#!/usr/bin/env python3
from pathlib import Path
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3Stamped
import cv2
import numpy as np
from ultralytics import YOLO


def _to_numpy(value):
    if hasattr(value, 'detach'):
        return value.detach().cpu().numpy()
    return np.asarray(value)


class CameraVisionNode(Node):
    def __init__(self):
        super().__init__('camera_vision_node')

        # Parameter
        self.declare_parameter('camera_id', 0)
        self.declare_parameter('frame_width', 854)
        self.declare_parameter('frame_height', 480)
        self.declare_parameter('deadzone', 0.30)          # rasio 0-1
        self.declare_parameter('show_debug', False)
        self.declare_parameter('flip_horizontal', True)
        self.declare_parameter('camera_backend', 'v4l2')
        self.declare_parameter('inference_rate', 10.0)    # Hz

        self.camera_id = self.get_parameter('camera_id').value
        self.frame_width = self.get_parameter('frame_width').value
        self.frame_height = self.get_parameter('frame_height').value
        self.deadzone = self.get_parameter('deadzone').value
        self.show_debug = self.get_parameter('show_debug').value
        self.flip_horizontal = self.get_parameter('flip_horizontal').value
        self.camera_backend = self.get_parameter('camera_backend').value
        self.inference_rate = self.get_parameter('inference_rate').value

        self.frame_center_x = self.frame_width // 2
        self.frame_center_y = self.frame_height // 2

        self.target_class_name = 'Human-body'
        self.model = None
        self.target_class_id = None

        self.error_pub = self.create_publisher(Vector3Stamped, '/vision_error', 10)
        self.add_on_set_parameters_callback(self.parameter_callback)

        # Kamera
        self.cap = self._open_camera()
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.frame_width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.frame_height)
        if not self.cap.isOpened():
            self.get_logger().error('Cannot open camera')
            raise RuntimeError('Camera not available')

        # Model
        model_path = (Path(__file__).resolve().parent / 'best.pt').resolve()
        if not model_path.exists():
            self.get_logger().error(f'Model not found: {model_path}')
            raise RuntimeError('Model not available')
        self.model = YOLO(str(model_path))
        self.target_class_id = self._resolve_target_class_id(self.target_class_name)
        if self.target_class_id is None:
            self.get_logger().error(f'Class not found: {self.target_class_name}')
            raise RuntimeError('Target class not available')

        # Rate limiter & state
        self._last_inference_time = self.get_clock().now()
        self._last_err_x = 0
        self._last_err_y = 0
        self._last_conf = 0.0
        self.last_detection = None   # (x1,y1,x2,y2,cx,cy,conf)

        # Debug
        self.video_writer = None
        self.video_output_path = Path.home() / 'metrics_logs/vision_debug.avi'
        self.video_fps = 20.0
        self.video_fourcc = cv2.VideoWriter_fourcc(*'XVID')

        self.timer = self.create_timer(0.05, self.process_frame)
        self.shutdown_requested = False
        self.get_logger().info('Camera vision node started')

    def _open_camera(self):
        backend = str(self.camera_backend).lower()
        candidates = [cv2.CAP_V4L2, cv2.CAP_ANY] if backend == 'v4l2' else [cv2.CAP_ANY]
        for api in candidates:
            cap = cv2.VideoCapture(self.camera_id, api)
            if cap.isOpened():
                self.get_logger().info(f'Camera opened (backend {api})')
                return cap
            cap.release()
        raise RuntimeError('Camera not available')

    def process_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn('Frame grab failed')
            return

        capture_time = self.get_clock().now()
        if self.flip_horizontal:
            frame = cv2.flip(frame, 1)

        frame_h, frame_w = frame.shape[:2]
        self.frame_center_x = frame_w // 2
        self.frame_center_y = frame_h // 2

        dead_x = int(frame_w * self.deadzone / 2.0)
        dead_y = int(frame_h * self.deadzone / 2.0)

        run_inference = True
        if self.inference_rate > 0.0:
            elapsed = (capture_time - self._last_inference_time).nanoseconds * 1e-9
            if elapsed < 1.0 / self.inference_rate:
                run_inference = False

        err_x_pub = self._last_err_x
        err_y_pub = self._last_err_y
        confidence = self._last_conf

        if run_inference:
            self._last_inference_time = capture_time
            results = self.model(frame, imgsz=480, conf=0.50, verbose=False)
            boxes = results[0].boxes if results else None
            target_boxes = np.empty((0, 4), dtype=float)
            target_conf = None

            if boxes is not None and len(boxes) > 0:
                boxes_xyxy = _to_numpy(boxes.xyxy)
                boxes_cls = _to_numpy(boxes.cls).reshape(-1).astype(int)
                boxes_conf = getattr(boxes, 'conf', None)
                if boxes_conf is not None:
                    boxes_conf = _to_numpy(boxes_conf).reshape(-1)

                if boxes_xyxy.size > 0:
                    mask = boxes_cls == self.target_class_id
                    target_boxes = boxes_xyxy[mask]
                    if boxes_conf is not None:
                        target_conf = boxes_conf[mask]

            if target_boxes.size > 0:
                areas = (target_boxes[:, 2] - target_boxes[:, 0]) * (target_boxes[:, 3] - target_boxes[:, 1])
                best_idx = int(np.argmax(areas))
                x1, y1, x2, y2 = target_boxes[best_idx]
                cx = int((x1 + x2) / 2)
                cy = int((y1 + y2) / 2)

                # Error relative to deadzone boundary (smooth correction)
                if cx > self.frame_center_x + dead_x:
                    err_x_raw = cx - (self.frame_center_x + dead_x)  # Positive: target right of deadzone
                elif cx < self.frame_center_x - dead_x:
                    err_x_raw = cx - (self.frame_center_x - dead_x)  # Negative: target left of deadzone
                else:
                    err_x_raw = 0

                if cy < self.frame_center_y - dead_y:
                    err_y_raw = (self.frame_center_y - dead_y) - cy  # Positive: target above deadzone
                elif cy > self.frame_center_y + dead_y:
                    err_y_raw = (self.frame_center_y + dead_y) - cy  # Negative: target below deadzone
                else:
                    err_y_raw = 0

                if target_conf is not None and len(target_conf) > best_idx:
                    confidence = float(target_conf[best_idx])
                else:
                    confidence = 1.0

                self.last_detection = (x1, y1, x2, y2, cx, cy, confidence)
            else:
                err_x_raw = 0
                err_y_raw = 0
                confidence = 0.0
                self.last_detection = None

            err_x_pub = err_x_raw
            err_y_pub = err_y_raw
            self._last_err_x = err_x_pub
            self._last_err_y = err_y_pub
            self._last_conf = confidence

        # Publish
        msg_vec = Vector3Stamped()
        msg_vec.header.stamp = capture_time.to_msg()
        msg_vec.vector.x = float(err_x_pub)
        msg_vec.vector.y = float(err_y_pub)
        msg_vec.vector.z = confidence
        self.error_pub.publish(msg_vec)

        # Debug visual
        if self.show_debug:
            disp = frame.copy()

            # Deadzone box
            pt1 = (self.frame_center_x - dead_x, self.frame_center_y - dead_y)
            pt2 = (self.frame_center_x + dead_x, self.frame_center_y + dead_y)
            cv2.rectangle(disp, pt1, pt2, (0, 255, 255), 1)

            # Crosshair pusat frame
            cv2.drawMarker(disp, (self.frame_center_x, self.frame_center_y),
                           (255, 0, 0), cv2.MARKER_CROSS, 20, 2)

            # Objek terdeteksi terakhir
            if self.last_detection is not None:
                x1, y1, x2, y2, cx, cy, conf = self.last_detection
                cv2.rectangle(disp, (int(x1), int(y1)), (int(x2), int(y2)),
                              (0, 255, 0), 2)
                cv2.circle(disp, (cx, cy), 5, (0, 0, 255), -1)
                cv2.putText(disp, f"conf:{conf:.2f}", (int(x1), max(20, int(y1)-10)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

            # Teks error (nilai yang dikirim ke PID)
            cv2.putText(disp, f"ERR PAN:{err_x_pub:+d} TILT:{err_y_pub:+d}",
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

            cv2.imshow("YOLO Tracking", disp)
            if self.video_writer is None:
                self.video_writer = cv2.VideoWriter(
                    str(self.video_output_path),
                    self.video_fourcc,
                    self.video_fps,
                    (frame_w, frame_h)
                )
            if self.video_writer is not None and self.video_writer.isOpened():
                self.video_writer.write(disp)

if cv2.waitKey(1) & 0xFF == ord('q'):
                self.shutdown_requested = True
                self.timer.cancel()

def destroy_node(self):
        self.timer.cancel()
        self.cap.release()
        if self.video_writer is not None:
            self.video_writer.release()
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraVisionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.timer.cancel()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()