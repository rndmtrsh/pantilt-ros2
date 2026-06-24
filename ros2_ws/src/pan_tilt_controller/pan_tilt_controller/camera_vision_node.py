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

        # --- Parameters matching launch file ---
        self.declare_parameter('camera_id', 0)
        self.declare_parameter('frame_width', 854)
        self.declare_parameter('frame_height', 480)
        self.declare_parameter('deadzone_h', 0.40)
        self.declare_parameter('deadzone_v', 0.40)
        self.declare_parameter('show_debug', False)
        self.declare_parameter('flip_horizontal', True)
        self.declare_parameter('camera_backend', 'v4l2')
        self.declare_parameter('capture_rate', 30.0)
        self.declare_parameter('inference_rate', 10.0)
        self.declare_parameter('max_jump', 200)            # increased from 80
        self.declare_parameter('reacquire_timeout', 0.5)
        self.declare_parameter('vertical_ref_ratio', 0.33)

        # Cache
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

        # Model YOLO
        model_path = (Path(__file__).resolve().parent / 'best.pt').resolve()
        if not model_path.exists():
            self.get_logger().error(f'Model not found: {model_path}')
            raise RuntimeError('Model not available')
        self.model = YOLO(str(model_path))
        self.target_class_id = self._resolve_target_class_id(self.target_class_name)
        if self.target_class_id is None:
            self.get_logger().error(f'Class not found: {self.target_class_name}')
            raise RuntimeError('Target class not available')

        # State variables
        self._last_inference_time = self.get_clock().now()
        self._last_err_x = 0
        self._last_err_y = 0
        self._last_conf = 0.0
        self.last_detection = None          # (x1,y1,x2,y2,cx,ref_y,conf)
        self.last_valid_center = None       # (cx, cy) for jump filter
        self.last_valid_time = self.get_clock().now()

        # Debug
        self.video_writer = None
        self.video_output_path = Path.home() / 'metrics_logs/vision_debug.avi'
        self.video_fps = 20.0
        self.video_fourcc = cv2.VideoWriter_fourcc(*'XVID')

        # Timer
        self.timer = self.create_timer(1.0 / self.capture_rate, self.process_frame)
        self.add_on_set_parameters_callback(self.parameter_callback)
        self.get_logger().info('Camera vision node started (separate H/V deadzones, improved jump filter)')

    def _open_camera(self):
        backend = str(self.camera_backend).lower()
        candidates = [cv2.CAP_V4L2, cv2.CAP_ANY] if backend == 'v4l2' else [cv2.CAP_ANY]
        for api in candidates:
            cap = cv2.VideoCapture(self.camera_id, api)
            if cap.isOpened():
                return cap
            cap.release()
        raise RuntimeError('Camera not available')

    def process_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            return

        capture_time = self.get_clock().now()
        if self.flip_horizontal:
            frame = cv2.flip(frame, 1)

        frame_h, frame_w = frame.shape[:2]

        # Deadzone half-sizes in pixels
        dead_x = int(frame_w * self.deadzone_h / 2.0)
        dead_y = int(frame_h * self.deadzone_v / 2.0)
        center_x = frame_w // 2
        center_y = frame_h // 2

        # Inference rate limiter
        run_inference = True
        if self.inference_rate > 0.0:
            elapsed = (capture_time - self._last_inference_time).nanoseconds * 1e-9
            if elapsed < 1.0 / self.inference_rate:
                run_inference = False

        # Default: tahan error terakhir
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

            # Pilih deteksi terbaik
            if target_boxes.size > 0:
                areas = (target_boxes[:, 2] - target_boxes[:, 0]) * (target_boxes[:, 3] - target_boxes[:, 1])
                best_idx = int(np.argmax(areas))
                x1, y1, x2, y2 = target_boxes[best_idx]
                cx = int((x1 + x2) / 2)
                cy = int((y1 + y2) / 2)
                conf = float(target_conf[best_idx]) if target_conf is not None and len(target_conf) > best_idx else 1.0

                # --- Jump filter (more tolerant now) ---
                time_since_last_valid = (capture_time - self.last_valid_time).nanoseconds * 1e-9
                accept = False
                if self.last_valid_center is None or time_since_last_valid > self.reacquire_timeout:
                    accept = True
                else:
                    dx = cx - self.last_valid_center[0]
                    dy = cy - self.last_valid_center[1]
                    if np.sqrt(dx*dx + dy*dy) <= self.max_jump:
                        accept = True

                if accept:
                    self.last_valid_center = (cx, cy)
                    self.last_valid_time = capture_time
                    ref_y = int(y1 + (y2 - y1) * self.vertical_ref_ratio)

                    # --- Smooth edge-based error ---
                    left_bound = center_x - dead_x
                    right_bound = center_x + dead_x
                    top_bound = center_y - dead_y
                    bottom_bound = center_y + dead_y

                    # Horizontal error: uses bbox centre (cx)
                    err_x_raw = 0.0
                    if cx < left_bound:
                        err_x_raw = cx - left_bound      # negative → pan left
                    elif cx > right_bound:
                        err_x_raw = cx - right_bound     # positive → pan right

                    # Vertical error: uses ref point (ref_y)
                    err_y_raw = 0.0
                    if ref_y < top_bound:
                        err_y_raw = top_bound - ref_y    # positive → tilt UP
                    elif ref_y > bottom_bound:
                        err_y_raw = bottom_bound - ref_y # negative → tilt DOWN

                    err_x_pub = err_x_raw
                    err_y_pub = err_y_raw
                    confidence = conf
                    self.last_detection = (x1, y1, x2, y2, cx, ref_y, conf)

            # Store for next non-inference frames
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

        # Debug window
        if self.show_debug:
            disp = frame.copy()
            cv2.rectangle(disp,
                          (center_x - dead_x, center_y - dead_y),
                          (center_x + dead_x, center_y + dead_y),
                          (0, 255, 255), 1)
            cv2.drawMarker(disp, (center_x, center_y),
                           (255, 0, 0), cv2.MARKER_CROSS, 20, 2)
            if self.last_detection is not None:
                x1, y1, x2, y2, cx, ref_y, conf = self.last_detection
                cv2.rectangle(disp, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
                cv2.circle(disp, (cx, ref_y), 5, (0, 0, 255), -1)
                cv2.putText(disp, f"conf:{conf:.2f}", (int(x1), max(20, int(y1)-10)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            cv2.putText(disp, f"ERR PAN:{err_x_pub:+.1f} TILT:{err_y_pub:+.1f}",
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
                rclpy.shutdown()

    def parameter_callback(self, params):
        from rcl_interfaces.msg import SetParametersResult
        for param in params:
            if param.name in ('deadzone_h', 'deadzone_v', 'vertical_ref_ratio',
                              'inference_rate', 'max_jump', 'reacquire_timeout',
                              'show_debug', 'flip_horizontal'):
                setattr(self, param.name, param.value)
            elif param.name == 'capture_rate':
                self.capture_rate = param.value
                self.timer.cancel()
                self.timer = self.create_timer(1.0 / self.capture_rate, self.process_frame)
        return SetParametersResult(successful=True)

    def _resolve_target_class_id(self, target_class_name):
        model_names = getattr(self.model, 'names', None)
        if isinstance(model_names, dict):
            for class_id, name in model_names.items():
                if name == target_class_name:
                    return int(class_id)
        elif isinstance(model_names, (list, tuple)):
            try:
                return model_names.index(target_class_name)
            except ValueError:
                return None
        return None

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