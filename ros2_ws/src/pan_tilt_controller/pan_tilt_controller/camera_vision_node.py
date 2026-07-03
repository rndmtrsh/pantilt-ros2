#!/usr/bin/env python3
from pathlib import Path
from datetime import datetime
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3Stamped
import cv2
import numpy as np
from ultralytics import YOLO


class CameraVisionNode(Node):
    def __init__(self):
        super().__init__('camera_vision_node')

        # --- Parameters  ---
        self.declare_parameter('camera_id', 1)
        self.declare_parameter('frame_width', 854)
        self.declare_parameter('frame_height', 480)
        self.declare_parameter('deadzone_h', 0.40)
        self.declare_parameter('deadzone_v', 0.40)
        self.declare_parameter('show_debug', False)
        self.declare_parameter('flip_horizontal', True)
        self.declare_parameter('camera_backend', 'v4l2')
        self.declare_parameter('capture_rate', 30.0)
        self.declare_parameter('inference_rate', 10.0)
        self.declare_parameter('max_jump', 200)            
        self.declare_parameter('reacquire_timeout', 0.5)
        self.declare_parameter('vertical_ref_ratio', 0.33)

        # --- Video recording parameters ---
        self.declare_parameter('record_video', False)   
        self.declare_parameter('video_output_dir', '~/metrics_logs')
        self.declare_parameter('video_fps', 24.0)

        # Cache parameters
        self.camera_id = self.get_parameter('camera_id').value
        self.frame_width = self.get_parameter('frame_width').value
        self.frame_height = self.get_parameter('frame_height').value
        self.deadzone_h = self.get_parameter('deadzone_h').value
        self.deadzone_v = self.get_parameter('deadzone_v').value
        self.show_debug = self.get_parameter('show_debug').value
        self.flip_horizontal = self.get_parameter('flip_horizontal').value
        self.camera_backend = self.get_parameter('camera_backend').value
        self.capture_rate = self.get_parameter('capture_rate').value
        self.inference_rate = self.get_parameter('inference_rate').value
        self.max_jump = self.get_parameter('max_jump').value
        self.reacquire_timeout = self.get_parameter('reacquire_timeout').value
        self.vertical_ref_ratio = self.get_parameter('vertical_ref_ratio').value

        # Video recording parameters
        self.record_video = self.get_parameter('record_video').value
        self.video_output_dir = Path(self.get_parameter('video_output_dir').value).expanduser().resolve()
        self.video_fps = self.get_parameter('video_fps').value

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
        self.last_detection = None
        self.last_valid_center = None
        self.last_valid_time = self.get_clock().now()

        # Video writer
        self.video_writer = None
        self.video_path = None
        if self.record_video:
            self.video_output_dir.mkdir(parents=True, exist_ok=True)
            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            self.video_path = self.video_output_dir / f'vision_track_{timestamp}.avi'
            self.get_logger().info(f'Video recording enabled, output: {self.video_path}')

        # Timer
        self.timer = self.create_timer(1.0 / self.capture_rate, self.process_frame)
        self.get_logger().info('Camera vision node started (video recording: {})'.format(
            'ON' if self.record_video else 'OFF'))

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

        dead_x = int(frame_w * self.deadzone_h / 2.0)
        dead_y = int(frame_h * self.deadzone_v / 2.0)
        center_x = frame_w // 2
        center_y = frame_h // 2

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
                conf = float(target_conf[best_idx]) if target_conf is not None and len(target_conf) > best_idx else 1.0

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

                    left_bound = center_x - dead_x
                    right_bound = center_x + dead_x
                    top_bound = center_y - dead_y
                    bottom_bound = center_y + dead_y

                    err_x_raw = 0.0
                    if cx < left_bound:
                        err_x_raw = cx - left_bound
                    elif cx > right_bound:
                        err_x_raw = cx - right_bound

                    err_y_raw = 0.0
                    if ref_y < top_bound:
                        err_y_raw = top_bound - ref_y
                    elif ref_y > bottom_bound:
                        err_y_raw = bottom_bound - ref_y

                    err_x_pub = err_x_raw
                    err_y_pub = err_y_raw
                    confidence = conf
                    self.last_detection = (x1, y1, x2, y2, cx, ref_y, conf)

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

        # --- Prepare display frame (always build for video recording) ---
        build_display = self.show_debug or self.record_video
        if build_display:
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

            if self.show_debug:
                cv2.imshow("YOLO Tracking", disp)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    rclpy.shutdown()

            if self.record_video:
                if self.video_writer is None:
                    fourcc = cv2.VideoWriter_fourcc(*'XVID')
                    self.video_writer = cv2.VideoWriter(
                        str(self.video_path),
                        fourcc,
                        self.video_fps,
                        (frame_w, frame_h)
                    )
                    if not self.video_writer.isOpened():
                        self.get_logger().error('Failed to open video writer')
                        self.video_writer = None
                    else:
                        self.get_logger().info(f'Video writer initialized: {self.video_path}')
                if self.video_writer is not None and self.video_writer.isOpened():
                    self.video_writer.write(disp)

    def parameter_callback(self, params):
        from rcl_interfaces.msg import SetParametersResult
        for param in params:
            if param.name == 'deadzone_h':
                self.deadzone_h = param.value
            elif param.name == 'deadzone_v':
                self.deadzone_v = param.value
            elif param.name == 'vertical_ref_ratio':
                self.vertical_ref_ratio = param.value
            elif param.name == 'inference_rate':
                self.inference_rate = param.value
            elif param.name == 'max_jump':
                self.max_jump = param.value
            elif param.name == 'reacquire_timeout':
                self.reacquire_timeout = param.value
            elif param.name == 'show_debug':
                self.show_debug = param.value
            elif param.name == 'flip_horizontal':
                self.flip_horizontal = param.value
            elif param.name == 'capture_rate':
                self.capture_rate = param.value
                # Aman: batalkan timer lama dan buat baru
                if hasattr(self, 'timer') and self.timer is not None:
                    try:
                        self.timer.cancel()
                    except rclpy._rclpy_pybind11.InvalidHandle:
                        pass
                self.timer = self.create_timer(1.0 / self.capture_rate, self.process_frame)
            elif param.name == 'record_video':
                self.record_video = param.value
                if self.record_video and self.video_writer is not None:
                    self.video_writer.release()
                    self.video_writer = None
                    timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
                    self.video_path = self.video_output_dir / f'vision_track_{timestamp}.avi'
                    self.get_logger().info(f'Video recording enabled, output: {self.video_path}')
                elif not self.record_video and self.video_writer is not None:
                    self.video_writer.release()
                    self.video_writer = None
                    self.get_logger().info('Video recording disabled')
            elif param.name == 'video_fps':
                self.video_fps = param.value
            elif param.name == 'video_output_dir':
                self.video_output_dir = Path(param.value).expanduser().resolve()
                if self.record_video:
                    self.video_output_dir.mkdir(parents=True, exist_ok=True)
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
        # Hancurkan timer dengan aman
        if hasattr(self, 'timer') and self.timer is not None:
            try:
                self.timer.cancel()
            except rclpy._rclpy_pybind11.InvalidHandle:
                # Timer sudah tidak valid, abaikan
                pass
            except Exception:
                pass

        # Lepaskan kamera
        if hasattr(self, 'cap') and self.cap is not None:
            self.cap.release()

        # Tutup video writer
        if hasattr(self, 'video_writer') and self.video_writer is not None:
            try:
                self.video_writer.release()
                self.get_logger().info(f'Video saved: {self.video_path}')
            except Exception:
                pass

        cv2.destroyAllWindows()

        # Panggil destroy_node dari base class
        try:
            super().destroy_node()
        except Exception:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = CameraVisionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def _to_numpy(value):
    if hasattr(value, 'detach'):
        return value.detach().cpu().numpy()
    return np.asarray(value)


if __name__ == '__main__':
    main()