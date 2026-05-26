#!/usr/bin/env python3
from pathlib import Path

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3
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
        self.declare_parameter('camera_id', 30)
        self.declare_parameter('frame_width', 640)
        self.declare_parameter('frame_height', 480)
        self.declare_parameter('deadzone', 40)
        self.declare_parameter('show_debug', False)
        self.declare_parameter('flip_horizontal', True)

        self.camera_id = self.get_parameter('camera_id').value
        self.frame_width = self.get_parameter('frame_width').value
        self.frame_height = self.get_parameter('frame_height').value
        self.deadzone = self.get_parameter('deadzone').value
        self.show_debug = self.get_parameter('show_debug').value
        self.flip_horizontal = self.get_parameter('flip_horizontal').value

        self.frame_center_x = self.frame_width // 2
        self.frame_center_y = self.frame_height // 2

        self.target_class_name = 'Human-body'
        self.model = None
        self.target_class_id = None

        # Publisher for error (Vector3: x=err_x, y=err_y, z=confidence)
        self.error_pub = self.create_publisher(Vector3, '/vision/error', 10)

        # Allow live parameter updates from ros2 param set
        self.add_on_set_parameters_callback(self.parameter_callback)

        # Open camera
        self.cap = cv2.VideoCapture(self.camera_id)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.frame_width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.frame_height)

        if not self.cap.isOpened():
            self.get_logger().error('Cannot open camera')
            raise RuntimeError('Camera not available')

        model_path = (Path(__file__).resolve().parent / 'best.pt').resolve()
        if not model_path.exists():
            self.get_logger().error(f'Model not found: {model_path}')
            raise RuntimeError('Model not available')

        self.model = YOLO(str(model_path))
        self.target_class_id = self._resolve_target_class_id(self.target_class_name)
        if self.target_class_id is None:
            self.get_logger().error(f'Class not found in model: {self.target_class_name}')
            raise RuntimeError('Target class not available')

        self.timer = self.create_timer(0.05, self.process_frame)
        self.get_logger().info('Camera vision node (YOLO) started')

    def process_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn('Failed to grab frame')
            return

        if self.flip_horizontal:
            frame = cv2.flip(frame, 1)

        frame_h, frame_w = frame.shape[:2]
        if frame_w != self.frame_width or frame_h != self.frame_height:
            self.frame_width = frame_w
            self.frame_height = frame_h
            self.frame_center_x = frame_w // 2
            self.frame_center_y = frame_h // 2

        err_x_pub = 0
        err_y_pub = 0
        confidence = 0.0

        results = self.model(frame, imgsz=480, conf=0.25, verbose=False)
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

        publish_debug = self.show_debug
        disp_frame = frame.copy() if publish_debug else None

        if target_boxes.size > 0:
            areas = (target_boxes[:, 2] - target_boxes[:, 0]) * (target_boxes[:, 3] - target_boxes[:, 1])
            best_idx = int(np.argmax(areas))
            x1, y1, x2, y2 = target_boxes[best_idx]
            cx = int((x1 + x2) / 2)
            cy = int((y1 + y2) / 2)

            err_x_raw = cx - self.frame_center_x
            err_y_raw = self.frame_center_y - cy  # positive = above center

            if abs(err_x_raw) >= self.deadzone:
                err_x_pub = err_x_raw
            if abs(err_y_raw) >= self.deadzone:
                err_y_pub = err_y_raw

            if target_conf is not None and len(target_conf) > best_idx:
                confidence = float(target_conf[best_idx])
            else:
                confidence = 1.0

            if publish_debug:
                x1i, y1i, x2i, y2i = map(int, (x1, y1, x2, y2))
                cv2.rectangle(disp_frame, (x1i, y1i), (x2i, y2i), (0, 255, 0), 2)
                cv2.circle(disp_frame, (cx, cy), 5, (0, 0, 255), -1)
                cv2.putText(disp_frame, f"conf:{confidence:.2f}", (x1i, max(20, y1i - 10)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                cv2.putText(disp_frame, f"err:(PAN:{err_x_raw:+d},TILT:{err_y_raw:+d})",
                            (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

        if publish_debug:
            cv2.drawMarker(disp_frame, (self.frame_center_x, self.frame_center_y),
                           (255, 0, 0), cv2.MARKER_CROSS, 20, 2)

        # Publish error (Vector3)
        msg_vec = Vector3()
        msg_vec.x = float(err_x_pub)
        msg_vec.y = float(err_y_pub)
        msg_vec.z = confidence
        self.error_pub.publish(msg_vec)

        if self.show_debug:
            cv2.imshow("YOLO Tracking", disp_frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                rclpy.shutdown()

    def parameter_callback(self, params):
        from rcl_interfaces.msg import SetParametersResult

        for param in params:
            if param.name == 'deadzone':
                self.deadzone = param.value
                self.get_logger().info(f'Updated deadzone = {self.deadzone}')

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
        self.cap.release()
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
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()