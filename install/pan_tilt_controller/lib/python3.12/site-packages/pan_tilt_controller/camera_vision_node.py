#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from geometry_msgs.msg import Vector3
import cv2
import numpy as np


class CameraVisionNode(Node):
    def __init__(self):
        super().__init__('camera_vision_node')

        # Parameter
        self.declare_parameter('camera_id', 0)
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

        # Visualisation large zone
        self.viz_zone_w = self.frame_width // 2
        self.viz_zone_h = self.frame_height // 2
        self.viz_zone_x = self.frame_center_x - self.viz_zone_w // 2
        self.viz_zone_y = self.frame_center_y - self.viz_zone_h // 2

        # Publisher for error (Vector3: x=err_x, y=err_y, z=confidence)
        self.error_pub = self.create_publisher(Vector3, '/vision/error', 10)

        # Debug image publisher
        self.debug_pub = self.create_publisher(Image, '/camera/debug_image', 10)
        self.bridge = CvBridge()

        # Open camera
        self.cap = cv2.VideoCapture(self.camera_id)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.frame_width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.frame_height)

        if not self.cap.isOpened():
            self.get_logger().error('Cannot open camera')
            raise RuntimeError('Camera not available')

        self.timer = self.create_timer(0.05, self.process_frame)
        self.get_logger().info('Camera vision node (green tracker) started')

    def process_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn('Failed to grab frame')
            return

        if self.flip_horizontal:
            frame = cv2.flip(frame, 1)

        # --- Green detection ---
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        low_green = np.array([35, 80, 80])
        high_green = np.array([85, 255, 255])
        mask = cv2.inRange(hsv, low_green, high_green)
        mask = cv2.erode(mask, None, iterations=1)
        mask = cv2.dilate(mask, None, iterations=2)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        err_x_pub = 0
        err_y_pub = 0
        confidence = 0.0

        # Default state (jika tidak ada deteksi)
        state = "NO TARGET"
        state_color = (0, 0, 255)

        disp_frame = frame.copy()

        if contours:
            largest = max(contours, key=cv2.contourArea)
            if cv2.contourArea(largest) > 500:
                x, y, w, h = cv2.boundingRect(largest)
                cx = x + w // 2
                cy = y + h // 2

                err_x_raw = cx - self.frame_center_x
                err_y_raw = self.frame_center_y - cy   # positive = above center

                # Deadzone
                if abs(err_x_raw) < self.deadzone:
                    err_x_pub = 0
                else:
                    err_x_pub = err_x_raw
                if abs(err_y_raw) < self.deadzone:
                    err_y_pub = 0
                else:
                    err_y_pub = err_y_raw

                confidence = 1.0

                if self.show_debug or self.debug_pub.get_subscription_count() > 0:
                    cv2.rectangle(disp_frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                    cv2.circle(disp_frame, (cx, cy), 5, (0, 0, 255), -1)
                    cv2.putText(disp_frame, f"err:(PAN:{err_x_raw:+d},TILT:{err_y_raw:+d})",
                                (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

                    # Check if inside visual zone
                    if (self.viz_zone_x <= cx <= self.viz_zone_x + self.viz_zone_w and
                        self.viz_zone_y <= cy <= self.viz_zone_y + self.viz_zone_h):
                        state = "STEADY"
                        state_color = (0, 255, 0)
                    else:
                        state = "MOVING"
                        state_color = (0, 165, 255)

        # --- Draw static elements (always, jika debug aktif) ---
        if self.show_debug or self.debug_pub.get_subscription_count() > 0:
            cv2.rectangle(disp_frame,
                          (self.viz_zone_x, self.viz_zone_y),
                          (self.viz_zone_x + self.viz_zone_w, self.viz_zone_y + self.viz_zone_h),
                          (255, 255, 0), 1)
            cv2.putText(disp_frame, state, (self.viz_zone_x, self.viz_zone_y - 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, state_color, 2)
            cv2.drawMarker(disp_frame, (self.frame_center_x, self.frame_center_y),
                           (255, 0, 0), cv2.MARKER_CROSS, 20, 2)

            debug_msg = self.bridge.cv2_to_imgmsg(disp_frame, encoding='bgr8')
            self.debug_pub.publish(debug_msg)

        # Publish error (Vector3)
        msg_vec = Vector3()
        msg_vec.x = float(err_x_pub)
        msg_vec.y = float(err_y_pub)
        msg_vec.z = confidence
        self.error_pub.publish(msg_vec)

        if self.show_debug:
            cv2.imshow("Green Tracking", disp_frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                rclpy.shutdown()

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