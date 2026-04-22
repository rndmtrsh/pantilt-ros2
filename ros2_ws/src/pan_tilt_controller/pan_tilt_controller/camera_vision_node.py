#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
import urllib.request

from pan_tilt_controller.msg import ErrorMsg


class CameraVisionNode(Node):
    def __init__(self):
        super().__init__('camera_vision_node')

        # Parameters
        self.declare_parameter('camera_id', 0)
        self.declare_parameter('frame_width', 640)
        self.declare_parameter('frame_height', 480)
        self.declare_parameter('deadzone', 40)
        self.declare_parameter('show_debug', False)

        self.camera_id = self.get_parameter('camera_id').value
        self.frame_width = self.get_parameter('frame_width').value
        self.frame_height = self.get_parameter('frame_height').value
        self.deadzone = self.get_parameter('deadzone').value
        self.show_debug = self.get_parameter('show_debug').value

        self.frame_center_x = self.frame_width // 2
        self.frame_center_y = self.frame_height // 2

        # Publisher for error
        self.error_pub = self.create_publisher(ErrorMsg, '/vision/error', 10)

        # Optional debug image publisher
        self.debug_pub = self.create_publisher(Image, '/camera/debug_image', 10)
        self.bridge = CvBridge()

        # Load Haar cascade for upper body
        cascade_path = 'haarcascade_upperbody.xml'
        if not os.path.exists(cascade_path):
            self.get_logger().info('Downloading haarcascade_upperbody.xml...')
            url = 'https://raw.githubusercontent.com/opencv/opencv/master/data/haarcascades/haarcascade_upperbody.xml'
            urllib.request.urlretrieve(url, cascade_path)

        self.cascade = cv2.CascadeClassifier(cascade_path)
        if self.cascade.empty():
            self.get_logger().error('Failed to load Haar cascade!')
            raise RuntimeError('Haar cascade not loaded')

        # Open camera
        self.cap = cv2.VideoCapture(self.camera_id)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.frame_width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.frame_height)

        if not self.cap.isOpened():
            self.get_logger().error('Cannot open camera')
            raise RuntimeError('Camera not available')

        # Timer to read frames
        self.timer = self.create_timer(0.05, self.process_frame)  # ~20 Hz
        self.get_logger().info('Camera vision node started')

    def process_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn('Failed to grab frame')
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        bodies = self.cascade.detectMultiScale(gray, scaleFactor=1.1, minNeighbors=3)

        err_x = 0
        err_y = 0
        confidence = 0.0

        if len(bodies) > 0:
            # Take the first detected upper body (largest area is not necessary)
            (x, y, w, h) = bodies[0]
            cx = x + w // 2
            cy = y + h // 2
            err_x = cx - self.frame_center_x
            err_y = cy - self.frame_center_y

            # Apply deadzone
            if abs(err_x) < self.deadzone:
                err_x = 0
            if abs(err_y) < self.deadzone:
                err_y = 0

            confidence = 1.0

            if self.show_debug:
                # Draw bounding box and center point
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                cv2.circle(frame, (cx, cy), 5, (0, 0, 255), -1)
                cv2.putText(frame, f'Err: ({err_x}, {err_y})', (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        else:
            if self.show_debug:
                cv2.putText(frame, 'No upper body', (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

        # Publish error message
        msg = ErrorMsg()
        msg.err_x = err_x
        msg.err_y = err_y
        msg.confidence = confidence
        self.error_pub.publish(msg)

        # Publish debug image if needed
        if self.debug_pub.get_subscription_count() > 0 or self.show_debug:
            debug_img = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            self.debug_pub.publish(debug_img)

        if self.show_debug:
            cv2.imshow('Camera Debug', frame)
            cv2.waitKey(1)

    def destroy_node(self):
        self.cap.release()
        if self.show_debug:
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