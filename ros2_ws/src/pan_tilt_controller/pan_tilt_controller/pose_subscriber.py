#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from turtlesim_msgs.msg import Pose

class PoseSubcriberNode(Node):
    def __init__(self):
        super().__init__("pose_subscriber")
        self.get_logger().info("Pose Subscriber Node has been started")
        self.pose_subscriber_ = self.create_subscription(
            Pose, "turtle1/pose", self.pose_callback, 10)

    def pose_callback(self, msg):
        self.get_logger().info(f"Pose received: x={msg.x}, y={msg.y}, theta={msg.theta}")

def main(args=None):
    rclpy.init(args=args)

    node = PoseSubcriberNode()
    rclpy.spin(node)

    rclpy.shutdown()