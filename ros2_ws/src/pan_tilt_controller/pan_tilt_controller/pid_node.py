#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

from pan_tilt_controller.msg import ErrorMsg


class PIDNode(Node):
    def __init__(self):
        super().__init__('pid_node')

        # Parameters
        self.declare_parameter('Kp', 0.5)
        self.declare_parameter('Ki', 0.01)
        self.declare_parameter('Kd', 0.1)
        self.declare_parameter('max_vel', 2000)
        self.declare_parameter('rate_limit', 200)

        self.Kp = self.get_parameter('Kp').value
        self.Ki = self.get_parameter('Ki').value
        self.Kd = self.get_parameter('Kd').value
        self.max_vel = self.get_parameter('max_vel').value
        self.rate_limit = self.get_parameter('rate_limit').value

        # PID state
        self.integral_x = 0.0
        self.integral_y = 0.0
        self.prev_err_x = 0.0
        self.prev_err_y = 0.0
        self.prev_pan_vel = 0.0
        self.prev_tilt_vel = 0.0

        # Subscriber and publisher
        self.error_sub = self.create_subscription(ErrorMsg, '/vision/error', self.error_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.get_logger().info('PID node started')

    def error_callback(self, msg):
        # Only compute if confidence > 0 (body detected)
        if msg.confidence == 0.0:
            # No detection, gradually reduce velocity (optional)
            pan_vel = 0.0
            tilt_vel = 0.0
        else:
            # PID for pan (X)
            pan_p = self.Kp * msg.err_x
            self.integral_x += msg.err_x
            pan_i = self.Ki * self.integral_x
            pan_d = self.Kd * (msg.err_x - self.prev_err_x)
            pan_vel = pan_p + pan_i + pan_d

            # PID for tilt (Y)
            tilt_p = self.Kp * msg.err_y
            self.integral_y += msg.err_y
            tilt_i = self.Ki * self.integral_y
            tilt_d = self.Kd * (msg.err_y - self.prev_err_y)
            tilt_vel = tilt_p + tilt_i + tilt_d

            # Clamp to max velocity
            pan_vel = max(-self.max_vel, min(self.max_vel, pan_vel))
            tilt_vel = max(-self.max_vel, min(self.max_vel, tilt_vel))

            # Rate limiting
            pan_vel = self.prev_pan_vel + max(-self.rate_limit, min(self.rate_limit, pan_vel - self.prev_pan_vel))
            tilt_vel = self.prev_tilt_vel + max(-self.rate_limit, min(self.rate_limit, tilt_vel - self.prev_tilt_vel))

            # Update previous errors
            self.prev_err_x = msg.err_x
            self.prev_err_y = msg.err_y

        # Publish command
        twist = Twist()
        twist.linear.x = pan_vel   # pan velocity
        twist.linear.y = tilt_vel  # tilt velocity
        self.cmd_pub.publish(twist)

        # Store current velocities for next rate limit
        self.prev_pan_vel = pan_vel
        self.prev_tilt_vel = tilt_vel

        # Simple anti-windup: if velocity saturated, do not accumulate integral
        if pan_vel == self.max_vel or pan_vel == -self.max_vel:
            self.integral_x -= msg.err_x
        if tilt_vel == self.max_vel or tilt_vel == -self.max_vel:
            self.integral_y -= msg.err_y


def main(args=None):
    rclpy.init(args=args)
    node = PIDNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()