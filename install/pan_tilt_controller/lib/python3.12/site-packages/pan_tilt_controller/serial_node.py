#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial # type: ignore
import time


class SerialNode(Node):
    def __init__(self):
        super().__init__('serial_node')

        # Parameters
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 115200)

        port = self.get_parameter('port').value
        baudrate = self.get_parameter('baudrate').value

        # Open serial port
        try:
            self.ser = serial.Serial(port, baudrate, timeout=1.0)
            time.sleep(2)  # Wait for Arduino/STM32 reset
            self.get_logger().info(f'Serial port {port} opened at {baudrate} baud')
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to open serial port: {e}')
            raise

        # Subscriber
        self.cmd_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_callback, 10)
        self.get_logger().info('Serial node started')

    def cmd_callback(self, msg):
        pan_vel = int(msg.linear.x)
        tilt_vel = int(msg.linear.y)

        # Format: P+1000,T-500\n
        cmd_str = f"P{pan_vel:+d},T{tilt_vel:+d}\n"
        self.get_logger().debug(f'Sending: {cmd_str.strip()}')

        try:
            self.ser.write(cmd_str.encode())
        except serial.SerialException as e:
            self.get_logger().error(f'Serial write error: {e}')

        # Optionally read response (if STM32 replies)
        # if self.ser.in_waiting > 0:
        #     response = self.ser.readline().decode().strip()
        #     self.get_logger().info(f'STM32 response: {response}')

    def destroy_node(self):
        if hasattr(self, 'ser') and self.ser.is_open:
            self.ser.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SerialNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()