#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3Stamped
from std_msgs.msg import Float32
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

        self.last_error_recv_time = None

        # Subscribers
        self.error_sub = self.create_subscription(Vector3Stamped, '/vision_error', self._error_cb, 10)
        self.cmd_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_callback, 10)
        self.latency_pub = self.create_publisher(Float32, '/latency/control_serial', 10)
        self.serial_write_pub = self.create_publisher(Float32, '/latency/serial_write', 10)
        self.get_logger().info('Serial node started')

    def _error_cb(self, msg):
        self.last_error_recv_time = self.get_clock().now()

    def cmd_callback(self, msg):
        pan_vel = int(msg.linear.x)
        tilt_vel = int(msg.linear.y)

        # Format: P+1000,T-500\n
        cmd_str = f"P{pan_vel:+d},T{tilt_vel:+d}\n"
        self.get_logger().debug(f'Sending: {cmd_str.strip()}')

        try:
            write_start = time.perf_counter()
            self.ser.write(cmd_str.encode())
            write_ms = (time.perf_counter() - write_start) * 1e3
        except serial.SerialException as e:
            self.get_logger().error(f'Serial write error: {e}')
            return

        write_msg = Float32()
        write_msg.data = float(write_ms)
        self.serial_write_pub.publish(write_msg)

        if self.last_error_recv_time is not None:
            now = self.get_clock().now()
            latency_ms = (now - self.last_error_recv_time).nanoseconds / 1e6
            latency_msg = Float32()
            latency_msg.data = float(latency_ms)
            self.latency_pub.publish(latency_msg)

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