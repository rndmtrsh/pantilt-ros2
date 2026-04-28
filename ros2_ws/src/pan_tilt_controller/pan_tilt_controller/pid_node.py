#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3
import time


class PIDNode(Node):
    def __init__(self):
        super().__init__('pid_node')

        self.declare_parameter('Kp', 0.5)
        self.declare_parameter('Ki', 0.01)
        self.declare_parameter('Kd', 0.1)
        self.declare_parameter('max_vel', 2000)
        self.declare_parameter('rate_limit', 200)
        self.declare_parameter('control_rate', 20)  # Hz

        self.Kp = self.get_parameter('Kp').value
        self.Ki = self.get_parameter('Ki').value
        self.Kd = self.get_parameter('Kd').value
        self.max_vel = self.get_parameter('max_vel').value
        self.rate_limit = self.get_parameter('rate_limit').value
        control_rate = self.get_parameter('control_rate').value
        self.dt = 1.0 / control_rate  # Periode timer dalam detik

        self.integral_x = 0.0
        self.integral_y = 0.0
        self.prev_err_x = 0.0
        self.prev_err_y = 0.0
        self.prev_pan_vel = 0.0
        self.prev_tilt_vel = 0.0

        # Error terbaru dari vision node
        self.current_err_x = 0.0
        self.current_err_y = 0.0
        self.confidence = 0.0

        # Subscribe to Vector3 (x=err_x, y=err_y, z=confidence)
        self.error_sub = self.create_subscription(Vector3, '/vision/error', self.error_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Create timer untuk control loop
        self.timer = self.create_timer(self.dt, self.control_loop)

        # Add callback untuk dynamic parameter updates
        self.add_on_set_parameters_callback(self.parameter_callback)

        self.get_logger().info(f'PID node started (pan direction reversed, rate={control_rate}Hz)')

    def error_callback(self, msg):
        """Simpan error terbaru dari vision node"""
        self.current_err_x = -msg.x  # invert X
        self.current_err_y = msg.y
        self.confidence = msg.z

    def control_loop(self):
        """Control loop utama dengan delta time yang terdefinisi"""
        if self.confidence == 0.0:
            pan_vel = 0.0
            tilt_vel = 0.0
        else:
            err_x = self.current_err_x
            err_y = self.current_err_y

            # --- PID PAN (sumbu X) dengan normalisasi waktu ---
            pan_p = self.Kp * err_x
            self.integral_x += err_x * self.dt 
            pan_i = self.Ki * self.integral_x
            pan_d = self.Kd * (err_x - self.prev_err_x) / self.dt 
            pan_vel = pan_p + pan_i + pan_d

            # --- PID TILT (sumbu Y) dengan normalisasi waktu ---
            tilt_p = self.Kp * err_y
            self.integral_y += err_y * self.dt 
            tilt_i = self.Ki * self.integral_y
            tilt_d = self.Kd * (err_y - self.prev_err_y) / self.dt  
            tilt_vel = tilt_p + tilt_i + tilt_d

            # Clamp kecepatan
            pan_vel = max(-self.max_vel, min(self.max_vel, pan_vel))
            tilt_vel = max(-self.max_vel, min(self.max_vel, tilt_vel))

            # Rate limiting (berdasarkan kecepatan terakhir yang DIKIRIM)
            pan_vel = self.prev_pan_vel + max(-self.rate_limit, min(self.rate_limit, pan_vel - self.prev_pan_vel))
            tilt_vel = self.prev_tilt_vel + max(-self.rate_limit, min(self.rate_limit, tilt_vel - self.prev_tilt_vel))

            # Update error sebelumnya
            self.prev_err_x = err_x
            self.prev_err_y = err_y

            # Anti‑windup (jika jenuh, kurangi integral yang baru ditambahkan)
            if pan_vel == self.max_vel or pan_vel == -self.max_vel:
                self.integral_x -= err_x * self.dt
            if tilt_vel == self.max_vel or tilt_vel == -self.max_vel:
                self.integral_y -= err_y * self.dt

        # Buat Twist, kirim ke serial
        twist = Twist()
        twist.linear.x = pan_vel
        twist.linear.y = tilt_vel
        self.cmd_pub.publish(twist)

        # Simpan kecepatan untuk rate limit berikutnya
        self.prev_pan_vel = pan_vel
        self.prev_tilt_vel = tilt_vel

    def parameter_callback(self, params):
        """Callback untuk mendengarkan perubahan parameter secara real-time"""
        from rcl_interfaces.msg import SetParametersResult
        
        for param in params:
            if param.name == 'Kp':
                self.Kp = param.value
                self.get_logger().info(f'Updated Kp = {self.Kp}')
            elif param.name == 'Ki':
                self.Ki = param.value
                self.get_logger().info(f'Updated Ki = {self.Ki}')
            elif param.name == 'Kd':
                self.Kd = param.value
                self.get_logger().info(f'Updated Kd = {self.Kd}')
            elif param.name == 'max_vel':
                self.max_vel = param.value
                self.get_logger().info(f'Updated max_vel = {self.max_vel}')
            elif param.name == 'rate_limit':
                self.rate_limit = param.value
                self.get_logger().info(f'Updated rate_limit = {self.rate_limit}')
        
        return SetParametersResult(successful=True)


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