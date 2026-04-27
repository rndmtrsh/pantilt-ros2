#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3


class PIDNode(Node):
    def __init__(self):
        super().__init__('pid_node')

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

        self.integral_x = 0.0
        self.integral_y = 0.0
        self.prev_err_x = 0.0
        self.prev_err_y = 0.0
        self.prev_pan_vel = 0.0
        self.prev_tilt_vel = 0.0

        # Subscribe to Vector3 (x=err_x, y=err_y, z=confidence)
        self.error_sub = self.create_subscription(Vector3, '/vision/error', self.error_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.get_logger().info('PID node started (pan direction reversed)')

    def error_callback(self, msg):
        # AMBIL ERROR MENTAH, LALU BALIK TANDA UNTUK PAN (sumbu X)
        err_x_raw = msg.x
        err_y = msg.y
        confidence = msg.z

        # Balik arah pan: objek di kanan (err_x positif) → motor pan bergerak KIRI (kecepatan negatif)
        # Jika diinginkan sebaliknya, komentari baris berikut
        err_x = -err_x_raw

        if confidence == 0.0:
            pan_vel = 0.0
            tilt_vel = 0.0
        else:
            # --- PID PAN (sumbu X) ---
            pan_p = self.Kp * err_x
            self.integral_x += err_x
            pan_i = self.Ki * self.integral_x
            pan_d = self.Kd * (err_x - self.prev_err_x)
            pan_vel = pan_p + pan_i + pan_d

            # --- PID TILT (sumbu Y) ---
            tilt_p = self.Kp * err_y
            self.integral_y += err_y
            tilt_i = self.Ki * self.integral_y
            tilt_d = self.Kd * (err_y - self.prev_err_y)
            tilt_vel = tilt_p + tilt_i + tilt_d

            # Clamp kecepatan
            pan_vel = max(-self.max_vel, min(self.max_vel, pan_vel))
            tilt_vel = max(-self.max_vel, min(self.max_vel, tilt_vel))

            # Rate limiting (berdasarkan kecepatan terakhir yang DIKIRIM)
            pan_vel = self.prev_pan_vel + max(-self.rate_limit, min(self.rate_limit, pan_vel - self.prev_pan_vel))
            tilt_vel = self.prev_tilt_vel + max(-self.rate_limit, min(self.rate_limit, tilt_vel - self.prev_tilt_vel))

            # Update error sebelumnya (pakai error yang sudah dibalik)
            self.prev_err_x = err_x
            self.prev_err_y = err_y

        # Buat Twist, kirim ke serial
        twist = Twist()
        twist.linear.x = pan_vel
        twist.linear.y = tilt_vel
        self.cmd_pub.publish(twist)

        # Simpan kecepatan untuk rate limit berikutnya
        self.prev_pan_vel = pan_vel
        self.prev_tilt_vel = tilt_vel

        # Anti‑windup (jika jenuh, kurangi integral yang baru ditambahkan)
        if pan_vel == self.max_vel or pan_vel == -self.max_vel:
            self.integral_x -= err_x
        if tilt_vel == self.max_vel or tilt_vel == -self.max_vel:
            self.integral_y -= err_y


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