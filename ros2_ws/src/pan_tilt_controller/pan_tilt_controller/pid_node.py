#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3Stamped
from std_msgs.msg import Float32
import time


class PIDNode(Node):
    def __init__(self):
        super().__init__('pid_node')

        # Parameter
        self.declare_parameter('Kp', 0.5)
        self.declare_parameter('Ki', 0.01)
        self.declare_parameter('Kd', 0.1)
        self.declare_parameter('max_vel', 2000)
        self.declare_parameter('accel_limit', 2500.0)   # akselerasi maks (vel/detik)
        self.declare_parameter('control_rate', 20)

        self.Kp = self.get_parameter('Kp').value
        self.Ki = self.get_parameter('Ki').value
        self.Kd = self.get_parameter('Kd').value
        self.max_vel = self.get_parameter('max_vel').value
        self.accel_limit = self.get_parameter('accel_limit').value
        control_rate = self.get_parameter('control_rate').value
        self.dt = 1.0 / control_rate

        # State PID
        self.integral_x = 0.0
        self.integral_y = 0.0
        self.prev_err_x = 0.0
        self.prev_err_y = 0.0
        self.prev_pan_vel = 0.0
        self.prev_tilt_vel = 0.0

        # Error terbaru
        self.current_err_x = 0.0
        self.current_err_y = 0.0
        self.confidence = 0.0

        # ROS
        self.error_sub = self.create_subscription(
            Vector3Stamped, '/vision_error', self.error_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.timer = self.create_timer(self.dt, self.control_loop)
        self.add_on_set_parameters_callback(self.parameter_callback)

        self.control_latency_pub = self.create_publisher(
            Float32, '/latency/control_compute', 10)
        self.get_logger().info(
            f'PID node started (accel_limit={self.accel_limit} vel/s, rate={control_rate}Hz)')

    def error_callback(self, msg):
        self.current_err_x = -msg.vector.x   # pan (invert)
        self.current_err_y = msg.vector.y    # tilt
        self.confidence = msg.vector.z

    def control_loop(self):
        start_time = time.perf_counter()

        # --- Tentukan target velocity (raw) ---
        if self.confidence == 0.0:
            # Kehilangan target → berhenti seketika
            raw_pan = 0.0
            raw_tilt = 0.0
            # Reset state sepenuhnya
            self.integral_x = 0.0
            self.integral_y = 0.0
            self.prev_err_x = 0.0
            self.prev_err_y = 0.0
            self.prev_pan_vel = 0.0
            self.prev_tilt_vel = 0.0

        elif self.current_err_x == 0.0 and self.current_err_y == 0.0:
            # Di dalam deadzone → target velocity 0, perlambatan halus
            raw_pan = 0.0
            raw_tilt = 0.0
            # Reset integrator agar tidak menumpuk, tapi jangan reset prev_vel
            self.integral_x = 0.0
            self.integral_y = 0.0
            self.prev_err_x = 0.0
            self.prev_err_y = 0.0

        else:
            err_x = self.current_err_x
            err_y = self.current_err_y

            # PID PAN
            pan_p = self.Kp * err_x
            self.integral_x += err_x * self.dt
            pan_i = self.Ki * self.integral_x
            pan_d = self.Kd * (err_x - self.prev_err_x) / self.dt
            raw_pan = pan_p + pan_i + pan_d

            # PID TILT
            tilt_p = self.Kp * err_y
            self.integral_y += err_y * self.dt
            tilt_i = self.Ki * self.integral_y
            tilt_d = self.Kd * (err_y - self.prev_err_y) / self.dt
            raw_tilt = tilt_p + tilt_i + tilt_d

            # Anti‑windup saat jenuh (sebelum akselerasi limit)
            if abs(raw_pan) >= self.max_vel:
                self.integral_x -= err_x * self.dt
            if abs(raw_tilt) >= self.max_vel:
                self.integral_y -= err_y * self.dt

            self.prev_err_x = err_x
            self.prev_err_y = err_y

        # Clamp kecepatan maksimum
        raw_pan = max(-self.max_vel, min(self.max_vel, raw_pan))
        raw_tilt = max(-self.max_vel, min(self.max_vel, raw_tilt))

        # --- Akselerasi halus (acceleration limiting) ---
        if self.accel_limit > 0.0:
            max_delta = self.accel_limit * self.dt

            # Pan
            delta_pan = raw_pan - self.prev_pan_vel
            if delta_pan > max_delta:
                delta_pan = max_delta
            elif delta_pan < -max_delta:
                delta_pan = -max_delta
            pan_vel = self.prev_pan_vel + delta_pan

            # Tilt
            delta_tilt = raw_tilt - self.prev_tilt_vel
            if delta_tilt > max_delta:
                delta_tilt = max_delta
            elif delta_tilt < -max_delta:
                delta_tilt = -max_delta
            tilt_vel = self.prev_tilt_vel + delta_tilt
        else:
            # Tanpa batas akselerasi → langsung ikuti raw
            pan_vel = raw_pan
            tilt_vel = raw_tilt

        # Simpan kecepatan terkirim
        self.prev_pan_vel = pan_vel
        self.prev_tilt_vel = tilt_vel

        # Hitung latency
        compute_ms = (time.perf_counter() - start_time) * 1e3

        # Publikasikan
        twist = Twist()
        twist.linear.x = pan_vel
        twist.linear.y = tilt_vel
        self.cmd_pub.publish(twist)

        latency_msg = Float32()
        latency_msg.data = float(compute_ms)
        self.control_latency_pub.publish(latency_msg)

    def parameter_callback(self, params):
        from rcl_interfaces.msg import SetParametersResult
        for param in params:
            if param.name == 'Kp':
                self.Kp = param.value
            elif param.name == 'Ki':
                self.Ki = param.value
            elif param.name == 'Kd':
                self.Kd = param.value
            elif param.name == 'max_vel':
                self.max_vel = param.value
            elif param.name == 'accel_limit':
                self.accel_limit = param.value
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