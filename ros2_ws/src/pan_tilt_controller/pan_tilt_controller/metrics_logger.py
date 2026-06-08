#!/usr/bin/env python3
import csv
import math
import time
from datetime import datetime
from pathlib import Path

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3Stamped
from std_msgs.msg import Float32


class MetricsLogger(Node):
    def __init__(self):
        super().__init__('metrics_logger')

        defaults = {
            'test_id': 'run',
            'distance': 0.0,
            'light_condition': 'unknown',
            'output_dir': '~/metrics_logs',
            'flush_interval_sec': 0.5,
            'latency_total_topic': '/latency/control_serial',
            'latency_control_topic': '/latency/control_compute',
            'latency_serial_topic': '/latency/serial_write',
        }
        for name, default in defaults.items():
            self.declare_parameter(name, default)

        self.test_id = self._get_param('test_id', str)
        self.distance = self._get_param('distance', float)
        self.light_condition = self._get_param('light_condition', str)
        self.output_dir = self._get_param('output_dir', str)
        self.flush_interval_sec = self._get_param('flush_interval_sec', float)
        self.latency_total_topic = self._get_param('latency_total_topic', str)
        self.latency_control_topic = self._get_param('latency_control_topic', str)
        self.latency_serial_topic = self._get_param('latency_serial_topic', str)

        self.start_time = self.get_clock().now()
        self.last_latency_total_ms = math.nan
        self.last_control_compute_ms = math.nan
        self.last_serial_write_ms = math.nan

        output_dir = Path(self.output_dir).expanduser().resolve()
        output_dir.mkdir(parents=True, exist_ok=True)
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        safe_test_id = self._sanitize(self.test_id)
        self.csv_path = output_dir / f'run_{safe_test_id}_{timestamp}.csv'

        self._header = [
            't_sec',
            'source',
            'capture_time_sec',
            'error_publish_time_sec',
            'err_x',
            'err_y',
            'confidence',
            'latency_ms',
            'control_compute_ms',
            'serial_write_ms',
            'test_id',
            'distance',
            'light_condition',
        ]

        self._writer_handle = self.csv_path.open('w', newline='')
        self._writer = csv.writer(self._writer_handle)
        self._writer.writerow(self._header)
        self._last_flush = time.monotonic()

        self.create_subscription(Vector3Stamped, '/vision_error', self._error_cb, 50)
        if self.latency_total_topic:
            self.create_subscription(Float32, self.latency_total_topic, self._latency_total_cb, 50)
        if self.latency_control_topic:
            self.create_subscription(Float32, self.latency_control_topic, self._latency_control_cb, 50)
        if self.latency_serial_topic:
            self.create_subscription(Float32, self.latency_serial_topic, self._latency_serial_cb, 50)

        self.get_logger().info(f'Logging metrics to {self.csv_path}')

    def _get_param(self, name, cast):
        return cast(self.get_parameter(name).value)

    def _elapsed_sec(self):
        elapsed = self.get_clock().now() - self.start_time
        return elapsed.nanoseconds / 1e9

    def _now_sec(self):
        return self.get_clock().now().nanoseconds / 1e9

    def _stamp_to_sec(self, stamp):
        return float(stamp.sec) + float(stamp.nanosec) / 1e9

    def _error_cb(self, msg):
        capture_time_sec = self._stamp_to_sec(msg.header.stamp)
        publish_time_sec = self._now_sec()
        row = [
            self._elapsed_sec(),
            'vision',
            float(capture_time_sec) if capture_time_sec is not None else math.nan,
            float(publish_time_sec) if publish_time_sec is not None else math.nan,
            float(msg.vector.x),
            float(msg.vector.y),
            float(msg.vector.z),
            float(self.last_latency_total_ms),
            float(self.last_control_compute_ms),
            float(self.last_serial_write_ms),
            self.test_id,
            self.distance,
            self.light_condition,
        ]
        self._write_row(row)

    def _latency_total_cb(self, msg):
        self.last_latency_total_ms = float(msg.data)

    def _latency_control_cb(self, msg):
        self.last_control_compute_ms = float(msg.data)

    def _latency_serial_cb(self, msg):
        self.last_serial_write_ms = float(msg.data)

    def _write_row(self, row):
        self._writer.writerow(row)
        if self.flush_interval_sec <= 0:
            self._writer_handle.flush()
            return
        now = time.monotonic()
        if now - self._last_flush >= self.flush_interval_sec:
            self._writer_handle.flush()
            self._last_flush = now

    def _sanitize(self, value):
        safe = ''.join(ch if ch.isalnum() or ch in ('-', '_') else '_' for ch in value)
        return safe or 'run'

    def destroy_node(self):
        if self._writer_handle is not None:
            self._writer_handle.flush()
            self._writer_handle.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MetricsLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
