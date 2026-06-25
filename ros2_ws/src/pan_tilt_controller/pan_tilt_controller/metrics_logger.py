#!/usr/bin/env python3
import csv
import math
import json
import time
from datetime import datetime, timezone
from pathlib import Path

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3Stamped
from std_msgs.msg import Float32

try:
    import psutil
except ImportError:
    psutil = None
    print("Warning: psutil not installed. CPU metrics will be NaN.")


class MetricsLogger(Node):
    def __init__(self):
        super().__init__('metrics_logger')

        # Parameter dengan default
        defaults = {
            'test_id': 'run',
            'distance': 0.0,
            'light_condition': 'unknown',
            'target_speed': 'slow',          
            'output_dir': '~/metrics_logs',
            'flush_interval_sec': 0.0,
            'latency_total_topic': '/latency/control_serial',
            'latency_control_topic': '/latency/control_compute',
            'latency_serial_topic': '/latency/serial_write',
            'latency_yolo_topic': '/latency/yolo_inference',
        }
        for name, default in defaults.items():
            self.declare_parameter(name, default)

        self.test_id = self._get_param('test_id', str)
        self.distance = self._get_param('distance', float)
        self.light_condition = self._get_param('light_condition', str)
        self.target_speed = self._get_param('target_speed', str)    # <--
        self.output_dir = self._get_param('output_dir', str)
        self.flush_interval_sec = self._get_param('flush_interval_sec', float)
        self.latency_yolo_topic = self._get_param('latency_yolo_topic', str)
        self.latency_total_topic = self._get_param('latency_total_topic', str)
        self.latency_control_topic = self._get_param('latency_control_topic', str)
        self.latency_serial_topic = self._get_param('latency_serial_topic', str)

        self.start_time = self.get_clock().now()
        self.last_latency_total_ms = math.nan
        self.last_control_compute_ms = math.nan
        self.last_serial_write_ms = math.nan
        self.row_count = 0

        # Statistik CPU
        self.cpu_sum = 0.0
        self.cpu_count = 0
        self.cpu_max = 0.0

        # Buat direktori output
        output_dir = Path(self.output_dir).expanduser().resolve()
        output_dir.mkdir(parents=True, exist_ok=True)

        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        safe_test_id = self._sanitize(self.test_id)
        self.csv_path = output_dir / f'run_{safe_test_id}_{timestamp}.csv'
        self.meta_path = output_dir / f'run_{safe_test_id}_{timestamp}.meta'

        # Header CSV (tambah kolom target_speed)
        self._header = [
            't_sec',
            'source',
            'capture_time_sec',
            'error_publish_time_sec',
            'err_x',
            'err_y',
            'confidence',
            'latency_total_ms',
            'latency_control_ms',
            'latency_serial_ms',
            'cpu_percent',
            'test_id',
            'distance',
            'light_condition',
            'target_speed',              # <-- KOLOM BARU
        ]

        self._writer_handle = self.csv_path.open('w', newline='')
        self._writer = csv.writer(self._writer_handle)
        self._writer.writerow(self._header)
        self._last_flush = time.monotonic()
        self._flush_immediately = (self.flush_interval_sec <= 0.0)

        # Subscriptions
        self.create_subscription(Vector3Stamped, '/vision_error', self._error_cb, 50)
        if self.latency_total_topic:
            self.create_subscription(Float32, self.latency_total_topic, self._latency_total_cb, 50)
        if self.latency_control_topic:
            self.create_subscription(Float32, self.latency_control_topic, self._latency_control_cb, 50)
        if self.latency_serial_topic:
            self.create_subscription(Float32, self.latency_serial_topic, self._latency_serial_cb, 50)
        if self.latency_yolo_topic:
            self.create_subscription(Float32, self.latency_yolo_topic, self._latency_yolo_cb, 50)

        if psutil is not None:
            psutil.cpu_percent(interval=None)

        self._write_metadata()
        self.get_logger().info(f' Logging metrics to {self.csv_path}')
        self.get_logger().info(f' Metadata saved to {self.meta_path}')

    def _write_metadata(self):
        meta = {
            'test_id': self.test_id,
            'distance': self.distance,
            'light_condition': self.light_condition,
            'target_speed': self.target_speed,    # <--
            'output_dir': str(self.output_dir),
            'flush_interval_sec': self.flush_interval_sec,
            'latency_topics': {
                'total': self.latency_total_topic,
                'control': self.latency_control_topic,
                'serial': self.latency_serial_topic,
            },
            'start_time_utc': datetime.now(timezone.utc).isoformat(),
            'ros_node_name': self.get_name(),
            'ros_namespace': self.get_namespace(),
        }
        with self.meta_path.open('w') as f:
            json.dump(meta, f, indent=2)

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

        if psutil is not None:
            cpu_percent = psutil.cpu_percent(interval=None)
        else:
            cpu_percent = math.nan

        if not math.isnan(cpu_percent):
            self.cpu_sum += cpu_percent
            self.cpu_count += 1
            if cpu_percent > self.cpu_max:
                self.cpu_max = cpu_percent

        row = [
            self._elapsed_sec(),
            'vision',
            capture_time_sec,
            publish_time_sec,
            float(msg.vector.x),
            float(msg.vector.y),
            float(msg.vector.z),
            float(self.last_latency_total_ms),
            float(self.last_control_compute_ms),
            float(self.last_serial_write_ms),
            float(cpu_percent),
            self.test_id,
            self.distance,
            self.light_condition,
            self.target_speed,          # <--
        ]
        self._write_row(row)

    def _latency_total_cb(self, msg):
        self.last_latency_total_ms = float(msg.data)

    def _latency_control_cb(self, msg):
        self.last_control_compute_ms = float(msg.data)

    def _latency_serial_cb(self, msg):
        self.last_serial_write_ms = float(msg.data)
    def _latency_yolo_cb(self, msg):
        pass

    def _write_row(self, row):
        self._writer.writerow(row)
        self.row_count += 1
        if self._flush_immediately:
            self._writer_handle.flush()
        else:
            now = time.monotonic()
            if now - self._last_flush >= self.flush_interval_sec:
                self._writer_handle.flush()
                self._last_flush = now

    def _sanitize(self, value):
        safe = ''.join(ch if ch.isalnum() or ch in ('-', '_') else '_' for ch in value)
        return safe or 'run'

    def destroy_node(self):
        # Update metadata dengan CPU stats
        if self.cpu_count > 0:
            cpu_avg = self.cpu_sum / self.cpu_count
            try:
                with self.meta_path.open('r') as f:
                    meta = json.load(f)
                meta['cpu_summary'] = {
                    'avg_percent': round(cpu_avg, 2),
                    'max_percent': round(self.cpu_max, 2),
                    'samples': self.cpu_count,
                }
                with self.meta_path.open('w') as f:
                    json.dump(meta, f, indent=2)
                self.get_logger().info(f' CPU summary saved: avg={cpu_avg:.1f}%, max={self.cpu_max:.1f}% ({self.cpu_count} samples)')
            except Exception as e:
                self.get_logger().warn(f'Failed to update metadata with CPU stats: {e}')

        if self._writer_handle is not None:
            shutdown_row = [
                self._elapsed_sec(),
                'shutdown',
                math.nan, math.nan, math.nan, math.nan, math.nan,
                math.nan, math.nan, math.nan, math.nan,
                self.test_id, self.distance, self.light_condition, self.target_speed,
            ]
            self._writer.writerow(shutdown_row)
            self._writer_handle.flush()
            self._writer_handle.close()
            self.get_logger().info(f' Logger stopped. {self.row_count} rows written to {self.csv_path}')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MetricsLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Interrupted by user')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()