#!/usr/bin/env python3
import csv
import math
import threading
import time
from datetime import datetime
from pathlib import Path
from queue import Empty, Full, Queue

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3Stamped, Twist


class MetricsLogger(Node):
    def __init__(self):
        super().__init__('metrics_logger')

        defaults = {
            'test_id': 'run',
            'distance': 0.0,
            'light_condition': 'unknown',
            'output_dir': '~/metrics_logs',
            'queue_size': 10000,
            'flush_interval_sec': 0.5,
            'latency_topic': '/detection/latency',
            'latency_msg_type': '',
            'latency_field': 'data',
        }
        for name, default in defaults.items():
            self.declare_parameter(name, default)

        self.test_id = self._get_param('test_id', str)
        self.distance = self._get_param('distance', float)
        self.light_condition = self._get_param('light_condition', str)
        self.output_dir = self._get_param('output_dir', str)
        self.queue_size = self._get_param('queue_size', int)
        self.flush_interval_sec = self._get_param('flush_interval_sec', float)
        self.latency_topic = self._get_param('latency_topic', str)
        self.latency_msg_type = self._get_param('latency_msg_type', str)
        self.latency_field = self._get_param('latency_field', str)

        self.start_time = self.get_clock().now()
        self.last_error = None
        self.last_cmd = None
        self.last_latency_ms = math.nan
        self._latency_warned = False

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
            'cmd_vel_x',
            'cmd_vel_y',
            'cmd_vel_z',
            'latency_ms',
            'test_id',
            'distance',
            'light_condition',
        ]

        self._queue = Queue(maxsize=max(1, self.queue_size))
        self._stop_event = threading.Event()
        self._dropped = 0
        self._last_drop_log = 0.0
        self._writer_thread = threading.Thread(target=self._writer_loop, daemon=True)
        self._writer_thread.start()

        self.create_subscription(Vector3Stamped, '/vision/error', self._error_cb, 50)
        self.create_subscription(Twist, '/cmd_vel', self._cmd_cb, 50)
        self._setup_latency_subscription()

        self.get_logger().info(f'Logging metrics to {self.csv_path}')

    def _setup_latency_subscription(self):
        if not self.latency_msg_type:
            return

        try:
            from rosidl_runtime_py.utilities import get_message
            msg_cls = get_message(self.latency_msg_type)
        except Exception as exc:
            self.get_logger().error(f'Latency subscription disabled: {exc}')
            return

        self.create_subscription(msg_cls, self.latency_topic, self._latency_cb, 50)

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
        self.last_error = msg
        capture_time_sec = self._stamp_to_sec(msg.header.stamp)
        publish_time_sec = self._now_sec()
        self._enqueue_row('vision', msg, self.last_cmd, self.last_latency_ms,
                          capture_time_sec, publish_time_sec)

    def _cmd_cb(self, msg):
        self.last_cmd = msg
        self._enqueue_row('cmd_vel', self.last_error, msg, self.last_latency_ms)

    def _latency_cb(self, msg):
        value = None
        if hasattr(msg, self.latency_field):
            value = getattr(msg, self.latency_field)
        else:
            for field in ('data', 'latency_ms', 'latency', 'value'):
                if hasattr(msg, field):
                    value = getattr(msg, field)
                    break

        if value is None:
            if not self._latency_warned:
                self._latency_warned = True
                self.get_logger().warn('Latency message field not found; skipping sample')
            return

        try:
            self.last_latency_ms = float(value)
        except (TypeError, ValueError):
            self.get_logger().warn('Latency value is not numeric; skipping sample')
            return

        self._enqueue_row('latency', self.last_error, self.last_cmd, self.last_latency_ms)

    def _enqueue_row(self, source, err_msg, cmd_msg, latency_ms,
                     capture_time_sec=math.nan, publish_time_sec=math.nan):
        err_x = float(err_msg.vector.x) if err_msg is not None else math.nan
        err_y = float(err_msg.vector.y) if err_msg is not None else math.nan
        confidence = float(err_msg.vector.z) if err_msg is not None else math.nan

        cmd_x = float(cmd_msg.linear.x) if cmd_msg is not None else math.nan
        cmd_y = float(cmd_msg.linear.y) if cmd_msg is not None else math.nan
        cmd_z = float(cmd_msg.angular.z) if cmd_msg is not None else math.nan

        row = [
            self._elapsed_sec(),
            source,
            float(capture_time_sec) if capture_time_sec is not None else math.nan,
            float(publish_time_sec) if publish_time_sec is not None else math.nan,
            err_x,
            err_y,
            confidence,
            cmd_x,
            cmd_y,
            cmd_z,
            float(latency_ms) if latency_ms is not None else math.nan,
            self.test_id,
            self.distance,
            self.light_condition,
        ]

        try:
            self._queue.put_nowait(row)
        except Full:
            self._dropped += 1
            now = time.monotonic()
            if now - self._last_drop_log > 1.0:
                self._last_drop_log = now
                self.get_logger().warn(f'Metrics queue full; dropped {self._dropped} rows')

    def _writer_loop(self):
        last_flush = time.monotonic()
        with self.csv_path.open('w', newline='') as handle:
            writer = csv.writer(handle)
            writer.writerow(self._header)
            while not self._stop_event.is_set() or not self._queue.empty():
                try:
                    row = self._queue.get(timeout=0.1)
                except Empty:
                    row = None

                if row is not None:
                    writer.writerow(row)

                now = time.monotonic()
                if now - last_flush >= self.flush_interval_sec:
                    handle.flush()
                    last_flush = now

            handle.flush()

    def _sanitize(self, value):
        safe = ''.join(ch if ch.isalnum() or ch in ('-', '_') else '_' for ch in value)
        return safe or 'run'

    def destroy_node(self):
        self._stop_event.set()
        if self._writer_thread.is_alive():
            self._writer_thread.join()
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
