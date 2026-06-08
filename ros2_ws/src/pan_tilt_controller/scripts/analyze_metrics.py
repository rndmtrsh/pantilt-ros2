#!/usr/bin/env python3
import argparse
import csv
import math
from pathlib import Path

import numpy as np


def _to_float(value):
    try:
        return float(value)
    except (TypeError, ValueError):
        return math.nan


def _load_rows(csv_path):
    with csv_path.open('r', newline='') as handle:
        reader = csv.DictReader(handle)
        rows = list(reader)
    return rows


def _extract_series(rows):
    if not rows:
        return None

    fields = rows[0].keys()
    has_source = 'source' in fields

    t_values = []
    err_x_values = []
    err_y_values = []
    conf_values = []
    latency_total_values = []
    control_values = []
    serial_values = []
    capture_values = []
    publish_values = []

    for row in rows:
        if has_source:
            source = row.get('source')
            if source not in (None, '', 'vision'):
                continue

        t_values.append(_to_float(row.get('t_sec')))
        err_x_values.append(_to_float(row.get('err_x')))
        err_y_values.append(_to_float(row.get('err_y')))
        conf_values.append(_to_float(row.get('confidence')))
        if 'latency_ms' in row:
            latency_total_values.append(_to_float(row.get('latency_ms')))
        if 'control_compute_ms' in row:
            control_values.append(_to_float(row.get('control_compute_ms')))
        if 'serial_write_ms' in row:
            serial_values.append(_to_float(row.get('serial_write_ms')))
        if 'capture_time_sec' in row:
            capture_values.append(_to_float(row.get('capture_time_sec')))
        if 'error_publish_time_sec' in row:
            publish_values.append(_to_float(row.get('error_publish_time_sec')))

    t = np.array(t_values, dtype=float)
    err_x = np.array(err_x_values, dtype=float)
    err_y = np.array(err_y_values, dtype=float)
    conf = np.array(conf_values, dtype=float)
    latency_total = np.array(latency_total_values, dtype=float) if latency_total_values else None
    control_compute = np.array(control_values, dtype=float) if control_values else None
    serial_write = np.array(serial_values, dtype=float) if serial_values else None
    capture = np.array(capture_values, dtype=float) if capture_values else None
    publish = np.array(publish_values, dtype=float) if publish_values else None

    distance = rows[0].get('distance', '')
    lighting = rows[0].get('light_condition', '')
    test_id = rows[0].get('test_id', '')

    return (
        t,
        err_x,
        err_y,
        conf,
        latency_total,
        control_compute,
        serial_write,
        capture,
        publish,
        distance,
        lighting,
        test_id,
    )


def _rms_error(err_x, err_y):
    mask = np.isfinite(err_x) & np.isfinite(err_y)
    if not np.any(mask):
        return math.nan
    err_mag = np.sqrt(err_x[mask] ** 2 + err_y[mask] ** 2)
    return float(np.sqrt(np.mean(err_mag ** 2)))


def _mean_std(values):
    mask = np.isfinite(values)
    if not np.any(mask):
        return math.nan, math.nan
    data = values[mask]
    return float(np.mean(data)), float(np.std(data))


def _loss_of_lock_per_minute(t, conf, threshold, min_duration):
    mask = np.isfinite(t) & np.isfinite(conf)
    t = t[mask]
    conf = conf[mask]
    if t.size < 2:
        return math.nan, 0

    events = 0
    in_low = False
    start_t = None

    for idx, value in enumerate(conf):
        low = value < threshold
        if low and not in_low:
            in_low = True
            start_t = t[idx]
        elif not low and in_low:
            duration = t[idx] - start_t
            if duration >= min_duration:
                events += 1
            in_low = False
            start_t = None

    if in_low:
        duration = t[-1] - start_t
        if duration >= min_duration:
            events += 1

    duration_sec = t[-1] - t[0]
    if duration_sec <= 0.0:
        return math.nan, events

    return float(events / (duration_sec / 60.0)), events


def _median_iqr(values):
    if values is None:
        return math.nan, math.nan, math.nan
    mask = np.isfinite(values)
    if not np.any(mask):
        return math.nan, math.nan, math.nan
    data = values[mask]
    q1 = float(np.percentile(data, 25))
    median = float(np.percentile(data, 50))
    q3 = float(np.percentile(data, 75))
    return median, q1, q3


def _vision_latency_ms(capture, publish):
    if capture is None or publish is None:
        return None
    mask = np.isfinite(capture) & np.isfinite(publish)
    if not np.any(mask):
        return None
    return (publish[mask] - capture[mask]) * 1e3


def _format_mean_std(mean, std, precision=2):
    if math.isnan(mean) or math.isnan(std):
        return 'n/a'
    return f'{mean:.{precision}f}+/-{std:.{precision}f}'


def _format_median_iqr(median, q1, q3, unit='ms', precision=2):
    if math.isnan(median) or math.isnan(q1) or math.isnan(q3):
        return 'n/a'
    return f'{median:.{precision}f} ({q1:.{precision}f}-{q3:.{precision}f}) {unit}'


def _format_distance(value):
    numeric = _to_float(value)
    if math.isnan(numeric):
        return str(value) if value is not None else ''
    return f'{numeric:g} m'


def analyze_file(csv_path, args):
    rows = _load_rows(csv_path)
    series = _extract_series(rows)
    if series is None:
        return None

    (
        t,
        err_x,
        err_y,
        conf,
        latency_total,
        control_compute,
        serial_write,
        capture,
        publish,
        distance,
        lighting,
        test_id,
    ) = series

    rms_error = _rms_error(err_x, err_y)
    mean_conf, std_conf = _mean_std(conf)
    loss_rate, _ = _loss_of_lock_per_minute(t, conf, args.lock_threshold, args.lock_min_duration)

    total_median, total_q1, total_q3 = _median_iqr(latency_total)
    control_median, control_q1, control_q3 = _median_iqr(control_compute)
    serial_median, serial_q1, serial_q3 = _median_iqr(serial_write)

    vision_latency = _vision_latency_ms(capture, publish)
    vision_median, vision_q1, vision_q3 = _median_iqr(vision_latency)

    return {
        'file': csv_path.name,
        'distance': _format_distance(distance),
        'lighting': lighting,
        'rms_error_px': rms_error,
        'confidence': _format_mean_std(mean_conf, std_conf, precision=2),
        'loss_of_lock_per_min': loss_rate,
        'ros2_latency': _format_median_iqr(total_median, total_q1, total_q3, unit='ms', precision=2),
        'vision_latency': _format_median_iqr(vision_median, vision_q1, vision_q3, unit='ms', precision=2),
        'control_compute': _format_median_iqr(control_median, control_q1, control_q3, unit='ms', precision=2),
        'serial_write': _format_median_iqr(serial_median, serial_q1, serial_q3, unit='ms', precision=2),
        'test_id': test_id,
    }


def main():
    parser = argparse.ArgumentParser(description='Summarize metrics CSV files.')
    parser.add_argument('folder', type=Path, help='Folder containing metrics CSV files')
    parser.add_argument('--lock-threshold', type=float, default=0.5)
    parser.add_argument('--lock-min-duration', type=float, default=0.5)
    parser.add_argument('--output', type=Path, default=None, help='Write the summary table to a CSV file')
    args = parser.parse_args()

    csv_files = sorted(args.folder.glob('*.csv'))
    if not csv_files:
        print('No CSV files found.')
        return

    results = []
    output_path = args.output.resolve() if args.output is not None else None
    for csv_path in csv_files:
        if output_path is not None and csv_path.resolve() == output_path:
            continue
        result = analyze_file(csv_path, args)
        if result is not None:
            results.append(result)

    if not results:
        print('No usable data found in CSV files.')
        return

    header = [
        'Distance',
        'Lighting',
        'Pixel Error (RMS)',
        'Confidence',
        'Loss of Lock / minute',
        'Latency (ROS2)',
        'Latency (YOLO Vision)',
        'control_compute',
        'serial_write_time',
    ]

    print('Latency columns show median (IQR) in ms.')
    print('\t'.join(header))
    output_rows = [header]
    for result in results:
        row = [
            result['distance'],
            result['lighting'],
            f"{result['rms_error_px']:.2f}" if math.isfinite(result['rms_error_px']) else 'n/a',
            result['confidence'],
            f"{result['loss_of_lock_per_min']:.2f}" if math.isfinite(result['loss_of_lock_per_min']) else 'n/a',
            result['ros2_latency'],
            result['vision_latency'],
            result['control_compute'],
            result['serial_write'],
        ]
        print('\t'.join(row))
        output_rows.append(row)

    if args.output is not None:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        with args.output.open('w', newline='') as handle:
            writer = csv.writer(handle)
            writer.writerows(output_rows)


if __name__ == '__main__':
    main()
