#!/usr/bin/env python3
import argparse
import csv
import json
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


def _load_metadata(meta_path):
    """Baca file .meta untuk mendapatkan parameter tambahan."""
    if not meta_path.exists():
        return {}
    try:
        with meta_path.open('r') as f:
            return json.load(f)
    except Exception:
        return {}


def _extract_series(rows):
    if not rows:
        return None

    fields = rows[0].keys()
    has_source = 'source' in fields

    t = []
    err_x = []
    err_y = []
    conf = []
    latency_total = []
    control_compute = []
    serial_write = []
    capture = []
    publish = []
    cpu = []

    for row in rows:
        if has_source:
            source = row.get('source')
            if source not in (None, '', 'vision'):
                continue

        t.append(_to_float(row.get('t_sec')))
        err_x.append(_to_float(row.get('err_x')))
        err_y.append(_to_float(row.get('err_y')))
        conf.append(_to_float(row.get('confidence')))
        cpu.append(_to_float(row.get('cpu_percent')))
        if 'latency_total_ms' in row:
            latency_total.append(_to_float(row.get('latency_total_ms')))
        if 'latency_control_ms' in row:
            control_compute.append(_to_float(row.get('latency_control_ms')))
        if 'latency_serial_ms' in row:
            serial_write.append(_to_float(row.get('latency_serial_ms')))
        if 'capture_time_sec' in row:
            capture.append(_to_float(row.get('capture_time_sec')))
        if 'error_publish_time_sec' in row:
            publish.append(_to_float(row.get('error_publish_time_sec')))

    t_arr = np.array(t, dtype=float)
    err_x_arr = np.array(err_x, dtype=float)
    err_y_arr = np.array(err_y, dtype=float)
    conf_arr = np.array(conf, dtype=float)
    cpu_arr = np.array(cpu, dtype=float) if cpu else None

    return {
        't': t_arr,
        'err_x': err_x_arr,
        'err_y': err_y_arr,
        'conf': conf_arr,
        'cpu': cpu_arr,
        'latency_total': np.array(latency_total, dtype=float) if latency_total else None,
        'control_compute': np.array(control_compute, dtype=float) if control_compute else None,
        'serial_write': np.array(serial_write, dtype=float) if serial_write else None,
        'capture': np.array(capture, dtype=float) if capture else None,
        'publish': np.array(publish, dtype=float) if publish else None,
    }


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


def _vision_latency_ms(capture, publish):
    if capture is None or publish is None:
        return None
    mask = np.isfinite(capture) & np.isfinite(publish)
    if not np.any(mask):
        return None
    return (publish[mask] - capture[mask]) * 1e3


def _compute_overshoot(err_x, err_y, t, min_amplitude=30.0):
    """
    Hitung overshoot maksimum: puncak error setelah transisi.
    Pendekatan: cari local maxima dari magnitude error di atas ambang.
    Ambil 2 tertinggi sebagai indikasi overshoot.
    """
    mask = np.isfinite(err_x) & np.isfinite(err_y) & np.isfinite(t)
    if not np.any(mask):
        return math.nan

    t = t[mask]
    err_mag = np.sqrt(err_x[mask] ** 2 + err_y[mask] ** 2)

    if t.size < 3:
        return math.nan

    # Cari local maxima (titik di mana error lebih besar dari tetangga)
    peaks = []
    for i in range(1, len(err_mag) - 1):
        if err_mag[i] > err_mag[i-1] and err_mag[i] > err_mag[i+1]:
            if err_mag[i] > min_amplitude:
                peaks.append(err_mag[i])

    if not peaks:
        return math.nan

    # Ambil nilai puncak tertinggi
    return float(np.max(peaks))


def _format_mean_std(mean, std, precision=2):
    if math.isnan(mean) or math.isnan(std):
        return 'n/a'
    return f'{mean:.{precision}f}+/-{std:.{precision}f}'


def _format_median_iqr(median, q1, q3, unit='ms', precision=2):
    if math.isnan(median) or math.isnan(q1) or math.isnan(q3):
        return 'n/a'
    return f'{median:.{precision}f} ({q1:.{precision}f}-{q3:.{precision}f}) {unit}'


def _format_value(value, precision=2):
    if math.isnan(value):
        return 'n/a'
    return f'{value:.{precision}f}'


def analyze_file(csv_path, args):
    rows = _load_rows(csv_path)
    if not rows:
        return None

    # Baca metadata untuk parameter tambahan
    meta_path = csv_path.with_suffix('.meta')
    meta = _load_metadata(meta_path)

    series = _extract_series(rows)
    if series is None:
        return None

    t = series['t']
    err_x = series['err_x']
    err_y = series['err_y']
    conf = series['conf']
    cpu = series['cpu']
    latency_total = series['latency_total']
    control_compute = series['control_compute']
    serial_write = series['serial_write']
    capture = series['capture']
    publish = series['publish']

    rms_error = _rms_error(err_x, err_y)
    mean_conf, std_conf = _mean_std(conf)
    loss_rate, _ = _loss_of_lock_per_minute(t, conf, args.lock_threshold, args.lock_min_duration)

    # CPU stats
    cpu_avg, cpu_std = _mean_std(cpu) if cpu is not None else (math.nan, math.nan)
    cpu_max = float(np.nanmax(cpu)) if cpu is not None and np.any(np.isfinite(cpu)) else math.nan

    # Overshoot
    overshoot = _compute_overshoot(err_x, err_y, t)

    # Latencies
    total_median, total_q1, total_q3 = _median_iqr(latency_total)
    control_median, control_q1, control_q3 = _median_iqr(control_compute)
    serial_median, serial_q1, serial_q3 = _median_iqr(serial_write)

    vision_latency = _vision_latency_ms(capture, publish)
    vision_median, vision_q1, vision_q3 = _median_iqr(vision_latency)

    # Ambil parameter dari metadata (fallback ke baris pertama)
    first_row = rows[0]
    distance = meta.get('distance', first_row.get('distance', ''))
    lighting = meta.get('light_condition', first_row.get('light_condition', ''))
    target_speed = meta.get('target_speed', first_row.get('target_speed', ''))
    test_id = meta.get('test_id', first_row.get('test_id', ''))

    return {
        'file': csv_path.name,
        'distance': distance,
        'lighting': lighting,
        'target_speed': target_speed,
        'rms_error_px': rms_error,
        'confidence': _format_mean_std(mean_conf, std_conf, precision=2),
        'loss_of_lock_per_min': loss_rate,
        'cpu_avg': cpu_avg,
        'cpu_max': cpu_max,
        'ros2_latency': _format_median_iqr(total_median, total_q1, total_q3, unit='ms', precision=2),
        'vision_latency': _format_median_iqr(vision_median, vision_q1, vision_q3, unit='ms', precision=2),
        'control_compute': _format_median_iqr(control_median, control_q1, control_q3, unit='ms', precision=2),
        'serial_write': _format_median_iqr(serial_median, serial_q1, serial_q3, unit='ms', precision=2),
        'overshoot': overshoot,
        'test_id': test_id,
    }


def main():
    parser = argparse.ArgumentParser(description='Summarize metrics CSV files.')
    parser.add_argument('folder', type=Path, help='Folder containing metrics CSV files')
    parser.add_argument('--lock-threshold', type=float, default=0.5,
                        help='Confidence threshold below which considered lost')
    parser.add_argument('--lock-min-duration', type=float, default=0.5,
                        help='Minimum duration (sec) of low confidence to count as loss')
    parser.add_argument('--output', type=Path, default=None,
                        help='Write the summary table to a CSV file')
    parser.add_argument('--group-by', choices=['distance', 'lighting', 'speed'], default='distance',
                        help='Group results by this parameter (for table view)')
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

    # Tampilkan hasil
    print('\n' + '=' * 80)
    print('📊 METRICS SUMMARY')
    print('=' * 80)

    # Tampilkan per file
    for res in results:
        print(f"\n📄 {res['file']}")
        print(f"   Test ID     : {res['test_id']}")
        print(f"   Distance    : {res['distance']}")
        print(f"   Lighting    : {res['lighting']}")
        print(f"   Speed       : {res['target_speed']}")
        print(f"   RMS Error   : {_format_value(res['rms_error_px'])} px")
        print(f"   Confidence  : {res['confidence']}")
        print(f"   Loss/min    : {_format_value(res['loss_of_lock_per_min'])}")
        print(f"   Overshoot   : {_format_value(res['overshoot'])} px")
        print(f"   CPU avg     : {_format_value(res['cpu_avg'])} %")
        print(f"   CPU max     : {_format_value(res['cpu_max'])} %")
        print(f"   ROS2 Lat    : {res['ros2_latency']}")
        print(f"   Vision Lat  : {res['vision_latency']}")
        print(f"   Control Lat : {res['control_compute']}")
        print(f"   Serial Lat  : {res['serial_write']}")

    # Output tabel dalam format CSV
    if args.output is not None:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        header = [
            'Distance', 'Lighting', 'Speed',
            'RMS_Error_px', 'Confidence', 'Loss_per_min', 'Overshoot_px',
            'CPU_avg_%', 'CPU_max_%',
            'ROS2_Latency_ms', 'Vision_Latency_ms',
            'Control_Compute_ms', 'Serial_Write_ms',
            'Test_ID'
        ]
        with args.output.open('w', newline='') as handle:
            writer = csv.writer(handle)
            writer.writerow(header)
            for res in results:
                writer.writerow([
                    res['distance'],
                    res['lighting'],
                    res['target_speed'],
                    _format_value(res['rms_error_px']),
                    res['confidence'],
                    _format_value(res['loss_of_lock_per_min']),
                    _format_value(res['overshoot']),
                    _format_value(res['cpu_avg']),
                    _format_value(res['cpu_max']),
                    res['ros2_latency'],
                    res['vision_latency'],
                    res['control_compute'],
                    res['serial_write'],
                    res['test_id'],
                ])
        print(f"\n✅ Summary table saved to: {args.output}")


if __name__ == '__main__':
    main()