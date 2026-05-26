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
    source_filter = None
    if 'source' in fields:
        source_filter = 'vision'

    t_values = []
    err_x_values = []
    err_y_values = []
    conf_values = []
    latency_values = []

    for row in rows:
        if source_filter and row.get('source') != source_filter:
            continue
        t_values.append(_to_float(row.get('t_sec')))
        err_x_values.append(_to_float(row.get('err_x')))
        err_y_values.append(_to_float(row.get('err_y')))
        conf_values.append(_to_float(row.get('confidence')))
        if 'latency_ms' in row:
            latency_values.append(_to_float(row.get('latency_ms')))

    t = np.array(t_values, dtype=float)
    err_x = np.array(err_x_values, dtype=float)
    err_y = np.array(err_y_values, dtype=float)
    conf = np.array(conf_values, dtype=float)
    latency = np.array(latency_values, dtype=float) if latency_values else None

    return t, err_x, err_y, conf, latency


def _rms_error(t, err_x, err_y, steady_state_sec):
    mask = np.isfinite(t) & np.isfinite(err_x) & np.isfinite(err_y) & (t <= steady_state_sec)
    if not np.any(mask):
        return math.nan
    err_mag = np.sqrt(err_x[mask] ** 2 + err_y[mask] ** 2)
    return float(np.sqrt(np.mean(err_mag ** 2)))


def _mean_confidence(conf):
    mask = np.isfinite(conf)
    if not np.any(mask):
        return math.nan
    return float(np.mean(conf[mask]))


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


def _settling_time(t, err_x, step_start, settle_band, hold_sec, final_window_sec, step_search_window_sec):
    mask = np.isfinite(t) & np.isfinite(err_x)
    t = t[mask]
    err_x = err_x[mask]
    if t.size < 3:
        return math.nan

    after_step_mask = t >= step_start
    if not np.any(after_step_mask):
        return math.nan

    t_after = t[after_step_mask]
    err_after = err_x[after_step_mask]
    diffs = np.abs(np.diff(err_after))
    if diffs.size == 0:
        return math.nan

    if step_search_window_sec > 0:
        search_end = step_start + step_search_window_sec
        search_mask = t_after[:-1] <= search_end
        if np.any(search_mask):
            search_diffs = diffs[search_mask]
            step_offset = int(np.argmax(search_diffs))
        else:
            step_offset = int(np.argmax(diffs))
    else:
        step_offset = int(np.argmax(diffs))

    step_time = t_after[min(step_offset + 1, t_after.size - 1)]

    end_time = t[-1]
    final_mask = t >= (end_time - final_window_sec)
    if not np.any(final_mask):
        return math.nan
    final_value = float(np.median(err_x[final_mask]))

    within = np.abs(err_x - final_value) <= settle_band
    start_idx = int(np.searchsorted(t, step_time, side='left'))
    idx = start_idx
    while idx < t.size:
        if within[idx]:
            seg_start = t[idx]
            idx_end = idx
            while idx_end < t.size and within[idx_end]:
                idx_end += 1
            seg_end = t[idx_end - 1]
            if seg_end - seg_start >= hold_sec:
                return float(seg_start - step_time)
            idx = idx_end
        else:
            idx += 1

    return math.nan


def _latency_stats(latency):
    if latency is None:
        return math.nan, math.nan

    mask = np.isfinite(latency)
    if not np.any(mask):
        return math.nan, math.nan

    values = latency[mask]
    return float(np.mean(values)), float(np.std(values))


def analyze_file(csv_path, args):
    rows = _load_rows(csv_path)
    series = _extract_series(rows)
    if series is None:
        return None

    t, err_x, err_y, conf, latency = series

    rms_error = _rms_error(t, err_x, err_y, args.steady_state_sec)
    settling_time = _settling_time(
        t,
        err_x,
        args.step_start,
        args.settle_band,
        args.settle_hold,
        args.final_window_sec,
        args.step_search_window_sec,
    )
    mean_conf = _mean_confidence(conf)
    loss_rate, loss_events = _loss_of_lock_per_minute(t, conf, args.lock_threshold, args.lock_min_duration)
    latency_mean, latency_std = _latency_stats(latency)

    return {
        'file': csv_path.name,
        'rms_error_px': rms_error,
        'settling_time_sec': settling_time,
        'mean_confidence': mean_conf,
        'loss_of_lock_per_min': loss_rate,
        'loss_of_lock_events': loss_events,
        'latency_mean_ms': latency_mean,
        'latency_std_ms': latency_std,
    }


def main():
    parser = argparse.ArgumentParser(description='Analyze metrics CSV files.')
    parser.add_argument('folder', type=Path, help='Folder containing metrics CSV files')
    parser.add_argument('--steady-state-sec', type=float, default=60.0)
    parser.add_argument('--step-start', type=float, default=60.0)
    parser.add_argument('--step-search-window-sec', type=float, default=30.0)
    parser.add_argument('--settle-band', type=float, default=10.0)
    parser.add_argument('--settle-hold', type=float, default=1.0)
    parser.add_argument('--final-window-sec', type=float, default=10.0)
    parser.add_argument('--lock-threshold', type=float, default=0.5)
    parser.add_argument('--lock-min-duration', type=float, default=0.5)
    parser.add_argument('--output', type=Path, default=None)
    args = parser.parse_args()

    csv_files = sorted(args.folder.glob('*.csv'))
    if not csv_files:
        print('No CSV files found.')
        return

    results = []
    for csv_path in csv_files:
        result = analyze_file(csv_path, args)
        if result is not None:
            results.append(result)

    if not results:
        print('No usable data found in CSV files.')
        return

    header = [
        'file',
        'rms_error_px',
        'settling_time_sec',
        'mean_confidence',
        'loss_of_lock_per_min',
        'loss_of_lock_events',
        'latency_mean_ms',
        'latency_std_ms',
    ]

    print(','.join(header))
    for result in results:
        row = [result[key] for key in header]
        print(','.join(str(value) for value in row))

    if args.output is not None:
        with args.output.open('w', newline='') as handle:
            writer = csv.writer(handle)
            writer.writerow(header)
            for result in results:
                writer.writerow([result[key] for key in header])


if __name__ == '__main__':
    main()
