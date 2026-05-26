#!/usr/bin/env python3
import argparse
import math
from pathlib import Path

from rclpy.serialization import deserialize_message
from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
from rosidl_runtime_py.utilities import get_message


def count_false_positives(bag_path, topic, threshold, storage_id):
    reader = SequentialReader()
    storage_options = StorageOptions(uri=str(bag_path), storage_id=storage_id)
    converter_options = ConverterOptions(
        input_serialization_format='cdr',
        output_serialization_format='cdr',
    )
    reader.open(storage_options, converter_options)

    msg_type = None
    for topic_type in reader.get_all_topics_and_types():
        if topic_type.name == topic:
            msg_type = topic_type.type
            break

    if msg_type is None:
        raise RuntimeError(f'Topic not found in bag: {topic}')

    msg_cls = get_message(msg_type)

    first_time = None
    last_time = None
    count = 0
    total = 0

    while reader.has_next():
        topic_name, data, timestamp = reader.read_next()
        if topic_name != topic:
            continue
        msg = deserialize_message(data, msg_cls)
        if not hasattr(msg, 'z'):
            raise RuntimeError('Message does not have a z field for confidence')

        confidence = float(msg.z)
        if first_time is None:
            first_time = timestamp
        last_time = timestamp
        total += 1
        if confidence > threshold:
            count += 1

    if first_time is None or last_time is None or total == 0:
        return math.nan, 0, 0, 0.0

    duration_sec = (last_time - first_time) / 1e9
    if duration_sec <= 0.0:
        rate = math.nan
    else:
        rate = count / (duration_sec / 60.0)

    return rate, count, total, duration_sec


def main():
    parser = argparse.ArgumentParser(description='Compute false positive rate from an empty-room bag.')
    parser.add_argument('bag', type=Path, help='Path to rosbag2 folder')
    parser.add_argument('--topic', type=str, default='/vision/error')
    parser.add_argument('--threshold', type=float, default=0.5)
    parser.add_argument('--storage-id', type=str, default='sqlite3')
    args = parser.parse_args()

    rate, count, total, duration_sec = count_false_positives(
        args.bag,
        args.topic,
        args.threshold,
        args.storage_id,
    )

    if math.isnan(rate):
        print('No valid messages found to compute rate.')
        return

    print(f'bag: {args.bag}')
    print(f'topic: {args.topic}')
    print(f'duration_sec: {duration_sec:.2f}')
    print(f'total_msgs: {total}')
    print(f'false_positive_msgs: {count}')
    print(f'false_positive_per_min: {rate:.3f}')


if __name__ == '__main__':
    main()
