#!/usr/bin/env python3
"""Convert exported HBUS PCD, JPEG and IMU data to a ROS1 bag.

The LG calibration uses LIDAR_FRONT. Select a different sensor explicitly
with --lidar-dir and supply calibration for that sensor before mapping.
"""

import argparse
import heapq
import json
import math
import os
from pathlib import Path
import sys
import tempfile

import cv2
import numpy as np
import rosbag
import rospy
from sensor_msgs.msg import CompressedImage, Image, Imu, PointCloud2, PointField
from std_msgs.msg import Header


POINT_TYPES = {
    ('I', 1): PointField.INT8, ('U', 1): PointField.UINT8,
    ('I', 2): PointField.INT16, ('U', 2): PointField.UINT16,
    ('I', 4): PointField.INT32, ('U', 4): PointField.UINT32,
    ('F', 4): PointField.FLOAT32, ('F', 8): PointField.FLOAT64,
}


def header(timestamp_ns, frame_id, sequence):
    seconds, nanoseconds = divmod(timestamp_ns, 1_000_000_000)
    if not 0 <= seconds <= 0xFFFFFFFF:
        raise ValueError('Timestamp outside ROS1 uint32 seconds range')
    return Header(seq=sequence, stamp=rospy.Time(seconds, nanoseconds), frame_id=frame_id)


def read_pcd(path, msg_header):
    """Preserve field names, types, counts and packed binary values."""
    metadata = {}
    with path.open('rb') as source:
        while True:
            line = source.readline()
            if not line:
                raise ValueError('{}: missing PCD DATA header'.format(path))
            line = line.decode('ascii').strip()
            if not line or line.startswith('#'):
                continue
            key, *values = line.split()
            metadata[key.upper()] = values
            if key.upper() == 'DATA':
                break
        payload = source.read()
    names = metadata['FIELDS']
    sizes = list(map(int, metadata['SIZE']))
    kinds = metadata['TYPE']
    counts = list(map(int, metadata.get('COUNT', ['1'] * len(names))))
    if not len(names) == len(sizes) == len(kinds) == len(counts):
        raise ValueError('{}: inconsistent PCD field metadata'.format(path))
    width = int(metadata['WIDTH'][0])
    height = int(metadata['HEIGHT'][0])
    points = int(metadata.get('POINTS', [str(width * height)])[0])
    if width < 0 or height < 1 or points != width * height:
        raise ValueError('{}: inconsistent PCD dimensions'.format(path))
    fields, dtype_fields, offset = [], [], 0
    for name, size, kind, count in zip(names, sizes, kinds, counts):
        if (kind, size) not in POINT_TYPES or count < 1:
            raise ValueError('{}: unsupported field {} {} {} {}'.format(
                path, name, kind, size, count))
        fields.append(PointField(name=name, offset=offset,
                                 datatype=POINT_TYPES[kind, size], count=count))
        dtype_fields.append((name, '<{}{}'.format(kind.lower(), size), (count,)))
        offset += size * count
    dtype = np.dtype(dtype_fields)
    encoding = metadata['DATA'][0].lower()
    if encoding == 'binary':
        if len(payload) != points * offset:
            raise ValueError('{}: expected {} PCD bytes, got {}'.format(
                path, points * offset, len(payload)))
        array = np.frombuffer(payload, dtype=dtype, count=points)
    elif encoding == 'ascii':
        tokens = payload.split()
        columns = sum(counts)
        if len(tokens) != points * columns:
            raise ValueError('{}: incorrect ASCII PCD value count'.format(path))
        values = np.asarray(tokens).reshape(points, columns)
        array = np.empty(points, dtype=dtype)
        column = 0
        for name, count in zip(names, counts):
            array[name] = values[:, column:column + count]
            column += count
        payload = array.tobytes()
    else:
        raise ValueError('{}: unsupported PCD encoding {} (use binary or ascii)'.format(
            path, encoding))
    dense = all(np.isfinite(array[name]).all() for name in ('x', 'y', 'z')
                if name in names)
    return PointCloud2(header=msg_header, height=height, width=width, fields=fields,
                       is_bigendian=False, point_step=offset, row_step=width * offset,
                       data=payload, is_dense=bool(dense))


def read_image(path, msg_header, compressed):
    if compressed:
        return CompressedImage(header=msg_header, format='bgr8; jpeg compressed bgr8',
                               data=path.read_bytes())
    bgr = cv2.imread(str(path), cv2.IMREAD_COLOR)
    if bgr is None:
        raise ValueError('{}: cannot decode image'.format(path))
    rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
    height, width = rgb.shape[:2]
    return Image(header=msg_header, height=height, width=width, encoding='rgb8',
                 is_bigendian=0, step=width * 3, data=rgb.tobytes())


def read_imu_rows(path, timestamp_unit):
    rows = []
    multiplier = {'ns': 1, 'us': 1000, 'ms': 1_000_000, 's': 1_000_000_000}[timestamp_unit]
    with path.open() as source:
        for line_number, line in enumerate(source, 1):
            line = line.split('#', 1)[0].strip()
            if not line:
                continue
            columns = line.split()
            if len(columns) != 11:
                raise ValueError('{}:{}: expected 11 columns'.format(path, line_number))
            try:
                timestamp_ns = int(columns[0]) * multiplier
                values = tuple(map(float, columns[1:]))
                if not all(math.isfinite(value) for value in values):
                    raise ValueError('non-finite IMU value')
                if sum(value * value for value in values[:4]) < 1e-12:
                    raise ValueError('zero quaternion')
            except ValueError as error:
                raise ValueError('{}:{}: {}'.format(path, line_number, error)) from error
            rows.append((timestamp_ns, values))
    if not rows:
        raise ValueError('{}: no IMU records'.format(path))
    return sorted(rows, key=lambda row: row[0])


def imu_message(values, msg_header, quaternion_order, gyro_unit, accel_unit):
    message = Imu(header=msg_header)
    quaternion = dict(zip(quaternion_order, values[:4]))
    for axis in 'xyzw':
        setattr(message.orientation, axis, quaternion[axis])
    gyro_scale = math.pi / 180 if gyro_unit == 'deg/s' else 1.0
    accel_scale = 9.80665 if accel_unit == 'g' else 1.0
    for index, axis in enumerate('xyz'):
        setattr(message.angular_velocity, axis, values[4 + index] * gyro_scale)
        setattr(message.linear_acceleration, axis, values[7 + index] * accel_scale)
    # All-zero covariance means unknown, as specified by sensor_msgs/Imu.
    return message


def timestamped_files(folder, suffixes):
    paths = [path for path in folder.iterdir() if path.suffix.lower() in suffixes]
    if not paths:
        raise ValueError('{}: no matching files'.format(folder))
    return sorted(((int(path.stem), path) for path in paths), key=lambda row: row[0])


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('input', nargs='?', type=Path, default=Path(__file__).resolve().parent,
                        help='Directory containing LIDAR_FRONT, CAM_FRONT_MID and IMU/IMU.txt')
    parser.add_argument('-o', '--output', type=Path, help='Output ROS1 bag (must not exist)')
    parser.add_argument('--gyro-unit', required=True, choices=['deg/s', 'rad/s'],
                        help='Input angular velocity unit; ROS output is always rad/s')
    parser.add_argument('--accel-unit', choices=['m/s2', 'g'], default='m/s2')
    parser.add_argument('--quaternion-order', choices=['wxyz', 'xyzw'], default='wxyz')
    parser.add_argument('--imu-time-unit', choices=['ns', 'us', 'ms', 's'], default='us')
    parser.add_argument('--compression', choices=['none', 'bz2', 'lz4'], default='lz4')
    parser.add_argument('--compressed-images', action='store_true',
                        help='Store original JPEG as CompressedImage instead of RGB Image')
    parser.add_argument('--lidar-topic', default='/lidar/points')
    parser.add_argument('--image-topic', default=None)
    parser.add_argument('--imu-topic', default='/imu/data')
    parser.add_argument('--lidar-dir', type=Path, default=Path('LIDAR_FRONT'),
                        help='PCD directory, relative to input or absolute (default: LIDAR_FRONT)')
    parser.add_argument('--lidar-frame',
                        help='PointCloud2 frame_id (default: lowercase name of selected lidar directory)')
    parser.add_argument('--image-frame', default='cam_front_mid_optical_frame')
    parser.add_argument('--imu-frame', default='imu_link')
    return parser.parse_args()


def convert(args):
    root = args.input.resolve()
    output = (args.output or root / 'hbus_ros1.bag').resolve()
    report_path = output.with_suffix(output.suffix + '.json')
    if output.exists() or report_path.exists():
        raise ValueError('Output or conversion report already exists: {}'.format(output))
    lidar_dir = (root / args.lidar_dir).resolve()
    lidar_frame = args.lidar_frame or lidar_dir.name.lower()
    clouds = timestamped_files(lidar_dir, {'.pcd'})
    images = timestamped_files(root / 'CAM_FRONT_MID', {'.jpg', '.jpeg', '.png'})
    if args.compressed_images and any(path.suffix.lower() not in {'.jpg', '.jpeg'}
                                      for _, path in images):
        raise ValueError('--compressed-images requires JPEG source files')
    imu = read_imu_rows(root / 'IMU' / 'IMU.txt', args.imu_time_unit)
    image_topic = args.image_topic or ('/camera/image_raw/compressed'
                                     if args.compressed_images else '/camera/image_raw')
    topics = [args.lidar_topic, image_topic, args.imu_topic]
    if len(set(topics)) != 3 or any(not topic or topic == '/' for topic in topics):
        raise ValueError('Specify three distinct, nonempty topic names')
    streams = [((stamp, 0, seq, path) for seq, (stamp, path) in enumerate(clouds)),
               ((stamp, 1, seq, path) for seq, (stamp, path) in enumerate(images)),
               ((stamp, 2, seq, values) for seq, (stamp, values) in enumerate(imu))]
    total = len(clouds) + len(images) + len(imu)
    output.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary = tempfile.mkstemp(prefix=output.name + '.', suffix='.active',
                                            dir=str(output.parent))
    os.close(descriptor)
    try:
        with rosbag.Bag(temporary, 'w', compression=args.compression) as bag:
            for count, (stamp, kind, sequence, item) in enumerate(heapq.merge(*streams), 1):
                if kind == 0:
                    message = read_pcd(item, header(stamp, lidar_frame, sequence))
                elif kind == 1:
                    message = read_image(item, header(stamp, args.image_frame, sequence),
                                         args.compressed_images)
                else:
                    message = imu_message(item, header(stamp, args.imu_frame, sequence),
                                          args.quaternion_order, args.gyro_unit, args.accel_unit)
                bag.write(topics[kind], message, message.header.stamp)
                if count % 500 == 0 or count == total:
                    print('Written {}/{} messages'.format(count, total), flush=True)
        os.rename(temporary, str(output))
    except BaseException:
        Path(temporary).unlink(missing_ok=True)
        raise
    report = {
        'input': str(root), 'output': str(output), 'ros_version': 1,
        'source_lidar_dir': str(lidar_dir),
        'compression': args.compression,
        'topics': {args.lidar_topic: {'type': 'sensor_msgs/PointCloud2', 'count': len(clouds),
                                     'frame_id': lidar_frame},
                   image_topic: {'type': 'sensor_msgs/CompressedImage' if args.compressed_images
                                 else 'sensor_msgs/Image', 'count': len(images),
                                 'frame_id': args.image_frame},
                   args.imu_topic: {'type': 'sensor_msgs/Imu', 'count': len(imu),
                                    'frame_id': args.imu_frame}},
        'start_ns': min(clouds[0][0], images[0][0], imu[0][0]),
        'end_ns': max(clouds[-1][0], images[-1][0], imu[-1][0]),
        'input_imu': {'columns': ['timestamp'] + ['q' + axis for axis in args.quaternion_order]
                      + ['gyro_' + axis for axis in 'xyz'] + ['accel_' + axis for axis in 'xyz'],
                      'timestamp_unit': args.imu_time_unit,
                      'gyro_unit': args.gyro_unit, 'accel_unit': args.accel_unit},
        'output_imu': {'gyro_unit': 'rad/s', 'accel_unit': 'm/s2',
                       'covariance': 'unknown (all zeros)'},
    }
    report_path.write_text(json.dumps(report, indent=2, ensure_ascii=False) + '\n')
    print('Bag: {} ({} bytes)'.format(output, output.stat().st_size), flush=True)
    print('Report: {}'.format(report_path), flush=True)


if __name__ == '__main__':
    try:
        convert(parse_args())
    except (OSError, ValueError, KeyError, rosbag.ROSBagException) as error:
        print('Conversion failed: {}'.format(error), file=sys.stderr)
        sys.exit(1)
