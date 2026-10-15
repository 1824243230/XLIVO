"""Regression checks for selecting the HBUS sensor matching LG calibration."""
import json
from pathlib import Path
import struct
import subprocess
import sys
import tempfile
import unittest

import cv2
import numpy as np
import rosbag


class LidarSelectionTest(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory(prefix='hbus_lidar_selection_')
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)
        stamp = '1784529006800088770'
        for folder, x in [('LIDAR_FRONT', 1.0), ('LIDAR_RIGHT', 11.0)]:
            directory = self.root / folder
            directory.mkdir()
            metadata = ('VERSION .7\nFIELDS x y z intensity timestampOffset\n'
                        'SIZE 4 4 4 4 4\nTYPE F F F F F\nCOUNT 1 1 1 1 1\n'
                        'WIDTH 1\nHEIGHT 1\nPOINTS 1\nDATA binary\n')
            (directory / (stamp + '.pcd')).write_bytes(
                metadata.encode('ascii') + struct.pack('<5f', x, 2, 3, 4, 500))
        (self.root / 'CAM_FRONT_MID').mkdir()
        cv2.imwrite(str(self.root / 'CAM_FRONT_MID' / (stamp + '.jpg')),
                    np.zeros((16, 16, 3), dtype=np.uint8))
        (self.root / 'IMU').mkdir()
        (self.root / 'IMU/IMU.txt').write_text('1784529006800088 1 0 0 0 0 0 0 0 0 9.81\n')

    def convert(self, *options):
        converter = Path(__file__).with_name('hbus_to_rosbag.py')
        subprocess.run([sys.executable, str(converter), str(self.root),
                        '--gyro-unit', 'deg/s', *options], check=True,
                       stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        with rosbag.Bag(str(self.root / 'hbus_ros1.bag')) as bag:
            _, message, _ = next(bag.read_messages(topics=['/lidar/points']))
        report = json.loads((self.root / 'hbus_ros1.bag.json').read_text())
        return message, report

    def test_default_uses_front_payload_and_front_frame(self):
        message, report = self.convert()
        self.assertEqual(struct.unpack('<5f', message.data)[0], 1.0)
        self.assertEqual(message.header.frame_id, 'lidar_front')
        self.assertEqual(report['source_lidar_dir'], str(self.root / 'LIDAR_FRONT'))

    def test_explicit_right_sensor_gets_right_payload_and_frame(self):
        message, report = self.convert('--lidar-dir', 'LIDAR_RIGHT')
        self.assertEqual(struct.unpack('<5f', message.data)[0], 11.0)
        self.assertEqual(message.header.frame_id, 'lidar_right')
        self.assertEqual(report['source_lidar_dir'], str(self.root / 'LIDAR_RIGHT'))


if __name__ == '__main__':
    unittest.main()
