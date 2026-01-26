#!/usr/bin/env python3
"""Save /explored_areas PointCloud2 topic to PLY file.

Runs continuously and saves the latest point cloud on each update.
When the topic stops publishing (bagfile ends), saves the final state.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import struct
import sys
import signal
import time


class PointCloudSaver(Node):
    def __init__(self, output_file, idle_timeout=5.0):
        super().__init__('pointcloud_saver')
        self.output_file = output_file
        self.idle_timeout = idle_timeout
        self.last_msg_time = None
        self.last_point_count = 0
        self.total_saves = 0

        self.subscription = self.create_subscription(
            PointCloud2,
            '/explored_areas',
            self.callback,
            10)

        # Timer to check if topic stopped publishing
        self.check_timer = self.create_timer(1.0, self.check_idle)
        self.get_logger().info(f'Listening to /explored_areas (will save to {output_file})')
        self.get_logger().info(f'Will exit {idle_timeout}s after last message...')

    def check_idle(self):
        if self.last_msg_time is not None:
            elapsed = time.time() - self.last_msg_time
            if elapsed > self.idle_timeout:
                self.get_logger().info(f'No messages for {self.idle_timeout}s. Final save complete.')
                self.get_logger().info(f'Total points: {self.last_point_count}, Saves: {self.total_saves}')
                rclpy.shutdown()

    def callback(self, msg):
        self.last_msg_time = time.time()

        # Parse PointCloud2
        points = []
        point_step = msg.point_step
        data = msg.data

        for i in range(msg.width):
            offset = i * point_step
            x = struct.unpack_from('f', data, offset)[0]
            y = struct.unpack_from('f', data, offset + 4)[0]
            z = struct.unpack_from('f', data, offset + 8)[0]
            points.append((x, y, z))

        # Write PLY file (overwrite with latest)
        with open(self.output_file, 'w') as f:
            f.write('ply\n')
            f.write('format ascii 1.0\n')
            f.write(f'element vertex {len(points)}\n')
            f.write('property float x\n')
            f.write('property float y\n')
            f.write('property float z\n')
            f.write('end_header\n')
            for p in points:
                f.write(f'{p[0]} {p[1]} {p[2]}\n')

        self.last_point_count = len(points)
        self.total_saves += 1

        if self.total_saves == 1:
            self.get_logger().info(f'First save: {len(points)} points')
        elif self.total_saves % 10 == 0:
            self.get_logger().info(f'Updated: {len(points)} points (save #{self.total_saves})')


def main():
    if len(sys.argv) < 2:
        print('Usage: save_explored_areas.py <output_file.ply> [idle_timeout_seconds]')
        print('  Continuously saves /explored_areas to PLY file.')
        print('  Exits after idle_timeout seconds of no messages (default: 5s)')
        sys.exit(1)

    output_file = sys.argv[1]
    idle_timeout = float(sys.argv[2]) if len(sys.argv) > 2 else 5.0

    rclpy.init()
    node = PointCloudSaver(output_file, idle_timeout)

    # Handle Ctrl+C gracefully
    def shutdown_handler(sig, frame):
        node.get_logger().info(f'Interrupted. Final point count: {node.last_point_count}')
        rclpy.shutdown()

    signal.signal(signal.SIGINT, shutdown_handler)

    rclpy.spin(node)


if __name__ == '__main__':
    main()
