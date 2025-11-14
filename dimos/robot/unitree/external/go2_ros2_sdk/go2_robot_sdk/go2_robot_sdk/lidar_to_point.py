# Copyright (c) 2024, RoboVerse community
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice, this
#    list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
import open3d as o3d


class LidarToPointCloud(Node):
    def __init__(self):
        super().__init__('lidar_to_pointcloud')

        self.declare_parameter('robot_ip_lst')
        self.declare_parameter('map_name')
        self.declare_parameter('map_save')

        self.robot_ip_lst = self.get_parameter(
            'robot_ip_lst').get_parameter_value().string_array_value
        self.map_name = self.get_parameter(
            'map_name').get_parameter_value().string_value
        self.save_map = self.get_parameter(
            'map_save').get_parameter_value().string_value

        self.points_len = 0
        self.map_full_name = f'{self.map_name}.ply'

        if self.save_map.lower() == "true":
            self.get_logger().info("Map will be saved")
            self.get_logger().info(f"Map name is {self.map_full_name}")

        self.conn_mode = "single" if len(self.robot_ip_lst) == 1 else "multi"

        if self.conn_mode == 'single':

            self.subscription = self.create_subscription(
                PointCloud2,
                '/robot0/point_cloud2',
                self.lidar_callback,
                10
            )

        else:
            for i in range(len(self.robot_ip_lst)):
                # Subscribe to the PointCloud2 topic
                self.subscription = self.create_subscription(
                    PointCloud2,
                    f'/robot{i}/point_cloud2',
                    self.lidar_callback,
                    10
                )
        self.points = set()
        self.publisher = self.create_publisher(
            PointCloud2, '/pointcloud/deque', 10)

        # Save map every 10 seconds
        if self.save_map.lower() == "true":
            self.timer = self.create_timer(10, self.save_gen_map)

    def lidar_callback(self, msg):
        # Extract the points from the PointCloud2 message
        for point in point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
            point_tuple = tuple(point)
            self.points.add(point_tuple)

        header = msg.header
        pointcloud = point_cloud2.create_cloud_xyz32(header, self.points)
        self.publisher.publish(pointcloud)

    def save_gen_map(self):
        # Convert the points to an Open3D point cloud and save it
        if not self.points:
            return

        if len(self.points) != self.points_len:
            self.points_len = len(self.points)
            point_cloud = o3d.geometry.PointCloud()
            point_cloud.points = o3d.utility.Vector3dVector(self.points)
            o3d.io.write_point_cloud(self.map_full_name, point_cloud)
            self.get_logger().info(
                f"Saved {len(self.points)} points to {self.map_full_name}.")


def main(args=None):
    rclpy.init(args=args)
    node = LidarToPointCloud()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
