# Copyright 2025 Dimensional Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
import pickle

import pytest
from dimos_lcm.foxglove_msgs.ImageAnnotations import ImageAnnotations
from dimos_lcm.sensor_msgs import CameraInfo

from dimos.core.transport import LCMTransport
from dimos.msgs.geometry_msgs import Transform
from dimos.msgs.sensor_msgs import PointCloud2
from dimos.msgs.sensor_msgs.Image import Image
from dimos.perception.detection2d import Detection2DArrayFix, DetectionPointcloud
from dimos.perception.detection2d.module import DetectionPointcloud, build_imageannotations
from dimos.protocol.service import lcmservice as lcm
from dimos.protocol.tf import TF
from dimos.robot.unitree_webrtc.modular.connection_module import ConnectionModule
from dimos.robot.unitree_webrtc.type.lidar import LidarMessage
from dimos.robot.unitree_webrtc.type.odometry import Odometry
from dimos.utils.data import get_data
from dimos.utils.testing import TimedSensorReplay

# Global path for the detection result pickle file
TEST_DIR = os.path.dirname(__file__)
DETECTION_RESULT_PKL = os.path.join(TEST_DIR, "detection_result.pkl")


@pytest.fixture
def moment():
    data_dir = "unitree_office_walk"
    get_data(data_dir)

    seek_seconds = 55.0

    lidar_frame = TimedSensorReplay(
        f"{data_dir}/lidar", autocast=LidarMessage.from_msg
    ).find_closest_seek(seek_seconds)

    image_frame = TimedSensorReplay(
        f"{data_dir}/video", autocast=Image.from_numpy
    ).find_closest_seek(seek_seconds)

    odom_frame = TimedSensorReplay(
        f"{data_dir}/odom", autocast=Odometry.from_msg
    ).find_closest_seek(seek_seconds)

    transforms = ConnectionModule._odom_to_tf(odom_frame)

    return [
        odom_frame,
        lidar_frame,
        image_frame,
        ConnectionModule._camera_info(),
        transforms,
    ]


def publish_detected_pc(detected_pc: list[PointCloud2]):
    for idx, detection in enumerate(detected_pc):
        detected_pointcloud_transport = LCMTransport(f"/detected_{idx}", PointCloud2)
        detected_pointcloud_transport.publish(detection)


def publish_lcm(
    lidar_frame: LidarMessage,
    image_frame: Image,
    odom_frame,
    camera_info,
    annotations,
    detected_pc: list[PointCloud2],
):
    lidar_frame_transport = LCMTransport("/lidar", LidarMessage)
    lidar_frame_transport.publish(lidar_frame)

    image_frame_transport = LCMTransport("/image", Image)
    image_frame_transport.publish(image_frame)

    odom_frame_transport = LCMTransport("/odom", Odometry)
    odom_frame_transport.publish(odom_frame)

    camera_info_transport = LCMTransport("/camera_info", CameraInfo)
    camera_info_transport.publish(camera_info)

    annotations_transport = LCMTransport("/annotations", ImageAnnotations)
    annotations_transport.publish(annotations)

    publish_detected_pc(detected_pc)


def test_basic(moment):
    lcm.autoconf()
    odom_frame, lidar_frame, image_frame, camera_info, transforms = moment
    tf = TF()
    tf.publish(*transforms)

    camera_transform = tf.get("camera_optical", "world")

    detector = DetectionPointcloud()
    [image_frame, detections, separate_detections_pointcloud, detections_pointcloud] = (
        detector.process_frame(image_frame, lidar_frame, camera_info, camera_transform)
    )

    detection_result = [separate_detections_pointcloud, camera_transform]

    # Assuming you have your detection_result object
    with open(DETECTION_RESULT_PKL, "wb") as f:
        pickle.dump(detection_result, f)

    publish_lcm(
        lidar_frame,
        image_frame,
        odom_frame,
        camera_info,
        build_imageannotations([image_frame, detections]),
        separate_detections_pointcloud,
    )

    print("detections:\n", "\n".join(map(str, separate_detections_pointcloud)))


# from https://www.open3d.org/docs/release/python_api/open3d.geometry.PointCloud.html
#
# hidden_point_removal(self: open3d.geometry.PointCloud, camera_location: numpy.ndarray[numpy.float64[3, 1]], radius: float) → tuple[open3d.geometry.TriangleMesh, list[int]]
#     Removes hidden points from a point cloud and returns a mesh of the remaining points. Based on Katz et al. ‘Direct Visibility of Point Sets’, 2007. Additional information about the choice of radius for noisy point clouds can be found in Mehra et. al. ‘Visibility of Noisy Point Cloud Data’, 2010.
#     Parameters:
#             camera_location (numpy.ndarray[numpy.float64[3, 1]]) – All points not visible from that location will be removed
#             radius (float) – The radius of the sperical projection
#     Returns:
#         tuple[open3d.geometry.TriangleMesh, list[int]]


def hidden_point_removal(camera_transform: Transform, pc: PointCloud2, radius: float = 100.0):
    """Remove points not visible from camera location.

    Args:
        camera_transform: Transform from camera to world frame
        pc: PointCloud2 in world frame
        radius: Radius for spherical projection (default 1000.0)

    Returns:
        PointCloud2 with only visible points
    """
    import numpy as np
    import open3d as o3d

    # Get camera position in world frame
    camera_position = camera_transform.translation
    print(f"Camera position: {camera_position}")

    # Convert to numpy array for open3d
    camera_pos_np = camera_position.to_numpy().reshape(3, 1)

    # Get the point cloud
    pcd = pc.pointcloud
    print(f"Input point cloud has {len(pcd.points)} points")

    # Perform hidden point removal
    _, visible_indices = pcd.hidden_point_removal(camera_pos_np, radius)

    print(f"Visible points: {len(visible_indices)} out of {len(pcd.points)}")

    # Create new point cloud with only visible points
    visible_pcd = pcd.select_by_index(visible_indices)

    return PointCloud2(visible_pcd, frame_id=pc.frame_id, ts=pc.ts)


def test_hidden_removal():
    lcm.autoconf()

    with open(DETECTION_RESULT_PKL, "rb") as f:
        detections, camera_transform = pickle.load(f)

        cast_detections = []

        for detection in detections:
            cast_detections.append(hidden_point_removal(camera_transform, detection))

        publish_detected_pc(cast_detections)
