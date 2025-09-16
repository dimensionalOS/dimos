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

from dimos.perception.detection2d.module2D import Detection2DModule
from dimos.perception.detection2d.module3D import Detection3DModule, build_imageannotations
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
    # data_dir = "unitree_office_walk"
    data_dir = "unitree_go2_lidar_corrected"
    get_data(data_dir)

    seek = 10

    lidar_frame = TimedSensorReplay(f"{data_dir}/lidar").find_closest_seek(seek)
    image_frame = TimedSensorReplay(
        f"{data_dir}/video",
    ).find_closest(lidar_frame.ts)

    odom_frame = TimedSensorReplay(f"{data_dir}/odom", autocast=Odometry.from_msg).find_closest(
        lidar_frame.ts
    )

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
        detected_pointcloud_transport: LCMTransport = LCMTransport(f"/detected_{idx}", PointCloud2)
        detected_pointcloud_transport.publish(detection.pointcloud)


def publish_lcm(
    lidar_frame: LidarMessage,
    image_frame: Image,
    odom_frame,
    camera_info,
    annotations,
    detected_pc: list[PointCloud2],
):
    lidar_frame_transport: LCMTransport = LCMTransport("/lidar", LidarMessage)
    lidar_frame_transport.publish(lidar_frame)

    image_frame_transport: LCMTransport = LCMTransport("/image", Image)
    image_frame_transport.publish(image_frame)

    odom_frame_transport: LCMTransport = LCMTransport("/odom", Odometry)
    odom_frame_transport.publish(odom_frame)

    camera_info_transport: LCMTransport = LCMTransport("/camera_info", CameraInfo)
    camera_info_transport.publish(camera_info)

    annotations_transport: LCMTransport = LCMTransport("/annotations", ImageAnnotations)
    annotations_transport.publish(annotations)

    publish_detected_pc(detected_pc)


def test_basic(moment):
    lcm.autoconf()
    odom_frame, lidar_frame, image_frame, camera_info, transforms = moment
    tf = TF()
    tf.publish(*transforms)

    camera_transform = tf.get("camera_optical", "world")

    detector = Detection3DModule()
    detections = Detection2DModule.process_frame(detector, image_frame)

    # Process detections to get Detection3D objects
    detection3d_list = detector.process_frame(
        detections, lidar_frame, camera_info, camera_transform
    )

    detection_result = [detection3d_list, camera_transform]

    # Assuming you have your detection_result object
    with open(DETECTION_RESULT_PKL, "wb") as f:
        pickle.dump(detection_result, f)

    # Save just the first detection3d for the load test
    if detection3d_list:
        detection3d_pkl = os.path.join(TEST_DIR, "detection3d.pkl")
        with open(detection3d_pkl, "wb") as f:
            pickle.dump(detection3d_list[0], f)

    publish_lcm(
        lidar_frame,
        image_frame,
        odom_frame,
        camera_info,
        build_imageannotations((image_frame, detections)),
        detection3d_list,
    )

    print("detections:\n", "\n".join(map(str, detection3d_list)))

    # how do we count points in this detection? should be 10 exactly
    # detection3d_list contains Detection3D objects
    detection3d = detection3d_list[0]
    num_points = len(detection3d.pointcloud.pointcloud.points)
    print(f"Number of points in first detection: {num_points}")
    assert num_points == 81, f"Expected 10 points, got {num_points}"


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
    camera_position = camera_transform.inverse().translation
    camera_pos_np = camera_position.to_numpy().reshape(3, 1)

    pcd = pc.pointcloud

    _, visible_indices = pcd.hidden_point_removal(camera_pos_np, radius)
    visible_pcd = pcd.select_by_index(visible_indices)

    return PointCloud2(visible_pcd, frame_id=pc.frame_id, ts=pc.ts)


@pytest.mark.tool
def test_hidden_removal():
    lcm.autoconf()

    with open(DETECTION_RESULT_PKL, "rb") as f:
        detections, camera_transform = pickle.load(f)

        cast_detections = []

        for detection in detections:
            cast_detections.append(hidden_point_removal(camera_transform, detection))

        publish_detected_pc(cast_detections)
