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

import pytest
from dimos_lcm.foxglove_msgs.ImageAnnotations import ImageAnnotations
from dimos_lcm.sensor_msgs import CameraInfo

from dimos.core.transport import LCMTransport
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


def publish_lcm(
    lidar_frame: LidarMessage,
    image_frame: Image,
    odom_frame,
    camera_info,
    annotations,
    detected_pc: list[PointCloud2],
):
    lidar_frame_transport = LCMTransport("/lidar", LidarMessage)
    lidar_frame_transport.broadcast(None, lidar_frame)

    image_frame_transport = LCMTransport("/image", Image)
    image_frame_transport.broadcast(None, image_frame)

    odom_frame_transport = LCMTransport("/odom", Odometry)
    odom_frame_transport.broadcast(None, odom_frame)

    camera_info_transport = LCMTransport("/camera_info", CameraInfo)
    camera_info_transport.broadcast(None, camera_info)

    for idx, detection in enumerate(detected_pc):
        detected_pointcloud_transport = LCMTransport(f"/detected_{idx}", PointCloud2)
        detected_pointcloud_transport.broadcast(None, detection)

    annotations_transport = LCMTransport("/annotations", ImageAnnotations)
    annotations_transport.broadcast(None, annotations)


def test_basic(moment):
    lcm.autoconf()
    tf = TF()
    odom_frame, lidar_frame, image_frame, camera_info, transforms = moment
    tf.publish(*transforms)

    print("Published transforms:", *transforms)

    camera_transform = tf.get("camera_optical", "world")

    detector = DetectionPointcloud()
    [image_frame, detections, separate_detections_pointcloud, detections_pointcloud] = (
        detector.process_frame(image_frame, lidar_frame, camera_info, camera_transform)
    )

    publish_lcm(
        lidar_frame,
        image_frame,
        odom_frame,
        camera_info,
        build_imageannotations([image_frame, detections]),
        separate_detections_pointcloud,
    )
