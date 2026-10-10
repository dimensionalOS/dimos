# Copyright 2026 Dimensional Inc.
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

# Copyright 2026 Dimensional Inc.
# SPDX-License-Identifier: Apache-2.0

from dimos_generated.builtin_interfaces.msg import Time
from dimos_generated.geometry_msgs.msg import (
    Point,
    Pose,
    PoseWithCovariance,
    Quaternion,
    Twist,
    TwistWithCovariance,
    Vector3,
)
from dimos_generated.nav_msgs.msg import Odometry
from dimos_generated.sensor_msgs.msg import CameraInfo, Image, PointCloud2
from dimos_generated.std_msgs.msg import Header
from dimos_message_build.registry import encode as cdr_encode
import numpy as np
import pytest

from dimos.hardware.sensors.camera.realsense.blueprints import RealSenseMountTf, _cloud
from dimos.hardware.sensors.lidar.fastlio2.recorder import FastLio2Recorder
from dimos.hardware.sensors.lidar.pointlio.recorder import PointlioRecorder
from dimos.mapping.relocalization.blueprints import RecordingPlayer, _fine_points
from dimos.msgs.geometry import transform_matrix
from dimos.msgs.pointcloud import pointcloud_from_xyz
from dimos.robot.assembly.mid360_realsense_30 import Mid360RealsenseRecorder
from dimos.robot.unitree.g1.g1_recorder import G1Recorder, G1RecorderConfig


@pytest.mark.asyncio
@pytest.mark.parametrize("recorder_type", [FastLio2Recorder, PointlioRecorder])
async def test_generated_recorders_anchor_clouds_to_last_odometry(recorder_type, tmp_path):
    recorder = recorder_type(db_path=tmp_path / "unused.db")
    try:
        cloud = pointcloud_from_xyz(
            np.array([[1, 2, 3]], dtype=np.float32),
            header=Header(stamp=Time(sec=0, nanosec=0), frame_id=""),
        )
        assert await recorder._lidar_pose(cloud) is None
        pose = Pose(
            position=Point(x=2, y=3, z=0.0), orientation=Quaternion(w=1, x=0.0, y=0.0, z=0.0)
        )
        odometry = Odometry(
            header=Header(frame_id="world", stamp=Time(sec=0, nanosec=0)),
            pose=PoseWithCovariance(pose=pose, covariance=np.zeros(36, dtype=np.float64)),
            child_frame_id="",
            twist=TwistWithCovariance(
                twist=Twist(
                    linear=Vector3(x=0.0, y=0.0, z=0.0), angular=Vector3(x=0.0, y=0.0, z=0.0)
                ),
                covariance=np.zeros(36, dtype=np.float64),
            ),
        )
        assert await recorder._odom_pose(odometry) == pose
        assert await recorder._lidar_pose(cloud) == pose
        assert not (tmp_path / "unused.db").exists()
    finally:
        recorder.stop()


def test_recorder_and_replay_ports_use_generated_message_identities():
    for recorder_type in [FastLio2Recorder, PointlioRecorder]:
        streams = {
            stream.name: stream.type for stream in recorder_type.blueprint().blueprints[0].streams
        }
        prefix = "fastlio" if recorder_type is FastLio2Recorder else "pointlio"
        assert streams[f"{prefix}_odometry"] is Odometry
        assert streams[f"{prefix}_lidar"] is PointCloud2
    streams = {
        stream.name: stream.type
        for stream in Mid360RealsenseRecorder.blueprint().blueprints[0].streams
    }
    assert streams["color_image"] is Image
    assert streams["realsense_depth_image"] is Image
    assert streams["realsense_pointcloud"] is PointCloud2
    assert streams["realsense_camera_info"] is CameraInfo
    streams = {
        stream.name: stream.type for stream in RecordingPlayer.blueprint().blueprints[0].streams
    }
    assert streams["lidar"] is PointCloud2


def test_realsense_mount_is_an_identity_and_generated_cloud_views_are_typed():
    mount = RealSenseMountTf()
    try:
        edge = mount.transforms()[0]
        assert edge.header.frame_id == "world" and edge.child_frame_id == "camera_link"
        np.testing.assert_allclose(transform_matrix(edge.transform), np.eye(4))
    finally:
        mount.stop()
    cloud = pointcloud_from_xyz(
        np.array([[1, 2, 3]], dtype=np.float32),
        header=Header(stamp=Time(sec=0, nanosec=0), frame_id=""),
    )
    before = cdr_encode(cloud)
    assert _cloud(cloud).positions is not None
    assert _fine_points(cloud).positions is not None
    assert cdr_encode(cloud) == before


def test_g1_depth_recorder_declares_generated_lossless_cdr():
    config = G1RecorderConfig()
    assert config.stream_codecs["realsense_depth_image"] == "lz4+cdr"
    streams = {s.name: s.type for s in G1Recorder.blueprint().blueprints[0].streams}
    assert streams["realsense_depth_image"] is Image
    assert streams["realsense_camera_info"] is CameraInfo
