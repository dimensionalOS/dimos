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

from dimos_generated.builtin_interfaces.msg import Time
from dimos_generated.geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
from dimos_generated.sensor_msgs.msg import CameraInfo, RegionOfInterest
from dimos_generated.std_msgs.msg import Header
from dimos_generated.tf2_msgs.msg import TFMessage
from dimos_message_build.registry import decode as cdr_decode, encode as cdr_encode
import numpy as np
import pytest

from dimos.core.global_config import GlobalConfig
from dimos.robot.unitree.dimsim_connection import _odom_to_tf
from dimos.robot.unitree.g1.mujoco_sim import G1SimConnection
from dimos.robot.unitree.mujoco_connection import MujocoConnection


@pytest.fixture
def source_pose():
    return PoseStamped(
        header=Header(frame_id="world", stamp=Time(sec=1700000000, nanosec=123456789)),
        pose=Pose(
            position=Point(x=1.25, y=2.5, z=0.7), orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        ),
    )


def test_dimsim_tf_preserves_source_pose_and_stamp(source_pose):
    message = cdr_decode(cdr_encode(TFMessage(transforms=_odom_to_tf(source_pose))), TFMessage)
    assert [(edge.header.frame_id, edge.child_frame_id) for edge in message.transforms] == [
        ("world", "base_link"),
        ("base_link", "camera_link"),
        ("camera_link", "camera_optical"),
        ("base_link", "lidar_link"),
    ]
    assert all(edge.header.stamp == source_pose.header.stamp for edge in message.transforms)
    assert message.transforms[0].transform.translation.x == 1.25
    assert message.transforms[1].transform.translation.x == 0.3


@pytest.fixture
def g1_connection(mocker):
    connection = G1SimConnection(g=GlobalConfig(robot_ip="127.0.0.1"))
    mocker.patch.object(connection, "connection", mocker.MagicMock(spec=MujocoConnection))
    mocker.patch.object(connection, "tf", mocker.MagicMock())
    mocker.patch.object(connection, "odom", mocker.MagicMock())
    mocker.patch.object(connection, "camera_info", mocker.MagicMock())
    try:
        yield connection
    finally:
        connection.stop()


def test_g1_simulator_tf_uses_pose_time_for_all_mounts(g1_connection, source_pose):
    g1_connection._publish_tf(source_pose)
    message = cdr_decode(g1_connection.tf.publish.call_args.args[0].encode(), TFMessage)
    assert [(edge.header.frame_id, edge.child_frame_id) for edge in message.transforms] == [
        ("world", "base_link"),
        ("base_link", "camera_link"),
        ("camera_link", "camera_optical"),
        ("map", "world"),
    ]
    assert all(edge.header.stamp == source_pose.header.stamp for edge in message.transforms)
    assert message.transforms[1].transform.translation.z == 0.6
    assert g1_connection.odom.publish.call_args.args[0] == source_pose


def test_g1_camera_info_stamps_each_publication_without_mutating_template(g1_connection, mocker):
    template = CameraInfo(
        header=Header(frame_id="camera_optical", stamp=Time(sec=0, nanosec=0)),
        width=640,
        height=288,
        distortion_model="",
        d=np.array([], dtype=np.float64),
        k=np.zeros(9, dtype=np.float64),
        r=np.zeros(9, dtype=np.float64),
        p=np.zeros(12, dtype=np.float64),
        binning_x=0,
        binning_y=0,
        roi=RegionOfInterest(x_offset=0, y_offset=0, height=0, width=0, do_rectify=False),
    )
    g1_connection.connection.camera_info_static = template
    mocker.patch("dimos.robot.unitree.g1.mujoco_sim.time.time_ns", return_value=1700000000123456789)
    mocker.patch.object(
        g1_connection._stop_event, "wait", side_effect=lambda _: g1_connection._stop_event.set()
    )
    g1_connection._publish_camera_info_loop()
    message = cdr_decode(g1_connection.camera_info.publish.call_args.args[0].encode(), CameraInfo)
    assert message.header.stamp == Time(sec=1700000000, nanosec=123456789)
    assert message.header.frame_id == "camera_optical"
    assert template.header.stamp == Time(sec=0, nanosec=0)
