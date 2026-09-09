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

"""The one place an R1 Pro stream's wire topic is chosen.

``r1pro_coordinator`` builds its transports from this, and a recording made off
those transports is named by the same topics, so :class:`R1ProReplay` looks its
streams up here rather than against a second hand-kept list that would rot the
moment a topic is renamed.
"""

from __future__ import annotations

from typing import NamedTuple

from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.CompressedImage import CompressedImage
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.Imu import Imu
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.sensor_msgs.MotorCommandArray import MotorCommandArray
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2


class Topic(NamedTuple):
    path: str
    msg_type: type
    # Keep only the newest sample. Right for sensor frames a late consumer
    # should not receive a backlog of; wrong for commands.
    latest_wins: bool = False


# Port name -> topic. A port absent from here is not on the R1's zenoh bus.
R1PRO_TOPICS: dict[str, Topic] = {
    # WholeBody bridge (hw_id="r1pro"). TransportWholeBodyAdapter builds
    # /{hw}/motor_states|imu|motor_command itself, so these three topics are
    # fixed by hardware_id, not a naming choice. Only one IMU goes to /r1pro/imu.
    "motor_states": Topic("/r1pro/motor_states", JointState),
    "imu_chassis": Topic("/r1pro/imu", Imu),
    "imu_torso": Topic("/imu_torso", Imu),
    "motor_command": Topic("/r1pro/motor_command", MotorCommandArray),
    # Twist bridge (hw_id="chassis").
    "chassis_cmd_vel": Topic("/chassis/cmd_vel", Twist),
    "chassis_odom": Topic("/chassis/odom", PoseStamped),
    # Wheel odometry (pose + twist) for navigation consumers.
    "odometry": Topic("/odometry", Odometry),
    # Public Twist bus: any module's cmd_vel Out drives the coordinator's
    # twist_command In.
    "cmd_vel": Topic("/cmd_vel", Twist),
    "twist_command": Topic("/cmd_vel", Twist),
    # Sensor pass-throughs.
    "head_left_color": Topic("/head_left_color", CompressedImage, latest_wins=True),
    "head_right_color": Topic("/head_right_color", CompressedImage, latest_wins=True),
    "head_depth": Topic("/head_depth", Image, latest_wins=True),
    "head_camera_info": Topic("/head_camera_info", CameraInfo, latest_wins=True),
    "lidar": Topic("/lidar", PointCloud2, latest_wins=True),
    "wrist_left_color": Topic("/wrist_left_color", CompressedImage, latest_wins=True),
    "wrist_left_depth": Topic("/wrist_left_depth", Image, latest_wins=True),
    "wrist_right_color": Topic("/wrist_right_color", CompressedImage, latest_wins=True),
    "wrist_right_depth": Topic("/wrist_right_depth", Image, latest_wins=True),
    # ControlCoordinator outs.
    "coordinator_joint_state": Topic("/coordinator/joint_state", JointState),
    "joint_command": Topic("/r1pro/joint_command", JointState),
}


def recorded_stream_name(port_name: str) -> str:
    """The stream name the generic recorder writes for *port_name*.

    It slugs the topic it subscribed to, so ``/r1pro/motor_states`` lands as
    ``r1pro_motor_states``. A port with no entry here rides its own name.
    """
    topic = R1PRO_TOPICS.get(port_name)
    path = topic.path if topic else port_name
    return path.strip("/").replace("/", "_")
