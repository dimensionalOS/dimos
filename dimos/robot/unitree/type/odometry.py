# Copyright 2025-2026 Dimensional Inc.
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
from copy import deepcopy
from typing import Literal, TypedDict

from dimos_generated.builtin_interfaces.msg import Time
from dimos_generated.geometry_msgs.msg import (
    Point,
    Pose,
    PoseStamped as GeneratedPoseStamped,
    Quaternion as GeneratedQuaternion,
)
from dimos_generated.std_msgs.msg import Header as GeneratedHeader

raw_odometry_msg_sample: "RawOdometryMessage" = {
    "type": "msg",
    "topic": "rt/utlidar/robot_pose",
    "data": {
        "header": {"stamp": {"sec": 1746565669, "nanosec": 448350564}, "frame_id": "odom"},
        "pose": {
            "position": {"x": 5.961965, "y": -2.916958, "z": 0.319509},
            "orientation": {"x": 0.002787, "y": -0.000902, "z": -0.970244, "w": -0.242112},
        },
    },
}


class TimeStamp(TypedDict):
    sec: int
    nanosec: int


class Header(TypedDict):
    stamp: TimeStamp
    frame_id: str


class RawPosition(TypedDict):
    x: float
    y: float
    z: float


class Orientation(TypedDict):
    x: float
    y: float
    z: float
    w: float


class PoseData(TypedDict):
    position: RawPosition
    orientation: Orientation


class OdometryData(TypedDict):
    header: Header
    pose: PoseData


class RawOdometryMessage(TypedDict):
    type: Literal["msg"]
    topic: str
    data: OdometryData


def pose_from_webrtc_odometry(
    message: RawOdometryMessage, *, header: GeneratedHeader | None = None
) -> GeneratedPoseStamped:
    """Copy the device's ROS-shaped pose, preserving its header unless explicitly replaced."""
    data = message["data"]
    source_header = data["header"]
    pose = data["pose"]
    return GeneratedPoseStamped(
        header=deepcopy(header)
        if header is not None
        else GeneratedHeader(
            stamp=Time(**source_header["stamp"]), frame_id=source_header["frame_id"]
        ),
        pose=Pose(
            position=Point(**pose["position"]),
            orientation=GeneratedQuaternion(**pose["orientation"]),
        ),
    )
