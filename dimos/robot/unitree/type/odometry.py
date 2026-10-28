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
from typing import Literal, TypedDict

from dimos.messages.builtin_interfaces.msg import Time
from dimos.messages.geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
from dimos.messages.std_msgs.msg import Header as MessageHeader

raw_odometry_msg_sample = {
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


def pose_from_message(msg: RawOdometryMessage) -> PoseStamped:
    """Parse the Unitree source stamp without converting through floating seconds."""
    data = msg["data"]
    pose = data["pose"]
    position, rotation = pose["position"], pose["orientation"]
    stamp = data["header"]["stamp"]
    return PoseStamped(
        MessageHeader(Time(stamp["sec"], stamp["nanosec"]), "world"),
        Pose(
            Point(position["x"], position["y"], position["z"]),
            Quaternion(rotation["x"], rotation["y"], rotation["z"], rotation["w"]),
        ),
    )
