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
import math
from datetime import datetime
from io import BytesIO
from typing import BinaryIO, Literal, TypeAlias, TypedDict

from scipy.spatial.transform import Rotation as R

from dimos.msgs.geometry_msgs import PoseStamped as LCMPoseStamped
from dimos.msgs.geometry_msgs import Quaternion, Vector3
from dimos.robot.unitree_webrtc.type.timeseries import (
    EpochLike,
    Timestamped,
    to_datetime,
    to_human_readable,
)
from dimos.types.timestamped import to_timestamp
from dimos.types.vector import Vector, VectorLike

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


class Odometry(LCMPoseStamped):
    name = "geometry_msgs.PoseStamped"

    @classmethod
    def from_msg(cls, msg: RawOdometryMessage) -> "Odometry":
        pose = msg["data"]["pose"]
        orientation = pose["orientation"]
        position = pose["position"]

        # Extract position
        pos = Vector3(position.get("x"), position.get("y"), position.get("z"))

        rot = Quaternion(
            orientation.get("x"),
            orientation.get("y"),
            orientation.get("z"),
            orientation.get("w"),
        )

        ts = to_timestamp(msg["data"]["header"]["stamp"])
        return Odometry(pos, rot, ts=ts, frame_id="lidar")

    def __repr__(self) -> str:
        return f"Odom pos({self.position}), rot({self.orientation})"
