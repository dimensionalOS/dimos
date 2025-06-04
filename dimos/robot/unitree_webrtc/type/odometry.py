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
from typing import Literal, TypedDict

from dimos.robot.unitree_webrtc.type.timeseries import (
    EpochLike,
    to_datetime,
    to_human_readable,
)
from dimos.types.position import Position
from dimos.types.vector import VectorLike

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


class Pose(TypedDict):
    position: RawPosition
    orientation: Orientation


class OdometryData(TypedDict):
    header: Header
    pose: Pose


class RawOdometryMessage(TypedDict):
    type: Literal["msg"]
    topic: str
    data: OdometryData


class Odometry(Position):
    def __init__(self, pos: VectorLike, rot: VectorLike, ts: EpochLike):
        super().__init__(pos, rot)
        self.ts = to_datetime(ts) if ts else datetime.now()

    @staticmethod
    def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
        """Convert quaternion to yaw angle (rotation around z-axis) in radians."""
        # Calculate yaw (rotation around z-axis)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return yaw

    @classmethod
    def from_msg(cls, msg: RawOdometryMessage) -> "Odometry":
        pose = msg["data"]["pose"]
        orientation = pose["orientation"]
        position = pose["position"]

        # Extract position
        pos = [position.get("x"), position.get("y"), position.get("z")]

        # Extract quaternion components
        qx = orientation.get("x")
        qy = orientation.get("y")
        qz = orientation.get("z")
        qw = orientation.get("w")

        # Convert quaternion to yaw angle and store in rot.z
        # Keep x,y as quaternion components for now, but z becomes the actual yaw angle
        yaw_radians = cls.quaternion_to_yaw(qx, qy, qz, qw)
        rot = [qx, qy, yaw_radians]

        return cls(pos, rot, msg["data"]["header"]["stamp"])

    def __repr__(self) -> str:
        return f"Odom ts({to_human_readable(self.ts)}) pos({self.pos}), rot({self.rot}) yaw({math.degrees(self.rot.z):.1f}°)"
