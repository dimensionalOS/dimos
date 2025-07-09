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
from dimos.types.vector import VectorLike, Vector
from dimos.robot.unitree_webrtc.type.timeseries import Timestamped, to_human_readable
from scipy.spatial.transform import Rotation as R

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

    @classmethod
    def from_msg(cls, msg: RawOdometryMessage) -> "Odometry":
        pose = msg["data"]["pose"]
        orientation = pose["orientation"]
        position = pose["position"]

        # Extract position
        pos = [position.get("x"), position.get("y"), position.get("z")]

        quat = [
            orientation.get("x"),
            orientation.get("y"),
            orientation.get("z"),
            orientation.get("w"),
        ]

        # Check if quaternion has zero norm (invalid)
        quat_norm = sum(x**2 for x in quat) ** 0.5
        if quat_norm < 1e-8:
            quat = [0.0, 0.0, 0.0, 1.0]

        rotation = R.from_quat(quat)
        rot = Vector(rotation.as_euler("xyz", degrees=False))

        return cls(pos, rot, msg["data"]["header"]["stamp"])

    def __repr__(self) -> str:
        return f"Odom ts({to_human_readable(self.ts)}) pos({self.pos}), rot({self.rot}) yaw({math.degrees(self.rot.z):.1f}°)"
