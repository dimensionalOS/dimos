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
from typing import Literal, TypedDict

from dimos.msgs.geometry_msgs import PoseStamped, Quaternion, Vector3
from dimos.robot.unitree_webrtc.type.timeseries import (
    Timestamped,
)
from dimos.types.timestamped import to_timestamp

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


class Odometry(PoseStamped, Timestamped):  # type: ignore[misc]
    name = "geometry_msgs.PoseStamped"

    def __init__(self, frame_id: str = "base_link", *args, **kwargs) -> None:  # type: ignore[no-untyped-def]
        super().__init__(frame_id=frame_id, *args, **kwargs)  # type: ignore[misc]

    @classmethod
    def from_msg(cls, msg: RawOdometryMessage) -> "Odometry":
        pose = msg["data"]["pose"]

        # Extract position
        pos = Vector3(
            pose["position"].get("x"),
            pose["position"].get("y"),
            pose["position"].get("z"),
        )

        rot = Quaternion(
            pose["orientation"].get("x"),
            pose["orientation"].get("y"),
            pose["orientation"].get("z"),
            pose["orientation"].get("w"),
        )

        ts = to_timestamp(msg["data"]["header"]["stamp"])
        return Odometry(position=pos, orientation=rot, ts=ts, frame_id="world")

    def __repr__(self) -> str:
        return f"Odom pos({self.position}), rot({self.orientation})"
