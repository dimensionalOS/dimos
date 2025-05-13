from typing import TypedDict, Literal
from datetime import datetime
from dataclasses import dataclass
from dimos.types.vector import Vector
from dimos.types.position import Position
from dimos.robot.unitree_webrtc.type.timeseries import Timestamped, to_human_readable

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


@dataclass
class Odometry(Timestamped, Position):
    ts: datetime

    @classmethod
    def from_msg(cls, msg: RawOdometryMessage) -> "Odometry":
        pose = msg["data"]["pose"]
        orientation = pose["orientation"]
        position = pose["position"]
        pos = Vector(position.get("x"), position.get("y"), position.get("z"))
        rot = Vector(orientation.get("x"), orientation.get("y"), orientation.get("z"))
        return cls(pos=pos, rot=rot, ts=msg["data"]["header"]["stamp"])

    def __repr__(self) -> str:
        return f"Odom ts({to_human_readable(self.ts)}) pos({self.pos}), rot({self.rot})"
