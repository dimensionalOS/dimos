from typing import TypedDict, List, Literal
from dataclasses import dataclass, field
from dimos.types.vector import Vector

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


class Position(TypedDict):
    x: float
    y: float
    z: float


class Orientation(TypedDict):
    x: float
    y: float
    z: float
    w: float


class Pose(TypedDict):
    position: Position
    orientation: Orientation


class OdometryData(TypedDict):
    header: Header
    pose: Pose


class RawOdometryMessage(TypedDict):
    type: Literal["msg"]
    topic: str
    data: OdometryData


@dataclass
class OdometryMessage:
    pos: Vector
    rot: Vector

    @classmethod
    def from_msg(cls, raw_message: RawOdometryMessage):
        pose = raw_message["data"]["pose"]
        orientation = pose["orientation"]
        pos = Vector(pose["position"])
        rot = Vector(orientation.get("x"), orientation.get("y"), orientation.get("z"))
        return cls(pos=pos, rot=rot)
