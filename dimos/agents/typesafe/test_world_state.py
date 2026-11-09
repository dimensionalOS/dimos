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

import math
import time

import numpy as np

from dimos.agents.typesafe.world_state import bearing_word, build_world_state, distance_word
from dimos.message_utils import geometry, pointcloud, time as message_time
from dimos.messages.geometry_msgs.msg import (
    Point,
    Pose,
    PoseStamped,
    PoseWithCovariance,
    Quaternion,
    Vector3,
)
from dimos.messages.std_msgs.msg import Header
from dimos.messages.vision_msgs.msg import (
    BoundingBox2D,
    BoundingBox3D,
    Detection2D,
    Detection2DArray,
    Detection3D,
    Detection3DArray,
    ObjectHypothesis,
    ObjectHypothesisWithPose,
    Point2D,
    Pose2D,
)


def pose(x: float, y: float, yaw_deg: float) -> PoseStamped:
    p = PoseStamped(
        header=Header(stamp=message_time.time_from_seconds(time.time()), frame_id="world"),
        pose=Pose(position=Point(x, y, 0.4), orientation=Quaternion(0.0, 0.0, 0.0, 1.0)),
    )
    p.pose.orientation = geometry.quaternion_from_euler(0, 0, math.radians(yaw_deg))
    return p


def det3d(label: str, x: float, y: float, ts: float | None = None) -> Detection3DArray:
    header = Header(message_time.time_from_seconds(time.time() if ts is None else ts), "world")
    center = Pose(Point(x, y, 0.3), Quaternion(0, 0, 0, 1))
    result = ObjectHypothesisWithPose(
        ObjectHypothesis(label, 0.9), PoseWithCovariance(center, np.zeros(36))
    )
    detection = Detection3D(header, [result], BoundingBox3D(center, Vector3(0.5, 0.5, 0.6)), "")
    return Detection3DArray(header, [detection])


def det2d(label: str, cx: float, w: float, h: float, ts: float | None = None) -> Detection2DArray:
    header = Header(message_time.time_from_seconds(time.time() if ts is None else ts), "camera")
    center = Pose(Point(0, 0, 0), Quaternion(0, 0, 0, 1))
    result = ObjectHypothesisWithPose(
        ObjectHypothesis(label, 0.8), PoseWithCovariance(center, np.zeros(36))
    )
    detection = Detection2D(
        header, [result], BoundingBox2D(Pose2D(Point2D(cx, 360.0), 0.0), w, h), ""
    )
    return Detection2DArray(header, [detection])


def _state(**kw):  # type: ignore[no-untyped-def]
    args = {
        "detections_3d": None,
        "detections_2d": None,
        "lidar": None,
        "goal_xy": None,
        "image_size": (1280, 720),
        "lidar_band": (-0.2, 0.8, 5.0),
    }
    args.update(kw)
    return build_world_state("go to the chair", args.pop("pose", pose(0, 0, 0)), "idle", **args)


def test_pose_to_json() -> None:
    assert {
        key: value for key, value in _state(pose=pose(1, 2, 90))["robot"].items() if key != "motion"
    } == {
        "position": {"x": 1.0, "y": 2.0, "z": 0.4},
        "yaw_deg": 90.0,
        "heading": "north",
    }
    assert _state(pose=pose(0, 0, -45))["robot"]["heading"] == "south_east"


def test_words() -> None:
    assert [bearing_word(a) for a in (0.0, math.pi / 2, -math.pi / 2, math.pi)] == [
        "ahead",
        "left",
        "right",
        "behind",
    ]
    assert [distance_word(d) for d in (0.2, 1.0, 3.0, 6.0)] == ["touching", "near", "mid", "far"]


def test_objects_3d_relative_to_pose() -> None:
    # robot at origin facing north: a chair at (-2, 0) is on its left, 2 m away
    (obj,) = _state(pose=pose(0, 0, 90), detections_3d=det3d("chair", -2.0, 0.0))["objects"]
    assert (
        obj["label"],
        obj["bearing"],
        obj["bearing_deg"],
        obj["distance"],
        obj["distance_m"],
    ) == ("chair", "left", 90.0, "mid", 2.0)


def test_objects_2d_bearing_and_size() -> None:
    (obj,) = _state(detections_2d=det2d("person", 1200.0, 640, 600))["objects"]
    assert (obj["bearing"], obj["size"]) == ("far_right", "filling_view")


def test_room_sectors_in_robot_frame() -> None:
    p = pose(1.0, 1.0, 90)  # facing north
    pts = np.array(
        [[1.0, 1.3, 0.5], [3.0, 1.0, 0.5], [1.0, -2.0, 0.5], [1.0, 1.0, 5.0]]
    )  # ahead 0.3, right 2, behind 3, above
    room = _state(pose=p, lidar=pointcloud.from_xyz(pts, header=message_time.header_now("world")))[
        "room"
    ]
    assert room["ahead"] == {"clear_m": 0.3, "state": "blocked"}
    assert room["right"] == {"clear_m": 2.0, "state": "clear"}
    assert (room["behind"]["clear_m"], room["left"]["clear_m"]) == (3.0, 5.0)
    assert "room" not in _state()


def test_goal_point_relative_to_pose() -> None:
    gp = _state(pose=pose(0, 0, 90), goal_xy=(-2.0, 0.0))["goal_point"]
    assert gp == {
        "x": -2.0,
        "y": 0.0,
        "bearing": "left",
        "bearing_deg": 90.0,
        "distance": "mid",
        "distance_m": 2.0,
    }
    assert "goal_point" not in _state()


def test_generated_3d_detection_keeps_marker_identity():
    detections = det3d(" chair ", 2.0, 0.0)
    detections.detections[0].id = " 23 "
    objects = _state(detections_3d=detections)["objects"]
    assert objects[0]["label"] == "chair id=23"
