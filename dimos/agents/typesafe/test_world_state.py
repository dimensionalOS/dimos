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

from dimos_lcm.vision_msgs import (
    BoundingBox2D,
    BoundingBox3D,
    Detection2D,
    Detection3D,
    ObjectHypothesis,
    ObjectHypothesisWithPose,
    Pose2D,
)
import numpy as np

from dimos.agents.typesafe.world_state import bearing_word, build_world_state, distance_word
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.msgs.std_msgs.Header import Header
from dimos.msgs.vision_msgs.Detection2DArray import Detection2DArray
from dimos.msgs.vision_msgs.Detection3DArray import Detection3DArray


def pose(x: float, y: float, yaw_deg: float) -> PoseStamped:
    p = PoseStamped(position=(x, y, 0.4), frame_id="world")
    p.orientation = Quaternion.from_euler(Vector3(0, 0, math.radians(yaw_deg)))
    return p


def det3d(label: str, x: float, y: float, ts: float | None = None) -> Detection3DArray:
    ts = time.time() if ts is None else ts
    d = Detection3D()
    d.header = Header(ts, "world")
    d.results = [ObjectHypothesisWithPose(hypothesis=ObjectHypothesis(class_id=label, score=0.9))]
    d.results_length = 1
    d.bbox = BoundingBox3D(center=Pose(position=(x, y, 0.3)), size=Vector3(0.5, 0.5, 0.6))
    return Detection3DArray(detections_length=1, header=Header(ts, "world"), detections=[d])


def _det2d(label: str, cx: float, w: float, h: float) -> Detection2DArray:
    d = Detection2D()
    d.header = Header(1.0, "camera")
    d.results = [ObjectHypothesisWithPose(hypothesis=ObjectHypothesis(class_id=label, score=0.8))]
    d.results_length = 1
    center = Pose2D()
    center.position.x, center.position.y = cx, 360.0
    d.bbox = BoundingBox2D(center=center, size_x=w, size_y=h)
    return Detection2DArray(detections_length=1, header=Header(1.0, "camera"), detections=[d])


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
    assert pose(1, 2, 90).to_json() == {
        "position": {"x": 1.0, "y": 2.0, "z": 0.4},
        "yaw_deg": 90.0,
        "heading": "north",
    }
    assert pose(0, 0, -45).to_json()["heading"] == "south_east"


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
    (obj,) = _state(detections_2d=_det2d("person", 1200.0, 640, 600))["objects"]
    assert (obj["bearing"], obj["size"]) == ("far_right", "filling_view")


def test_room_sectors_in_robot_frame() -> None:
    p = pose(1.0, 1.0, 90)  # facing north
    pts = np.array(
        [[1.0, 1.3, 0.5], [3.0, 1.0, 0.5], [1.0, -2.0, 0.5], [1.0, 1.0, 5.0]]
    )  # ahead 0.3, right 2, behind 3, above
    room = _state(pose=p, lidar=PointCloud2.from_numpy(pts, frame_id="world"))["room"]
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
