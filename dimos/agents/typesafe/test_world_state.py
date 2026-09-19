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


def _pose(x: float, y: float, yaw_deg: float) -> PoseStamped:
    p = PoseStamped(position=(x, y, 0.4), frame_id="world")
    p.orientation = Quaternion.from_euler(Vector3(0, 0, math.radians(yaw_deg)))
    return p


def _det3d(label: str, x: float, y: float) -> Detection3DArray:
    d = Detection3D()
    d.header = Header(1.0, "world")
    d.results = [ObjectHypothesisWithPose(hypothesis=ObjectHypothesis(class_id=label, score=0.9))]
    d.results_length = 1
    d.bbox = BoundingBox3D(center=Pose(position=(x, y, 0.3)), size=Vector3(0.5, 0.5, 0.6))
    return Detection3DArray(detections_length=1, header=Header(1.0, "world"), detections=[d])


def _det2d(label: str, cx: float, w: float, h: float) -> Detection2DArray:
    d = Detection2D()
    d.header = Header(1.0, "camera")
    d.results = [ObjectHypothesisWithPose(hypothesis=ObjectHypothesis(class_id=label, score=0.8))]
    d.results_length = 1
    center = Pose2D()
    center.position.x, center.position.y = cx, 360.0
    d.bbox = BoundingBox2D(center=center, size_x=w, size_y=h)
    return Detection2DArray(detections_length=1, header=Header(1.0, "camera"), detections=[d])


def test_pose_to_json_heading() -> None:
    j = _pose(1, 2, 90).to_json()
    assert j["position"] == {"x": 1.0, "y": 2.0, "z": 0.4}
    assert (j["yaw_deg"], j["heading"]) == (90.0, "north")
    assert _pose(0, 0, -45).to_json()["heading"] == "south_east"


def test_bearing_and_distance_words() -> None:
    assert [bearing_word(a) for a in (0.0, math.pi / 2, -math.pi / 2, math.pi)] == [
        "ahead",
        "left",
        "right",
        "behind",
    ]
    assert [distance_word(d) for d in (0.2, 1.0, 3.0, 6.0)] == ["touching", "near", "mid", "far"]


def test_objects_3d_relative_to_pose() -> None:
    # robot at origin facing north: a chair at (-2, 0) is on its left.
    state = build_world_state(
        "go to the chair",
        _pose(0, 0, 90),
        detections_3d=_det3d("chair", -2.0, 0.0),
        detections_2d=None,
        lidar=None,
        robot={},
    )
    (obj,) = state["objects"]
    assert (obj["label"], obj["bearing"], obj["distance"], obj["distance_m"]) == (
        "chair",
        "left",
        "mid",
        1.75,  # to the 0.5 m box's edge
    )
    assert obj["bearing_deg"] == 90.0
    assert state["unavailable"] == ["room"]


def test_objects_2d_bearing_and_size() -> None:
    state = build_world_state(
        "x",
        _pose(0, 0, 0),
        detections_3d=None,
        detections_2d=_det2d("person", 1200.0, 640, 600),
        lidar=None,
        robot={},
    )
    (obj,) = state["objects"]
    assert (obj["bearing"], obj["size"]) == ("far_right", "filling_view")


def test_pointcloud_sectors_in_robot_frame() -> None:
    pose = _pose(1.0, 1.0, 90)  # facing north
    pts = np.array(
        [[1.0, 1.3, 0.5], [3.0, 1.0, 0.5], [1.0, -2.0, 0.5], [1.0, 1.0, 5.0]]
    )  # ahead 0.3, right 2, behind 3, above
    s = PointCloud2.from_numpy(pts, frame_id="world").to_json(pose)
    assert s["ahead"] == {"clear_m": 0.3, "state": "blocked"}
    assert s["right"] == {"clear_m": 2.0, "state": "clear"}
    assert s["behind"]["clear_m"] == 3.0
    assert s["left"]["clear_m"] == 5.0


def test_goal_object_listed_beyond_the_nearest_cap() -> None:
    dets = Detection3DArray(header=Header(1.0, "world"))
    for i in range(25):
        dets.detections.append(_det3d("cabinet", 1.0 + i * 0.1, 0.0).detections[0])
    dets.detections.append(_det3d("chair", 40.0, 0.0).detections[0])
    dets.detections_length = len(dets.detections)
    state = build_world_state(
        "go to the chair at (40.00, 0.00)",
        _pose(0, 0, 0),
        detections_3d=dets,
        detections_2d=None,
        lidar=None,
        robot={},
    )
    labels = [o["label"] for o in state["objects"]]
    assert len(labels) == 20 and labels[-1] == "chair"


def test_goal_coordinates_pick_one_of_several_same_label_objects() -> None:
    dets = _det3d("chair", 2.0, 0.0)
    dets.detections.append(_det3d("chair", 40.0, 0.0).detections[0])
    dets.detections.append(_det3d("table", 3.0, 0.0).detections[0])
    dets.detections_length = 3
    state = build_world_state(
        "go to the chair at (40.00, 0.00)",
        _pose(0, 0, 0),
        detections_3d=dets,
        detections_2d=None,
        lidar=None,
        robot={},
    )
    assert [(o["label"], o["distance_m"]) for o in state["objects"]] == [
        ("table", 2.75),
        ("chair", 39.75),
    ]


def test_body_frame_scan_is_not_transformed() -> None:
    # Robot far from the world origin; a body-frame point 0.3 m ahead must still block ahead.
    pts = np.array([[0.3, 0.0, 0.5], [0.0, -2.0, 0.5]])
    state = build_world_state(
        "go to the chair",
        _pose(12.0, -7.0, 90),
        detections_3d=None,
        detections_2d=None,
        lidar=PointCloud2.from_numpy(pts, frame_id="base_link"),
        robot={},
    )
    s = state["room"]["sectors"]
    assert s["ahead"] == {"clear_m": 0.3, "state": "blocked"}
    assert s["right"]["clear_m"] == 2.0 and s["left"]["clear_m"] == 5.0
