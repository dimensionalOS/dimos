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

from dimos.agents.typesafe.world_state import (
    Memory,
    bearing_word,
    build_world_state,
    distance_word,
    scan_reach,
    way_to_target,
)
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
        "behind_left",
    ]
    assert [distance_word(d) for d in (0.2, 1.0, 3.0, 6.0)] == ["touching", "near", "mid", "far"]


def test_bearing_word_ahead_is_narrow() -> None:
    words = [bearing_word(math.radians(d)) for d in (10, 20, -20, 60, -100)]
    assert words == ["ahead", "ahead_left", "ahead_right", "ahead_left", "right"]


def test_way_to_target_names_the_blocker_and_the_open_sides() -> None:
    # Facing north; a wall across the line to the target with a doorway at x in [1, 2].
    walls = [("wall", (-5.0, 1.0, 1.0, 1.2)), ("wall", (2.0, 1.0, 6.0, 1.2))]
    target = (-0.3, 3.0, 0.3, 3.5)
    way = way_to_target(walls, target, _pose(0, 0, 90))
    assert (way["state"], way["blocked_by"], way["blocked_at_m"]) == ("blocked", "wall", 0.8)
    left, right = way["open_sides"]  # fixed order: left, then right
    assert (left["side"], left["kind"]) == ("left", "open")
    # From the side the bearing is to the point in front of the opening, abreast of it
    # straight through it.
    assert (right["side"], right["kind"], right["bearing"]) == ("right", "doorway", "right")
    assert (right["width_m"], right["target_beyond"]) == (1.0, True)
    no_gap = way_to_target([("wall", (-5.0, 1.0, 6.0, 1.2))], target, _pose(0, 0, 90))
    assert [o["kind"] for o in no_gap["open_sides"]] == ["open", "open"]  # left, then right
    abreast = way_to_target(walls, target, _pose(1.5, 0, 90))["open_sides"]
    assert [(o["kind"], o["bearing"]) for o in abreast if o["side"] == "right"] == [
        ("doorway", "ahead")
    ]
    # A cupboard in front of the opening: no free straight line to it, so it is not listed.
    cupboard = [*walls, ("cupboard", (0.9, 0.2, 2.1, 0.7))]
    assert all(o["kind"] != "doorway" for o in way_to_target(cupboard, target, _pose(0, 0, 90))["open_sides"])  # fmt: skip
    assert way_to_target([], target, _pose(0, 0, 90)) == {"state": "clear", "room": "clear"}
    # A chair tucked against the target is part of it, a wall touching it is still a wall.
    chair = [("chair", (-0.3, 2.5, 0.3, 2.9))]
    assert way_to_target(chair, target, _pose(0, 0, 90)) == {"state": "clear", "room": "clear"}
    stool = [("stool", (-0.3, 2.7, 0.3, 2.9))]  # the robot comes to rest at it, at the target
    assert way_to_target(stool, target, _pose(0, 2.2, 90)) == {"state": "clear", "room": "blocked"}
    wall = [("wall", (-5.0, 2.8, 5.0, 2.95))]
    assert way_to_target(wall, target, _pose(0, 0, 90))["blocked_by"] == "wall"


def test_way_at_the_target_is_clear_and_its_furniture_blocks_only_when_it_stops_the_robot() -> None:
    target = (-0.3, 3.0, 0.3, 3.5)
    behind = [("wall", (-5.0, 3.5, 5.0, 3.7))]  # the wall the target stands against
    assert way_to_target(behind, target, _pose(0, 1.5, 90))["state"] == "clear"
    through = [("wall", (-5.0, 2.8, 5.0, 3.1))]  # overlaps the target's box, robot touching
    assert way_to_target(through, target, _pose(0, 2.6, 90)) == {
        "state": "clear",
        "room": "blocked",
    }
    plant = [("plant", (-0.3, 2.0, 0.3, 2.9))]  # tucked against the target, 1 m deep
    assert way_to_target(plant, target, _pose(0, 0, 90))["state"] == "clear"
    assert way_to_target(plant, target, _pose(0, 1.6, 90))["blocked_by"] == "plant"


def test_last_metre_beside_a_wall_is_clear_and_narrowed_not_blocked() -> None:
    target = (-0.3, 1.2, 0.3, 1.7)
    beside = [("wall", (-1.0, 0.3, -0.2, 0.5))]  # its end 0.2 m left of the line, not on it
    way = way_to_target(beside, target, _pose(0, 0, 90))
    assert way == {"state": "clear", "room": "blocked", "narrowed_on": "left"}
    far = way_to_target(beside, (-0.3, 3.0, 0.3, 3.5), _pose(0, 0, 90))
    assert far["state"] == "blocked"  # from further off it is gone around like anything else


def test_listed_sides_agree_with_the_scan_and_are_never_empty() -> None:
    # Facing north at a long wall with no opening: by the footprints both sides are open.
    wall = [("wall", (-6.0, 1.0, 6.0, 1.2))]
    target, pose = (-0.3, 3.0, 0.3, 3.5), _pose(0, 0, 90)
    by_map = way_to_target(wall, target, pose)["open_sides"]
    assert [(o["side"], o["detour_deg"]) for o in by_map] == [("left", 65), ("right", 65)]
    # The scan reads something the footprints lack 0.4 m away all over the left: no direction
    # there is listed; the right is untouched.
    arc = np.radians(np.arange(40, 181, 2.5))  # robot frame: ahead-left round to behind
    pts = np.column_stack((0.4 * np.cos(arc), 0.4 * np.sin(arc), np.full(len(arc), 0.3)))
    scan = scan_reach(PointCloud2.from_numpy(pts, frame_id="base_link"), None, -0.2, 0.8, 5.0)
    fused = way_to_target(wall, target, pose, scan)["open_sides"]
    assert [(o["side"], o["detour_deg"]) for o in fused] == [("right", 65)]
    # Only the scan sees something on a free line, well short of the target: a blocker with sides.
    dot = PointCloud2.from_numpy(np.array([[0.4, 0.0, 0.3]]), frame_id="base_link")
    unnamed = way_to_target([], target, pose, scan_reach(dot, None, -0.2, 0.8, 5.0))
    assert (unnamed["blocked_by"], unnamed["blocked_at_m"]) == ("obstacle", 0.4)
    assert {o["side"] for o in unnamed["open_sides"]} == {"left", "right"}
    # Boxed in with 1.2 m to either side: nothing is open, the longest direction is listed.
    pocket = [*wall, ("shelf", (-1.5, -2.0, -1.2, 1.0)), ("shelf", (1.2, -2.0, 1.5, 1.0)), ("shelf", (-1.5, -1.5, 1.5, -1.2))]  # fmt: skip
    spare = way_to_target(pocket, target, pose)["open_sides"]
    assert spare and all(o["kind"] == "free" and o["clear_m"] < 1.5 for o in spare)


def test_standing_in_a_doorway_it_is_through_the_way_faced() -> None:
    walls = [("wall", (-5.0, 1.0, 1.0, 1.2)), ("wall", (2.0, 1.0, 6.0, 1.2)), ("wall", (-3.0, -3.0, -2.8, 1.0))]  # fmt: skip
    away = (-6.0, -1.3, -5.5, -0.7)  # the target is west, behind another wall
    # In the opening facing north the way through is ahead, though the target is behind.
    inside = [o for o in way_to_target(walls, away, _pose(1.5, 1.1, 90))["open_sides"] if o["kind"] == "doorway"]  # fmt: skip
    assert [(o["bearing"], o["target_beyond"]) for o in inside] == [("ahead", False)]


def test_memory_marks_where_it_has_driven_and_tells_still_early() -> None:
    walls = [("wall", (-5.0, 1.0, -2.0, 1.2)), ("wall", (-1.0, 1.0, 1.0, 1.2)), ("wall", (2.0, 1.0, 6.0, 1.2))]  # fmt: skip
    target, mem = (-0.3, 3.0, 0.3, 3.5), Memory()
    # It comes back from beyond the right-hand opening and stands at the start again.
    for t, (x, y) in enumerate([(1.5, 1.8), (1.5, 1.0), (1.0, 0.5), (0.0, 0.0)]):
        mem.recent(float(t), "go", _pose(x, y, 90), 3.0)
    pose = _pose(0, 0, 90)
    way = way_to_target(walls, target, pose)
    mem.going_around(3.0, way, None)
    assert [o.get("been_there") for o in way["open_sides"]] == [None, None]  # too fresh to count
    turned_right = {"x": "none", "y": "none", "yaw": "turn_right"}
    way = way_to_target(walls, target, pose)
    assert mem.going_around(3.5, way, turned_right) == {"side": "right", "for_s": 0}
    mem.recent(20.0, "go", pose, 3.0)
    way = way_to_target(walls, target, pose)
    assert mem.going_around(20.0, way, turned_right)["side"] == "right"  # told, not taken away
    assert [o.get("been_there") for o in way["open_sides"]] == [None, True]
    idle = Memory()
    patterns = [idle.recent(t / 2, "go", pose, 3.0)["pattern"] for t in range(5)]
    assert patterns == ["starting", "starting", "starting", "still", "still"]
    # Driving picks with nothing moved for 3 s read stuck; one such pick after a turn does not.
    forward = {"x": "forward", "y": "none", "yaw": "none"}
    pushed = [idle.recent(3 + t / 2, "go", pose, 3.0, forward)["pattern"] for t in range(8)]
    assert pushed[0] == "still" and pushed[-1] == "stuck"


def test_what_the_scan_saw_is_remembered_when_the_robot_looks_away() -> None:
    mem, here = Memory(), _pose(0, 0, 0)
    hit_b, hit_r = np.array([0.0]), np.array([0.4])  # something 0.4 m dead ahead
    assert abs(mem.seen_lately(0.0, here, hit_b, hit_r)[0] - 0.4) < 0.06
    none = np.array([])
    turned = mem.seen_lately(1.0, _pose(0, 0, 90), none, none)  # now it is on the right
    assert abs(turned[(-90 // 5) % 72] - 0.4) < 0.06 and turned[0] == np.inf
    assert np.isinf(mem.seen_lately(40.0, _pose(0, 0, 90), none, none)).all()  # forgotten


def test_memory_keeps_the_side_the_picks_steered_to() -> None:
    walls = [("wall", (-5.0, 1.0, -2.0, 1.2)), ("wall", (-1.0, 1.0, 1.0, 1.2)), ("wall", (2.0, 1.0, 6.0, 1.2))]  # fmt: skip
    target = (-0.3, 3.0, 0.3, 3.5)
    mem, pose = Memory(), _pose(0, 0, 90)
    way = way_to_target(walls, target, pose)
    assert {o["side"] for o in way["open_sides"]} == {"left", "right"}
    assert mem.going_around(0.0, way, None) is None  # nothing picked yet
    turned_right = {"x": "none", "y": "none", "yaw": "turn_right"}
    assert mem.going_around(0.5, way, turned_right) == {"side": "right", "for_s": 0}
    assert mem.going_around(4.5, way, {"x": "none", "y": "none", "yaw": "none"})["for_s"] == 4
    # A flip of the pick mid-turn does not re-read the side; nor does a neighbouring box of the
    # same obstacle taking over as the blocker while the way past stays where it was.
    turned_left = {"x": "none", "y": "none", "yaw": "turn_left"}
    assert mem.going_around(4.6, {**way, "_blocker": "b"}, turned_left)["side"] == "right"
    moved = [{**o, "bearing_deg": o["bearing_deg"] + 120} for o in way["open_sides"]]
    jumped = {**way, "_blocker": "c", "open_sides": moved}
    other = Memory()
    other.going_around(0.0, dict(way), None)
    assert other.going_around(0.5, dict(way), turned_right)["side"] == "right"
    assert other.going_around(1.0, jumped, turned_right) is None  # elsewhere: a new choice
    only_left = {**way, "open_sides": [o for o in way["open_sides"] if o["side"] == "left"]}
    assert mem.going_around(5.0, only_left, turned_right) is None  # that side closed
    recent = [mem.recent(float(t), "go", _pose(0, 0, 90 * t), 3.0) for t in range(6)][-1]
    assert (recent["moved_m"], recent["turned_deg"], recent["pattern"]) == (
        0.0,
        450,
        "turning_on_the_spot",
    )


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
    assert obj["bearing_deg"] == 90 and obj["target"] is True and obj["width_m"] == 0.5
    assert "position" not in obj and "position" not in state["robot"]
    # No scan: free space all around still comes from the detections' footprints.
    assert state["free_space"]["left"] == {"clear_m": 1.8, "state": "clear", "by": "chair"}
    assert state["free_space"]["right"] == {"clear_m": 5.0, "state": "clear"}
    assert state["way_to_target"]["state"] == "clear"


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
    assert labels == ["chair"] + ["cabinet"] * 5 and state["goal"] == "go to the chair"


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
        ("chair", 39.75),
        ("table", 2.75),
    ]


def test_only_floor_level_obstacles_are_listed() -> None:
    def box(label: str, z: float, size: tuple[float, float, float]) -> Detection3D:
        d = _det3d(label, 1.0, 0.0).detections[0]
        d.bbox = BoundingBox3D(center=Pose(position=(1.0, 0.0, z)), size=Vector3(*size))
        return d

    dets = _det3d("chair", 3.0, 0.0)
    dets.detections += [
        box("ceiling_lamp", 2.9, (0.5, 0.5, 0.3)),
        box("bed upstairs", 3.5, (2.0, 1.5, 0.6)),
        box("carpet", 0.26, (2.0, 2.0, 0.02)),
        box("bottle", 0.5, (0.1, 0.1, 0.3)),
        box("table", 0.6, (1.0, 1.0, 0.7)),
        box("room floor", 0.6, (1.0, 1.0, 0.7)),
        box("floor_lamp", 0.9, (0.4, 0.4, 1.6)),
    ]
    dets.detections_length = len(dets.detections)
    state = build_world_state(
        "go to the chair",
        _pose(0, 0, 0),  # z = 0.4: the floor is at 0.25
        detections_3d=dets,
        detections_2d=None,
        lidar=None,
        robot={},
    )
    assert [o["label"] for o in state["objects"]] == ["chair", "table", "floor_lamp"]


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
    s = state["free_space"]
    assert s["ahead"] == {"clear_m": 0.3, "state": "blocked"}
    assert s["right"]["clear_m"] == 2.0
    assert s["left"] == {"state": "unseen"}  # no return at all: not looked at, never "clear"


def test_seen_empty_sector_is_clear() -> None:
    pts = np.array([[2.0, 0.0, 0.0]])  # only the floor ahead: seen, nothing in the body band
    state = build_world_state(
        "go to the chair",
        _pose(0, 0, 0),
        detections_3d=None,
        detections_2d=None,
        lidar=PointCloud2.from_numpy(pts, frame_id="base_link"),
        robot={},
        lidar_band=(0.1, 0.8, 5.0),
    )
    assert state["free_space"]["ahead"] == {"clear_m": 5.0, "state": "clear"}
