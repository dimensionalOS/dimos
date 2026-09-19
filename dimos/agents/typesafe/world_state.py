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
"""Fold the latest perception messages into the JSON state the model reads."""

from __future__ import annotations

import math
import re
from typing import TYPE_CHECKING, TypedDict

from typing_extensions import NotRequired

from dimos.msgs.geometry_msgs.PoseStamped import XyzJson
from dimos.msgs.sensor_msgs.PointCloud2 import SECTOR_NAMES, SectorJson
from dimos.msgs.vision_msgs.Detection2DArray import BBoxJson

if TYPE_CHECKING:
    from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
    from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
    from dimos.msgs.vision_msgs.Detection2DArray import Detection2DArray
    from dimos.msgs.vision_msgs.Detection3DArray import Detection3DArray

MAX_OBJECTS = 20
_BEARINGS_2D = ("far_left", "left", "center", "right", "far_right")
_SIZES_2D = ((0.4, "filling_view"), (0.15, "large"), (0.03, "medium"), (0.0, "small"))
_GOAL_XY = re.compile(r"\(\s*(-?\d+(?:\.\d+)?)\s*,\s*(-?\d+(?:\.\d+)?)\s*\)")
_DISTANCES = ((0.5, "touching"), (1.5, "near"), (4.0, "mid"), (math.inf, "far"))


class ObjectState(TypedDict):
    label: str
    score: float
    bearing: str
    distance: NotRequired[str]  # 3D only
    distance_m: NotRequired[float]
    bearing_deg: NotRequired[float]
    position: NotRequired[XyzJson]
    size: NotRequired[str]  # 2D only
    bbox: NotRequired[BBoxJson]


class RobotState(TypedDict, total=False):
    motion: str
    last_drive: dict[str, str]
    ts: float
    frame_id: str
    position: XyzJson
    yaw_deg: float
    heading: str


class WorldState(TypedDict):
    task: NotRequired[str]
    goal: str
    robot: RobotState
    objects: list[ObjectState]
    room: NotRequired[dict[str, dict[str, SectorJson]]]
    unavailable: NotRequired[list[str]]


def bearing_word(rel_angle: float) -> str:
    """8-way word for an angle in the robot frame (0 = ahead, +pi/2 = left)."""
    return SECTOR_NAMES[round(rel_angle / (math.pi / 4)) % 8]


def distance_word(d: float) -> str:
    return next(word for limit, word in _DISTANCES if d < limit)


def _objects_3d(dets: Detection3DArray, pose: PoseStamped, goal: str) -> list[ObjectState]:
    out: list[ObjectState] = []
    for d in dets.to_json():
        dx, dy = d["position"]["x"] - pose.x, d["position"]["y"] - pose.y
        # To the box's nearest edge: the centre of a large object is never reachable.
        dist = math.hypot(
            max(0.0, abs(dx) - d["size"]["x"] / 2), max(0.0, abs(dy) - d["size"]["y"] / 2)
        )
        rel = math.atan2(dy, dx) - pose.yaw
        out.append(
            {
                "label": d["label"],
                "score": d["score"],
                "position": d["position"],
                "bearing": bearing_word(rel),
                "bearing_deg": round(math.degrees(math.atan2(math.sin(rel), math.cos(rel))), 1),
                "distance": distance_word(dist),
                "distance_m": round(dist, 2),
            }
        )
    out.sort(key=lambda o: o["distance_m"])
    # The nearest ones, but whatever the goal names is always listed, however far. A goal
    # with coordinates ("the chair at (x, y)") names one object: the same-label others go.
    named = [o for o in out if o["label"] and o["label"].lower() in goal.lower()]
    at = _GOAL_XY.search(goal)
    if at and named:
        gx, gy = float(at.group(1)), float(at.group(2))
        best = min(
            named, key=lambda o: math.hypot(o["position"]["x"] - gx, o["position"]["y"] - gy)
        )
        out = [o for o in out if o is best or o not in named]
        named = [best]
    rest = [o for o in out if o not in named]
    keep = named[:MAX_OBJECTS] + rest[: MAX_OBJECTS - min(len(named), MAX_OBJECTS)]
    return sorted(keep, key=lambda o: o["distance_m"])


def _objects_2d(dets: Detection2DArray, image_size: tuple[int, int]) -> list[ObjectState]:
    w, h = image_size
    out: list[ObjectState] = []
    for d in dets.to_json():
        area = d["bbox"]["w"] * d["bbox"]["h"] / (w * h)
        out.append(
            {
                "label": d["label"],
                "score": d["score"],
                "bbox": d["bbox"],
                "bearing": _BEARINGS_2D[min(4, int(5 * d["bbox"]["cx"] / w))],
                "size": next(word for limit, word in _SIZES_2D if area > limit),
            }
        )
    return sorted(out, key=lambda o: -o["bbox"]["w"] * o["bbox"]["h"])[:MAX_OBJECTS]


def build_world_state(
    goal: str,
    pose: PoseStamped,
    *,
    task: str | None = None,
    detections_3d: Detection3DArray | None,
    detections_2d: Detection2DArray | None,
    lidar: PointCloud2 | None,
    robot: RobotState,
    image_size: tuple[int, int] = (1280, 720),
    lidar_band: tuple[float, float, float] = (-0.2, 0.8, 5.0),
) -> WorldState:
    if detections_3d is not None:
        objects = _objects_3d(detections_3d, pose, goal)
    elif detections_2d is not None:
        objects = _objects_2d(detections_2d, image_size)
    else:
        objects = []
    robot_state: RobotState = {**robot, **pose.to_json()}
    state: WorldState = {"goal": goal, "robot": robot_state, "objects": objects}
    if task:
        state = {"task": task, **state}
    if lidar is not None:
        z_min, z_max, max_range = lidar_band
        # A scan in the robot's own frame (habitat `lidar`, base_link) needs no transform.
        origin = pose if lidar.frame_id == pose.frame_id else None
        state["room"] = {
            "sectors": lidar.to_json(origin, z_min=z_min, z_max=z_max, max_range=max_range)
        }
    else:
        state["unavailable"] = ["room"]
    return state
