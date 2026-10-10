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
"""The JSON state the navigation questions read: goal, pose, objects with word buckets, room sectors."""

from __future__ import annotations

import math
from typing import TYPE_CHECKING, TypedDict

from typing_extensions import NotRequired

from dimos.agents.typesafe.constants import (
    BEARING_WORDS_2D,
    DISTANCE_WORDS,
    MAX_OBJECTS,
    SECTOR_NAMES,
    SIZE_WORDS_2D,
)
from dimos.msgs.geometry_msgs.PoseStamped import PoseJson, XyzJson
from dimos.msgs.sensor_msgs.PointCloud2 import SectorJson
from dimos.msgs.vision_msgs.Detection2DArray import BBoxJson

if TYPE_CHECKING:
    from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
    from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
    from dimos.msgs.vision_msgs.Detection2DArray import Detection2DArray
    from dimos.msgs.vision_msgs.Detection3DArray import Detection3DArray


class ObjectState(TypedDict):
    label: str
    score: float
    bearing: str
    position: NotRequired[XyzJson]  # 3D
    bearing_deg: NotRequired[float]
    distance: NotRequired[str]
    distance_m: NotRequired[float]
    bbox: NotRequired[BBoxJson]  # 2D
    size: NotRequired[str]


class RobotState(PoseJson):
    motion: str


class GoalPoint(TypedDict):
    """World-frame XY the goal resolved to, with the words objects carry."""

    x: float
    y: float
    bearing: str
    bearing_deg: float
    distance: str
    distance_m: float


class WorldState(TypedDict):
    goal: str
    robot: RobotState
    objects: list[ObjectState]
    goal_point: NotRequired[GoalPoint]
    room: NotRequired[dict[str, SectorJson]]


def bearing_word(rel: float) -> str:
    """8-way word for an angle in the robot frame (0 ahead, +pi/2 left)."""
    return SECTOR_NAMES[round(rel / (math.pi / 4)) % 8]


def distance_word(d: float) -> str:
    return next(word for limit, word in DISTANCE_WORDS if d < limit)


def relative(pose: PoseStamped, x: float, y: float) -> tuple[str, float, str, float]:
    """(bearing word, bearing deg, distance word, distance m) of a world XY from the pose."""
    dx, dy = x - pose.x, y - pose.y
    dist, rel = math.hypot(dx, dy), math.atan2(dy, dx) - pose.yaw
    deg = round(math.degrees(math.atan2(math.sin(rel), math.cos(rel))), 1)
    return bearing_word(rel), deg, distance_word(dist), round(dist, 2)


def _objects_3d(dets: Detection3DArray, pose: PoseStamped) -> list[ObjectState]:
    out: list[ObjectState] = []
    for d in dets.to_json():
        bearing, deg, distance, dist = relative(pose, d["position"]["x"], d["position"]["y"])
        out.append(
            {
                "label": d["label"],
                "score": d["score"],
                "position": d["position"],
                "bearing": bearing,
                "bearing_deg": deg,
                "distance": distance,
                "distance_m": dist,
            }
        )
    return sorted(out, key=lambda o: o["distance_m"])[:MAX_OBJECTS]


def _objects_2d(dets: Detection2DArray, image_size: tuple[int, int]) -> list[ObjectState]:
    w, h = image_size
    out: list[ObjectState] = []
    for d in dets.to_json():
        b = d["bbox"]
        area = b["w"] * b["h"] / (w * h)
        out.append(
            {
                "label": d["label"],
                "score": d["score"],
                "bbox": b,
                "bearing": BEARING_WORDS_2D[min(4, int(5 * b["cx"] / w))],
                "size": next(word for limit, word in SIZE_WORDS_2D if area > limit),
            }
        )
    return sorted(out, key=lambda o: -o["bbox"]["w"] * o["bbox"]["h"])[:MAX_OBJECTS]


def build_world_state(
    goal: str,
    pose: PoseStamped,
    motion: str,
    *,
    detections_3d: Detection3DArray | None,
    detections_2d: Detection2DArray | None,
    lidar: PointCloud2 | None,
    goal_xy: tuple[float, float] | None,
    image_size: tuple[int, int],
    lidar_band: tuple[float, float, float],
) -> WorldState:
    if detections_3d is not None:
        objects = _objects_3d(detections_3d, pose)
    elif detections_2d is not None:
        objects = _objects_2d(detections_2d, image_size)
    else:
        objects = []
    state: WorldState = {
        "goal": goal,
        "robot": {**pose.to_json(), "motion": motion},
        "objects": objects,
    }
    if goal_xy is not None:
        bearing, deg, distance, dist = relative(pose, *goal_xy)
        state["goal_point"] = {
            "x": round(goal_xy[0], 2),
            "y": round(goal_xy[1], 2),
            "bearing": bearing,
            "bearing_deg": deg,
            "distance": distance,
            "distance_m": dist,
        }
    if lidar is not None:
        z_min, z_max, max_range = lidar_band
        state["room"] = lidar.to_json(
            pose, sectors=SECTOR_NAMES, z_min=z_min, z_max=z_max, max_range=max_range
        )
    return state
