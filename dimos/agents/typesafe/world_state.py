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

import numpy as np
from typing_extensions import NotRequired

from dimos.agents.typesafe.constants import (
    BEARING_WORDS_2D,
    DISTANCE_WORDS,
    MAX_OBJECTS,
    SECTOR_NAMES,
    SIZE_WORDS_2D,
)
from dimos.message_utils import geometry, pointcloud

if TYPE_CHECKING:
    from dimos.messages.geometry_msgs.msg import PoseStamped
    from dimos.messages.sensor_msgs.msg import PointCloud2
    from dimos.messages.vision_msgs.msg import Detection2DArray, Detection3DArray


class XyzJson(TypedDict):
    x: float
    y: float
    z: float


class PoseJson(TypedDict):
    position: XyzJson
    yaw_deg: float
    heading: str


class SectorJson(TypedDict):
    clear_m: float
    state: str


class BBoxJson(TypedDict):
    cx: float
    cy: float
    w: float
    h: float


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
    dx, dy = x - pose.pose.position.x, y - pose.pose.position.y
    dist, rel = math.hypot(dx, dy), math.atan2(dy, dx) - geometry.yaw(pose.pose.orientation)
    deg = round(math.degrees(math.atan2(math.sin(rel), math.cos(rel))), 1)
    return bearing_word(rel), deg, distance_word(dist), round(dist, 2)


def _objects_3d(dets: Detection3DArray, pose: PoseStamped) -> list[ObjectState]:
    out: list[ObjectState] = []
    for detection in dets.detections:
        center = detection.bbox.center.position
        marker_id = str(detection.id).strip()
        class_id = next(
            (
                str(result.hypothesis.class_id).strip()
                for result in detection.results
                if str(result.hypothesis.class_id).strip()
            ),
            "",
        )
        label = (
            f"{class_id} id={marker_id}"
            if marker_id and class_id
            else class_id
            if class_id
            else f"id={marker_id}"
            if marker_id
            else ""
        )
        d: ObjectState = {
            "label": label,
            "score": round(
                max((result.hypothesis.score for result in detection.results), default=0.0), 2
            ),
            "position": {"x": round(center.x, 2), "y": round(center.y, 2), "z": round(center.z, 2)},
            "bearing": "",
        }
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
    for detection in dets.detections:
        center = detection.bbox.center.position
        b: BBoxJson = {
            "cx": round(center.x, 1),
            "cy": round(center.y, 1),
            "w": round(detection.bbox.size_x, 1),
            "h": round(detection.bbox.size_y, 1),
        }
        label = next(
            (
                str(result.hypothesis.class_id)
                for result in detection.results
                if result.hypothesis.class_id
            ),
            "",
        )
        score = round(
            max((result.hypothesis.score for result in detection.results), default=0.0), 2
        )
        area = b["w"] * b["h"] / (w * h)
        out.append(
            {
                "label": label,
                "score": score,
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
    yaw_degrees = math.degrees(geometry.yaw(pose.pose.orientation))
    headings = (
        "east",
        "north_east",
        "north",
        "north_west",
        "west",
        "south_west",
        "south",
        "south_east",
    )
    position = pose.pose.position
    state: WorldState = {
        "goal": goal,
        "robot": {
            "position": {
                "x": round(position.x, 2),
                "y": round(position.y, 2),
                "z": round(position.z, 2),
            },
            "yaw_deg": round(yaw_degrees, 1),
            "heading": headings[round(yaw_degrees / 45.0) % 8],
            "motion": motion,
        },
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
        pts = pointcloud.xyz(lidar) - geometry.vector_array(pose.pose.position)
        c, sine = (
            np.cos(-geometry.yaw(pose.pose.orientation)),
            np.sin(-geometry.yaw(pose.pose.orientation)),
        )
        pts = np.column_stack(
            (c * pts[:, 0] - sine * pts[:, 1], sine * pts[:, 0] + c * pts[:, 1], pts[:, 2])
        )
        radius = np.hypot(pts[:, 0], pts[:, 1])
        keep = (pts[:, 2] > z_min) & (pts[:, 2] < z_max) & (radius > 0.05) & (radius < max_range)
        pts, radius = pts[keep], radius[keep]
        sectors = np.round(
            np.arctan2(pts[:, 1], pts[:, 0]) / (2 * np.pi / len(SECTOR_NAMES))
        ).astype(int) % len(SECTOR_NAMES)
        room: dict[str, SectorJson] = {}
        for index, name in enumerate(SECTOR_NAMES):
            clearance = (
                float(radius[sectors == index].min()) if (sectors == index).any() else max_range
            )
            room[name] = {
                "clear_m": round(clearance, 2),
                "state": "blocked" if clearance < 0.5 else "tight" if clearance < 1.0 else "clear",
            }
        state["room"] = room
    return state
