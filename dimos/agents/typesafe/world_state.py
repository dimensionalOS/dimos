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
from dimos.msgs.geometry import yaw
from dimos.msgs.pointcloud import pointcloud_xyz

if TYPE_CHECKING:
    from dimos_generated.geometry_msgs.msg import PoseStamped
    from dimos_generated.sensor_msgs.msg import PointCloud2
    from dimos_generated.vision_msgs.msg import Detection2DArray, Detection3DArray


class XyzJson(TypedDict):
    x: float
    y: float
    z: float


class PoseJson(TypedDict):
    position: XyzJson
    yaw_deg: float
    heading: str


class BBoxJson(TypedDict):
    cx: float
    cy: float
    w: float
    h: float


class SectorJson(TypedDict):
    clear_m: float
    state: str


def pose_json(pose: PoseStamped) -> PoseJson:
    position = pose.pose.position
    degrees = math.degrees(yaw(pose.pose.orientation))
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
    return {
        "position": {
            "x": round(position.x, 2),
            "y": round(position.y, 2),
            "z": round(position.z, 2),
        },
        "yaw_deg": round(degrees, 1),
        "heading": headings[round(degrees / 45.0) % 8],
    }


def room_sectors(
    cloud: PointCloud2, pose: PoseStamped, z_min: float, z_max: float, max_range: float
) -> dict[str, SectorJson]:
    position = pose.pose.position
    points = pointcloud_xyz(cloud) - np.array([position.x, position.y, position.z])
    c, s = np.cos(-yaw(pose.pose.orientation)), np.sin(-yaw(pose.pose.orientation))
    points = np.column_stack(
        (c * points[:, 0] - s * points[:, 1], s * points[:, 0] + c * points[:, 1], points[:, 2])
    )
    radii = np.hypot(points[:, 0], points[:, 1])
    keep = (points[:, 2] > z_min) & (points[:, 2] < z_max) & (radii > 0.05) & (radii < max_range)
    points, radii = points[keep], radii[keep]
    bins = np.round(
        np.arctan2(points[:, 1], points[:, 0]) / (2 * np.pi / len(SECTOR_NAMES))
    ).astype(int) % len(SECTOR_NAMES)
    out: dict[str, SectorJson] = {}
    for index, name in enumerate(SECTOR_NAMES):
        clear_m = float(radii[bins == index].min()) if (bins == index).any() else max_range
        out[name] = {
            "clear_m": round(clear_m, 2),
            "state": "blocked" if clear_m < 0.5 else "tight" if clear_m < 1.0 else "clear",
        }
    return out


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
    dist, rel = math.hypot(dx, dy), math.atan2(dy, dx) - yaw(pose.pose.orientation)
    deg = round(math.degrees(math.atan2(math.sin(rel), math.cos(rel))), 1)
    return bearing_word(rel), deg, distance_word(dist), round(dist, 2)


def _objects_3d(dets: Detection3DArray, pose: PoseStamped) -> list[ObjectState]:
    out: list[ObjectState] = []
    for detection in dets.detections:
        center = detection.bbox.center.position
        position: XyzJson = {
            "x": round(center.x, 2),
            "y": round(center.y, 2),
            "z": round(center.z, 2),
        }
        label = next(
            (
                result.hypothesis.class_id
                for result in detection.results
                if result.hypothesis.class_id
            ),
            "",
        )
        marker_id = detection.id.strip()
        label = label.strip()
        if marker_id:
            label = f"{label} id={marker_id}" if label else f"id={marker_id}"
        score = round(
            max((result.hypothesis.score for result in detection.results), default=0.0), 2
        )
        bearing, deg, distance, dist = relative(pose, position["x"], position["y"])
        out.append(
            {
                "label": label,
                "score": score,
                "position": position,
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
                result.hypothesis.class_id
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
    state: WorldState = {
        "goal": goal,
        "robot": {**pose_json(pose), "motion": motion},
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
        state["room"] = room_sectors(lidar, pose, z_min, z_max, max_range)
    return state
