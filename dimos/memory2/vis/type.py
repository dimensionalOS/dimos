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

"""Visualization types for the memory2 drawing language.

Each vis type wraps one or more dimos.msgs with rendering intent + style.
For example, Pose(posestamped) says "render this PoseStamped as a circle +
heading arrow", while Arrow(posestamped) says "render it as an arrow only."

SVG renderer collapses to 2D (top-down XY projection, Z ignored).
Rerun renderer can use the wrapped msgs' .to_rerun() methods directly.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING, Union

if TYPE_CHECKING:
    from dimos.msgs.geometry_msgs.Point import Point as GeoPoint
    from dimos.msgs.geometry_msgs.Pose import Pose as GeoPose
    from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
    from dimos.msgs.geometry_msgs.Vector3 import Vector3
    from dimos.msgs.nav_msgs.OccupancyGrid import OccupancyGrid
    from dimos.msgs.nav_msgs.Path import Path
    from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
    from dimos.msgs.sensor_msgs.Image import Image


@dataclass
class Pose:
    """Circle + heading arrow at a pose.

    Default vis type for PoseStamped.
    SVG: <circle> at .msg.x/.y + heading <line> from .msg.yaw
    Rerun: msg.to_rerun() (Transform3D) + msg.to_rerun_arrow()
    """

    msg: PoseStamped | GeoPose
    color: str = "#1abc9c"
    size: float = 0.3
    label: str | None = None


@dataclass
class Arrow:
    """Heading arrow only (no dot).

    SVG: <line> + <polygon> arrowhead from .msg.x/.y along .msg.yaw
    Rerun: msg.to_rerun_arrow()
    """

    msg: PoseStamped | GeoPose
    color: str = "#e67e22"
    length: float = 0.5


@dataclass
class Point:
    """Dot at a position.

    Default vis type for geometry_msgs.Point / PointStamped.
    SVG: <circle> + optional <text> label
    Rerun: rr.Points3D
    """

    msg: GeoPoint | GeoPose
    color: str = "#e74c3c"
    radius: float = 0.05
    label: str | None = None


@dataclass
class Box3D:
    """3D bounding box, rendered as rectangle in top-down view.

    Built from Detection3D.bbox or manually from center + size.
    SVG: <rect> centered at .center.x/.y with .size.x/.y
    Rerun: rr.Boxes3D
    """

    center: GeoPose
    size: Vector3
    color: str = "#f1c40f"
    label: str | None = None


@dataclass
class Camera:
    """Camera frustum at a pose, with optional image and intrinsics.

    SVG: FOV wedge at .pose.x/.y/.yaw (if camera_info), else dot + thumbnail
    Rerun: rr.Pinhole + rr.Transform3D + optional rr.Image
    """

    pose: PoseStamped
    image: Image | None = None
    camera_info: CameraInfo | None = None
    color: str = "#9b59b6"
    label: str | None = None


@dataclass
class Polyline:
    """Styled polyline wrapping a Path msg.

    SVG: <polyline> through .msg.poses[*].x/.y
    Rerun: rr.LineStrips3D
    """

    msg: Path
    color: str = "#3498db"
    width: float = 0.05


@dataclass
class Text:
    """Text annotation at a world position.

    SVG: <text>
    Rerun: rr.TextLog
    """

    position: tuple[float, float, float]
    text: str
    font_size: float = 12.0
    color: str = "#333333"


# Union of all types that can appear in a Drawing2D
SceneElement = Union[
    Pose,
    Arrow,
    Point,
    Box3D,
    Camera,
    Polyline,
    Text,
    "OccupancyGrid",  # pass-through, rendered as base map raster
    "PointCloud2",  # pass-through, rerun renders full 3D, SVG collapses to occupancy grid
    "EmbeddedObservation",  # pass-through, renderer decides presentation
]


# --- GraphTime element types ---


@dataclass
class Series:
    """Line connecting (t, y) points."""

    ts: list[float]
    values: list[float]
    color: str = "#3498db"
    width: float = 1.5
    label: str | None = None


@dataclass
class Markers:
    """Scatter dots at (t, y) points."""

    ts: list[float]
    values: list[float]
    color: str = "#e74c3c"
    radius: float = 0.5
    label: str | None = None


@dataclass
class HLine:
    """Horizontal reference line."""

    y: float
    color: str = "#888888"
    style: str = "dashed"
    label: str | None = None


GraphElement = Union[Series, Markers, HLine]
