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

"""Element types for the Space drawing language.

Each element wraps one or more dimos.msgs with rendering intent + style.
For example, Pose(posestamped) says "render this PoseStamped as a circle +
heading arrow", while Arrow(posestamped) says "render it as an arrow only."

SVG renderer collapses to 2D (top-down XY projection, Z ignored).
Rerun renderer can use the wrapped msgs' .to_rerun() methods directly.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING, Any, Union

from dimos.memory2.vis.color import Color, DeferredColor

ColorLike = Union[str, Color, DeferredColor]

if TYPE_CHECKING:
    from dimos.memory2.type.observation import Observation
    from dimos.msgs.geometry_msgs.Point import Point as GeoPoint
    from dimos.msgs.geometry_msgs.Pose import Pose as GeoPose
    from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
    from dimos.msgs.geometry_msgs.Vector3 import Vector3
    from dimos.msgs.nav_msgs.OccupancyGrid import OccupancyGrid
    from dimos.msgs.nav_msgs.Path import Path
    from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
    from dimos.msgs.sensor_msgs.Image import Image
    from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2


@dataclass
class Pose:
    """Circle + heading arrow at a pose.

    Default element for PoseStamped.
    SVG: <circle> at .msg.x/.y + heading <line> from .msg.yaw
    Rerun: msg.to_rerun() (Transform3D) + msg.to_rerun_arrow()
    """

    msg: PoseStamped | GeoPose
    color: ColorLike = "#1abc9c"
    size: float = 0.3
    label: str | None = None
    opacity: float = 1.0


@dataclass
class Arrow:
    """Heading arrow only (no dot).

    SVG: <line> + <polygon> arrowhead from .msg.x/.y along .msg.yaw
    Rerun: msg.to_rerun_arrow()
    """

    msg: PoseStamped | GeoPose
    color: ColorLike = "#e67e22"
    length: float = 0.5
    opacity: float = 1.0


@dataclass
class Point:
    """Dot at a position.

    Default element for geometry_msgs.Point / PointStamped.
    SVG: <circle> + optional <text> label
    Rerun: rr.Points3D
    """

    msg: GeoPoint | GeoPose
    color: ColorLike = "#e74c3c"
    radius: float = 0.05
    label: str | None = None
    opacity: float = 1.0


@dataclass
class Box3D:
    """3D bounding box, rendered as rectangle in top-down view.

    Built from Detection3D.bbox or manually from center + size.
    SVG: <rect> centered at .center.x/.y with .size.x/.y
    Rerun: rr.Boxes3D
    """

    center: GeoPose
    size: Vector3
    color: ColorLike = "#f1c40f"
    label: str | None = None
    opacity: float = 1.0


@dataclass
class Camera:
    """Camera frustum at a pose, with optional image and intrinsics.

    SVG: FOV wedge at .pose.x/.y/.yaw (if camera_info), else dot + thumbnail
    Rerun: rr.Pinhole + rr.Transform3D + optional rr.Image
    """

    pose: PoseStamped
    image: Image | None = None
    camera_info: CameraInfo | None = None
    color: ColorLike = "#9b59b6"
    label: str | None = None
    opacity: float = 1.0


@dataclass
class Polyline:
    """Styled polyline wrapping a Path msg.

    SVG: <polyline> through .msg.poses[*].x/.y
    Rerun: rr.LineStrips3D
    """

    msg: Path
    color: ColorLike = "#3498db"
    width: float = 0.05
    opacity: float = 1.0


@dataclass
class Text:
    """Text annotation at a world position.

    SVG: <text>
    Rerun: rr.TextLog
    """

    position: tuple[float, float, float]
    text: str
    font_size: float = 12.0
    color: ColorLike = "#333333"
    opacity: float = 1.0


SpaceElement = Union[
    Pose,
    Arrow,
    Point,
    Box3D,
    Camera,
    Polyline,
    Text,
    "OccupancyGrid",  # pass-through, rendered as base map raster
    "PointCloud2",  # pass-through, rerun renders full 3D, SVG collapses to occupancy grid
    "Observation[Any]",  # pass-through, renderer decides presentation (covers EmbeddedObservation)
]
