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
from typing import TYPE_CHECKING, Any, Literal, Union

import numpy as np

from dimos.memory2.vis.color import Color, DeferredColor

ColorLike = Union[str, Color, DeferredColor]

PointShape = Literal["dot", "cross", "x", "square"]

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
    """Marker at a position.

    Default element for geometry_msgs.Point / PointStamped.
    SVG: <circle>/<line>×2/<rect> depending on ``shape`` + optional <text> label.
    Rerun: rr.Points3D (shape is collapsed to a dot — rerun doesn't expose
    per-marker glyphs).

    ``shape`` selects the marker glyph; ``halo`` adds a black underlay for
    legibility over busy raster backgrounds (uses the SVG ``paint-order``
    trick / the cv2 thick-black-then-thin-color double pass).
    """

    msg: GeoPoint | GeoPose
    color: ColorLike = "#e74c3c"
    radius: float = 0.05
    label: str | None = None
    opacity: float = 1.0
    shape: PointShape = "dot"
    halo: bool = False


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


@dataclass
class Polygon:
    """Closed shape in world coords with optional fill + stroke.

    Default for partition rendering (rooms, regions) where the agent needs
    semi-transparent fills with crisp outlines. ``fill_opacity=0.35`` matches
    the cv2 overlay alpha used in `dimos/memory2/experimental/memory2_agent/map_view.py`.

    SVG: <polygon> with fill/fill-opacity + stroke/stroke-width.
    Raster: cv2.fillPoly → cv2.addWeighted blend, then cv2.polylines outline.
    Rerun: rr.LineStrips3D of the closed boundary (fill not represented).

    Polygons with fewer than 3 vertices are skipped (no crash, no shape
    emitted), matching the existing renderer-tolerant pattern.
    """

    vertices: list[tuple[float, float]]
    fill: ColorLike | None = None
    stroke: ColorLike | None = None
    fill_opacity: float = 0.35
    stroke_width: float = 0.05
    label: str | None = None
    opacity: float = 1.0


@dataclass
class RasterOverlay:
    """World-frame RGBA bitmap, alpha-composited onto the canvas.

    Generalises the OccupancyGrid raster pattern so callers can drop in any
    pre-rendered mask (e.g. an "overclaim" highlight, a heatmap, a costmap).
    The overlay's lower-left corner is at ``origin``; each pixel covers
    ``resolution`` × ``resolution`` metres.

    SVG: <image href="data:image/png;base64,...">.
    Raster: per-pixel alpha blend against the destination region.
    Rerun: silently skipped (no first-class textured-plane archetype).
    """

    rgba: np.ndarray
    origin: tuple[float, float]
    resolution: float
    opacity: float = 1.0
    label: str | None = None


@dataclass
class Wedge:
    """Outlined viewing cone (FOV sector) at a world position.

    Drawn as a triangle: origin → left edge tip → right edge tip, plus an
    optional centerline. Used for camera frustum overlays; cleaner than
    overloading :class:`Camera` because most callers don't have a real
    ``CameraInfo``.

    SVG: <polygon fill="none" stroke=…> + optional centerline.
    Raster: 3× cv2.line (left, center, right).
    Rerun: rr.LineStrips3D.

    Wedges with non-positive ``fov`` or ``length`` are skipped.
    """

    origin: tuple[float, float]
    yaw: float
    fov: float
    length: float
    color: ColorLike = "#e67e22"
    stroke_width: float = 0.03
    label: str | None = None
    opacity: float = 1.0


SpaceElement = Union[
    Pose,
    Arrow,
    Point,
    Box3D,
    Camera,
    Polyline,
    Text,
    Polygon,
    RasterOverlay,
    Wedge,
    "OccupancyGrid",  # pass-through, rendered as base map raster
    "PointCloud2",  # pass-through, rerun renders full 3D, SVG collapses to occupancy grid
    "Observation[Any]",  # pass-through, renderer decides presentation (covers EmbeddedObservation)
]
