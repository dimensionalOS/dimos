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

"""SVG renderer for Drawing2D.

Top-down XY projection (Z ignored). World Y-up → SVG Y-down.
"""

from __future__ import annotations

import base64
from dataclasses import dataclass
import io
import math
from pathlib import Path
from typing import TYPE_CHECKING, Any

import numpy as np
from PIL import Image as PILImage

from dimos.memory2.type.observation import EmbeddedObservation
from dimos.memory2.vis.type import (
    Arrow,
    Box3D,
    Camera,
    Point,
    Polyline,
    Pose,
    SceneElement,
    Text,
)
from dimos.msgs.nav_msgs.OccupancyGrid import OccupancyGrid
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2

if TYPE_CHECKING:
    from dimos.memory2.vis.drawing2d.drawing2d import Drawing2D


@dataclass
class ViewTransform:
    """Maps world XY coordinates to SVG pixel coordinates with Y-flip."""

    wx_min: float
    wx_max: float
    wy_min: float
    wy_max: float
    svg_width: float
    svg_height: float

    @classmethod
    def from_elements(
        cls, elements: list[SceneElement], width_px: float = 800, padding: float = 0.5
    ) -> ViewTransform:
        xs: list[float] = []
        ys: list[float] = []
        for el in elements:
            _collect_bounds(el, xs, ys)

        if not xs or not ys:
            return cls(0, 1, 0, 1, width_px, width_px)

        xmin, xmax = min(xs) - padding, max(xs) + padding
        ymin, ymax = min(ys) - padding, max(ys) + padding

        world_w = xmax - xmin or 1.0
        world_h = ymax - ymin or 1.0
        aspect = world_h / world_w
        svg_h = width_px * aspect

        return cls(xmin, xmax, ymin, ymax, width_px, svg_h)

    def w2s(self, wx: float, wy: float) -> tuple[float, float]:
        """World (x, y) → SVG (sx, sy) with Y-flip."""
        sx = (wx - self.wx_min) / (self.wx_max - self.wx_min) * self.svg_width
        sy = (1 - (wy - self.wy_min) / (self.wy_max - self.wy_min)) * self.svg_height
        return sx, sy

    def scale(self, world_dist: float) -> float:
        """Convert a world-space distance to SVG pixels."""
        return world_dist / (self.wx_max - self.wx_min) * self.svg_width


def _collect_bounds(el: Any, xs: list[float], ys: list[float]) -> None:
    if isinstance(el, (Pose, Arrow)):
        xs.append(el.msg.x)
        ys.append(el.msg.y)
    elif isinstance(el, Point):
        xs.append(el.msg.x)
        ys.append(el.msg.y)
    elif isinstance(el, Polyline):
        for p in el.msg.poses:
            xs.append(p.x)
            ys.append(p.y)
    elif isinstance(el, Box3D):
        cx, cy = el.center.x, el.center.y
        hw, hh = el.size.x / 2, el.size.y / 2
        xs.extend([cx - hw, cx + hw])
        ys.extend([cy - hh, cy + hh])
    elif isinstance(el, Camera):
        xs.append(el.pose.x)
        ys.append(el.pose.y)
    elif isinstance(el, Text):
        xs.append(el.position[0])
        ys.append(el.position[1])
    elif isinstance(el, EmbeddedObservation):
        ps = el.pose_stamped
        xs.append(ps.x)
        ys.append(ps.y)
    elif isinstance(el, OccupancyGrid):
        ox, oy = el.origin.x, el.origin.y
        xs.extend([ox, ox + el.width * el.resolution])
        ys.extend([oy, oy + el.height * el.resolution])


def _render_point(el: Point, vt: ViewTransform) -> str:
    sx, sy = vt.w2s(el.msg.x, el.msg.y)
    r = max(vt.scale(el.radius), 2)
    parts = [f'<circle cx="{sx:.1f}" cy="{sy:.1f}" r="{r:.1f}" fill="{el.color}" opacity="0.85"/>']
    if el.label:
        parts.append(
            f'<text x="{sx + r + 2:.1f}" y="{sy + 4:.1f}" '
            f'font-size="11" fill="{el.color}">{_esc(el.label)}</text>'
        )
    return "\n".join(parts)


def _render_pose(el: Pose, vt: ViewTransform) -> str:
    sx, sy = vt.w2s(el.msg.x, el.msg.y)
    r = max(vt.scale(el.size * 0.3), 3)
    yaw = el.msg.yaw

    # Circle at position
    parts = [f'<circle cx="{sx:.1f}" cy="{sy:.1f}" r="{r:.1f}" fill="{el.color}" opacity="0.85"/>']

    # Heading arrow
    arrow_len = vt.scale(el.size)
    dx = math.cos(yaw) * arrow_len
    dy = -math.sin(yaw) * arrow_len  # Y-flip
    parts.append(
        f'<line x1="{sx:.1f}" y1="{sy:.1f}" x2="{sx + dx:.1f}" y2="{sy + dy:.1f}" '
        f'stroke="{el.color}" stroke-width="2" marker-end="url(#ah)"/>'
    )

    if el.label:
        parts.append(
            f'<text x="{sx + r + 2:.1f}" y="{sy + 4:.1f}" '
            f'font-size="11" fill="{el.color}">{_esc(el.label)}</text>'
        )
    return "\n".join(parts)


def _render_arrow(el: Arrow, vt: ViewTransform) -> str:
    sx, sy = vt.w2s(el.msg.x, el.msg.y)
    yaw = el.msg.yaw
    arrow_len = vt.scale(el.length)
    dx = math.cos(yaw) * arrow_len
    dy = -math.sin(yaw) * arrow_len
    return (
        f'<line x1="{sx:.1f}" y1="{sy:.1f}" x2="{sx + dx:.1f}" y2="{sy + dy:.1f}" '
        f'stroke="{el.color}" stroke-width="2.5" marker-end="url(#ah)"/>'
    )


def _render_polyline(el: Polyline, vt: ViewTransform) -> str:
    pts = " ".join(f"{vt.w2s(p.x, p.y)[0]:.1f},{vt.w2s(p.x, p.y)[1]:.1f}" for p in el.msg.poses)
    sw = max(vt.scale(el.width), 1)
    return (
        f'<polyline points="{pts}" fill="none" '
        f'stroke="{el.color}" stroke-width="{sw:.1f}" stroke-linejoin="round"/>'
    )


def _render_box3d(el: Box3D, vt: ViewTransform) -> str:
    cx, cy = el.center.x, el.center.y
    hw, hh = el.size.x / 2, el.size.y / 2
    sx1, sy1 = vt.w2s(cx - hw, cy + hh)  # top-left in world → SVG
    sx2, sy2 = vt.w2s(cx + hw, cy - hh)  # bottom-right in world → SVG
    w = abs(sx2 - sx1)
    h = abs(sy2 - sy1)
    x = min(sx1, sx2)
    y = min(sy1, sy2)
    parts = [
        f'<rect x="{x:.1f}" y="{y:.1f}" width="{w:.1f}" height="{h:.1f}" '
        f'fill="none" stroke="{el.color}" stroke-width="2"/>'
    ]
    if el.label:
        parts.append(
            f'<text x="{x + 3:.1f}" y="{y - 3:.1f}" '
            f'font-size="11" fill="{el.color}">{_esc(el.label)}</text>'
        )
    return "\n".join(parts)


def _render_camera(el: Camera, vt: ViewTransform) -> str:
    sx, sy = vt.w2s(el.pose.x, el.pose.y)
    yaw = el.pose.yaw

    if el.camera_info and el.camera_info.K[4] > 0:
        # FOV wedge
        fy = el.camera_info.K[4]
        fov_y = 2 * math.atan(el.camera_info.height / (2 * fy))
        fov_half = fov_y / 2
        wedge_len = vt.scale(0.8)

        a1 = yaw + fov_half
        a2 = yaw - fov_half
        x1 = sx + math.cos(a1) * wedge_len
        y1 = sy - math.sin(a1) * wedge_len
        x2 = sx + math.cos(a2) * wedge_len
        y2 = sy - math.sin(a2) * wedge_len

        parts = [
            f'<polygon points="{sx:.1f},{sy:.1f} {x1:.1f},{y1:.1f} {x2:.1f},{y2:.1f}" '
            f'fill="{el.color}" fill-opacity="0.2" stroke="{el.color}" stroke-width="1.5"/>'
        ]
    else:
        # No intrinsics: just a dot
        parts = [f'<circle cx="{sx:.1f}" cy="{sy:.1f}" r="5" fill="{el.color}" opacity="0.8"/>']

    if el.label:
        parts.append(
            f'<text x="{sx + 6:.1f}" y="{sy + 4:.1f}" '
            f'font-size="11" fill="{el.color}">{_esc(el.label)}</text>'
        )
    return "\n".join(parts)


def _render_text(el: Text, vt: ViewTransform) -> str:
    sx, sy = vt.w2s(el.position[0], el.position[1])
    return (
        f'<text x="{sx:.1f}" y="{sy:.1f}" '
        f'font-size="{el.font_size}" fill="{el.color}">{_esc(el.text)}</text>'
    )


def _render_occupancy_grid(el: OccupancyGrid, vt: ViewTransform) -> str:
    if el.grid.size == 0:
        return ""

    rgba = np.flipud(el._generate_rgba_texture())
    img = PILImage.fromarray(rgba, "RGBA")
    buf = io.BytesIO()
    img.save(buf, format="PNG")
    b64 = base64.b64encode(buf.getvalue()).decode("ascii")

    ox, oy = el.origin.x, el.origin.y
    world_w = el.width * el.resolution
    world_h = el.height * el.resolution

    sx, sy = vt.w2s(ox, oy + world_h)  # top-left in world → SVG
    sw = vt.scale(world_w)
    sh = vt.scale(world_h)

    return (
        f'<image x="{sx:.1f}" y="{sy:.1f}" width="{sw:.1f}" height="{sh:.1f}" '
        f'href="data:image/png;base64,{b64}" image-rendering="pixelated"/>'
    )


_ARROWHEAD_MARKER = (
    '<defs><marker id="ah" markerWidth="8" markerHeight="6" '
    'refX="8" refY="3" orient="auto"><polygon points="0 0, 8 3, 0 6" '
    'fill="context-stroke"/></marker></defs>'
)


def render(
    drawing: Drawing2D,
    path: str | Path | None = None,
    width_px: float = 800,
    padding: float = 0.5,
) -> str:
    """Render a Drawing2D to an SVG string, optionally writing to *path*."""
    elements = drawing.elements
    vt = ViewTransform.from_elements(elements, width_px=width_px, padding=padding)

    parts: list[str] = [
        f'<svg xmlns="http://www.w3.org/2000/svg" '
        f'width="{vt.svg_width:.0f}" height="{vt.svg_height:.0f}" '
        f'viewBox="0 0 {vt.svg_width:.0f} {vt.svg_height:.0f}" '
        f'style="background:#f8f8f8">',
        _ARROWHEAD_MARKER,
    ]

    # Render in insertion order (z-index = add order)
    for el in elements:
        parts.append(_render_element(el, vt))

    parts.append("</svg>")
    svg = "\n".join(parts)

    if path is not None:
        Path(path).write_text(svg)

    return svg


def _render_element(el: SceneElement, vt: ViewTransform) -> str:
    if isinstance(el, Point):
        return _render_point(el, vt)
    elif isinstance(el, Pose):
        return _render_pose(el, vt)
    elif isinstance(el, Arrow):
        return _render_arrow(el, vt)
    elif isinstance(el, Polyline):
        return _render_polyline(el, vt)
    elif isinstance(el, Box3D):
        return _render_box3d(el, vt)
    elif isinstance(el, Camera):
        return _render_camera(el, vt)
    elif isinstance(el, Text):
        return _render_text(el, vt)
    elif isinstance(el, OccupancyGrid):
        return _render_occupancy_grid(el, vt)
    elif isinstance(el, PointCloud2):
        from dimos.mapping.pointclouds.occupancy import general_occupancy

        return _render_occupancy_grid(general_occupancy(el), vt)
    elif isinstance(el, EmbeddedObservation):
        return _render_arrow(Arrow(msg=el.pose_stamped), vt)
    else:
        return f"<!-- unsupported: {type(el).__name__} -->"


def _esc(s: str) -> str:
    """Escape text for SVG XML."""
    return s.replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;")
