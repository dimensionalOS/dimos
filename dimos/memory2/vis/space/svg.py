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

"""SVG renderer for Space.

Top-down XY projection (Z ignored). Renders in world coordinates with Y-flip.
The SVG viewBox is computed from actual rendered content, so all element types
automatically contribute to the viewport bounds.
"""

from __future__ import annotations

import base64
from dataclasses import dataclass
import io
import math
from pathlib import Path
from typing import TYPE_CHECKING

import numpy as np
from PIL import Image as PILImage

from dimos.mapping.occupancy.visualizations import generate_rgba_texture
from dimos.memory2.type.observation import Observation
from dimos.memory2.vis.color import Color
from dimos.memory2.vis.space.elements import (
    Arrow,
    Box3D,
    Camera,
    Point,
    Polyline,
    Pose,
    SpaceElement,
    Text,
)
from dimos.msgs.nav_msgs.OccupancyGrid import OccupancyGrid
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2

if TYPE_CHECKING:
    from dimos.memory2.vis.space.space import Space


@dataclass
class Bounds:
    """Accumulates world-space bounding box during rendering."""

    xmin: float = float("inf")
    xmax: float = float("-inf")
    ymin: float = float("inf")
    ymax: float = float("-inf")

    def include(self, x: float, y: float) -> None:
        self.xmin = min(self.xmin, x)
        self.xmax = max(self.xmax, x)
        self.ymin = min(self.ymin, y)
        self.ymax = max(self.ymax, y)

    @property
    def empty(self) -> bool:
        return self.xmin > self.xmax

    @property
    def width(self) -> float:
        return max(self.xmax - self.xmin, 1.0)

    @property
    def height(self) -> float:
        return max(self.ymax - self.ymin, 1.0)


def _y(wy: float) -> float:
    """Flip Y axis: world Y-up → SVG Y-down."""
    return -wy


def _style(el: object) -> tuple[str, float]:
    """Return (hex, combined-alpha) from an element's color and opacity."""
    c = Color.coerce(getattr(el, "color", "#000000"))
    opacity = float(getattr(el, "opacity", 1.0))
    return c.hex(), c.a * opacity


# Element renderers — all emit world-coordinate SVG and grow Bounds


def _render_point(el: Point, b: Bounds) -> str:
    x, y = el.msg.x, _y(el.msg.y)
    r = el.radius
    b.include(x - r, y - r)
    b.include(x + r, y + r)
    fill, alpha = _style(el)
    parts = [f'<circle cx="{x:.4f}" cy="{y:.4f}" r="{r:.4f}" fill="{fill}" opacity="{alpha:.3f}"/>']
    if el.label:
        parts.append(
            f'<text x="{x + r:.4f}" y="{y:.4f}" '
            f'font-size="{r * 1.5:.4f}" fill="{fill}" opacity="{alpha:.3f}">{_esc(el.label)}</text>'
        )
    return "\n".join(parts)


def _render_arrow(el: Arrow, b: Bounds) -> str:
    x, y = el.msg.x, _y(el.msg.y)
    yaw = el.msg.yaw
    length = el.length
    half_base = length * 0.4

    # Tip of the triangle
    tx = x + math.cos(yaw) * length
    ty = y - math.sin(yaw) * length  # sin negated for Y-flip
    # Two base corners (perpendicular to yaw)
    bx1 = x + math.cos(yaw + math.pi / 2) * half_base
    by1 = y - math.sin(yaw + math.pi / 2) * half_base
    bx2 = x + math.cos(yaw - math.pi / 2) * half_base
    by2 = y - math.sin(yaw - math.pi / 2) * half_base

    for px, py in [(x, y), (tx, ty), (bx1, by1), (bx2, by2)]:
        b.include(px, py)

    stroke, alpha = _style(el)
    return (
        f'<polygon points="{tx:.4f},{ty:.4f} {bx1:.4f},{by1:.4f} {bx2:.4f},{by2:.4f}" '
        f'fill="none" stroke="{stroke}" opacity="{alpha:.3f}" '
        f'stroke-width="{length * 0.08:.4f}" stroke-linejoin="round"/>'
    )


def _render_pose(el: Pose, b: Bounds) -> str:
    arrow = Arrow(msg=el.msg, length=el.size, color=el.color, opacity=el.opacity)
    parts = [_render_arrow(arrow, b)]
    if el.label:
        x, y = el.msg.x, _y(el.msg.y)
        fill, alpha = _style(el)
        parts.append(
            f'<text x="{x + el.size * 0.5:.4f}" y="{y:.4f}" '
            f'font-size="{el.size * 0.8:.4f}" fill="{fill}" opacity="{alpha:.3f}">{_esc(el.label)}</text>'
        )
    return "\n".join(parts)


def _render_polyline(el: Polyline, b: Bounds) -> str:
    pts = []
    for p in el.msg.poses:
        x, y = p.x, _y(p.y)
        b.include(x, y)
        pts.append(f"{x:.4f},{y:.4f}")
    stroke, alpha = _style(el)
    return (
        f'<polyline points="{" ".join(pts)}" fill="none" '
        f'stroke="{stroke}" opacity="{alpha:.3f}" '
        f'stroke-width="{el.width:.4f}" stroke-linejoin="round"/>'
    )


def _render_box3d(el: Box3D, b: Bounds) -> str:
    cx, cy = el.center.x, el.center.y
    hw, hh = el.size.x / 2, el.size.y / 2
    # Top-left in world → SVG
    x = cx - hw
    y = _y(cy + hh)
    w = el.size.x
    h = el.size.y
    b.include(x, y)
    b.include(x + w, y + h)
    stroke, alpha = _style(el)
    parts = [
        f'<rect x="{x:.4f}" y="{y:.4f}" width="{w:.4f}" height="{h:.4f}" '
        f'fill="none" stroke="{stroke}" opacity="{alpha:.3f}" '
        f'stroke-width="{min(w, h) * 0.04:.4f}"/>'
    ]
    if el.label:
        font_size = max(h * 0.3, 0.2)
        parts.append(
            f'<text x="{x:.4f}" y="{y - h * 0.05:.4f}" '
            f'font-size="{font_size:.4f}" fill="{stroke}" opacity="{alpha:.3f}">{_esc(el.label)}</text>'
        )
    return "\n".join(parts)


def _render_camera(el: Camera, b: Bounds) -> str:
    x, y = el.pose.x, _y(el.pose.y)
    yaw = el.pose.yaw
    stroke, alpha = _style(el)

    if el.camera_info and el.camera_info.K[4] > 0:
        fy = el.camera_info.K[4]
        fov_y = 2 * math.atan(el.camera_info.height / (2 * fy))
        fov_half = fov_y / 2
        wedge_len = 0.8

        a1 = yaw + fov_half
        a2 = yaw - fov_half
        x1 = x + math.cos(a1) * wedge_len
        y1 = y - math.sin(a1) * wedge_len
        x2 = x + math.cos(a2) * wedge_len
        y2 = y - math.sin(a2) * wedge_len

        for px, py in [(x, y), (x1, y1), (x2, y2)]:
            b.include(px, py)

        parts = [
            f'<polygon points="{x:.4f},{y:.4f} {x1:.4f},{y1:.4f} {x2:.4f},{y2:.4f}" '
            f'fill="none" stroke="{stroke}" opacity="{alpha:.3f}" stroke-width="0.03"/>'
        ]
    else:
        r = 0.15
        b.include(x - r, y - r)
        b.include(x + r, y + r)
        parts = [
            f'<circle cx="{x:.4f}" cy="{y:.4f}" r="{r:.4f}" fill="{stroke}" opacity="{alpha:.3f}"/>'
        ]

    if el.label:
        parts.append(
            f'<text x="{x + 0.2:.4f}" y="{y:.4f}" '
            f'font-size="0.3" fill="{stroke}" opacity="{alpha:.3f}">{_esc(el.label)}</text>'
        )
    return "\n".join(parts)


def _render_text(el: Text, b: Bounds) -> str:
    x, y = el.position[0], _y(el.position[1])
    b.include(x, y)
    fill, alpha = _style(el)
    return (
        f'<text x="{x:.4f}" y="{y:.4f}" '
        f'font-size="{el.font_size:.4f}" fill="{fill}" opacity="{alpha:.3f}">{_esc(el.text)}</text>'
    )


def _render_occupancy_grid(el: OccupancyGrid, b: Bounds) -> str:
    if el.grid.size == 0:
        return ""

    rgba = np.flipud(generate_rgba_texture(el))
    img = PILImage.fromarray(rgba, "RGBA")
    buf = io.BytesIO()
    img.save(buf, format="PNG")
    b64 = base64.b64encode(buf.getvalue()).decode("ascii")

    ox, oy = el.origin.x, el.origin.y
    world_w = el.width * el.resolution
    world_h = el.height * el.resolution

    # SVG top-left: world top-left with Y-flip
    sx = ox
    sy = _y(oy + world_h)

    b.include(sx, sy)
    b.include(sx + world_w, sy + world_h)

    return (
        f'<image x="{sx:.4f}" y="{sy:.4f}" width="{world_w:.4f}" height="{world_h:.4f}" '
        f'href="data:image/png;base64,{b64}" image-rendering="pixelated"/>'
    )


# Dispatch + top-level render


def _render_element(el: SpaceElement, b: Bounds) -> str:
    if isinstance(el, Point):
        return _render_point(el, b)
    elif isinstance(el, Pose):
        return _render_pose(el, b)
    elif isinstance(el, Arrow):
        return _render_arrow(el, b)
    elif isinstance(el, Polyline):
        return _render_polyline(el, b)
    elif isinstance(el, Box3D):
        return _render_box3d(el, b)
    elif isinstance(el, Camera):
        return _render_camera(el, b)
    elif isinstance(el, Text):
        return _render_text(el, b)
    elif isinstance(el, OccupancyGrid):
        return _render_occupancy_grid(el, b)
    elif isinstance(el, PointCloud2):
        from dimos.mapping.occupancy.inflation import simple_inflate
        from dimos.mapping.pointclouds.occupancy import height_cost_occupancy

        return _render_occupancy_grid(simple_inflate(height_cost_occupancy(el), 0.05), b)
    elif isinstance(el, Observation):
        if el.pose is None:
            return ""
        if el.data_type == float:
            return _render_arrow(Arrow(msg=el.pose_stamped, color="#ff0000"), b)
        else:
            return _render_arrow(Arrow(msg=el.pose_stamped), b)

    else:
        return f"<!-- unsupported: {type(el).__name__} -->"


def render(
    space: Space,
    path: str | Path | None = None,
    width_px: float = 800,
    padding: float = 0.5,
) -> str:
    """Render a Space to an SVG string, optionally writing to *path*."""
    b = Bounds()
    fragments: list[str] = []

    for el in space.elements:
        fragments.append(_render_element(el, b))

    if b.empty:
        b.include(0, 0)
        b.include(1, 1)

    b.xmin -= padding
    b.xmax += padding
    b.ymin -= padding
    b.ymax += padding

    aspect = b.height / b.width
    svg_h = width_px * aspect

    parts: list[str] = [
        f'<svg xmlns="http://www.w3.org/2000/svg" '
        f'width="{width_px:.0f}" height="{svg_h:.0f}" '
        f'viewBox="{b.xmin:.4f} {b.ymin:.4f} {b.width:.4f} {b.height:.4f}" '
        f'style="background:#f8f8f8">',
    ]
    parts.extend(fragments)
    parts.append("</svg>")
    svg = "\n".join(parts)

    if path is not None:
        Path(path).write_text(svg)

    return svg


def _esc(s: str) -> str:
    """Escape text for SVG XML."""
    return s.replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;")
