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
import io
import math
from pathlib import Path
from typing import TYPE_CHECKING

import numpy as np
from PIL import Image as PILImage

from dimos.mapping.occupancy.visualizations import generate_rgba_texture
from dimos.memory2.type.observation import Observation
from dimos.memory2.vis.color import Color
from dimos.memory2.vis.space.bounds import Bounds
from dimos.memory2.vis.space.elements import (
    Arrow,
    Box3D,
    Camera,
    ColorLike,
    Point,
    Polygon,
    Polyline,
    Pose,
    RasterOverlay,
    SpaceElement,
    Text,
    Wedge,
)
from dimos.msgs.nav_msgs.OccupancyGrid import OccupancyGrid
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2

__all__ = ["Bounds", "render"]

if TYPE_CHECKING:
    from dimos.memory2.vis.space.space import Space


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
    halo_r = r * 1.4 if el.halo else r
    b.include(x - halo_r, y - halo_r)
    b.include(x + halo_r, y + halo_r)
    fill, alpha = _style(el)

    parts: list[str] = []
    if el.shape == "dot":
        if el.halo:
            parts.append(
                f'<circle cx="{x:.4f}" cy="{y:.4f}" r="{halo_r:.4f}" '
                f'fill="#000000" opacity="{alpha:.3f}"/>'
            )
        parts.append(
            f'<circle cx="{x:.4f}" cy="{y:.4f}" r="{r:.4f}" fill="{fill}" opacity="{alpha:.3f}"/>'
        )
    elif el.shape == "cross":
        sw = r * 0.5
        if el.halo:
            halo_sw = sw * 2.2
            parts.append(
                f'<line x1="{x - r:.4f}" y1="{y:.4f}" x2="{x + r:.4f}" y2="{y:.4f}" '
                f'stroke="#000000" stroke-width="{halo_sw:.4f}" stroke-linecap="round" opacity="{alpha:.3f}"/>'
            )
            parts.append(
                f'<line x1="{x:.4f}" y1="{y - r:.4f}" x2="{x:.4f}" y2="{y + r:.4f}" '
                f'stroke="#000000" stroke-width="{halo_sw:.4f}" stroke-linecap="round" opacity="{alpha:.3f}"/>'
            )
        parts.append(
            f'<line x1="{x - r:.4f}" y1="{y:.4f}" x2="{x + r:.4f}" y2="{y:.4f}" '
            f'stroke="{fill}" stroke-width="{sw:.4f}" stroke-linecap="round" opacity="{alpha:.3f}"/>'
        )
        parts.append(
            f'<line x1="{x:.4f}" y1="{y - r:.4f}" x2="{x:.4f}" y2="{y + r:.4f}" '
            f'stroke="{fill}" stroke-width="{sw:.4f}" stroke-linecap="round" opacity="{alpha:.3f}"/>'
        )
    elif el.shape == "x":
        sw = r * 0.5
        diag = r * 0.70710678  # r / sqrt(2)
        if el.halo:
            halo_sw = sw * 2.2
            parts.append(
                f'<line x1="{x - diag:.4f}" y1="{y - diag:.4f}" x2="{x + diag:.4f}" y2="{y + diag:.4f}" '
                f'stroke="#000000" stroke-width="{halo_sw:.4f}" stroke-linecap="round" opacity="{alpha:.3f}"/>'
            )
            parts.append(
                f'<line x1="{x - diag:.4f}" y1="{y + diag:.4f}" x2="{x + diag:.4f}" y2="{y - diag:.4f}" '
                f'stroke="#000000" stroke-width="{halo_sw:.4f}" stroke-linecap="round" opacity="{alpha:.3f}"/>'
            )
        parts.append(
            f'<line x1="{x - diag:.4f}" y1="{y - diag:.4f}" x2="{x + diag:.4f}" y2="{y + diag:.4f}" '
            f'stroke="{fill}" stroke-width="{sw:.4f}" stroke-linecap="round" opacity="{alpha:.3f}"/>'
        )
        parts.append(
            f'<line x1="{x - diag:.4f}" y1="{y + diag:.4f}" x2="{x + diag:.4f}" y2="{y - diag:.4f}" '
            f'stroke="{fill}" stroke-width="{sw:.4f}" stroke-linecap="round" opacity="{alpha:.3f}"/>'
        )
    elif el.shape == "square":
        if el.halo:
            parts.append(
                f'<rect x="{x - halo_r:.4f}" y="{y - halo_r:.4f}" '
                f'width="{halo_r * 2:.4f}" height="{halo_r * 2:.4f}" '
                f'fill="#000000" opacity="{alpha:.3f}"/>'
            )
        parts.append(
            f'<rect x="{x - r:.4f}" y="{y - r:.4f}" '
            f'width="{r * 2:.4f}" height="{r * 2:.4f}" '
            f'fill="{fill}" opacity="{alpha:.3f}"/>'
        )

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


def _rgba_image_to_svg(
    rgba: np.ndarray,
    *,
    origin_world: tuple[float, float],
    resolution: float,
    opacity: float,
    b: Bounds,
) -> str:
    """Shared <image> emitter for world-frame RGBA bitmaps.

    Takes lower-left ``origin_world`` (world Y-up), the per-pixel
    ``resolution`` in metres, and renders the bitmap into SVG with the
    correct top-left + Y-flipped placement. Grows ``b`` with the four
    overlay corners.
    """
    if rgba.size == 0:
        return ""

    flipped = np.flipud(rgba)
    img = PILImage.fromarray(flipped, "RGBA")
    buf = io.BytesIO()
    img.save(buf, format="PNG")
    b64 = base64.b64encode(buf.getvalue()).decode("ascii")

    ox, oy = origin_world
    world_w = rgba.shape[1] * resolution
    world_h = rgba.shape[0] * resolution

    sx = ox
    sy = _y(oy + world_h)

    b.include(sx, sy)
    b.include(sx + world_w, sy + world_h)

    opacity_attr = "" if opacity >= 0.999 else f' opacity="{opacity:.3f}"'
    return (
        f'<image x="{sx:.4f}" y="{sy:.4f}" width="{world_w:.4f}" height="{world_h:.4f}" '
        f'href="data:image/png;base64,{b64}" image-rendering="pixelated"{opacity_attr}/>'
    )


def _render_occupancy_grid(el: OccupancyGrid, b: Bounds) -> str:
    if el.grid.size == 0:
        return ""
    rgba = generate_rgba_texture(el)
    return _rgba_image_to_svg(
        rgba,
        origin_world=(el.origin.x, el.origin.y),
        resolution=el.resolution,
        opacity=1.0,
        b=b,
    )


def _render_raster_overlay(el: RasterOverlay, b: Bounds) -> str:
    return _rgba_image_to_svg(
        el.rgba,
        origin_world=el.origin,
        resolution=el.resolution,
        opacity=float(el.opacity),
        b=b,
    )


def _color_attr(c: ColorLike | None) -> tuple[str, float]:
    """Return (hex, alpha) for an optional ColorLike; (#000000, 0) for None."""
    if c is None:
        return "#000000", 0.0
    col = Color.coerce(c)
    return col.hex(), col.a


def _render_polygon(el: Polygon, b: Bounds) -> str:
    pts: list[tuple[float, float]] = [(wx, _y(wy)) for (wx, wy) in el.vertices]

    if len(pts) < 3:
        return f"<!-- polygon: <3 vertices ({len(pts)}) -->"
    if el.fill is None and el.stroke is None:
        return "<!-- polygon: no fill or stroke -->"

    # Only grow bounds once we know the polygon is actually visible.
    for (px, py) in pts:
        b.include(px, py)

    pts_str = " ".join(f"{px:.4f},{py:.4f}" for (px, py) in pts)

    fill_hex, fill_a = _color_attr(el.fill)
    stroke_hex, stroke_a = _color_attr(el.stroke)
    op = float(el.opacity)

    attrs: list[str] = [f'points="{pts_str}"']
    if el.fill is not None:
        attrs.append(f'fill="{fill_hex}"')
        attrs.append(f'fill-opacity="{fill_a * el.fill_opacity * op:.3f}"')
    else:
        attrs.append('fill="none"')
    if el.stroke is not None:
        attrs.append(f'stroke="{stroke_hex}"')
        attrs.append(f'stroke-opacity="{stroke_a * op:.3f}"')
        attrs.append(f'stroke-width="{el.stroke_width:.4f}"')
        attrs.append('stroke-linejoin="round"')

    parts = [f'<polygon {" ".join(attrs)}/>']

    if el.label:
        # Anchor at the top-most vertex (smallest world y → smallest SVG y
        # after Y-flip means largest world y; we want the one that reads
        # topmost on screen, so pick the one with the smallest SVG y).
        anchor_x, anchor_y = min(pts, key=lambda p: p[1])
        label_color = stroke_hex if el.stroke is not None else fill_hex
        label_alpha = (stroke_a if el.stroke is not None else fill_a) * op
        parts.append(
            f'<text x="{anchor_x:.4f}" y="{anchor_y:.4f}" '
            f'font-size="{max(el.stroke_width * 6, 0.2):.4f}" '
            f'fill="{label_color}" opacity="{label_alpha:.3f}">{_esc(el.label)}</text>'
        )
    return "\n".join(parts)


def _render_wedge(el: Wedge, b: Bounds) -> str:
    if el.fov <= 0 or el.length <= 0:
        return "<!-- wedge: non-positive fov or length -->"

    ox, oy = el.origin[0], _y(el.origin[1])
    half = el.fov / 2.0
    # Left edge tip (yaw + half_fov), center tip (yaw), right edge tip (yaw - half_fov)
    def _tip(angle: float) -> tuple[float, float]:
        return (
            el.origin[0] + math.cos(angle) * el.length,
            _y(el.origin[1] + math.sin(angle) * el.length),
        )

    lx, ly = _tip(el.yaw + half)
    cx, cy = _tip(el.yaw)
    rx, ry = _tip(el.yaw - half)

    for px, py in [(ox, oy), (lx, ly), (cx, cy), (rx, ry)]:
        b.include(px, py)

    stroke, alpha = _style(el)
    parts = [
        f'<polygon points="{ox:.4f},{oy:.4f} {lx:.4f},{ly:.4f} {rx:.4f},{ry:.4f}" '
        f'fill="none" stroke="{stroke}" opacity="{alpha:.3f}" '
        f'stroke-width="{el.stroke_width:.4f}" stroke-linejoin="round"/>',
        # centerline
        f'<line x1="{ox:.4f}" y1="{oy:.4f}" x2="{cx:.4f}" y2="{cy:.4f}" '
        f'stroke="{stroke}" stroke-width="{el.stroke_width:.4f}" '
        f'opacity="{alpha:.3f}" stroke-linecap="round"/>',
    ]
    if el.label:
        parts.append(
            f'<text x="{ox + el.stroke_width * 4:.4f}" y="{oy:.4f}" '
            f'font-size="{max(el.stroke_width * 8, 0.2):.4f}" '
            f'fill="{stroke}" opacity="{alpha:.3f}">{_esc(el.label)}</text>'
        )
    return "\n".join(parts)


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
    elif isinstance(el, Polygon):
        return _render_polygon(el, b)
    elif isinstance(el, Wedge):
        return _render_wedge(el, b)
    elif isinstance(el, RasterOverlay):
        return _render_raster_overlay(el, b)
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
    width_px: int = 800,
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
