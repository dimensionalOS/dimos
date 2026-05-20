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

"""cv2 raster renderer for Space.

Mirrors the structure of `svg.py`: walks `space.elements`, accumulates a
world-space `Bounds`, then paints each element onto a BGR `np.ndarray` using
OpenCV primitives. Used by `Space.to_bgr` / `Space.to_png` to produce raster
images for LLM image pipelines (vision models read raster reliably; SVG less
so).

World→pixel mapping:
    px = round((wx - bounds.xmin) * res_px)
    py = round((bounds.ymax - wy) * res_px)
"""

from __future__ import annotations

import math
from typing import TYPE_CHECKING, Any

import cv2
import numpy as np

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
    Text,
    Wedge,
)
from dimos.msgs.nav_msgs.OccupancyGrid import OccupancyGrid
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2

if TYPE_CHECKING:
    from dimos.memory2.vis.space.space import Space


# ---------------------------------------------------------------------------
# Color helpers
# ---------------------------------------------------------------------------


def _bgr_u8(c: ColorLike) -> tuple[int, int, int]:
    """Convert any accepted color form to a (B, G, R) uint8 tuple."""
    col = Color.coerce(c)
    r, g, b = col.rgb_u8()
    return (b, g, r)


def _alpha(el: object, base: float = 1.0) -> float:
    """Combine element opacity with a base factor, clamped to [0, 1]."""
    return max(0.0, min(1.0, float(getattr(el, "opacity", 1.0)) * base))


def _thick(value_world: float, res_px: float, minimum: int = 1) -> int:
    """Convert a world-unit thickness to a pixel-thickness, clamped at >= minimum."""
    return max(minimum, round(value_world * res_px))


# ---------------------------------------------------------------------------
# Bounds pass — compute world-frame extent from elements
# ---------------------------------------------------------------------------


def _grow_bounds(el: Any, b: Bounds, pc_cache: dict[int, OccupancyGrid]) -> None:
    """Grow `b` to cover the world-frame extent of `el`. Mirrors svg.py's
    per-element b.include() calls; kept separate so the raster backend can
    decide canvas size before drawing.

    ``pc_cache`` is populated for every visited PointCloud2 so the draw pass
    can reuse the inflated grid instead of recomputing it.
    """
    if isinstance(el, Point):
        r = el.radius * (1.4 if el.halo else 1.0)
        b.include(el.msg.x - r, el.msg.y - r)
        b.include(el.msg.x + r, el.msg.y + r)
    elif isinstance(el, Pose):
        b.include(el.msg.x, el.msg.y)
        if el.size > 0:
            b.include(
                el.msg.x + math.cos(el.msg.yaw) * el.size, el.msg.y + math.sin(el.msg.yaw) * el.size
            )
    elif isinstance(el, Arrow):
        b.include(el.msg.x, el.msg.y)
        b.include(
            el.msg.x + math.cos(el.msg.yaw) * el.length, el.msg.y + math.sin(el.msg.yaw) * el.length
        )
    elif isinstance(el, Polyline):
        for p in el.msg.poses:
            b.include(p.x, p.y)
    elif isinstance(el, Polygon):
        # Skip invisible polygons: <3 vertices, or neither fill nor stroke set.
        if len(el.vertices) < 3 or (el.fill is None and el.stroke is None):
            return
        for wx, wy in el.vertices:
            b.include(wx, wy)
    elif isinstance(el, Wedge):
        if el.fov <= 0 or el.length <= 0:
            return
        half = el.fov / 2.0
        for ang in (el.yaw + half, el.yaw, el.yaw - half):
            b.include(
                el.origin[0] + math.cos(ang) * el.length, el.origin[1] + math.sin(ang) * el.length
            )
        b.include(el.origin[0], el.origin[1])
    elif isinstance(el, Box3D):
        hw, hh = el.size.x / 2, el.size.y / 2
        b.include(el.center.x - hw, el.center.y - hh)
        b.include(el.center.x + hw, el.center.y + hh)
    elif isinstance(el, Camera):
        b.include(el.pose.x, el.pose.y)
    elif isinstance(el, Text):
        b.include(el.position[0], el.position[1])
    elif isinstance(el, RasterOverlay):
        ox, oy = el.origin
        b.include(ox, oy)
        b.include(ox + el.rgba.shape[1] * el.resolution, oy + el.rgba.shape[0] * el.resolution)
    elif isinstance(el, OccupancyGrid):
        if el.grid.size > 0:
            ox, oy = el.origin.x, el.origin.y
            b.include(ox, oy)
            b.include(ox + el.width * el.resolution, oy + el.height * el.resolution)
    elif isinstance(el, PointCloud2):
        # Collapse to OccupancyGrid extent; same approach as svg.py. Cache the
        # inflated grid so _draw_pointcloud doesn't recompute it.
        from dimos.mapping.occupancy.inflation import simple_inflate
        from dimos.mapping.pointclouds.occupancy import height_cost_occupancy

        grid = simple_inflate(height_cost_occupancy(el), _POINTCLOUD_INFLATE_M)
        pc_cache[id(el)] = grid
        if grid.grid.size > 0:
            ox, oy = grid.origin.x, grid.origin.y
            b.include(ox, oy)
            b.include(ox + grid.width * grid.resolution, oy + grid.height * grid.resolution)
    elif isinstance(el, Observation):
        if el.pose is not None:
            b.include(el.pose[0], el.pose[1])


# ---------------------------------------------------------------------------
# Draw pass — paint each element onto BGR ndarray
# ---------------------------------------------------------------------------


class _Canvas:
    """Bundle of canvas state passed to each per-element draw function.

    ``pc_cache`` maps ``id(PointCloud2 element) → inflated OccupancyGrid`` so
    the bounds pass and the draw pass don't duplicate work.
    """

    __slots__ = ("bgr", "bounds", "pc_cache", "res_px")

    def __init__(
        self,
        bgr: np.ndarray,
        bounds: Bounds,
        res_px: float,
        pc_cache: dict[int, OccupancyGrid] | None = None,
    ) -> None:
        self.bgr = bgr
        self.bounds = bounds
        self.res_px = res_px
        self.pc_cache: dict[int, OccupancyGrid] = pc_cache if pc_cache is not None else {}

    def w2p(self, wx: float, wy: float) -> tuple[int, int]:
        """World (x, y) → integer pixel (px, py); Y-flipped."""
        px = round((wx - self.bounds.xmin) * self.res_px)
        py = round((self.bounds.ymax - wy) * self.res_px)
        return px, py


def _draw_text_halo(
    bgr: np.ndarray,
    text: str,
    org: tuple[int, int],
    color_bgr: tuple[int, int, int],
    scale: float = 0.5,
) -> None:
    """Standard black-halo putText pair used throughout dimos."""
    cv2.putText(bgr, text, org, cv2.FONT_HERSHEY_SIMPLEX, scale, (0, 0, 0), 3, cv2.LINE_AA)
    cv2.putText(bgr, text, org, cv2.FONT_HERSHEY_SIMPLEX, scale, color_bgr, 1, cv2.LINE_AA)


def _draw_point(el: Point, c: _Canvas) -> None:
    cx, cy = c.w2p(el.msg.x, el.msg.y)
    r_px = max(2, round(el.radius * c.res_px))
    halo_px = max(r_px + 1, round(r_px * 1.4))
    col = _bgr_u8(el.color)
    alpha = _alpha(el)

    overlay = c.bgr if alpha >= 0.999 else c.bgr.copy()

    if el.shape == "dot":
        if el.halo:
            cv2.circle(overlay, (cx, cy), halo_px, (0, 0, 0), -1, cv2.LINE_AA)
        cv2.circle(overlay, (cx, cy), r_px, col, -1, cv2.LINE_AA)
    elif el.shape == "cross":
        sw = max(1, round(r_px * 0.45))
        if el.halo:
            halo_sw = sw + 2
            cv2.line(overlay, (cx - r_px, cy), (cx + r_px, cy), (0, 0, 0), halo_sw, cv2.LINE_AA)
            cv2.line(overlay, (cx, cy - r_px), (cx, cy + r_px), (0, 0, 0), halo_sw, cv2.LINE_AA)
        cv2.line(overlay, (cx - r_px, cy), (cx + r_px, cy), col, sw, cv2.LINE_AA)
        cv2.line(overlay, (cx, cy - r_px), (cx, cy + r_px), col, sw, cv2.LINE_AA)
    elif el.shape == "x":
        sw = max(1, round(r_px * 0.45))
        diag = round(r_px * 0.70710678)
        if el.halo:
            halo_sw = sw + 2
            cv2.line(
                overlay,
                (cx - diag, cy - diag),
                (cx + diag, cy + diag),
                (0, 0, 0),
                halo_sw,
                cv2.LINE_AA,
            )
            cv2.line(
                overlay,
                (cx - diag, cy + diag),
                (cx + diag, cy - diag),
                (0, 0, 0),
                halo_sw,
                cv2.LINE_AA,
            )
        cv2.line(overlay, (cx - diag, cy - diag), (cx + diag, cy + diag), col, sw, cv2.LINE_AA)
        cv2.line(overlay, (cx - diag, cy + diag), (cx + diag, cy - diag), col, sw, cv2.LINE_AA)
    elif el.shape == "square":
        if el.halo:
            cv2.rectangle(
                overlay, (cx - halo_px, cy - halo_px), (cx + halo_px, cy + halo_px), (0, 0, 0), -1
            )
        cv2.rectangle(overlay, (cx - r_px, cy - r_px), (cx + r_px, cy + r_px), col, -1)

    if alpha < 0.999:
        cv2.addWeighted(overlay, alpha, c.bgr, 1.0 - alpha, 0, dst=c.bgr)

    if el.label:
        _draw_text_halo(c.bgr, el.label, (cx + r_px + 4, cy + 4), col, scale=0.45)


def _draw_arrow_world(
    c: _Canvas,
    origin_world: tuple[float, float],
    yaw: float,
    length_world: float,
    color_bgr: tuple[int, int, int],
    alpha: float,
) -> None:
    cx, cy = c.w2p(*origin_world)
    tx = origin_world[0] + math.cos(yaw) * length_world
    ty = origin_world[1] + math.sin(yaw) * length_world
    px, py = c.w2p(tx, ty)
    thickness = _thick(length_world * 0.08, c.res_px, minimum=2)
    if alpha >= 0.999:
        cv2.arrowedLine(
            c.bgr, (cx, cy), (px, py), color_bgr, thickness, cv2.LINE_AA, tipLength=0.35
        )
    else:
        overlay = c.bgr.copy()
        cv2.arrowedLine(
            overlay, (cx, cy), (px, py), color_bgr, thickness, cv2.LINE_AA, tipLength=0.35
        )
        cv2.addWeighted(overlay, alpha, c.bgr, 1.0 - alpha, 0, dst=c.bgr)


def _draw_arrow(el: Arrow, c: _Canvas) -> None:
    _draw_arrow_world(
        c,
        (el.msg.x, el.msg.y),
        el.msg.yaw,
        el.length,
        _bgr_u8(el.color),
        _alpha(el),
    )


def _draw_pose(el: Pose, c: _Canvas) -> None:
    if el.size <= 0:
        return
    col = _bgr_u8(el.color)
    alpha = _alpha(el)
    _draw_arrow_world(c, (el.msg.x, el.msg.y), el.msg.yaw, el.size, col, alpha)
    if el.label:
        cx, cy = c.w2p(el.msg.x, el.msg.y)
        _draw_text_halo(c.bgr, el.label, (cx + 4, cy + 4), col, scale=0.45)


def _draw_polyline(el: Polyline, c: _Canvas) -> None:
    pts = np.array([c.w2p(p.x, p.y) for p in el.msg.poses], dtype=np.int32)
    if len(pts) < 2:
        return
    col = _bgr_u8(el.color)
    thickness = _thick(el.width, c.res_px, minimum=1)
    alpha = _alpha(el)
    if alpha >= 0.999:
        cv2.polylines(c.bgr, [pts], False, col, thickness, cv2.LINE_AA)
    else:
        overlay = c.bgr.copy()
        cv2.polylines(overlay, [pts], False, col, thickness, cv2.LINE_AA)
        cv2.addWeighted(overlay, alpha, c.bgr, 1.0 - alpha, 0, dst=c.bgr)


def _draw_polygon(el: Polygon, c: _Canvas) -> None:
    if len(el.vertices) < 3:
        return
    if el.fill is None and el.stroke is None:
        return

    pts = np.array([c.w2p(wx, wy) for (wx, wy) in el.vertices], dtype=np.int32)
    op = _alpha(el)

    if el.fill is not None:
        fill_col = _bgr_u8(el.fill)
        fill_alpha = max(0.0, min(1.0, float(el.fill_opacity) * op))
        if fill_alpha > 0:
            overlay = c.bgr.copy()
            cv2.fillPoly(overlay, [pts], fill_col)
            cv2.addWeighted(overlay, fill_alpha, c.bgr, 1.0 - fill_alpha, 0, dst=c.bgr)

    if el.stroke is not None:
        stroke_col = _bgr_u8(el.stroke)
        thickness = _thick(el.stroke_width, c.res_px, minimum=1)
        if op >= 0.999:
            cv2.polylines(c.bgr, [pts], True, stroke_col, thickness, cv2.LINE_AA)
        else:
            overlay = c.bgr.copy()
            cv2.polylines(overlay, [pts], True, stroke_col, thickness, cv2.LINE_AA)
            cv2.addWeighted(overlay, op, c.bgr, 1.0 - op, 0, dst=c.bgr)

    if el.label:
        # Anchor at the top-most pixel (smallest py).
        anchor_idx = int(np.argmin(pts[:, 1]))
        ax, ay = int(pts[anchor_idx, 0]), int(pts[anchor_idx, 1])
        label_col = _bgr_u8(el.stroke) if el.stroke is not None else _bgr_u8(el.fill)
        _draw_text_halo(c.bgr, el.label, (ax + 6, ay + 18), label_col, scale=0.5)


def _draw_wedge(el: Wedge, c: _Canvas) -> None:
    if el.fov <= 0 or el.length <= 0:
        return
    half = el.fov / 2.0
    cx, cy = c.w2p(el.origin[0], el.origin[1])
    col = _bgr_u8(el.color)
    thickness = _thick(el.stroke_width, c.res_px, minimum=1)
    alpha = _alpha(el)
    overlay = c.bgr if alpha >= 0.999 else c.bgr.copy()
    for ang in (el.yaw + half, el.yaw, el.yaw - half):
        tx = el.origin[0] + math.cos(ang) * el.length
        ty = el.origin[1] + math.sin(ang) * el.length
        ex, ey = c.w2p(tx, ty)
        cv2.line(overlay, (cx, cy), (ex, ey), col, thickness, cv2.LINE_AA)
    if alpha < 0.999:
        cv2.addWeighted(overlay, alpha, c.bgr, 1.0 - alpha, 0, dst=c.bgr)
    if el.label:
        _draw_text_halo(c.bgr, el.label, (cx + 6, cy - 6), col, scale=0.5)


def _draw_box3d(el: Box3D, c: _Canvas) -> None:
    hw, hh = el.size.x / 2, el.size.y / 2
    p1 = c.w2p(el.center.x - hw, el.center.y - hh)
    p2 = c.w2p(el.center.x + hw, el.center.y + hh)
    col = _bgr_u8(el.color)
    thickness = max(1, round(min(el.size.x, el.size.y) * 0.04 * c.res_px))
    alpha = _alpha(el)
    if alpha >= 0.999:
        cv2.rectangle(c.bgr, p1, p2, col, thickness, cv2.LINE_AA)
    else:
        overlay = c.bgr.copy()
        cv2.rectangle(overlay, p1, p2, col, thickness, cv2.LINE_AA)
        cv2.addWeighted(overlay, alpha, c.bgr, 1.0 - alpha, 0, dst=c.bgr)
    if el.label:
        _draw_text_halo(c.bgr, el.label, (p1[0], p1[1] - 4), col, scale=0.5)


def _draw_camera(el: Camera, c: _Canvas) -> None:
    cx, cy = c.w2p(el.pose.x, el.pose.y)
    col = _bgr_u8(el.color)
    alpha = _alpha(el)
    if el.camera_info and el.camera_info.K[4] > 0:
        fy = el.camera_info.K[4]
        fov_y = 2 * math.atan(el.camera_info.height / (2 * fy))
        # Re-use wedge drawing for consistency.
        wedge = Wedge(
            origin=(el.pose.x, el.pose.y),
            yaw=el.pose.yaw,
            fov=fov_y,
            length=_CAMERA_WEDGE_LENGTH_M,
            color=el.color,
            stroke_width=0.03,
            label=el.label,
            opacity=el.opacity,
        )
        _draw_wedge(wedge, c)
    else:
        r_px = max(2, round(_CAMERA_DOT_RADIUS_M * c.res_px))
        overlay = c.bgr if alpha >= 0.999 else c.bgr.copy()
        cv2.circle(overlay, (cx, cy), r_px, col, -1, cv2.LINE_AA)
        if alpha < 0.999:
            cv2.addWeighted(overlay, alpha, c.bgr, 1.0 - alpha, 0, dst=c.bgr)
        if el.label:
            _draw_text_halo(c.bgr, el.label, (cx + r_px + 4, cy + 4), col, scale=0.5)


def _draw_text(el: Text, c: _Canvas) -> None:
    cx, cy = c.w2p(el.position[0], el.position[1])
    col = _bgr_u8(el.color)
    scale = max(0.3, min(1.2, el.font_size / 24.0))
    _draw_text_halo(c.bgr, el.text, (cx, cy), col, scale=scale)


def _draw_raster_overlay_rgba(
    rgba: np.ndarray,
    origin: tuple[float, float],
    resolution: float,
    opacity: float,
    c: _Canvas,
) -> None:
    """Alpha-composite a world-frame RGBA bitmap onto the canvas.

    Shared by `RasterOverlay` and the (opaque) `OccupancyGrid` path.
    """
    if rgba.size == 0:
        return
    H, W = rgba.shape[:2]
    ox, oy = origin
    # World extent → canvas pixel rectangle.
    x0, y1 = c.w2p(ox, oy)  # lower-left in world → bottom-left in canvas
    x1, y0 = c.w2p(ox + W * resolution, oy + H * resolution)  # upper-right
    dst_w = max(1, x1 - x0)
    dst_h = max(1, y1 - y0)

    # Resize the overlay to the destination pixel box. cv2.resize takes (w, h).
    # Y-flip the rgba so that world Y-up reads correctly on canvas Y-down.
    rgba_flipped = np.flipud(rgba)
    resized = cv2.resize(rgba_flipped, (dst_w, dst_h), interpolation=cv2.INTER_AREA)

    # Clip to canvas bounds.
    H_c, W_c = c.bgr.shape[:2]
    cx0, cy0 = max(0, x0), max(0, y0)
    cx1, cy1 = min(W_c, x0 + dst_w), min(H_c, y0 + dst_h)
    if cx1 <= cx0 or cy1 <= cy0:
        return

    sx0, sy0 = cx0 - x0, cy0 - y0
    sx1, sy1 = sx0 + (cx1 - cx0), sy0 + (cy1 - cy0)
    src = resized[sy0:sy1, sx0:sx1]
    src_rgb = src[..., :3]
    # OpenCV is BGR; the RGBA input is RGB → swap.
    src_bgr = src_rgb[..., ::-1]
    src_a = (src[..., 3:4].astype(np.float32) / 255.0) * float(opacity)
    dst_region = c.bgr[cy0:cy1, cx0:cx1]
    blended = src_bgr.astype(np.float32) * src_a + dst_region.astype(np.float32) * (1.0 - src_a)
    c.bgr[cy0:cy1, cx0:cx1] = np.clip(blended, 0, 255).astype(np.uint8)


def _draw_raster_overlay(el: RasterOverlay, c: _Canvas) -> None:
    _draw_raster_overlay_rgba(el.rgba, el.origin, el.resolution, float(el.opacity), c)


def _draw_occupancy_grid(el: OccupancyGrid, c: _Canvas) -> None:
    if el.grid.size == 0:
        return
    rgba = generate_rgba_texture(el)
    _draw_raster_overlay_rgba(
        rgba,
        (el.origin.x, el.origin.y),
        el.resolution,
        1.0,
        c,
    )


def _draw_pointcloud(el: PointCloud2, c: _Canvas) -> None:
    grid = c.pc_cache.get(id(el))
    if grid is None:
        # No bounds-pass cache hit (e.g. callers driving _draw_pointcloud
        # directly in tests): fall back to recomputing.
        from dimos.mapping.occupancy.inflation import simple_inflate
        from dimos.mapping.pointclouds.occupancy import height_cost_occupancy

        grid = simple_inflate(height_cost_occupancy(el), _POINTCLOUD_INFLATE_M)
    _draw_occupancy_grid(grid, c)


def _draw_observation(el: Observation, c: _Canvas) -> None:
    if el.pose is None:
        return
    color: ColorLike = "#ff0000" if el.data_type == float else "#e67e22"
    _draw_arrow_world(
        c,
        (el.pose[0], el.pose[1]),
        # Pose yaw via the observation's pose_stamped.
        el.pose_stamped.yaw,
        0.5,
        _bgr_u8(color),
        1.0,
    )


def _draw_element(el: Any, c: _Canvas) -> None:
    if isinstance(el, Point):
        _draw_point(el, c)
    elif isinstance(el, Pose):
        _draw_pose(el, c)
    elif isinstance(el, Arrow):
        _draw_arrow(el, c)
    elif isinstance(el, Polyline):
        _draw_polyline(el, c)
    elif isinstance(el, Polygon):
        _draw_polygon(el, c)
    elif isinstance(el, Wedge):
        _draw_wedge(el, c)
    elif isinstance(el, RasterOverlay):
        _draw_raster_overlay(el, c)
    elif isinstance(el, Box3D):
        _draw_box3d(el, c)
    elif isinstance(el, Camera):
        _draw_camera(el, c)
    elif isinstance(el, Text):
        _draw_text(el, c)
    elif isinstance(el, OccupancyGrid):
        _draw_occupancy_grid(el, c)
    elif isinstance(el, PointCloud2):
        _draw_pointcloud(el, c)
    elif isinstance(el, Observation):
        _draw_observation(el, c)
    # Unsupported types are silently skipped (mirrors svg.py's fallback).


# ---------------------------------------------------------------------------
# Top-level render
# ---------------------------------------------------------------------------


_DEFAULT_BACKGROUND_BGR: tuple[int, int, int] = (248, 248, 248)  # matches SVG #f8f8f8

# height_px is clamped to width_px * this. A very tall, narrow scene would
# otherwise blow up canvas memory; the clamp silently distorts vertical
# aspect once it kicks in, but the alternative is OOM. Bump if a real use
# case needs taller-than-wide rendering past 10×.
_MAX_HEIGHT_RATIO = 10

# Default camera glyph sizes (world metres). Used only when Camera lacks a
# CameraInfo wide enough to derive a wedge length; chosen to be visible at
# typical indoor-map scales without overwhelming the scene.
_CAMERA_WEDGE_LENGTH_M = 0.8
_CAMERA_DOT_RADIUS_M = 0.15

# PointCloud2 → OccupancyGrid inflation radius (metres). Matches the SVG
# backend's choice so both renderers collapse a 3D cloud the same way.
_POINTCLOUD_INFLATE_M = 0.05


def render(
    space: Space,
    *,
    width_px: int = 800,
    padding_m: float = 0.0,
    background_bgr: tuple[int, int, int] = _DEFAULT_BACKGROUND_BGR,
) -> np.ndarray:
    """Render a Space to a BGR ``np.ndarray``.

    ``width_px`` sets the canvas width in pixels; the height is derived from
    the world-frame aspect ratio (clamped to ``width_px * 10`` for safety).
    ``padding_m`` is an extra world-space margin around the content; defaults
    to 0 so an OccupancyGrid base map renders tight to its pixel bounds (no
    light-gray frame around the content).
    """
    b = Bounds()
    pc_cache: dict[int, OccupancyGrid] = {}
    for el in space.elements:
        _grow_bounds(el, b, pc_cache)

    if b.empty:
        b.include(0, 0)
        b.include(1, 1)

    b.xmin -= padding_m
    b.xmax += padding_m
    b.ymin -= padding_m
    b.ymax += padding_m

    width_px = max(1, int(width_px))
    res_px = width_px / b.width
    height_px = round(b.height * res_px)
    height_px = max(1, min(height_px, width_px * _MAX_HEIGHT_RATIO))

    bgr = np.full((height_px, width_px, 3), background_bgr, dtype=np.uint8)
    canvas = _Canvas(bgr, b, res_px, pc_cache)

    for el in space.elements:
        _draw_element(el, canvas)

    return bgr
