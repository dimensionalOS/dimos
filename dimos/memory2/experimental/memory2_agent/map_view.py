# Copyright 2026 Dimensional Inc.
# SPDX-License-Identifier: Apache-2.0

"""Top-down occupancy renderer + memory-view recall helpers.

`MapRenderer.show_map(when)` — top-down occupancy with the agent arrow.
`recall_view(...)` — find the best-matching past camera frames given a
position (by time or by landmark) and a direction (relative or absolute).

The base image (lidar fusion -> occupancy -> BGR) is cached on first use.
"""

from __future__ import annotations

import base64
from dataclasses import dataclass
import math
import threading
from typing import Any

import cv2
import numpy as np

from dimos.mapping.pointclouds.occupancy import simple_occupancy
from dimos.mapping.voxels import VoxelMapTransformer
from dimos.memory2.store.sqlite import SqliteStore
from dimos.memory2.type.observation import Observation
from dimos.memory2.vis.space.elements import (
    Arrow as SpaceArrow,
    Point as SpacePoint,
    Polygon as SpacePolygon,
    Polyline as SpacePolyline,
    RasterOverlay as SpaceRasterOverlay,
    Wedge as SpaceWedge,
)
from dimos.memory2.vis.space.space import Space
from dimos.msgs.geometry_msgs.Point import Point as GeoPoint
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.nav_msgs.Path import Path as DimosPath
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2

SCALE = 5  # output canvas upscale factor (target = grid.width * SCALE px)
VOXEL_SIZE = 0.10  # m, lidar fusion + occupancy resolution

# Agent marker styling for the Space-rendered map. Two-color (white body
# under a red arrow) so the agent dot is unmistakable on busy maps. Sizes
# chosen empirically against the office recording.
_AGENT_BODY_RADIUS_M = 0.30
_AGENT_ARROW_LEN_M = 0.50


class MapRenderer:
    """Caches the fused occupancy grid; builds Spaces for top-down renders.

    The expensive part is the lidar→voxel→occupancy fusion (~2 min on a 60 s
    recording). That result is cached. Everything else — pose resolution,
    Space construction, raster rendering — is fast and rebuilt per call so
    callers can mix in their own elements (Polygons, Wedges, Points).

    Output target is ``width_px = grid.width * SCALE`` so the agent's existing
    image pipeline sees the same canvas dimensions as the original cv2 path.
    """

    def __init__(self, store: SqliteStore) -> None:
        self._store = store
        self._grid: Any = None
        self._odom_poses: list[tuple[float, tuple[float, ...]]] | None = None
        self._odom_traj: DimosPath | None = None
        # Serialises the lazy lidar+odom materialisations so parallel tool
        # calls (e.g. show_map(when="start") + show_map(when="end") in one
        # turn) don't race two passes through the shared sqlite connection.
        # The underlying SqliteBlobStore connection has check_same_thread=
        # False but is not safe for concurrent execute()s — overlapping
        # cursors on the same connection can return None for rows that
        # exist, which then surface as KeyError("No blob for stream=...").
        self._materialise_lock = threading.Lock()

    def _ensure_grid(self) -> None:
        if self._grid is not None:
            return
        with self._materialise_lock:
            if self._grid is not None:
                return
            lidar = self._store.stream("lidar", PointCloud2)
            global_cloud = lidar.transform(VoxelMapTransformer(voxel_size=VOXEL_SIZE)).last().data
            self._grid = simple_occupancy(global_cloud, resolution=VOXEL_SIZE)

    def _ensure_odom(self) -> None:
        if self._odom_poses is not None:
            return
        with self._materialise_lock:
            if self._odom_poses is not None:
                return
            observations = self._store.stream("odom").to_list()
            self._odom_poses = [(o.ts, o.pose) for o in observations]
            self._odom_traj = DimosPath(poses=[o.data for o in observations])

    def resolve_pose(self, when: str) -> tuple[float, tuple[float, ...]] | None:
        """Return (ts, pose) for the requested moment, or None on bad input.

        Accepted ``when`` values:
            "now" / "end" / "latest"     -> last odom pose
            "start" / "first" / "begin"  -> first odom pose
            "<float>"                    -> closest odom pose to that ts
                                            (interpreted as unix ts if > 1e9,
                                             else as seconds from recording start)
        """
        self._ensure_odom()
        assert self._odom_poses is not None
        if not self._odom_poses:
            return None
        token = when.strip().lower()
        if token in ("now", "end", "latest", "last"):
            return self._odom_poses[-1]
        if token in ("start", "first", "begin"):
            return self._odom_poses[0]
        try:
            t = float(token)
        except ValueError:
            return None
        if t < 1e9:
            t = self._odom_poses[0][0] + t
        return min(self._odom_poses, key=lambda r: abs(r[0] - t))

    # -- Space-based API -----------------------------------------------------

    def _agent_marker_elements(self, pose: tuple[float, ...]) -> list[Any]:
        """Build the agent marker (white body + red arrow) as Space elements."""
        x, y = pose[0], pose[1]
        qx, qy, qz, qw = pose[3:7]
        ps = PoseStamped(x, y, 0.0, qx, qy, qz, qw)
        return [
            SpacePoint(
                GeoPoint(x, y, 0.0),
                color="#ffffff",
                radius=_AGENT_BODY_RADIUS_M,
                halo=True,
                shape="dot",
            ),
            SpaceArrow(ps, color="#ff0000", length=_AGENT_ARROW_LEN_M),
        ]

    def base_space(self, *, include_trajectory: bool = True) -> Space:
        """Return a Space with the occupancy base map and (optionally) the
        odom trajectory polyline. No agent marker.
        """
        self._ensure_grid()
        self._ensure_odom()
        space = Space().base_map(self._grid)
        if include_trajectory and self._odom_traj is not None:
            space.add(SpacePolyline(msg=self._odom_traj, color="#ff0000", width=0.05))
        return space

    def space(
        self,
        when: str,
        *,
        include_trajectory: bool = True,
    ) -> tuple[Space, float, tuple[float, ...]] | None:
        """Build a Space with base map + trajectory + agent marker at *when*.

        Returns (space, ts, pose) — callers can ``space.add(...)`` more
        elements (Points, Polygons, Wedges) before calling ``space.to_bgr()``.
        """
        resolved = self.resolve_pose(when)
        if resolved is None:
            return None
        ts, pose = resolved
        space = self.base_space(include_trajectory=include_trajectory)
        for el in self._agent_marker_elements(pose):
            space.add(el)
        return space, ts, pose

    def render_target_width(self) -> int:
        """Pixel width to use when rasterizing — matches the original cv2 canvas
        size (``grid.width * SCALE``) so the agent's image pipeline gets the
        same resolution it did with the cv2 renderer.
        """
        self._ensure_grid()
        return int(self._grid.width * SCALE)


# Shared points overlay for top-down map renders

POINT_COLORS_BGR: dict[str, tuple[int, int, int]] = {
    "red": (0, 0, 255),
    "green": (60, 200, 60),
    "blue": (255, 50, 50),
    "yellow": (0, 255, 255),
    "cyan": (255, 255, 0),
    "magenta": (255, 50, 255),
    "orange": (0, 165, 255),
    "white": (255, 255, 255),
}
POINT_COLORS_ALLOWED: list[str] = list(POINT_COLORS_BGR.keys())
POINT_DEFAULT_COLOR = "yellow"
POINT_SOFT_CAP = 16


# Palette-name → memory2 color string (the named-palette set is identical
# except for "white"; memory2 doesn't have it).
_POINT_COLOR_TO_MEMORY2: dict[str, str] = {
    "red": "red",
    "green": "green",
    "blue": "blue",
    "yellow": "yellow",
    "cyan": "cyan",
    "magenta": "magenta",
    "orange": "orange",
    "white": "#ffffff",
}

# Point radius for agent-overlay markers (world meters). Picked so dots are
# clearly visible on the office recording without dominating the map.
_POINT_OVERLAY_RADIUS_M = 0.22


def add_points_to_space(
    space: Space,
    points: list[dict[str, Any]] | None,
    grid: Any,
) -> str:
    """Append agent-supplied overlay points to *space* and return the legend.

    Numeric index labels live on the image (rendered next to each dot),
    full description lives in the returned legend text. Honours the agent
    tool's input schema: ``{x, y, label?, color?}`` per entry, the same
    colour palette (see :data:`POINT_COLORS_ALLOWED`), and a soft cap
    (:data:`POINT_SOFT_CAP`). Off-map and invalid-colour entries are
    reported in the legend.
    """
    if not points:
        return ""

    pts = list(points)
    dropped = 0
    if len(pts) > POINT_SOFT_CAP:
        dropped = len(pts) - POINT_SOFT_CAP
        pts = pts[:POINT_SOFT_CAP]

    H_g = grid.height
    res = grid.resolution
    ox = grid.origin.position.x
    oy = grid.origin.position.y

    invalid_colors: list[str] = []
    lines: list[str] = []
    for idx, p in enumerate(pts, 1):
        try:
            wx = float(p["x"])
            wy = float(p["y"])
        except (KeyError, TypeError, ValueError):
            lines.append(f"  {idx}  <invalid x/y> — skipped")
            continue
        color_name = (p.get("color") or POINT_DEFAULT_COLOR).lower()
        if color_name not in POINT_COLORS_BGR:
            invalid_colors.append(color_name)
            color_name = POINT_DEFAULT_COLOR
        label = (p.get("label") or "").strip()

        gx = int((wx - ox) / res)
        gy = int((wy - oy) / res)
        if not (0 <= gx < grid.width and 0 <= gy < H_g):
            lines.append(
                f"  {idx}  ({color_name})  at ({wx:+.2f}, {wy:+.2f})"
                + (f"  — {label}" if label else "")
                + "  [OFF MAP]"
            )
            continue

        # Numeric index as the on-image label so the legend (text) and the
        # marker (image) line up one-to-one.
        space.add(
            SpacePoint(
                GeoPoint(wx, wy, 0.0),
                color=_POINT_COLOR_TO_MEMORY2[color_name],
                radius=_POINT_OVERLAY_RADIUS_M,
                shape="dot",
                halo=True,
                label=str(idx),
            )
        )

        line = f"  {idx:>2}  ({color_name:<7s})  at ({wx:+.2f}, {wy:+.2f})"
        if label:
            line += f"  — {label}"
        lines.append(line)

    legend = f"Points ({len(pts)}):\n" + "\n".join(lines)
    if dropped:
        legend += f"\n  [dropped {dropped} points beyond soft cap of {POINT_SOFT_CAP}]"
    if invalid_colors:
        legend += (
            "\n  [invalid color(s) "
            + ", ".join(sorted(set(invalid_colors)))
            + f"; defaulted to '{POINT_DEFAULT_COLOR}'. Allowed: "
            + ", ".join(POINT_COLORS_ALLOWED)
            + "]"
        )
    return legend


def encode_space_as_multimodal(
    space: Space, caption: str, *, width_px: int
) -> list[dict[str, Any]]:
    """Render *space* to PNG and return LangChain multimodal content blocks."""
    png = space.to_png(width_px=width_px, padding_m=0.0)
    b64 = base64.b64encode(png).decode("ascii")
    return [
        {"type": "text", "text": caption},
        {"type": "image_url", "image_url": {"url": f"data:image/png;base64,{b64}"}},
    ]


def encode_bgr_as_multimodal(bgr: np.ndarray, caption: str) -> list[dict[str, Any]]:
    """Wrap a raw BGR image (e.g. an annotated ego camera frame) into
    LangChain multimodal content blocks. Used for in-image annotations that
    aren't world-frame (walkthrough captions, query-projection crosses).
    """
    ok, buf = cv2.imencode(".png", bgr)
    if not ok:
        raise RuntimeError("cv2.imencode failed")
    b64 = base64.b64encode(bytes(buf)).decode("ascii")
    return [
        {"type": "text", "text": caption},
        {"type": "image_url", "image_url": {"url": f"data:image/png;base64,{b64}"}},
    ]


# walkthrough — annotated low-res frame sequence between two moments


def _resolve_ts(when: str, odom_observations: list) -> float | None:
    token = when.strip().lower()
    if token in ("now", "end", "latest", "last"):
        return odom_observations[-1].ts
    if token in ("start", "first", "begin"):
        return odom_observations[0].ts
    try:
        v = float(token)
    except ValueError:
        return None
    if v < 1e9:
        return odom_observations[0].ts + v
    return v


# verify_room_partition — render agent-described room rectangles over the map


@dataclass
class PartitionStats:
    id: Any
    desc: str
    polygon: list[tuple[float, float]]
    area_m2: float
    odom_samples_inside: int
    n_visible_inside: int  # FREE cells (lidar saw open space) in polygon
    n_unknown_inside: int  # UNKNOWN cells (lidar never observed) in polygon — overclaim
    overclaim_m2: float  # n_unknown_inside * cell_area
    overlap_with: dict[Any, float]


def verify_room_partition(
    store: SqliteStore,
    renderer: MapRenderer,
    rooms: list[dict[str, Any]],
    *,
    odom_stride: int = 5,
    max_odom_distance_m: float = 3.0,
) -> tuple[Space, list[PartitionStats], int, int, int]:
    """Analyse and render agent-described room polygons against the map.

    Returns ``(space, per_room_stats, n_odom_outside, n_odom_total,
    n_visible_outside)``. The Space contains: base map + odom trajectory +
    per-room Polygon + RasterOverlay for the "overclaim" red wash where
    polygons cover lidar-UNKNOWN cells + green/red Point per odom sample
    (inside/outside) + orange Point per salient lidar-visible blob outside
    any room (with the blob's m² as the label). Caller renders with
    ``space.to_bgr(width_px=renderer.render_target_width())``.

    ``rooms[i]`` may specify the shape as either:
        "polygon": [[x, y], [x, y], ...]   (>=3 corners, world coords)
    OR  "rect":    [x_min, y_min, x_max, y_max]   (treated as a 4-corner polygon)

    and may also contain:
        "id":   any label
        "desc": short description
    """
    from dimos.memory2.vis.color import PALETTE

    renderer._ensure_grid()
    grid = renderer._grid
    assert grid is not None

    res = grid.resolution
    ox = grid.origin.position.x
    oy = grid.origin.position.y
    H_g, W_g = grid.height, grid.width

    def _world_poly_to_grid_mask(poly_world: list[tuple[float, float]]) -> np.ndarray:
        grid_pts = []
        for wx, wy in poly_world:
            grid_pts.append([int((wx - ox) / res), int((wy - oy) / res)])
        mask = np.zeros((H_g, W_g), dtype=np.uint8)
        if len(grid_pts) >= 3:
            cv2.fillPoly(mask, [np.array(grid_pts, dtype=np.int32)], 255)
        return mask

    # Resolve each room's polygon (world coords). Accept either "polygon"
    # or "rect" (a rect is just a 4-corner axis-aligned polygon). Rooms with
    # neither a valid "polygon" (>=3 corners) nor a valid "rect" are dropped.
    # Keep the surviving room dicts paired with their polygons in `valid_rooms`
    # so every per-room array below shares one index space — dropping a room
    # from `polys_world` while still indexing the original `rooms` by position
    # caused IndexErrors and drifted room labels/stats.
    valid_rooms: list[dict[str, Any]] = []
    polys_world: list[list[tuple[float, float]]] = []
    for r in rooms:
        if "polygon" in r and isinstance(r["polygon"], list) and len(r["polygon"]) >= 3:
            polys_world.append([(float(p[0]), float(p[1])) for p in r["polygon"]])
            valid_rooms.append(r)
        elif "rect" in r and len(r["rect"]) == 4:
            x_a, y_a, x_b, y_b = r["rect"]
            xmin, xmax = min(x_a, x_b), max(x_a, x_b)
            ymin, ymax = min(y_a, y_b), max(y_a, y_b)
            polys_world.append([(xmin, ymin), (xmax, ymin), (xmax, ymax), (xmin, ymax)])
            valid_rooms.append(r)
    n_rooms = len(valid_rooms)

    # Rasterise each polygon once — these masks drive area, membership,
    # overlap detection, and the per-room visible-cell counts.
    room_masks: list[np.ndarray] = [_world_poly_to_grid_mask(p) for p in polys_world]
    room_masks_bool: list[np.ndarray] = [(m > 0) for m in room_masks]
    room_areas_m2: list[float] = [float(m.sum()) * res * res for m in room_masks_bool]
    any_room_mask = np.zeros((H_g, W_g), dtype=bool)
    for m in room_masks_bool:
        any_room_mask |= m

    # Per-room UNKNOWN cell count (cells lidar never observed that the
    # polygon nonetheless claims) — the "overclaim" — and a combined
    # overclaim mask for the visual overlay.
    unknown_mask_grid = grid.grid == -1
    per_room_unknown = [int((m & unknown_mask_grid).sum()) for m in room_masks_bool]
    overclaim_combined_grid = np.zeros((H_g, W_g), dtype=bool)
    for m in room_masks_bool:
        overclaim_combined_grid |= m & unknown_mask_grid

    # Pairwise overlaps (cells -> m^2)
    overlap_with_per_room: list[dict[Any, float]] = [{} for _ in range(n_rooms)]
    for i in range(n_rooms):
        for j in range(i + 1, n_rooms):
            inter = room_masks_bool[i] & room_masks_bool[j]
            ov_cells = int(inter.sum())
            if ov_cells > 0:
                ov_m2 = ov_cells * res * res
                id_i = valid_rooms[i].get("id", i + 1)
                id_j = valid_rooms[j].get("id", j + 1)
                overlap_with_per_room[i][id_j] = ov_m2
                overlap_with_per_room[j][id_i] = ov_m2

    # --- Begin Space construction ----------------------------------------
    space = renderer.base_space(include_trajectory=True)

    # Per-room fill + outline (Polygon handles fill_opacity + label anchor).
    for i, poly_world in enumerate(polys_world):
        room = valid_rooms[i]
        col = PALETTE[i % len(PALETTE)].hex()
        ident = room.get("id", i + 1)
        desc = (room.get("desc", "") or "")[:22]
        space.add(
            SpacePolygon(
                vertices=poly_world,
                fill=col,
                stroke=col,
                fill_opacity=0.35,
                stroke_width=0.06,
                label=f"#{ident}  {desc}",
            )
        )

    # Overclaim red wash — RGBA at grid resolution, world-frame placement.
    # Y-flipped relative to grid array convention so origin = lower-left
    # (matches OccupancyGrid + RasterOverlay's coordinate contract).
    if overclaim_combined_grid.any():
        rgba = np.zeros((H_g, W_g, 4), dtype=np.uint8)
        mask = overclaim_combined_grid
        rgba[mask, 0] = 220  # R
        rgba[mask, 1] = 40  # G
        rgba[mask, 2] = 40  # B
        rgba[mask, 3] = 115  # alpha ~ 0.45
        space.add(
            SpaceRasterOverlay(
                rgba=rgba,
                origin=(ox, oy),
                resolution=res,
            )
        )

    def _inside_any(x: float, y: float) -> int | None:
        gx = int((x - ox) / res)
        gy = int((y - oy) / res)
        if not (0 <= gx < W_g and 0 <= gy < H_g):
            return None
        for i, m in enumerate(room_masks_bool):
            if m[gy, gx]:
                return i
        return None

    # Mark odom samples — green if inside some room, red if outside.
    odom_obs = store.stream("odom").to_list()
    n_total = 0
    n_outside = 0
    per_room_odom = [0] * n_rooms
    for k, obs in enumerate(odom_obs):
        if k % odom_stride != 0 or obs.pose is None:
            continue
        n_total += 1
        x, y = obs.pose[0], obs.pose[1]
        idx = _inside_any(x, y)
        if idx is None:
            n_outside += 1
            space.add(
                SpacePoint(
                    GeoPoint(x, y, 0.0),
                    color="red",
                    radius=0.06,
                    halo=True,
                    shape="dot",
                )
            )
        else:
            per_room_odom[idx] += 1
            space.add(
                SpacePoint(
                    GeoPoint(x, y, 0.0),
                    color="green",
                    radius=0.04,
                    halo=False,
                    shape="dot",
                )
            )

    # Clean the FREE mask: morphological opening removes single-cell
    # free-cell islands (lidar noise). Cheap first pass.
    raw_free_mask = (grid.grid == 0).astype(np.uint8) * 255
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
    free_mask_clean = cv2.morphologyEx(raw_free_mask, cv2.MORPH_OPEN, kernel)
    free_mask = free_mask_clean > 0
    free_outside_mask = (free_mask & ~any_room_mask).astype(np.uint8) * 255

    # Per-room visible-cell counts (free cells inside each polygon)
    per_room_visible = [int((m & free_mask).sum()) for m in room_masks_bool]

    # Connected components on the free-and-outside mask
    n_visible_outside_cells = int(free_outside_mask.sum() // 255)
    salient_blobs: list[tuple[float, float, int]] = []  # (gx, gy, area_cells)
    if n_visible_outside_cells > 0:
        num, labels_cc, stats_cc, centroids_cc = cv2.connectedComponentsWithStats(
            free_outside_mask, connectivity=8
        )
        for k in range(1, num):
            area_cells = int(stats_cc[k, cv2.CC_STAT_AREA])
            # Salience threshold: at least ~0.3 m^2 (30 cells at 0.1 m)
            min_cells = max(8, int(0.3 / (res * res)))
            if area_cells < min_cells:
                continue
            salient_blobs.append((float(centroids_cc[k, 0]), float(centroids_cc[k, 1]), area_cells))

    # Visibility filter: a blob only counts if at least one lidar capture
    # pose had unobstructed line of sight to it through the occupancy.
    # The walls in the fused occupancy are gappy (single-cell pinholes),
    # so we dilate the OCCUPIED mask first (one cv2 close) to seal those
    # holes virtually before the ray-cast. We also require the rays from
    # SEVERAL nearby poses to be clear, not just one — a single lucky-
    # angle ray through a residual gap shouldn't validate the whole blob.
    lidar_obs = store.stream("lidar").to_list()
    lidar_poses_xy = [(float(o.pose[0]), float(o.pose[1])) for o in lidar_obs if o.pose is not None]
    occ_dilated = cv2.dilate(
        (grid.grid == 100).astype(np.uint8) * 255,
        cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)),
        iterations=1,
    )

    def _ray_blocked_by_dilated(p1: tuple[float, float], p2: tuple[float, float]) -> bool:
        """Walk a ray through the DILATED occupied mask; blocked if >=2
        consecutive dilated-occupied cells lie along the path."""
        x1, y1 = p1
        x2, y2 = p2
        dist = math.hypot(x2 - x1, y2 - y1)
        if dist < 1e-6:
            return False
        n = max(2, int(dist / 0.05) + 1)
        run = 0
        for i in range(1, n):
            t = i / n
            wx = x1 + (x2 - x1) * t
            wy = y1 + (y2 - y1) * t
            gxi = int((wx - ox) / res)
            gyi = int((wy - oy) / res)
            if 0 <= gxi < W_g and 0 <= gyi < H_g and occ_dilated[gyi, gxi] > 0:
                run += 1
                if run >= 2:
                    return True
            else:
                run = 0
        return False

    def _blob_visible(cgx: float, cgy: float) -> bool:
        wx = ox + (cgx + 0.5) * res
        wy = oy + (cgy + 0.5) * res
        # Sort lidar poses by distance, take the 12 nearest within 8 m.
        nearby = []
        for px, py in lidar_poses_xy:
            dx, dy = px - wx, py - wy
            d2 = dx * dx + dy * dy
            if d2 <= 64.0:
                nearby.append((d2, px, py))
        nearby.sort()
        nearby = nearby[:12]
        # Need AT LEAST 2 of the nearest poses to have a clear ray. One
        # lucky-angle hit is not enough — it usually means a residual
        # gap in the wall, not a real sightline.
        clear = 0
        for _, px, py in nearby:
            if not _ray_blocked_by_dilated((px, py), (wx, wy)):
                clear += 1
                if clear >= 2:
                    return True
        return False

    salient_blobs = [b for b in salient_blobs if _blob_visible(b[0], b[1])]
    salient_blobs.sort(key=lambda b: -b[2])
    salient_blobs = salient_blobs[:8]

    # Salient blob markers — orange dot scaled by sqrt(area), label = m^2.
    for cgx, cgy, area_cells in salient_blobs:
        area_m2 = area_cells * res * res
        wx = ox + (cgx + 0.5) * res
        wy = oy + (cgy + 0.5) * res
        # World-frame radius matches the cv2 pixel-radius (~5..14 px @ SCALE=5)
        # so the marker looks similar at the agent's default canvas width.
        radius_m = max(0.10, min(0.30, 0.05 + (area_m2**0.5) * 0.04))
        space.add(
            SpacePoint(
                GeoPoint(wx, wy, 0.0),
                color="orange",
                radius=radius_m,
                halo=True,
                shape="dot",
                label=f"{area_m2:.1f} m^2",
            )
        )
    n_visible_outside = len(salient_blobs)

    # Per-room stats
    stats: list[PartitionStats] = []
    for i, poly_world in enumerate(polys_world):
        room = valid_rooms[i]
        stats.append(
            PartitionStats(
                id=room.get("id", i + 1),
                desc=room.get("desc", ""),
                polygon=poly_world,
                area_m2=room_areas_m2[i],
                odom_samples_inside=per_room_odom[i],
                n_visible_inside=per_room_visible[i],
                n_unknown_inside=per_room_unknown[i],
                overclaim_m2=per_room_unknown[i] * res * res,
                overlap_with=overlap_with_per_room[i],
            )
        )

    return space, stats, n_outside, n_total, n_visible_outside


# frames_facing — recorded frames whose viewing cone could contain a point


# Go2 head camera intrinsics (from dimos/robot/unitree/go2/connection.py)
GO2_FX = 819.553492
GO2_FY = 820.646595
GO2_CX = 625.284099
GO2_CY = 336.808987
GO2_IMG_W = 1280
GO2_IMG_H = 720
# Horizontal FOV — 2 * atan(W/2 / fx) ~ 76 deg
GO2_HFOV_DEG = 2.0 * math.degrees(math.atan((GO2_IMG_W / 2.0) / GO2_FX))


def _project_world_xy_to_pixel(
    *,
    cam_pose: tuple[float, ...],
    query_x: float,
    query_y: float,
    query_z: float | None = None,
) -> tuple[int, int, float] | None:
    """Project a world-frame (x, y) (with assumed `query_z` height, default
    floor z=0) into the camera image. Returns (pixel_x_at_floor, pixel_x,
    z_cam) where pixel_x is the column for the query direction
    independent of height, and z_cam is the depth in metres (positive =
    in front). Returns None if the point is behind the camera.
    """
    cam_x, cam_y, cam_z = cam_pose[0], cam_pose[1], cam_pose[2]
    qx, qy, qz, qw = cam_pose[3:7]
    yaw = math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))

    dx = query_x - cam_x
    dy = query_y - cam_y
    # Camera optical frame: +Z forward, +X right, +Y down. World yaw=0 means
    # the camera looks along +x_world; camera-X is world's right.
    z_cam = dx * math.cos(yaw) + dy * math.sin(yaw)
    if z_cam <= 0.05:
        return None
    x_cam = dx * math.sin(yaw) - dy * math.cos(yaw)

    pixel_x = GO2_FX * x_cam / z_cam + GO2_CX
    qz_eff = 0.0 if query_z is None else query_z
    y_cam_floor = cam_z - qz_eff  # camera y is down, so positive when query below
    pixel_y_floor = GO2_FY * y_cam_floor / z_cam + GO2_CY
    return round(pixel_x), round(pixel_y_floor), float(z_cam)


def _annotate_query_in_frame(
    bgr_or_img: Any,
    cam_pose: tuple[float, ...],
    query_x: float,
    query_y: float,
) -> np.ndarray | None:
    """Return the BGR frame with a red vertical line at the query's
    image column and a red dot at the floor-projected pixel. Returns
    None if the query is behind the camera or off-screen by far."""
    # Accept either a DimosImage (with .data ndarray) or a raw ndarray
    if hasattr(bgr_or_img, "data"):
        rgb = bgr_or_img.data
        bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
    else:
        bgr = bgr_or_img.copy()

    proj = _project_world_xy_to_pixel(cam_pose=cam_pose, query_x=query_x, query_y=query_y)
    if proj is None:
        return None  # query behind camera; caller falls back to unannotated frame
    px, py_floor, z_cam = proj
    H, W = bgr.shape[:2]
    if px < -W // 2 or px > W + W // 2:
        return None  # very far off image; not informative

    # Clip drawing position to image bounds; mark sideways arrow if off-screen
    draw_x = max(0, min(W - 1, px))
    draw_y = max(0, min(H - 1, py_floor))
    # Red cross with black halo at the projected pixel
    arm = 18
    for color, thick in ((0, 0, 0), 5), ((0, 0, 255), 2):  # black halo then red
        cv2.line(
            bgr,
            (draw_x - arm, draw_y - arm),
            (draw_x + arm, draw_y + arm),
            color,
            thick,
            cv2.LINE_AA,
        )
        cv2.line(
            bgr,
            (draw_x - arm, draw_y + arm),
            (draw_x + arm, draw_y - arm),
            color,
            thick,
            cv2.LINE_AA,
        )
    # Annotation text
    label = f"query ({query_x:.1f}, {query_y:.1f})  d={z_cam:.1f}m"
    if px < 0:
        label = "<- " + label
    elif px >= W:
        label = label + " ->"
    tx = max(8, min(W - 260, draw_x + arm + 4))
    ty = max(28, draw_y - arm - 6)
    cv2.putText(bgr, label, (tx, ty), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 4, cv2.LINE_AA)
    cv2.putText(bgr, label, (tx, ty), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 1, cv2.LINE_AA)
    return bgr


@dataclass
class VisibilityCandidate:
    obs: Observation
    score: float
    angular_offset_deg: float
    distance_m: float


def _ray_occluded(
    grid: Any,
    p1: tuple[float, float],
    p2: tuple[float, float],
    step_m: float = 0.05,
    max_occupied_run: int = 4,
) -> bool:
    """True if the ray from p1 to p2 hits a contiguous run of more than
    `max_occupied_run` OCCUPIED cells (i.e. a real wall, not just clutter).

    The fused occupancy is noisy — chair legs, shelf brackets, and other
    short obstacles produce isolated occupied cells along long rays. A
    pure "any occupied → blocked" check rejects almost every distant
    line of sight. Counting contiguous occupied cells lets clutter pass
    through while still catching solid walls.
    """
    x1, y1 = p1
    x2, y2 = p2
    dist = math.hypot(x2 - x1, y2 - y1)
    if dist < 1e-6:
        return False
    n = max(2, int(dist / step_m) + 1)
    run = 0
    for i in range(1, n):
        t = i / n
        wx = x1 + (x2 - x1) * t
        wy = y1 + (y2 - y1) * t
        gx, gy, _ = grid.world_to_grid([wx, wy, 0.0])
        gxi, gyi = int(gx), int(gy)
        if 0 <= gxi < grid.width and 0 <= gyi < grid.height and grid.grid[gyi, gxi] == 100:
            run += 1
            if run > max_occupied_run:
                return True
        else:
            run = 0
    return False


def frames_that_could_see_point(
    store: SqliteStore,
    renderer: MapRenderer,
    *,
    x: float,
    y: float,
    k: int = 4,
    max_range_m: float = 8.0,
    min_distance_m: float = 1.0,
    check_occlusion: bool = True,
    fov_horizontal_deg: float = GO2_HFOV_DEG,
    dedup_time_s: float = 3.0,
) -> tuple[list[VisibilityCandidate], Space | None]:
    """Find color_image frames whose 2D viewing cone could contain (x, y).

    Returns ``(picks, space)`` where ``space`` is a Space with the base map,
    odom trajectory, one Wedge per pick (palette-coloured, labelled by
    index), and a yellow X Point at the query position. ``space`` is None
    only if the lidar grid hasn't been built yet (shouldn't happen in
    practice — callers go through MapRenderer).
    """
    renderer._ensure_grid()
    grid = renderer._grid
    if grid is None:
        return [], None

    fov_rad = math.radians(fov_horizontal_deg)
    half_fov = fov_rad / 2.0

    candidates: list[VisibilityCandidate] = []
    for obs in store.stream("color_image").to_list():
        if obs.pose is None:
            continue
        cam_x, cam_y = obs.pose[0], obs.pose[1]
        qx, qy, qz, qw = obs.pose[3:7]
        cam_yaw = _yaw_from_quat(qx, qy, qz, qw)
        bearing = math.atan2(y - cam_y, x - cam_x)
        ang = _angle_diff(bearing, cam_yaw)
        if ang > half_fov:
            continue
        d = math.hypot(x - cam_x, y - cam_y)
        # min_distance_m drops cameras sitting essentially ON the query
        # point. Those frames score well (tiny d) but the projected cross
        # lands at the camera's own feet — useless for "does the cross sit
        # on the object?" verification. Caller can pass 0.0 to inspect.
        if d > max_range_m or d < max(min_distance_m, 1e-3):
            continue
        if check_occlusion and _ray_occluded(grid, (cam_x, cam_y), (x, y)):
            continue
        score = (ang / half_fov) + (d / max_range_m)
        candidates.append(
            VisibilityCandidate(
                obs=obs,
                score=score,
                angular_offset_deg=math.degrees(ang),
                distance_m=d,
            )
        )

    candidates.sort(key=lambda c: c.score)

    picked: list[VisibilityCandidate] = []
    for c in candidates:
        if all(abs(c.obs.ts - p.obs.ts) >= dedup_time_s for p in picked):
            picked.append(c)
            if len(picked) >= k:
                break

    space = _build_visibility_space(
        renderer,
        x=x,
        y=y,
        picks=picked,
        fov_rad=fov_rad,
        max_range_m=max_range_m,
    )
    return picked, space


# Wedge palette for FOV cones — saturated oranges so several cones can
# stack without losing readability against the magenta+blue map.
_WEDGE_PALETTE: tuple[str, ...] = ("orange", "vermilion", "amber", "magenta")


def _build_visibility_space(
    renderer: MapRenderer,
    *,
    x: float,
    y: float,
    picks: list[VisibilityCandidate],
    fov_rad: float,
    max_range_m: float,
) -> Space:
    """Build a Space showing the query point + per-candidate viewing cones
    on top of the base map and odom trajectory.
    """
    space = renderer.base_space(include_trajectory=True)

    for i, c in enumerate(picks):
        cam_x, cam_y = c.obs.pose[0], c.obs.pose[1]
        qx, qy, qz, qw = c.obs.pose[3:7]
        cam_yaw = _yaw_from_quat(qx, qy, qz, qw)
        space.add(
            SpaceWedge(
                origin=(cam_x, cam_y),
                yaw=cam_yaw,
                fov=fov_rad,
                length=max_range_m,
                color=_WEDGE_PALETTE[i % len(_WEDGE_PALETTE)],
                stroke_width=0.05,
                label=str(i + 1),
            )
        )

    # Query marker — yellow X with halo and inline label so the agent
    # can spot it even in a busy cone forest.
    space.add(
        SpacePoint(
            GeoPoint(x, y, 0.0),
            color="yellow",
            radius=0.30,
            shape="x",
            halo=True,
            label=f"query ({x:.1f}, {y:.1f})",
        )
    )

    return space


WALKTHROUGH_TIMESTAMPS_MAX = 32
WALKTHROUGH_FRAMES_MAX = 16
WALKTHROUGH_MIN_STEP_S = 0.5
WALKTHROUGH_MAX_STEP_S = 3.0


def _resolve_walkthrough_range(
    label: str,
    store: SqliteStore,
    t_start: str,
    t_end: str,
    step_seconds: float,
    max_count: int,
) -> tuple[float, float, float, int, list] | str:
    """Shared range/step resolution. Returns (ts0, ts1, step, count, odom_obs)
    on success, or an error string on failure. Refuses ranges that would
    require more than `max_count` samples — agent must chunk instead.
    """
    odom_obs = store.stream("odom").to_list()
    if not odom_obs:
        return f"{label}: odom stream is empty"
    ts0 = _resolve_ts(t_start, odom_obs)
    ts1 = _resolve_ts(t_end, odom_obs)
    if ts0 is None or ts1 is None:
        return f"{label}: could not parse t_start={t_start!r}, t_end={t_end!r}"
    if ts1 < ts0:
        ts0, ts1 = ts1, ts0
    step = float(step_seconds)
    if step > WALKTHROUGH_MAX_STEP_S:
        return (
            f"{label}: step_seconds={step:.1f}s exceeds cap {WALKTHROUGH_MAX_STEP_S:.1f}s. "
            f"Use step_seconds in [{WALKTHROUGH_MIN_STEP_S}, {WALKTHROUGH_MAX_STEP_S}] and "
            f"chunk longer ranges into multiple calls."
        )
    step = max(step, WALKTHROUGH_MIN_STEP_S)
    span = ts1 - ts0
    count = int(span / step) + 1
    if count < 2:
        count = 2
    if count > max_count:
        max_span = (max_count - 1) * step
        return (
            f"{label}: range {span:.1f}s with step {step:.1f}s would need "
            f"{count} samples (cap {max_count}). Issue multiple calls over "
            f"sub-ranges no wider than {max_span:.1f}s each. step_seconds "
            f"is capped at {WALKTHROUGH_MAX_STEP_S}s — you must chunk."
        )
    return ts0, ts1, step, count, odom_obs


def walkthrough_timestamps_only(
    store: SqliteStore,
    *,
    t_start: str,
    t_end: str,
    step_seconds: float = 2.0,
) -> list[tuple[int, float, float]] | str:
    """Return evenly-spaced timestamps between t_start and t_end at ~step_seconds.

    Each item is (index, ts, t_rel_seconds). No images, no pose, no yaw —
    just the schedule the caller can step through with show_image.
    """
    resolved = _resolve_walkthrough_range(
        "walkthrough_timestamps",
        store,
        t_start,
        t_end,
        step_seconds,
        WALKTHROUGH_TIMESTAMPS_MAX,
    )
    if isinstance(resolved, str):
        return resolved
    ts0, ts1, _step, n, odom_obs = resolved
    t_zero = odom_obs[0].ts
    out = []
    for i in range(n):
        ts = ts0 + (ts1 - ts0) * i / (n - 1)
        out.append((i, ts, ts - t_zero))
    return out


def walkthrough_frames(
    store: SqliteStore,
    *,
    t_start: str,
    t_end: str,
    step_seconds: float = 2.0,
    stream: str = "color_image",
    scale: float = 0.30,
) -> list[tuple[np.ndarray, float, tuple[float, ...]]] | str:
    """Sample evenly-spaced frames between t_start and t_end at ~step_seconds.

    Returns a list of (annotated_bgr, ts, pose) tuples, or an error string.
    Each frame is downsized by `scale` and overlaid with t=, pos=, yaw=.
    """
    import math

    import cv2 as _cv2  # local alias to avoid shadowing in tests

    resolved = _resolve_walkthrough_range(
        "walkthrough",
        store,
        t_start,
        t_end,
        step_seconds,
        WALKTHROUGH_FRAMES_MAX,
    )
    if isinstance(resolved, str):
        return resolved
    ts0, ts1, _step, n, odom_obs = resolved
    sample_ts = [ts0 + (ts1 - ts0) * i / (n - 1) for i in range(n)]

    img_obs = store.stream(stream).to_list()
    if not img_obs:
        return f"walkthrough: stream {stream!r} is empty"

    # Drop frames recorded before odometry was established (pose=None). The
    # nearest-frame pick below would otherwise hand a poseless obs to
    # obs.pose[...] and raise TypeError, which propagates uncaught through the
    # walkthrough tool. Filtering up front (rather than a per-sample `continue`)
    # also keeps every sample slot filled by the nearest *posed* frame instead
    # of dropping a tile — poseless frames cluster at t~0, so a `continue`
    # would lose the first tile of any walkthrough that starts at the beginning.
    img_obs = [o for o in img_obs if o.pose is not None]
    if not img_obs:
        return f"walkthrough: stream {stream!r} has no frames with a pose"

    t_zero = odom_obs[0].ts
    out: list[tuple[np.ndarray, float, tuple[float, ...]]] = []
    for s_ts in sample_ts:
        obs = min(img_obs, key=lambda o: abs(o.ts - s_ts))
        x, y = obs.pose[0], obs.pose[1]
        qx, qy, qz, qw = obs.pose[3:7]
        yaw_deg = math.degrees(math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz)))

        bgr = _cv2.cvtColor(obs.data.data, _cv2.COLOR_RGB2BGR)
        if scale != 1.0:
            new_w = int(bgr.shape[1] * scale)
            new_h = int(bgr.shape[0] * scale)
            bgr = _cv2.resize(bgr, (new_w, new_h), interpolation=_cv2.INTER_AREA)

        t_rel = obs.ts - t_zero
        cap = f"t={t_rel:5.1f}s  pos=({x:+.2f}, {y:+.2f})  yaw={yaw_deg:+.0f}deg"
        _cv2.putText(bgr, cap, (8, 18), _cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 0), 3, _cv2.LINE_AA)
        _cv2.putText(
            bgr, cap, (8, 18), _cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1, _cv2.LINE_AA
        )

        out.append((bgr, obs.ts, obs.pose))
    return out


def encode_walkthrough_blocks(
    frames: list[tuple[np.ndarray, float, tuple[float, ...]]],
    *,
    header: str,
    jpeg_quality: int = 70,
) -> list[dict[str, Any]]:
    """Pack a walkthrough into LangChain multimodal blocks (jpeg-encoded)."""
    blocks: list[dict[str, Any]] = [{"type": "text", "text": header}]
    for i, (bgr, ts, pose) in enumerate(frames, 1):
        ok, buf = cv2.imencode(".jpg", bgr, [cv2.IMWRITE_JPEG_QUALITY, jpeg_quality])
        if not ok:
            continue
        b64 = base64.b64encode(bytes(buf)).decode("ascii")
        cap = f"frame {i}/{len(frames)}  ts={ts:.2f}  pose=({pose[0]:+.2f}, {pose[1]:+.2f})"
        blocks.append({"type": "text", "text": cap})
        blocks.append(
            {
                "type": "image_url",
                "image_url": {"url": f"data:image/jpeg;base64,{b64}"},
            }
        )
    return blocks


# recall_view — find recorded camera frames matching a (position, direction)


def _yaw_from_quat(qx: float, qy: float, qz: float, qw: float) -> float:
    """Planar yaw (z-axis rotation) from a unit quaternion."""
    return math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))


def _angle_diff(a: float, b: float) -> float:
    """Smallest absolute angular distance between two yaws in radians."""
    d = (a - b + math.pi) % (2 * math.pi) - math.pi
    return abs(d)


@dataclass
class FrameMatch:
    obs: Observation
    score: float
    yaw_diff_deg: float
    xy_dist_m: float
    time_diff_s: float


def _resolve_position_and_heading(
    renderer: MapRenderer,
    *,
    at_ts: str,
) -> tuple[tuple[float, float], float, float, str]:
    """Resolve at_ts to (xy, body_yaw, anchor_ts, description)."""
    resolved = renderer.resolve_pose(at_ts)
    if resolved is None:
        raise ValueError(f"could not resolve at_ts={at_ts!r}")
    ts, pose = resolved
    x, y = pose[0], pose[1]
    qx, qy, qz, qw = pose[3:7]
    body_yaw = _yaw_from_quat(qx, qy, qz, qw)
    desc = f"at_ts={at_ts!r} -> ({x:.2f}, {y:.2f}, yaw={math.degrees(body_yaw):.1f}°)"
    return (x, y), body_yaw, ts, desc


def _resolve_target_yaw(
    body_yaw: float, direction: str | None, yaw_deg: float | None
) -> tuple[float, str]:
    has_dir = direction is not None
    has_yaw = yaw_deg is not None
    if has_dir == has_yaw:
        raise ValueError("specify exactly one of direction / yaw_deg")
    if has_yaw:
        return math.radians(yaw_deg), f"yaw_deg={yaw_deg:.1f}"  # type: ignore[arg-type]
    offsets = {
        "forward": 0.0,
        "back": math.pi,
        "left": math.pi / 2,
        "right": -math.pi / 2,
    }
    if direction not in offsets:  # type: ignore[operator]
        raise ValueError(f"direction must be one of {sorted(offsets)}, got {direction!r}")
    target = body_yaw + offsets[direction]  # type: ignore[index]
    return (
        target,
        f"direction={direction!r} (body_yaw={math.degrees(body_yaw):.1f}° -> target={math.degrees(target):.1f}°)",
    )


def recall_view(
    store: SqliteStore,
    renderer: MapRenderer,
    *,
    at_ts: str,
    direction: str | None = None,
    yaw_deg: float | None = None,
    k: int = 3,
    max_yaw_deg: float = 45.0,
    max_xy: float = 3.0,
    dedup_time_s: float = 3.0,
) -> tuple[list[FrameMatch], str]:
    """Rank recorded color_image frames by how well each matches the
    requested (position, direction) memory query.

    Position comes from `at_ts` — the robot's pose at that moment.
    Exactly one of {direction, yaw_deg} must be given.

    Hard filters before ranking: yaw_diff <= max_yaw_deg and xy_dist <= max_xy.
    Top-k after de-dup that keeps frames at least dedup_time_s apart in time.
    """
    (qx, qy), body_yaw, anchor_ts, pos_desc = _resolve_position_and_heading(renderer, at_ts=at_ts)
    target_yaw, dir_desc = _resolve_target_yaw(body_yaw, direction, yaw_deg)

    max_yaw_rad = math.radians(max_yaw_deg)
    candidates: list[FrameMatch] = []
    for obs in store.stream("color_image").to_list():
        if obs.pose is None:
            continue
        fx, fy = obs.pose[0], obs.pose[1]
        fqx, fqy, fqz, fqw = obs.pose[3:7]
        frame_yaw = _yaw_from_quat(fqx, fqy, fqz, fqw)
        yaw_d = _angle_diff(frame_yaw, target_yaw)
        if yaw_d > max_yaw_rad:
            continue
        xy_d = math.hypot(fx - qx, fy - qy)
        if xy_d > max_xy:
            continue
        time_d = abs(obs.ts - anchor_ts)
        score = yaw_d / max_yaw_rad + xy_d / max_xy + time_d / 30.0
        candidates.append(
            FrameMatch(
                obs=obs,
                score=score,
                yaw_diff_deg=math.degrees(yaw_d),
                xy_dist_m=xy_d,
                time_diff_s=time_d,
            )
        )

    candidates.sort(key=lambda m: m.score)

    # De-dup by time so we get diversity, not k near-duplicate frames.
    picked: list[FrameMatch] = []
    for cand in candidates:
        if all(abs(cand.obs.ts - p.obs.ts) >= dedup_time_s for p in picked):
            picked.append(cand)
            if len(picked) >= k:
                break

    desc = (
        f"recall_view: {pos_desc} | {dir_desc} | "
        f"filters: yaw<{max_yaw_deg}° xy<{max_xy}m | "
        f"candidates={len(candidates)} returned={len(picked)}"
    )
    return picked, desc
