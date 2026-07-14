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

"""Architectural floor plans (DXF + JPEG) from dimos lidar data.

Library entrypoint:

    from dimos.mapping.floorplan.generator import FloorplanOptions, generate_floorplan
    result = generate_floorplan(FloorplanOptions(rrd=Path("building.rrd")))
    print(result.summary())   # levels, walls, doors, stairs, mezzanines, files

CLI wrapper: scripts/demo_lidar_floorplan.py. Agent-callable skill:
dimos/skills/mapping/floorplan_skill.py (exposed over MCP).

Input is either a *running* dimos instance (LCM topics, e.g. after
`dimos --simulation run unitree-go2 --daemon`) or a Rerun recording.
Multi-story recordings are split into levels automatically — the robot's
trajectory z reveals the floors it walked on — and each level is drawn as its
own cleanly sectioned plan; roof points above the top story are excluded and
mezzanines/sunken areas are indicated on their parent level's sheet.

The drawing follows architectural drafting norms:

  - walls drawn as double lines with poché (solid fill in the raster,
    ANSI31 hatch in the DXF), heaviest line weight — they are cut elements
  - movable objects kept as thin outlines (lightest weight, layer A-FURN);
    people/transients removed
  - door openings detected as gaps in wall runs, drawn with swing arcs
  - dimension strings in millimetres on the major wall runs
  - north arrow, metric scale bar, title block with sheet numbers
  - DXF layers follow the AIA CAD Layer Guidelines (A-WALL, A-WALL-PATT,
    A-DOOR, A-FURN, A-ANNO-DIMS, A-ANNO-SYMB, A-ANNO-TTLB)

Wall/furniture separation uses height evidence: a cell is structural when the
lidar sees returns well above furniture height there. Indoor is distinguished
from outdoor by ceiling evidence — indoor cells have returns above head
height; outdoor clutter (trees, vehicles, ground) is excluded from the plan.
An optional GPT-4o review pass (`--ai-review`, on by default when
OPENAI_API_KEY is set) checks the classification of each level from a
rendered debug image.

Outputs:  <out>.dxf, <out>.jpg (+ optional <out>.model.rrd 3D model and
stylized renditions).
"""

from __future__ import annotations

import base64
import concurrent.futures
from dataclasses import dataclass, field, replace
from datetime import datetime
import itertools
import json
import math
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import threading
import time
from typing import Any

import cv2
from dotenv import load_dotenv
import numpy as np

from dimos.core.transport import LCMTransport
from dimos.mapping.pointclouds.live import LidarPointCloudClient
from dimos.mapping.reconstruction.scene_model import (
    ClosedRegion,
    SceneModel,
    VoxelOccupancy,
)
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.std_msgs.Bool import Bool
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

# Blueprint palette (cyanotype)
BG = "#123252"
GRID_MINOR = "#1e4470"
GRID_MAJOR = "#2a5488"
INK = "#dce9f5"
ACCENT = "#8fb8dd"
RED = "#ff6b57"

WALL_THICKNESS_DEFAULT = 0.15  # m — standard metric interior partition
DOOR_GAP = (0.55, 1.25)  # m — gap widths interpreted as door openings
MERGE_GAP = 0.35  # m — smaller gaps in a wall run are noise, bridge them
MIN_WALL_RUN = 0.8  # m — shortest run drawn as a wall
FLOOR_SEPARATION = 1.8  # m — minimum story height for level detection
STORY_CAP = 2.6  # m — top of the last level's occupancy band above its floor
# m — no real building has consecutive floors this far apart; a bigger gap means
# the trajectory's z drifted (dead-reckoning odometry on a long tour splits one
# real floor into several spurious z-clusters, e.g. 3 floors detected as 6)
MAX_STORY_GAP = 12.0

RobotMarker = tuple[float, float, float | None]  # x, y, yaw (None = unknown)


# --------------------------------------------------------------------------- #
# Collection
# --------------------------------------------------------------------------- #


class _PoseTracker:
    """Tracks /odom poses while lidar collection runs.

    Keeps the full position history, not just the latest pose: floor
    detection is trajectory-dwell-based (detect_floor_elevations), so a
    multi-level live collection needs everywhere the robot has been during
    the window, exactly like the .rrd path gets from its recorded path.
    """

    # At a typical 30-50Hz odom rate this caps memory at ~1MB while covering
    # hours of collection; on overflow, decimate 2x (keep every other pose)
    # rather than dropping the oldest -- dwell detection wants uniform-ish
    # coverage of the whole window, not just its tail.
    MAX_POSES = 50_000

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._pose: PoseStamped | None = None
        self._positions: list[tuple[float, float, float]] = []

    def on_odom(self, pose: PoseStamped) -> None:
        with self._lock:
            self._pose = pose
            self._positions.append(
                (float(pose.position.x), float(pose.position.y), float(pose.position.z))
            )
            if len(self._positions) > self.MAX_POSES:
                self._positions = self._positions[::2]

    @property
    def pose(self) -> PoseStamped | None:
        with self._lock:
            return self._pose

    @property
    def trajectory(self) -> np.ndarray:
        """(N, 3) positions seen so far (empty if no odom arrived)."""
        with self._lock:
            if not self._positions:
                return np.zeros((0, 3), dtype=np.float32)
            return np.asarray(self._positions, dtype=np.float32)


def collect_points(
    duration: float, lidar_topic: str, spin: bool, explore: bool
) -> tuple[np.ndarray, PoseStamped | None, np.ndarray]:
    lidar_client = LidarPointCloudClient(lidar_topic=lidar_topic, global_map_topic="/global_map")
    lidar_client.start()

    pose_tracker = _PoseTracker()
    odom_tp: LCMTransport[PoseStamped] = LCMTransport("/odom", PoseStamped)
    odom_tp.subscribe(pose_tracker.on_odom)

    cmd_tp: LCMTransport[Twist] | None = None
    explore_tp: LCMTransport[Bool] | None = None
    stop_explore_tp: LCMTransport[Bool] | None = None
    if spin:
        cmd_tp = LCMTransport("/cmd_vel", Twist)
        cmd_tp.start()
    if explore:
        explore_tp = LCMTransport("/explore_cmd", Bool)
        stop_explore_tp = LCMTransport("/stop_explore_cmd", Bool)
        explore_tp.start()
        stop_explore_tp.start()

    logger.info(f"Collecting lidar for {duration:.0f}s (spin={spin}, explore={explore})")
    t_start = time.time()
    t_end = t_start + duration
    last_explore_ping = 0.0
    while time.time() < t_end:
        now = time.time()
        if cmd_tp is not None:
            cmd_tp.broadcast(None, Twist(angular=Vector3(0.0, 0.0, 0.5)))
        if explore_tp is not None and now - last_explore_ping > 15.0:
            explore_tp.broadcast(None, Bool(True))  # frontier explorer keep-alive
            last_explore_ping = now
        time.sleep(0.4)

    if stop_explore_tp is not None:
        stop_explore_tp.broadcast(None, Bool(True))
        stop_explore_tp.stop()
    if explore_tp is not None:
        explore_tp.stop()
    if cmd_tp is not None:
        cmd_tp.broadcast(None, Twist())
        cmd_tp.stop()

    points = lidar_client.snapshot()
    pose = pose_tracker.pose
    trajectory = pose_tracker.trajectory
    lidar_client.stop()
    odom_tp.stop()
    logger.info(
        f"Collected {len(points)} points from {lidar_client.message_count} lidar messages "
        f"and {len(trajectory)} trajectory poses"
    )
    return points, pose, trajectory


# --------------------------------------------------------------------------- #
# Input: Rerun recording
# --------------------------------------------------------------------------- #


# .rrd input goes through the shared SceneModel (wishlist item 8): main()
# builds the model once and every per-level evidence raster in build_grids()
# is one of its horizontal_section() plan cuts.


# --------------------------------------------------------------------------- #
# Floor segmentation
# --------------------------------------------------------------------------- #


@dataclass
class Mezzanine:
    """A partial-footprint platform within a story: mezzanine, raised or sunken area."""

    z: float  # its FFL, absolute
    parent_floor_z: float
    footprints: list[np.ndarray] = field(default_factory=list)  # world xy polygons

    @property
    def rel(self) -> float:
        return self.z - self.parent_floor_z

    @property
    def label(self) -> str:
        kind = "MEZZANINE" if self.rel > 0 else "SUNKEN"
        return f"{kind} FFL {self.z:+.2f} m ({self.rel:+.2f} m)"


@dataclass
class Level:
    index: int  # 1-based, bottom-up
    floor_z: float
    ceiling_z: float  # top of this level's occupancy band
    mezzanines: list[Mezzanine] = field(default_factory=list)

    @property
    def name(self) -> str:
        return f"LEVEL {self.index}"


def detect_floor_elevations(path_z: np.ndarray, manual: str | None) -> list[float]:
    """Floor elevations from robot-trajectory dwell peaks (the robot walks on floors).

    A floor only needs a *visit*, not a long stay — the robot may cross an
    upper story briefly. Keep the dwell threshold low (1% of samples) and rely
    on the story-height separation to reject stair landings between floors.
    """
    if manual:
        return sorted(float(v) for v in manual.split(","))
    if len(path_z) < 20:
        return []
    bins = np.arange(path_z.min() - 0.05, path_z.max() + 0.35, 0.1)
    hist, edges = np.histogram(path_z, bins=bins)
    # np.convolve("same") returns kernel-length output when the histogram is
    # shorter than the kernel (a near-flat trajectory) — skip smoothing then
    if len(hist) >= 5:
        smooth = np.convolve(hist, np.ones(5) / 5, mode="same")
    else:
        smooth = hist.astype(np.float64)
    centers = (edges[:-1] + edges[1:]) / 2

    floors: list[float] = []
    candidates: list[tuple[float, float]] = []
    for i in np.argsort(smooth)[::-1]:
        # a floor needs only a brief visit; stair samples smear across many
        # bins and stay below this, and FLOOR_SEPARATION rejects landings
        if smooth[i] < max(0.004 * len(path_z), 8.0):
            break
        z = float(centers[i])
        candidates.append((z, float(smooth[i])))
        if all(abs(z - f) >= FLOOR_SEPARATION for f in floors):
            floors.append(z)
    logger.info(
        "Floor candidates (z, dwell): "
        + ", ".join(f"({z:+.2f}, {dw:.0f})" for z, dw in sorted(candidates))
    )
    return sorted(floors)


def check_floor_plausibility(floors: list[float]) -> str | None:
    """Warning string when auto-detected floors can't be a real building.

    Odometry drift on a long tour smears the trajectory's z, so revisits of
    the same physical floor land at different elevations and one real story
    splits into several spurious ones (observed: a 3-story office detected as
    6 "floors" spanning 43 m). The signature is a consecutive floor gap no
    real building has; the fix is better data, not drawing 6 sheets silently.
    """
    gaps = [(hi - lo, lo, hi) for lo, hi in zip(floors, floors[1:]) if hi - lo > MAX_STORY_GAP]
    if not gaps:
        return None
    worst = max(g[0] for g in gaps)
    return (
        f"{len(floors)} floors detected, but consecutive floors "
        f"{gaps[0][1]:+.1f} m -> {gaps[0][2]:+.1f} m are {worst:.0f} m apart — no real "
        "building spaces stories like that, so the trajectory's z has almost certainly "
        "drifted (dead-reckoning odometry splits each real floor into several spurious "
        "ones). Use a drift-corrected recording (e.g. a point-LIO / loop-closed SLAM "
        "export) or pass explicit floor elevations via levels=."
    )


def build_levels(floors: list[float]) -> list[Level]:
    levels = []
    for i, floor_z in enumerate(floors):
        ceiling = floors[i + 1] - 0.5 if i + 1 < len(floors) else floor_z + STORY_CAP
        ceiling = min(ceiling, floor_z + STORY_CAP)
        levels.append(Level(index=i + 1, floor_z=floor_z, ceiling_z=ceiling))
    return levels


def detect_mezzanines(model: SceneModel, levels: list[Level]) -> None:
    """Find intermediate platforms and attach them to their parent level.

    A mezzanine (or split-level / sunken area) shows up as trajectory dwell
    0.5-1.8 m away from a primary floor — too close to be its own story, so
    the primary pass rejects it. To distinguish a real platform from the
    look-alikes, a candidate must pass four tests:

      1. *sharp dwell*: a local maximum with prominence — a ramp or stair
         climb smears dwell across z and never forms a distinct peak
      2. *slab evidence*: the cloud has an areal platform at that elevation
      3. *walkable footprint* ≥ 3 m² around where the robot actually stood —
         a stair landing has dwell and a slab but only ~1 m²
      4. *overhead returns* above the platform — outdoor terrain at a lower
         elevation has dwell and ground but no ceiling

    Detected platforms are drawn on the parent level's sheet with a dashed
    outline and their own FFL label (they are not separate sheets).
    """
    path_z = model.trajectory[:, 2]
    if len(path_z) < 20 or not levels:
        return
    floors = [lv.floor_z for lv in levels]
    bins = np.arange(path_z.min() - 0.05, path_z.max() + 0.35, 0.1)
    hist, edges = np.histogram(path_z, bins=bins)
    if len(hist) < 5:
        return  # near-flat trajectory: no room for an intermediate platform
    smooth = np.convolve(hist, np.ones(5) / 5, mode="same")
    centers = (edges[:-1] + edges[1:]) / 2
    prominence_reach = 4  # bins = 0.4 m

    accepted: list[float] = []
    for i in np.argsort(smooth)[::-1]:
        if smooth[i] < max(0.008 * len(path_z), 8.0):
            break
        z_c = float(centers[i])
        # not part of (or too close to) a primary floor or an accepted platform
        if min(abs(z_c - f) for f in floors) < 0.5:
            continue
        if any(abs(z_c - a) < 0.5 for a in accepted):
            continue
        # 1. sharp dwell: local max with prominence (rejects ramps/stair smear)
        lo_i, hi_i = max(i - prominence_reach, 0), min(i + prominence_reach, len(smooth) - 1)
        if smooth[i] < smooth[lo_i] * 1.3 or smooth[i] < smooth[hi_i] * 1.3:
            continue

        below = [f for f in floors if f < z_c - 0.4]
        parent_z = max(below) if below else min(floors)  # below lowest = sunken
        rel = z_c - parent_z
        if abs(rel) >= FLOOR_SEPARATION and rel > 0:
            continue  # far above a floor → would have been a story of its own

        # 2+3. slab with a walkable footprint where the robot dwelt
        res = 0.1
        slab = model.horizontal_section(z=z_c - 0.05, thickness=0.35, resolution=res)
        occ = (slab.occupancy(2).astype(np.uint8)) * 255
        occ = cv2.morphologyEx(  # type: ignore[assignment]
            occ, cv2.MORPH_CLOSE, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (9, 9))
        )
        dwell_xy = model.trajectory[np.abs(path_z - z_c) < 0.3][:, :2]
        if not len(dwell_xy):
            continue
        x0, y0 = slab.extent[0], slab.extent[1]
        h, w = occ.shape
        dwell_mask = np.zeros_like(occ)
        dc = ((dwell_xy[:, 0] - x0) / res).astype(np.int32).clip(0, w - 1)
        dr = ((dwell_xy[:, 1] - y0) / res).astype(np.int32).clip(0, h - 1)
        dwell_mask[dr, dc] = 255
        dwell_mask = cv2.dilate(dwell_mask, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (9, 9)))  # type: ignore[assignment]
        n, labels = cv2.connectedComponents(occ)
        footprints: list[np.ndarray] = []
        area_m2 = 0.0
        for comp_i in range(1, n):
            comp = (labels == comp_i).astype(np.uint8) * 255
            if not cv2.bitwise_and(comp, dwell_mask).any():
                continue
            comp_area = float(np.count_nonzero(comp)) * res * res
            if comp_area < 3.0:
                continue  # slab fragments and furniture-top slivers, not the platform
            area_m2 += comp_area
            contours, _ = cv2.findContours(comp, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for contour in contours:
                approx = cv2.approxPolyDP(contour, epsilon=0.3 / res, closed=True)
                poly = approx.reshape(-1, 2).astype(np.float64)
                poly[:, 0] = x0 + poly[:, 0] * res
                poly[:, 1] = y0 + poly[:, 1] * res
                footprints.append(poly)
        if area_m2 < 3.0:
            continue  # stair landing / clutter, not a walkable platform

        # 4. overhead returns above the platform (indoor) — outdoor ground fails
        over = model.horizontal_section(
            z=z_c + 2.2, thickness=2.0, resolution=res, bounds=slab.extent
        )
        overlap = np.count_nonzero(over.occupancy(1) & (occ > 0)) / max(np.count_nonzero(occ), 1)  # type: ignore[operator]
        if overlap < 0.05:
            continue

        parent = max(
            (lv for lv in levels if lv.floor_z == parent_z),
            key=lambda lv: lv.index,
            default=levels[0],
        )
        mezz = Mezzanine(z=z_c, parent_floor_z=parent.floor_z, footprints=footprints)
        parent.mezzanines.append(mezz)
        accepted.append(z_c)
        logger.info(
            f"{parent.name}: detected {mezz.label} — {area_m2:.1f} m² platform, "
            f"dwell {smooth[i]:.0f}"
        )


# --------------------------------------------------------------------------- #
# Classification: permanent structure vs movable objects
# --------------------------------------------------------------------------- #


@dataclass
class Grids:
    all_mask: np.ndarray  # uint8 — every occupied cell in the level band
    high_mask: np.ndarray  # uint8 — cells with returns above furniture height
    indoor_mask: np.ndarray | None  # uint8 — cells under a ceiling (None = no evidence)
    envelope_mask: np.ndarray | None  # uint8 — building footprint at the wall face
    origin: tuple[float, float]
    resolution: float


@dataclass
class Cluster:
    label: str  # "W3" / "M7"
    mask: np.ndarray  # uint8 component mask
    centroid: tuple[float, float]  # world m
    extent: tuple[float, float]  # minAreaRect (long, short) in m
    high_frac: float


def build_grids(
    model: SceneModel,
    z_min: float,
    z_max: float,
    high_z: float,
    ceil_band: tuple[float, float] | None,
    traj_xy: np.ndarray | None,
    resolution: float,
    min_hits: int,
) -> Grids | None:
    """Evidence rasters for one level, built from SceneModel plan cuts (item 8).

    Every band is a `horizontal_section()` over the same fixed bounds, so the
    resulting masks are cell-aligned and combine directly.
    """
    points = model.points
    band_sel = (points[:, 2] >= z_min) & (points[:, 2] <= z_max)
    if int(band_sel.sum()) < 100:
        return None

    xy = points[band_sel][:, :2]
    margin = 0.6
    x0, y0 = xy.min(axis=0) - margin
    x1, y1 = xy.max(axis=0) + margin
    if ((x1 - x0) / resolution) * ((y1 - y0) / resolution) > 6000 * 6000:
        raise SystemExit("Grid too large — outliers? Tighten the z band.")
    bounds = (float(x0), float(y0), float(x1), float(y1))

    def _cut(lo: float, hi: float) -> np.ndarray:
        section = model.horizontal_section(
            z=(lo + hi) / 2, thickness=hi - lo, resolution=resolution, bounds=bounds
        )
        return section.density

    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
    all_mask = (_cut(z_min, z_max) >= min_hits).astype(np.uint8) * 255
    all_mask = cv2.morphologyEx(all_mask, cv2.MORPH_CLOSE, kernel)  # type: ignore[assignment]
    h, w = all_mask.shape

    # Structural evidence: any return above furniture height. Kept deliberately
    # permissive (>=1 hit, no all_mask AND) — far walls are sparsely sampled;
    # the 0.6 m opening in vectorize() removes isolated noise cells anyway.
    high_mask = (_cut(high_z, z_max) >= 1).astype(np.uint8) * 255
    high_mask = cv2.morphologyEx(high_mask, cv2.MORPH_CLOSE, kernel)  # type: ignore[assignment]

    # Indoor evidence: returns overhead (a ceiling). Outdoor areas — ground,
    # trees, vehicles — have nothing above head height.
    indoor_mask: np.ndarray | None = None
    if ceil_band is not None:
        raw = (_cut(ceil_band[0], ceil_band[1]) >= 1).astype(np.uint8) * 255
        coverage = np.count_nonzero(cv2.bitwise_and(raw, all_mask)) / max(  # type: ignore[operator]
            np.count_nonzero(all_mask), 1
        )
        if coverage >= 0.10:
            big = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (9, 9))
            indoor = cv2.morphologyEx(raw, cv2.MORPH_CLOSE, big)
            # fill enclosed holes (rooms whose ceiling the lidar under-sampled)
            flood = indoor.copy()
            fmask = np.zeros((h + 2, w + 2), np.uint8)
            cv2.floodFill(flood, fmask, (0, 0), 255)
            indoor = cv2.bitwise_or(indoor, cv2.bitwise_not(flood))
            # a real interior is AREAL ceiling coverage; a neighboring facade or
            # fence has only a thin ribbon of overhead returns directly above
            # itself — erode it away before extending back out
            thin = max(3, int(0.35 / resolution))
            indoor = cv2.erode(indoor, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (thin, thin)))
            # extend past the envelope so exterior wall faces stay included
            reach = max(3, int(0.85 / resolution))
            indoor = cv2.dilate(
                indoor, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (reach, reach))
            )
            # neighboring buildings also have "ceilings" — this building is the
            # indoor region the robot actually drove through. Run the
            # connectivity test on a heavily bridged copy: ceiling evidence is
            # patchy (unvisited rooms form separate islands), so gaps up to
            # ~2.5 m are considered the same building, while a facade across a
            # street (10 m+) stays a separate region and is dropped.
            if traj_xy is not None and len(traj_xy):
                tmask = np.zeros_like(indoor)
                tc = ((traj_xy[:, 0] - x0) / resolution).astype(np.int32).clip(0, w - 1)
                tr = ((traj_xy[:, 1] - y0) / resolution).astype(np.int32).clip(0, h - 1)
                tmask[tr, tc] = 255
                tmask = cv2.dilate(tmask, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (21, 21)))
                bridge_px = max(5, int(2.5 / resolution)) | 1
                bridged = cv2.morphologyEx(
                    indoor,
                    cv2.MORPH_CLOSE,
                    cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (bridge_px, bridge_px)),
                )
                n_c, comp_labels = cv2.connectedComponents(bridged)
                keep = set(np.unique(comp_labels[tmask > 0])) - {0}
                if keep:
                    # the bridged region *is* the building footprint — gaps it
                    # closed are unvisited interior, and their walls belong in
                    indoor = (np.isin(comp_labels, list(keep)).astype(np.uint8)) * 255
            indoor_mask = indoor
        else:
            logger.info(
                f"Ceiling evidence covers only {coverage:.0%} of occupancy — "
                "skipping indoor/outdoor filtering for this level"
            )

    # The building footprint at the exterior wall face: undo the outward
    # dilation. Glass curtain walls give the lidar almost no direct returns —
    # the ceiling extent is the reliable envelope evidence.
    envelope_mask: np.ndarray | None = None
    if indoor_mask is not None:
        reach = max(3, int(0.85 / resolution))
        envelope_mask = cv2.erode(
            indoor_mask, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (reach, reach))
        )
        high_mask = cv2.bitwise_and(high_mask, indoor_mask)  # type: ignore[assignment]

    # despeckle: isolated high-return specks (sensor noise, plant tops) are
    # not structure and only clutter clustering and the AI review
    n, labels, stats, _ = cv2.connectedComponentsWithStats(high_mask)
    min_cells = max(4, int(0.04 / (resolution * resolution)))
    for i in range(1, n):
        if stats[i, cv2.CC_STAT_AREA] < min_cells:
            high_mask[labels == i] = 0

    return Grids(
        all_mask, high_mask, indoor_mask, envelope_mask, (float(x0), float(y0)), resolution
    )


def split_clusters(grids: Grids) -> tuple[list[Cluster], list[Cluster]]:
    """Connected components: W* (tall → wall candidates), M* (low-only → movable)."""
    res = grids.resolution
    x0, y0 = grids.origin

    def _components(mask: np.ndarray, prefix: str) -> list[Cluster]:
        n, labels = cv2.connectedComponents(mask)
        out: list[Cluster] = []
        for i in range(1, n):
            comp = (labels == i).astype(np.uint8) * 255
            area_cells = int(np.count_nonzero(comp))
            if area_cells < 4:
                continue
            ys, xs = np.nonzero(comp)
            pts = np.column_stack([xs, ys]).astype(np.float32)
            (_, _), (rw, rh), _ = cv2.minAreaRect(pts)
            long_m, short_m = max(rw, rh) * res, min(rw, rh) * res
            high_overlap = np.count_nonzero(cv2.bitwise_and(comp, grids.high_mask))
            out.append(
                Cluster(
                    label=f"{prefix}{len(out)}",
                    mask=comp,
                    centroid=(x0 + xs.mean() * res, y0 + ys.mean() * res),
                    extent=(float(long_m), float(short_m)),
                    high_frac=high_overlap / area_cells,
                )
            )
        return out

    walls = _components(grids.high_mask, "W")
    # movable = occupied areas with no tall evidence nearby
    tall_dilated = cv2.dilate(grids.high_mask, cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5)))
    movable_mask = cv2.bitwise_and(grids.all_mask, cv2.bitwise_not(tall_dilated))
    if grids.indoor_mask is not None:
        # outdoor clutter (trees, vehicles, ground returns) never reaches the plan
        movable_mask = cv2.bitwise_and(movable_mask, grids.indoor_mask)
    movables = _components(movable_mask, "M")
    return walls, movables


# --------------------------------------------------------------------------- #
# Wall vectorization: dominant-axis rectification, runs, door gaps
# --------------------------------------------------------------------------- #


@dataclass
class Wall:
    p1: tuple[float, float]
    p2: tuple[float, float]
    thickness: float = WALL_THICKNESS_DEFAULT
    glazed: bool = False  # glass: drawn as thin double line, no poché (A-GLAZ)

    @property
    def length(self) -> float:
        return math.hypot(self.p2[0] - self.p1[0], self.p2[1] - self.p1[1])

    def corners(self) -> np.ndarray:
        dx, dy = self.p2[0] - self.p1[0], self.p2[1] - self.p1[1]
        norm = math.hypot(dx, dy) or 1.0
        nx, ny = -dy / norm * self.thickness / 2, dx / norm * self.thickness / 2
        return np.array(
            [
                [self.p1[0] + nx, self.p1[1] + ny],
                [self.p2[0] + nx, self.p2[1] + ny],
                [self.p2[0] - nx, self.p2[1] - ny],
                [self.p1[0] - nx, self.p1[1] - ny],
            ]
        )


@dataclass
class Door:
    hinge: tuple[float, float]
    jamb: tuple[float, float]  # opposite side of the opening
    normal: tuple[float, float]  # unit vector, swing side
    glazed: bool = False  # glass door: thin leaf on A-GLAZ

    @property
    def width(self) -> float:
        return math.hypot(self.jamb[0] - self.hinge[0], self.jamb[1] - self.hinge[1])


@dataclass
class RoomPolygon:
    """One partitioned space: a closed area bounded by walls and closed doors."""

    polygon: np.ndarray  # (K, 2) world m, first == last
    area_m2: float
    centroid: tuple[float, float]


@dataclass
class Stair:
    p1: tuple[float, float]  # centerline, lower end
    p2: tuple[float, float]  # centerline, upper end
    width: float = 1.2
    label: str = "UP"


@dataclass
class Plan:
    walls: list[Wall] = field(default_factory=list)
    doors: list[Door] = field(default_factory=list)
    stairs: list[Stair] = field(default_factory=list)
    furniture: list[np.ndarray] = field(default_factory=list)  # closed polygons, world m
    # partial-height platforms on this level: (footprint polygon, label)
    level_changes: list[tuple[np.ndarray, str]] = field(default_factory=list)
    # sealed connected-area boundaries from the voxel model (closed loops, world m)
    rooms: list[np.ndarray] = field(default_factory=list)
    # per-room polygons: walls + closed doors as partition boundaries (segment_rooms)
    partitions: list[RoomPolygon] = field(default_factory=list)
    dominant_angle: float = 0.0

    def translated(self, dx: float, dy: float) -> Plan:
        return Plan(
            walls=[
                replace(w, p1=(w.p1[0] + dx, w.p1[1] + dy), p2=(w.p2[0] + dx, w.p2[1] + dy))
                for w in self.walls
            ],
            doors=[
                replace(
                    d,
                    hinge=(d.hinge[0] + dx, d.hinge[1] + dy),
                    jamb=(d.jamb[0] + dx, d.jamb[1] + dy),
                )
                for d in self.doors
            ],
            partitions=[
                replace(
                    r, polygon=r.polygon + np.array([dx, dy]), centroid=(r.centroid[0] + dx, r.centroid[1] + dy)
                )
                for r in self.partitions
            ],
            stairs=[
                replace(s, p1=(s.p1[0] + dx, s.p1[1] + dy), p2=(s.p2[0] + dx, s.p2[1] + dy))
                for s in self.stairs
            ],
            furniture=[poly + np.array([dx, dy]) for poly in self.furniture],
            level_changes=[
                (poly + np.array([dx, dy]), label) for poly, label in self.level_changes
            ],
            rooms=[poly + np.array([dx, dy]) for poly in self.rooms],
            dominant_angle=self.dominant_angle,
        )

    def bounds(self) -> tuple[float, float, float, float]:
        geo = [w.corners() for w in self.walls] + list(self.furniture)
        geo.extend(poly for poly, _ in self.level_changes)
        for s in self.stairs:
            geo.append(np.array([s.p1, s.p2]))
        if not geo:
            return 0.0, 0.0, 1.0, 1.0
        pts = np.vstack(geo)
        return (
            float(pts[:, 0].min()),
            float(pts[:, 1].min()),
            float(pts[:, 0].max()),
            float(pts[:, 1].max()),
        )


def segment_rooms(
    plan: Plan,
    region: ClosedRegion | None = None,
    resolution: float = 0.05,
    min_area_m2: float = 1.0,
    junction_gap_m: float = 0.5,
) -> list[RoomPolygon]:
    """Partition the plan into discrete rooms from wall + closed-door geometry.

    Rasterizes the *vectorized* walls — cleaner than raw lidar evidence since
    it runs after dangling/duplicate-run cleanup — and treats doors as
    closed, so each side of a doorway reads as its own room. A small closing
    pass bridges the sub-wall-thickness gaps left at T-junctions and corners
    by vectorizing each wall as a separate run; it's far too small to bridge
    an actual doorway, which stays open until the explicit door-closing step
    below seals it.

    Vectorized walls can still have real, multi-meter holes (an unscanned
    stretch of exterior wall, or a `close_wall_gaps` bridge segment the AI
    critique pass deleted as a stray fragment) that no reasonable T-junction
    kernel should paper over. When `region` (the voxel-sealed whole-building
    boundary — see `closed_region`, independently reliable once it seals) is
    given, its interior is used as a hard outer bound: cells outside it are
    barriers regardless of what the vectorized walls show, so a hole in the
    wall vector can no longer leak the flood-fill out of the building. Walls
    and doors still do all the work of dividing that interior into rooms.

    Regions smaller than `min_area_m2` are dropped as noise (sensor shadow
    pockets, sliver artifacts from wall vectorization) rather than rooms.
    """
    if not plan.walls:
        return []

    x0, y0, x1, y1 = plan.bounds()
    margin = 0.5
    x0, y0, x1, y1 = x0 - margin, y0 - margin, x1 + margin, y1 + margin
    w = max(1, int((x1 - x0) / resolution) + 1)
    h = max(1, int((y1 - y0) / resolution) + 1)
    if w * h > 4000 * 4000:
        return []  # degenerate bounds — refuse to allocate an absurd grid

    def _to_px(pts: np.ndarray) -> np.ndarray:
        return np.column_stack(
            [(pts[:, 0] - x0) / resolution, (pts[:, 1] - y0) / resolution]
        ).astype(np.int32)

    grid = np.zeros((h, w), np.uint8)
    for wall in plan.walls:
        cv2.fillPoly(grid, [_to_px(wall.corners())], 255)

    # bridge small T-junction / corner gaps left by vectorizing separate wall
    # runs. Never let this exceed a fraction of the narrowest real door on
    # this plan — otherwise a legitimate doorway could get closing-bridged
    # shut before the explicit door-sealing step below even runs.
    gap_m = junction_gap_m
    if plan.doors:
        gap_m = min(gap_m, 0.6 * min(d.width for d in plan.doors))
    close_px = max(1, int(round(gap_m / resolution))) | 1
    grid = cv2.morphologyEx(grid, cv2.MORPH_CLOSE, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (close_px, close_px)))

    thick_px = max(2, int(round(0.1 / resolution)))
    for door in plan.doors:
        p1 = _to_px(np.array([door.hinge]))[0]
        p2 = _to_px(np.array([door.jamb]))[0]
        cv2.line(grid, tuple(p1), tuple(p2), 255, thick_px)

    if region is not None and region.interior.any():
        # resample the voxel-sealed interior onto this grid: anywhere it says
        # "outside the building" is a barrier, independent of whatever holes
        # the vectorized walls have there
        rows, cols = np.indices((h, w))
        wx = x0 + cols * resolution
        wy = y0 + rows * resolution
        rcol = ((wx - region.origin[0]) / region.resolution).astype(np.int32)
        rrow = ((wy - region.origin[1]) / region.resolution).astype(np.int32)
        ry, rx = region.interior.shape
        in_bounds = (rcol >= 0) & (rcol < rx) & (rrow >= 0) & (rrow < ry)
        is_interior = np.zeros((h, w), dtype=bool)
        is_interior[in_bounds] = region.interior[rrow[in_bounds], rcol[in_bounds]]
        grid[~is_interior] = 255

    free = (grid == 0).astype(np.uint8)
    flood_mask = np.zeros((h + 2, w + 2), np.uint8)
    flooded = free.copy() * 255
    cv2.floodFill(flooded, flood_mask, (0, 0), 128)
    exterior = flooded == 128
    interior_free = free.astype(bool) & ~exterior

    n, labels, stats, centroids = cv2.connectedComponentsWithStats(
        interior_free.astype(np.uint8), connectivity=4
    )
    rooms: list[RoomPolygon] = []
    for i in range(1, n):
        area_m2 = float(stats[i, cv2.CC_STAT_AREA]) * resolution * resolution
        if area_m2 < min_area_m2:
            continue
        component = (labels == i).astype(np.uint8) * 255
        contours, _ = cv2.findContours(component, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            continue
        cnt = max(contours, key=cv2.contourArea)
        approx = cv2.approxPolyDP(cnt, epsilon=2.0, closed=True).reshape(-1, 2).astype(np.float64)
        if len(approx) < 3:
            continue
        world = np.column_stack([x0 + approx[:, 0] * resolution, y0 + approx[:, 1] * resolution])
        cx, cy = centroids[i]
        rooms.append(
            RoomPolygon(
                polygon=np.vstack([world, world[:1]]),
                area_m2=area_m2,
                centroid=(x0 + cx * resolution, y0 + cy * resolution),
            )
        )
    rooms.sort(key=lambda r: r.area_m2, reverse=True)
    return rooms


def _line_intersect(a: list[Any], b: list[Any]) -> tuple[float, float] | None:
    """Intersect two snapped edge descriptors: ["H", y], ["V", x], or ["D", p, dir]."""

    def _as_line(e: list[Any]) -> tuple[np.ndarray, np.ndarray]:
        if e[0] == "H":
            return np.array([0.0, e[1]]), np.array([1.0, 0.0])
        if e[0] == "V":
            return np.array([e[1], 0.0]), np.array([0.0, 1.0])
        return e[1], e[2]

    p1, d1 = _as_line(a)
    p2, d2 = _as_line(b)
    cross = d1[0] * d2[1] - d1[1] * d2[0]
    if abs(cross) < 1e-9:
        return None
    t = ((p2[0] - p1[0]) * d2[1] - (p2[1] - p1[1]) * d2[0]) / cross
    pt = p1 + t * d1
    return float(pt[0]), float(pt[1])


def _orthogonalize(poly: np.ndarray, theta: float, tol_deg: float = 27.0) -> np.ndarray:
    """Square up a boundary polygon: snap near-axis edges to the dominant axes
    and rebuild the corners as clean line intersections. Genuinely diagonal
    facades (further than tol from both axes) are kept as-is."""
    if len(poly) < 3:
        return poly
    c, s = math.cos(theta), math.sin(theta)
    rx = c * poly[:, 0] + s * poly[:, 1]
    ry = -s * poly[:, 0] + c * poly[:, 1]
    pts = np.column_stack([rx, ry])

    desc: list[list[Any]] = []
    n = len(pts)
    for i in range(n):
        p1, p2 = pts[i], pts[(i + 1) % n]
        v = p2 - p1
        length = float(np.hypot(*v))
        if length < 1e-6:
            continue
        ang = math.degrees(math.atan2(v[1], v[0])) % 180.0
        if ang <= tol_deg or ang >= 180.0 - tol_deg:
            desc.append(["H", float((p1[1] + p2[1]) / 2), length])
        elif abs(ang - 90.0) <= tol_deg:
            desc.append(["V", float((p1[0] + p2[0]) / 2), length])
        else:
            desc.append(["D", p1.astype(float), v / length, length])
    if len(desc) < 3:
        return poly

    # merge cyclically-consecutive edges snapped to the same axis
    changed = True
    while changed and len(desc) >= 3:
        changed = False
        for i in range(len(desc)):
            j = (i + 1) % len(desc)
            a, b = desc[i], desc[j]
            if a[0] in ("H", "V") and a[0] == b[0]:
                wa, wb = a[-1], b[-1]
                a[1] = (a[1] * wa + b[1] * wb) / (wa + wb)
                a[-1] = wa + wb
                del desc[j]
                changed = True
                break

    verts = []
    m = len(desc)
    for i in range(m):
        pt = _line_intersect(desc[i], desc[(i + 1) % m])
        if pt is None:
            return poly  # degenerate — keep the un-squared polygon
        verts.append(pt)
    varr = np.array(verts)
    # a near-parallel intersection can fling a corner far away — bail out then
    d2 = ((varr[:, None, :] - pts[None, :, :]) ** 2).sum(axis=-1)
    if float(np.sqrt(d2.min(axis=1)).max()) > 2.0:
        return poly
    wx = c * varr[:, 0] - s * varr[:, 1]
    wy = s * varr[:, 0] + c * varr[:, 1]
    return np.column_stack([wx, wy])


def envelope_walls(
    env_mask: np.ndarray,
    origin: tuple[float, float],
    resolution: float,
    dominant_angle: float,
) -> list[Wall]:
    """Exterior walls from the building-footprint boundary.

    Glass facades barely register in lidar, so the envelope is traced from
    where the ceiling ends instead of from direct wall returns, then squared
    to the building's dominant axes. Drawn thicker than partitions, per
    convention for exterior walls.
    """
    x0, y0 = origin
    # smooth the wobble out of the raster boundary before polygonizing
    k = max(3, int(0.45 / resolution)) | 1
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (k, k))
    smooth = cv2.morphologyEx(
        cv2.morphologyEx(env_mask, cv2.MORPH_CLOSE, kernel), cv2.MORPH_OPEN, kernel
    )
    contours, _ = cv2.findContours(smooth, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)
    walls: list[Wall] = []
    min_area = 20.0 / (resolution * resolution)  # ignore blobs under 20 m²
    for contour in contours:
        if cv2.contourArea(contour) < min_area:
            continue
        approx = cv2.approxPolyDP(contour, epsilon=0.5 / resolution, closed=True)
        poly = approx.reshape(-1, 2).astype(np.float64)
        poly[:, 0] = x0 + poly[:, 0] * resolution
        poly[:, 1] = y0 + poly[:, 1] * resolution
        poly = _orthogonalize(poly, dominant_angle)
        for i in range(len(poly)):
            p1 = (float(poly[i][0]), float(poly[i][1]))
            p2 = (float(poly[(i + 1) % len(poly)][0]), float(poly[(i + 1) % len(poly)][1]))
            if math.hypot(p2[0] - p1[0], p2[1] - p1[1]) < 0.4:
                continue
            walls.append(Wall(p1, p2, thickness=0.22))
    return walls


def _pt_seg_dist(p: tuple[float, float], w: Wall) -> tuple[float, float]:
    """Distance from point to wall segment and the projection parameter t (m along wall)."""
    vx, vy = w.p2[0] - w.p1[0], w.p2[1] - w.p1[1]
    length = math.hypot(vx, vy)
    if length < 1e-9:
        return math.hypot(p[0] - w.p1[0], p[1] - w.p1[1]), 0.0
    t = ((p[0] - w.p1[0]) * vx + (p[1] - w.p1[1]) * vy) / length
    tc = min(max(t, 0.0), length)
    fx, fy = w.p1[0] + vx / length * tc, w.p1[1] + vy / length * tc
    return math.hypot(p[0] - fx, p[1] - fy), t


def tidy_plan(plan: Plan) -> dict[str, int]:
    """Deterministic repair so the plan behaves like a real building.

    - dedupe: two near-parallel runs tracing the same wall → keep the longer
    - snap: an endpoint that almost reaches another wall is extended to meet
      it exactly (closes T-junctions and corners; door gaps along the *same*
      line are preserved — the projection lands outside the sibling's span)
    - orphans: short fragments touching nothing are survey noise, drop them
    """
    stats = {"dedupe": 0, "snap": 0, "orphan": 0}
    walls = plan.walls

    # --- dedupe near-parallel duplicates
    keep: list[Wall] = []
    dropped: set[int] = set()
    order = sorted(range(len(walls)), key=lambda i: -walls[i].length)
    for oi, i in enumerate(order):
        if i in dropped:
            continue
        wi = walls[i]
        ai = math.atan2(wi.p2[1] - wi.p1[1], wi.p2[0] - wi.p1[0])
        for j in order[oi + 1 :]:
            if j in dropped:
                continue
            wj = walls[j]
            aj = math.atan2(wj.p2[1] - wj.p1[1], wj.p2[0] - wj.p1[0])
            dang = abs((ai - aj + math.pi / 2) % math.pi - math.pi / 2)
            if dang > math.radians(8):
                continue
            d1, t1 = _pt_seg_dist(wj.p1, wi)
            d2, t2 = _pt_seg_dist(wj.p2, wi)
            if max(d1, d2) > 0.22:
                continue
            # overlap fraction of the shorter run along the longer one
            lo, hi = sorted((t1, t2))
            overlap = min(hi, wi.length) - max(lo, 0.0)
            if overlap > 0.7 * wj.length:
                dropped.add(j)
                stats["dedupe"] += 1
        keep.append(wi)
    walls = keep

    # --- snap endpoints to nearby walls (T-junctions, corners)
    for w in walls:
        for attr in ("p1", "p2"):
            p = getattr(w, attr)
            best: tuple[float, Wall, float] | None = None
            for other in walls:
                if other is w:
                    continue
                d, t = _pt_seg_dist(p, other)
                if d < 0.45 and -0.3 <= t <= other.length + 0.3:
                    if best is None or d < best[0]:
                        best = (d, other, t)
            if best is not None and best[0] > 0.02:
                _, other, t = best
                tc = min(max(t, 0.0), other.length)
                ux = (other.p2[0] - other.p1[0]) / other.length
                uy = (other.p2[1] - other.p1[1]) / other.length
                setattr(w, attr, (other.p1[0] + ux * tc, other.p1[1] + uy * tc))
                stats["snap"] += 1

    # --- drop orphan fragments
    final: list[Wall] = []
    for w in walls:
        if w.length >= 1.0:
            final.append(w)
            continue
        touches = False
        for other in walls:
            if other is w:
                continue
            if _pt_seg_dist(w.p1, other)[0] < 0.5 or _pt_seg_dist(w.p2, other)[0] < 0.5:
                touches = True
                break
        if touches:
            final.append(w)
        else:
            stats["orphan"] += 1

    plan.walls = final
    return stats


def close_wall_gaps(
    plan: Plan, region: ClosedRegion, max_gap: float = 2.5, tol: float = 0.30
) -> int:
    """Seal the plan's connected-area boundary using the voxel closed loops.

    Walks each room loop and finds spans not explained by existing geometry:
    a loop vertex is *covered* when it lies within `tol` of a wall segment or
    inside a door opening (doors must stay open — the trajectory proved them
    passable). Consecutive uncovered runs shorter than `max_gap` become new
    wall segments, so rooms read sealed on the plan without touching
    vectorize()/carve_passages().
    """
    door_spans: list[tuple[tuple[float, float], tuple[float, float], float]] = []
    for door in plan.doors:
        cx = (door.hinge[0] + door.jamb[0]) / 2
        cy = (door.hinge[1] + door.jamb[1]) / 2
        door_spans.append(((cx, cy), door.normal, door.width / 2 + tol))

    def _covered(p: tuple[float, float]) -> bool:
        for (cx, cy), _n, r in door_spans:
            if math.hypot(p[0] - cx, p[1] - cy) <= r:
                return True
        return any(_pt_seg_dist(p, w)[0] <= tol for w in plan.walls)

    added = 0
    for loop in plan.rooms:
        # resample each loop edge so coverage is tested every ~tol meters
        dense: list[tuple[float, float]] = []
        for a, b in itertools.pairwise(loop):
            seg_len = float(np.hypot(*(b - a)))
            n = max(1, int(seg_len / max(tol, 1e-6)))
            for t in np.linspace(0.0, 1.0, n, endpoint=False):
                pt = a + t * (b - a)
                dense.append((float(pt[0]), float(pt[1])))
        if not dense:
            continue
        flags = [_covered(p) for p in dense]
        if all(flags):
            continue
        # rotate so index 0 is covered (a run never wraps the seam)
        if not all(not f for f in flags):
            first = next(i for i, f in enumerate(flags) if f)
            dense = dense[first:] + dense[:first]
            flags = flags[first:] + flags[:first]
        run: list[tuple[float, float]] = []
        for p, f in zip([*dense, dense[0]], [*flags, True], strict=False):
            if not f:
                run.append(p)
                continue
            if len(run) >= 2:
                gap_len = math.hypot(run[-1][0] - run[0][0], run[-1][1] - run[0][1])
                if 2 * tol < gap_len <= max_gap:
                    plan.walls.append(Wall(p1=run[0], p2=run[-1]))
                    added += 1
            run = []
    return added


def _indoor_barrier(vox: VoxelOccupancy, grids: Grids) -> np.ndarray | None:
    """Not-indoors resampled onto the voxel grid, as a flood barrier.

    Keeps a ground floor's interior flood from escaping through open entrance
    doorways: the ceiling-evidence indoor mask bounds the building even where
    the lidar saw no wall (glass, openings).
    """
    if grids.indoor_mask is None:
        return None
    _, ny, nx = vox.shape
    gh, gw = grids.indoor_mask.shape
    xs = vox.origin[0] + (np.arange(nx) + 0.5) * vox.voxel
    ys = vox.origin[1] + (np.arange(ny) + 0.5) * vox.voxel
    ci = ((xs - grids.origin[0]) / grids.resolution).astype(np.int32)
    ri = ((ys - grids.origin[1]) / grids.resolution).astype(np.int32)
    in_c = (ci >= 0) & (ci < gw)
    in_r = (ri >= 0) & (ri < gh)
    rr_, cc_ = np.meshgrid(ri.clip(0, gh - 1), ci.clip(0, gw - 1), indexing="ij")
    indoor = grids.indoor_mask[rr_, cc_] > 0
    indoor[~in_r, :] = False
    indoor[:, ~in_c] = False
    return ~indoor


def launch_viewer(path: Path) -> bool:
    """Open `path` in dimos-viewer (non-blocking); fall back to stock rerun."""
    try:
        import rerun_bindings

        rerun_bindings.spawn(executable_name="dimos-viewer", extra_args=[str(path)])
        return True
    except Exception as e:
        logger.debug(f"rerun_bindings spawn failed ({e}); trying PATH executables")
    for exe_name in ("dimos-viewer", "rerun"):
        exe = shutil.which(exe_name)
        if exe:
            subprocess.Popen([exe, str(path)])
            return True
    logger.warning(f"No viewer found on PATH — view with: dimos-viewer {path}")
    return False


def detect_stairs(trajectory: np.ndarray, levels: list[Level]) -> dict[int, list[Stair]]:
    """Stairs from the trajectory's climbs between walkable platforms.

    Platforms are the primary floors *plus every mezzanine/sunken area*: walk
    the ordered path, and whenever it leaves one platform band and arrives at
    an adjacent one, the transit vertices trace the stair. Multiple traversals
    of the same stair are clustered. A floor↔floor stair appears on both
    sheets (UP below, DN above); a floor↔mezzanine stair lives on the parent
    level's sheet.
    """
    if len(trajectory) < 3:
        return {}
    # (z, sheet index) for every walkable platform, sorted by elevation
    platforms: list[tuple[float, int]] = [(lv.floor_z, lv.index) for lv in levels]
    for lv in levels:
        platforms.extend((mz.z, lv.index) for mz in lv.mezzanines)
    if len(platforms) < 2:
        return {}
    platforms.sort()
    plat_z = np.array([p[0] for p in platforms])

    def _platform_of(z: float) -> int | None:
        i = int(np.argmin(np.abs(plat_z - z)))
        return i if abs(plat_z[i] - z) < 0.45 else None

    traversals: list[tuple[int, int, np.ndarray]] = []
    last_plat: int | None = None
    transit: list[np.ndarray] = []
    for pt in trajectory:
        p = _platform_of(float(pt[2]))
        if p is None:
            transit.append(pt)
            continue
        if last_plat is not None and p != last_plat and transit:
            if abs(p - last_plat) == 1:  # adjacent platforms in elevation order
                traversals.append((min(p, last_plat), max(p, last_plat), np.array(transit)))
        if p != last_plat or transit:
            transit = []
        last_plat = p

    # cluster traversals of the same stair by midpoint
    stairs: dict[int, list[Stair]] = {}
    used: list[tuple[int, int, np.ndarray]] = []
    for lo_i, hi_i, pts in traversals:
        if len(pts) < 2:
            continue
        mid = pts[:, :2].mean(axis=0)
        if any(lo_i == u[0] and np.hypot(*(mid - u[2][:, :2].mean(axis=0))) < 3.0 for u in used):
            continue
        used.append((lo_i, hi_i, pts))
        # principal direction of the transit path, oriented low z → high z
        xy = pts[:, :2]
        centered = xy - xy.mean(axis=0)
        _, _, vt = np.linalg.svd(centered, full_matrices=False)
        direction = vt[0]
        proj = centered @ direction
        p_lo = (
            xy[int(np.argmin(proj))]
            if pts[int(np.argmin(proj)), 2] < pts[int(np.argmax(proj)), 2]
            else xy[int(np.argmax(proj))]
        )
        p_hi = (
            xy[int(np.argmax(proj))]
            if not np.array_equal(p_lo, xy[int(np.argmax(proj))])
            else xy[int(np.argmin(proj))]
        )
        lo_sheet = platforms[lo_i][1]
        hi_sheet = platforms[hi_i][1]
        stairs.setdefault(lo_sheet, []).append(Stair(p1=tuple(p_lo), p2=tuple(p_hi), label="UP"))
        if hi_sheet != lo_sheet:
            stairs.setdefault(hi_sheet, []).append(
                Stair(p1=tuple(p_lo), p2=tuple(p_hi), label="DN")
            )
    return stairs


def classify_glazing(env_walls: list[Wall], grids: Grids) -> None:
    """Envelope runs with almost no direct lidar returns are glass.

    A solid wall reflects the lidar along its whole length; glass lets most
    beams through, leaving only mullion/frame returns. Sample the occupancy
    mask along each envelope centerline and flag sparse runs as glazed.
    """
    res = grids.resolution
    x0, y0 = grids.origin
    h, w = grids.all_mask.shape
    occupied = cv2.dilate(grids.all_mask, cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5)))
    for wall in env_walls:
        n = max(int(wall.length / 0.15), 2)
        ts = np.linspace(0.0, 1.0, n)
        xs = wall.p1[0] + (wall.p2[0] - wall.p1[0]) * ts
        ys = wall.p1[1] + (wall.p2[1] - wall.p1[1]) * ts
        cols = ((xs - x0) / res).astype(np.int32).clip(0, w - 1)
        rows = ((ys - y0) / res).astype(np.int32).clip(0, h - 1)
        coverage = float((occupied[rows, cols] > 0).mean())
        wall.glazed = coverage < 0.45


class SessionImagery:
    """Camera frames from a recording session, queryable by what they look at.

    Correlates the stamped trajectory (`gtsam_odom.tum`) with the recorded
    `color_image` stream (`mem2.db`) so ambiguous plan geometry can be
    visually inspected: give it a wall midpoint, get back the frame where the
    robot stood nearby facing that point.
    """

    def __init__(self, session_dir: Path) -> None:
        self.session_dir = session_dir
        tum = session_dir / "gtsam_odom.tum"
        self.db_path = session_dir / "mem2.db"
        if not tum.exists() or not self.db_path.exists():
            raise FileNotFoundError(f"need gtsam_odom.tum and mem2.db in {session_dir}")
        data = np.loadtxt(str(tum))  # t x y z qx qy qz qw
        qx, qy, qz, qw = data[:, 4], data[:, 5], data[:, 6], data[:, 7]
        self.t = data[:, 0]
        self.xyz = data[:, 1:4]
        self.yaw = np.arctan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))
        self._stream: Any = None
        self._store: Any = None

    def _images(self) -> Any:  # lazy: the store is 10 GB, open once
        if self._stream is None:
            from dimos.memory2.store.sqlite import SqliteStore

            self._store = SqliteStore(path=str(self.db_path), must_exist=True)
            self._store.start()
            self._stream = self._store.replay().stream("color_image")
        return self._stream

    def close(self) -> None:
        """Release the recording store (a multi-GB sqlite handle)."""
        if self._store is not None:
            try:
                self._store.stop()
            except Exception:
                pass
            self._store = None
            self._stream = None

    def best_view(
        self, target: tuple[float, float], floor_z: float
    ) -> tuple[np.ndarray, float] | None:
        """Frame taken 1.5-7 m from `target`, on its level, facing it head-on."""
        dx = target[0] - self.xyz[:, 0]
        dy = target[1] - self.xyz[:, 1]
        dist = np.hypot(dx, dy)
        bearing = np.arctan2(dy, dx)
        off_axis = np.abs((bearing - self.yaw + math.pi) % (2 * math.pi) - math.pi)
        ok = (
            (dist >= 1.5)
            & (dist <= 7.0)
            & (off_axis < math.radians(35))
            & (np.abs(self.xyz[:, 2] - floor_z) < 1.6)
        )
        if not ok.any():
            return None
        score = off_axis + 0.05 * dist
        score[~ok] = np.inf
        best = int(np.argmin(score))
        img = self._images().find_closest(float(self.t[best]), tolerance=0.6)
        if img is None:
            return None
        rgb = np.asarray(img.to_rgb().data)
        return rgb, float(self.t[best])


def _ask_surfaces(images: list[np.ndarray], save_to: list[Path]) -> list[str]:
    """Vision model: what is the surface ahead — glass, solid, or open — for each

    photo, numbered in order. One batched call instead of one per photo: the
    model reads all crops at once and returns a same-length verdict array.
    """
    b64s = []
    for rgb, path in zip(images, save_to):
        small = cv2.resize(rgb, (896, int(896 * rgb.shape[0] / rgb.shape[1])))
        cv2.imwrite(str(path), cv2.cvtColor(small, cv2.COLOR_RGB2BGR))
        b64s.append(base64.b64encode(path.read_bytes()).decode())

    verdict = _openai_json(
        "A ground robot's lidar got almost no returns from the wall area directly "
        "ahead in each of these photos (the camera faces it), numbered in order "
        "starting at 0. For an architectural floor plan, classify each surface: "
        '"glass" (glass door / curtain wall / large window), "solid" (an ordinary '
        'opaque wall — the lidar gap is a data artifact), or "open" (no wall: an '
        'open passage or missing geometry). Reply JSON: {"surfaces": '
        '[{"index": 0, "surface": "glass|solid|open", "detail": "<a few words>"}, ...]} '
        "with exactly one entry per photo.",
        b64s,
        tier="fast",
        timeout=90,
    )
    results = ["unknown"] * len(images)
    if verdict is None:
        return results
    for entry in verdict.get("surfaces", []):
        if not isinstance(entry, dict):
            continue
        idx = entry.get("index")
        if isinstance(idx, int) and 0 <= idx < len(results):
            results[idx] = str(entry.get("surface", "unknown"))
            logger.info(
                "Visual inspection",
                surface=results[idx],
                detail=entry.get("detail", ""),
                image=str(save_to[idx]),
            )
    return results


def visually_confirm_glazing(
    env_walls: list[Wall],
    session: SessionImagery,
    level: Level,
    out: Path,
    max_queries: int = 4,
) -> list[Wall]:
    """Check the longest glazed-candidate runs against the session camera, in one call.

    glass → keep glazed; solid → draw as normal wall; open → remove the
    segment (there is no wall there at all).
    """
    candidates = sorted((w for w in env_walls if w.glazed), key=lambda w: -w.length)[:max_queries]
    views: list[tuple[Wall, np.ndarray]] = []
    for wall in candidates:
        mid = ((wall.p1[0] + wall.p2[0]) / 2, (wall.p1[1] + wall.p2[1]) / 2)
        view = session.best_view(mid, level.floor_z)
        if view is not None:
            views.append((wall, view[0]))
    if not views:
        return env_walls

    save_to = [out.with_suffix(f".inspect-L{level.index}-{i}.jpg") for i in range(len(views))]
    surfaces = _ask_surfaces([rgb for _, rgb in views], save_to)

    drop: set[int] = set()
    for (wall, _rgb), surface in zip(views, surfaces):
        if surface == "solid":
            wall.glazed = False
        elif surface == "open":
            drop.add(id(wall))
    return [w for w in env_walls if id(w) not in drop]


BLUEPRINT_STYLES = {
    "drafted": (
        "a hand-drafted architectural floor plan in ink on aged vellum: fine ruled "
        "linework, hand-lettered labels and dimensions, subtle paper texture, drafting "
        "smudges, title block preserved"
    ),
    "cyanotype": (
        "a classic cyanotype blueprint: crisp white linework on deep Prussian-blue paper "
        "with slight print grain and faded edges, exactly as a 1950s reprographic copy"
    ),
    "presentation": (
        "a modern architectural presentation plan: clean white background, solid black "
        "poché walls, warm light-grey floor fills, subtle drop shadows, elegant thin "
        "annotations — the style of a contemporary architecture firm's portfolio"
    ),
}


def _best_image_model(client: Any) -> str:
    if "image" in _model_cache:
        return _model_cache["image"]
    choice = "gpt-image-1"
    try:
        available = sorted(
            m.id
            for m in client.models.list().data
            if m.id.startswith("gpt-image") and "mini" not in m.id
        )
        if available:
            choice = available[-1]
    except Exception:
        pass
    _model_cache["image"] = choice
    logger.info(f"OpenAI model (image): {choice}")
    return choice


def generate_blueprint_renditions(
    sheet_jpgs: list[tuple[str, Path]], out: Path, styles: list[str]
) -> list[Path]:
    """Stylized versions of the finished sheet(s) via the OpenAI image model.

    Uses images.edit so the model restyles the actual drawing — wall layout,
    openings, labels — instead of hallucinating a new building. Unknown style
    names are passed through as custom prompts.

    `sheet_jpgs` is a list of (label, source_jpg) pairs — one full-resolution
    sheet per level rather than one cramped combined multi-level sheet, so
    each level's rendition actually resolves its detail (label="" and a
    single entry for single-level buildings, where the combined sheet already
    is the per-level sheet). Every (level, style) pair is an independent
    request, run concurrently — sequential images.edit calls (each up to
    300s) would otherwise stack to minutes per style, times levels.
    """
    try:
        from openai import OpenAI
    except ImportError:
        logger.warning("openai package not installed — skipping renditions")
        return []

    model = _best_image_model(OpenAI(max_retries=0))

    def _render_one(label: str, sheet_jpg: Path, style: str) -> Path | None:
        look = BLUEPRINT_STYLES.get(style, style)
        prompt = (
            "Redraw this architectural floor plan drawing as "
            f"{look}. Preserve the exact wall layout, door openings and swing "
            "arcs, stairs, dimension strings, north arrows, level labels and "
            "title block text — change only the rendering style."
        )
        slug = "".join(c if c.isalnum() else "-" for c in style.lower())[:24].strip("-")
        suffix = f"-{label}" if label else ""
        target = out.parent / f"{out.name}.render-{slug}{suffix}.png"
        tag = f"style: {style}" + (f", {label}" if label else "")
        try:
            with open(sheet_jpg, "rb") as f:
                result = OpenAI(max_retries=0).images.edit(
                    model=model,
                    image=f,
                    prompt=prompt,
                    size="1536x1024",
                    timeout=300,
                )
            b64 = result.data[0].b64_json  # type: ignore[index]
            if not b64:
                raise RuntimeError("image model returned no data")
            target.write_bytes(base64.b64decode(b64))
            logger.info(f"Wrote {target} ({tag})")
            return target
        except Exception as e:
            logger.warning(f"rendition failed ({tag}): {e}")
            return None

    jobs = [(label, jpg, style) for label, jpg in sheet_jpgs for style in styles]
    # Capped, not len(jobs)-wide: a many-level building times several styles
    # shouldn't fire an unbounded burst of concurrent OpenAI requests. Submission
    # (not execution) is staggered by 1s so a many-job batch doesn't slam the API
    # in the same instant -- still concurrent, just not a simultaneous burst.
    with concurrent.futures.ThreadPoolExecutor(max_workers=min(len(jobs), 12) or 1) as pool:
        futures = []
        for i, job in enumerate(jobs):
            if i > 0:
                time.sleep(1.0)
            futures.append(pool.submit(lambda j=job: _render_one(*j)))
        results = [f.result() for f in futures]
    return [r for r in results if r is not None]


def plan_needs_cleaning(plan: Plan) -> tuple[bool, dict[str, int]]:
    """Measure plan defects; the expensive AI cleaning stage runs only if needed.

    Counted defects: dangling endpoints (a wall end touching nothing — real
    walls meet other walls), short orphan fragments, and duplicated
    near-parallel runs that tidy_plan's stricter thresholds left behind.
    """
    walls = plan.walls
    issues = {"dangling": 0, "fragments": 0, "duplicates": 0}
    for i, w in enumerate(walls):
        if w.length < 0.8:
            issues["fragments"] += 1
        for attr in ("p1", "p2"):
            p = getattr(w, attr)
            if all(_pt_seg_dist(p, other)[0] > 0.4 for other in walls if other is not w):
                issues["dangling"] += 1
        ai_ang = math.atan2(w.p2[1] - w.p1[1], w.p2[0] - w.p1[0])
        for other in walls[i + 1 :]:
            dang = abs(
                (
                    ai_ang
                    - math.atan2(other.p2[1] - other.p1[1], other.p2[0] - other.p1[0])
                    + math.pi / 2
                )
                % math.pi
                - math.pi / 2
            )
            if dang > math.radians(8):
                continue
            d1, t1 = _pt_seg_dist(other.p1, w)
            d2, t2 = _pt_seg_dist(other.p2, w)
            if max(d1, d2) < 0.3:
                lo, hi = sorted((t1, t2))
                if min(hi, w.length) - max(lo, 0.0) > 0.5 * other.length:
                    issues["duplicates"] += 1

    n_endpoints = max(2 * len(walls), 1)
    needs = (
        issues["dangling"] / n_endpoints > 0.15
        or issues["fragments"] > 6
        or issues["duplicates"] > 2
    )
    return needs, issues


def _render_review_image(sheet: LevelSheet, path: Path) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    x_lo, y_lo, x_hi, y_hi = sheet.plan.bounds()
    span = max(x_hi - x_lo, y_hi - y_lo) + 3.0
    fig, ax = plt.subplots(figsize=(11, 11), facecolor=BG)
    _draw_level(ax, sheet, span)
    for i, wall in enumerate(sheet.plan.walls):
        mx = (wall.p1[0] + wall.p2[0]) / 2
        my = (wall.p1[1] + wall.p2[1]) / 2
        ax.text(
            mx,
            my,
            str(i),
            color="#ffd166",
            fontsize=6,
            family="monospace",
            ha="center",
            va="center",
            zorder=9,
        )
    fig.tight_layout()
    fig.savefig(path, dpi=140, facecolor=BG, format="jpeg")
    plt.close(fig)


def critique_plan(sheet: LevelSheet, out: Path) -> None:
    """Final architectural review by the strongest available OpenAI model.

    The model sees the rendered level (walls labeled by id) plus the raw
    geometry and proposes repairs through a constrained op set; every op is
    re-validated geometrically before it is applied.
    """
    plan = sheet.plan
    img_path = out.with_suffix(f".review-L{sheet.level.index}.jpg")
    _render_review_image(sheet, img_path)
    b64 = base64.b64encode(img_path.read_bytes()).decode()

    walls_json = [
        {
            "id": i,
            "p1": [round(w.p1[0], 2), round(w.p1[1], 2)],
            "p2": [round(w.p2[0], 2), round(w.p2[1], 2)],
            "len_m": round(w.length, 2),
            "glazed": w.glazed,
        }
        for i, w in enumerate(plan.walls)
    ]
    prompt = (
        f"You are an architect reviewing an auto-vectorized floor plan of "
        f"{sheet.level.name} (from a lidar scan; image attached, walls labeled by id; "
        "solid walls have poché, glazing is thin double lines, door arcs shown, stair "
        "treads shown). Real buildings have continuous walls that meet at junctions and "
        "enclose rooms; scan artifacts show up as stray fragments, duplicated parallel "
        "runs, or fragments crossing open space diagonally for no reason.\n\n"
        "Wall geometry (meters):\n"
        + json.dumps(walls_json)
        + "\n\nPropose corrections using ONLY these ops:\n"
        '  {"op":"delete_wall","id":N}            — spurious fragment / artifact\n'
        '  {"op":"merge","ids":[A,B]}             — same wall split in two, join them\n'
        '  {"op":"extend","id":A,"target":B}      — wall A should meet wall B (close gap)\n'
        '  {"op":"set_glazed","id":A,"glazed":true|false}\n'
        "Be conservative: when unsure, leave geometry alone. Never delete long envelope "
        "runs. At most 25 ops.\n"
        'Reply JSON: {"actions":[...], "assessment":"<one sentence>"}'
    )
    verdict = _openai_json(prompt, b64, tier="max", timeout=420)
    if verdict is None:
        logger.warning("plan critique unavailable — keeping plan as-is")
        return

    actions = list(verdict.get("actions", []))[:25]
    applied = {"delete_wall": 0, "merge": 0, "extend": 0, "set_glazed": 0, "rejected": 0}
    to_delete: set[int] = set()

    def _wall(i: Any) -> Wall | None:
        return plan.walls[i] if isinstance(i, int) and 0 <= i < len(plan.walls) else None

    for act in actions:
        op = act.get("op")
        if op == "delete_wall":
            w = _wall(act.get("id"))
            if w is not None and w.length < 12.0:
                to_delete.add(act["id"])
                applied[op] += 1
            else:
                applied["rejected"] += 1
        elif op == "merge":
            ids = act.get("ids", [])
            a = _wall(ids[0]) if len(ids) == 2 else None
            b = _wall(ids[1]) if len(ids) == 2 else None
            if a is None or b is None or ids[0] in to_delete or ids[1] in to_delete:
                applied["rejected"] += 1
                continue
            ang_a = math.atan2(a.p2[1] - a.p1[1], a.p2[0] - a.p1[0])
            ang_b = math.atan2(b.p2[1] - b.p1[1], b.p2[0] - b.p1[0])
            dang = abs((ang_a - ang_b + math.pi / 2) % math.pi - math.pi / 2)
            d1, _ = _pt_seg_dist(b.p1, a)
            d2, _ = _pt_seg_dist(b.p2, a)
            if dang < math.radians(12) and min(d1, d2) < 0.5:
                # span both along a's direction
                ux, uy = math.cos(ang_a), math.sin(ang_a)
                pts = [a.p1, a.p2, b.p1, b.p2]
                ts = [(p[0] - a.p1[0]) * ux + (p[1] - a.p1[1]) * uy for p in pts]
                lo, hi = min(ts), max(ts)
                a.p1, a.p2 = (
                    (a.p1[0] + ux * lo, a.p1[1] + uy * lo),
                    (a.p1[0] + ux * hi, a.p1[1] + uy * hi),
                )
                to_delete.add(ids[1])
                applied[op] += 1
            else:
                applied["rejected"] += 1
        elif op == "extend":
            w = _wall(act.get("id"))
            t = _wall(act.get("target"))
            if w is None or t is None:
                applied["rejected"] += 1
                continue
            d1, tp1 = _pt_seg_dist(w.p1, t)
            d2, tp2 = _pt_seg_dist(w.p2, t)
            attr, d, tp = ("p1", d1, tp1) if d1 <= d2 else ("p2", d2, tp2)
            if 0.02 < d <= 1.5:
                tc = min(max(tp, 0.0), t.length)
                ux = (t.p2[0] - t.p1[0]) / t.length
                uy = (t.p2[1] - t.p1[1]) / t.length
                setattr(w, attr, (t.p1[0] + ux * tc, t.p1[1] + uy * tc))
                applied[op] += 1
            else:
                applied["rejected"] += 1
        elif op == "set_glazed":
            w = _wall(act.get("id"))
            if w is not None and isinstance(act.get("glazed"), bool):
                w.glazed = act["glazed"]
                applied[op] += 1
            else:
                applied["rejected"] += 1
        else:
            applied["rejected"] += 1

    plan.walls = [w for i, w in enumerate(plan.walls) if i not in to_delete]
    logger.info(
        f"{sheet.level.name} critique: {verdict.get('assessment', '')}",
        **{k: v for k, v in applied.items() if v},
    )


def _seg_cross(
    p1: tuple[float, float], p2: tuple[float, float], a: np.ndarray, b: np.ndarray
) -> float | None:
    """Return the parameter t in [0,1] along p1→p2 where segment a→b crosses it."""
    r = (p2[0] - p1[0], p2[1] - p1[1])
    q = (b[0] - a[0], b[1] - a[1])
    cross = r[0] * q[1] - r[1] * q[0]
    if abs(cross) < 1e-9:
        return None
    dx, dy = a[0] - p1[0], a[1] - p1[1]
    t = (dx * q[1] - dy * q[0]) / cross
    u = (dx * r[1] - dy * r[0]) / cross
    if 0.0 <= t <= 1.0 and 0.0 <= u <= 1.0:
        return float(t)
    return None


def carve_passages(plan: Plan, traj_xy: np.ndarray | None) -> int:
    """Cut door openings wherever the robot's path crosses a drawn wall.

    The trajectory is ground truth for passable openings: the robot physically
    drove through, so a wall cannot be solid there. Openings get swing arcs;
    envelope crossings (thick walls) are cut wider as entrances.
    """
    if traj_xy is None or len(traj_xy) < 2:
        return 0
    segs = [
        (traj_xy[i], traj_xy[i + 1])
        for i in range(len(traj_xy) - 1)
        if np.hypot(*(traj_xy[i + 1] - traj_xy[i])) < 5.0  # skip resets/teleports
    ]

    carved = 0
    new_walls: list[Wall] = []
    for wall in plan.walls:
        length = wall.length
        if length < 0.7:
            new_walls.append(wall)
            continue
        crossings = sorted(
            t * length for a, b in segs if (t := _seg_cross(wall.p1, wall.p2, a, b)) is not None
        )
        if not crossings:
            new_walls.append(wall)
            continue

        clusters: list[list[float]] = [[crossings[0]]]
        for cv in crossings[1:]:
            if cv - clusters[-1][-1] <= 1.0:
                clusters[-1].append(cv)
            else:
                clusters.append([cv])

        door_w = 1.5 if wall.thickness >= 0.2 else 0.9  # entrance vs interior door
        ux = (wall.p2[0] - wall.p1[0]) / length
        uy = (wall.p2[1] - wall.p1[1]) / length

        def _at(
            t: float,
            _p1: tuple[float, float] = wall.p1,
            _ux: float = ux,
            _uy: float = uy,
        ) -> tuple[float, float]:
            return (_p1[0] + _ux * t, _p1[1] + _uy * t)

        cuts: list[tuple[float, float]] = []
        for cluster in clusters:
            center = sum(cluster) / len(cluster)
            lo = max(center - door_w / 2, 0.0)
            hi = min(center + door_w / 2, length)
            if hi - lo >= 0.4:
                cuts.append((lo, hi))
        if not cuts:
            new_walls.append(wall)
            continue

        cursor = 0.0
        for lo, hi in cuts:
            if lo - cursor >= 0.35:
                new_walls.append(Wall(_at(cursor), _at(lo), wall.thickness, wall.glazed))
            plan.doors.append(
                Door(hinge=_at(lo), jamb=_at(hi), normal=(-uy, ux), glazed=wall.glazed)
            )
            carved += 1
            cursor = hi
        if length - cursor >= 0.35:
            new_walls.append(Wall(_at(cursor), _at(length), wall.thickness, wall.glazed))

    plan.walls = new_walls
    return carved


def furniture_outlines(movables: list[Cluster], grids: Grids, drop: set[str]) -> list[np.ndarray]:
    """Simplified outlines of movable clusters — drawn thin per drafting convention."""
    polys: list[np.ndarray] = []
    res = grids.resolution
    x0, y0 = grids.origin
    for cluster in movables:
        if cluster.label in drop:
            continue
        contours, _ = cv2.findContours(cluster.mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for contour in contours:
            if cv2.contourArea(contour) < (0.12 / res) ** 2:
                continue
            approx = cv2.approxPolyDP(contour, epsilon=max(1.0, 0.04 / res), closed=True)
            poly = approx.reshape(-1, 2).astype(np.float64)
            poly[:, 0] = x0 + poly[:, 0] * res
            poly[:, 1] = y0 + poly[:, 1] * res
            polys.append(poly)
    return polys


def _dominant_angle(mask: np.ndarray, resolution: float) -> float:
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
    edges = cv2.morphologyEx(mask, cv2.MORPH_GRADIENT, kernel)
    lines = cv2.HoughLinesP(
        edges,
        rho=1,
        theta=np.pi / 180,
        threshold=20,
        minLineLength=int(MIN_WALL_RUN / resolution),
        maxLineGap=int(0.2 / resolution),
    )
    if lines is None:
        return 0.0
    bins = np.zeros(90)
    for c1, r1, c2, r2 in lines[:, 0]:
        length = math.hypot(c2 - c1, r2 - r1)
        angle = math.degrees(math.atan2(r2 - r1, c2 - c1)) % 90.0
        bins[int(angle) % 90] += length
    # smooth circularly over 3 bins, pick mode
    smoothed = bins + np.roll(bins, 1) + np.roll(bins, -1)
    theta = float(np.argmax(smoothed))
    if theta > 45:
        theta -= 90
    return math.radians(theta)


def _merge_intervals(
    intervals: list[tuple[float, float, float]],
) -> tuple[list[tuple[float, float, float]], list[tuple[float, float]]]:
    """Merge [start, end, thickness] intervals along one line; return runs + door gaps."""
    if not intervals:
        return [], []
    intervals = sorted(intervals)
    runs = [list(intervals[0])]
    doors: list[tuple[float, float]] = []
    for start, end, t in intervals[1:]:
        gap = start - runs[-1][1]
        if gap <= MERGE_GAP:
            runs[-1][1] = max(runs[-1][1], end)
            runs[-1][2] = max(runs[-1][2], t)
        else:
            if DOOR_GAP[0] <= gap <= DOOR_GAP[1] and runs[-1][1] - runs[-1][0] >= 0.4:
                doors.append((runs[-1][1], start))
            runs.append([start, end, t])
    return [tuple(r) for r in runs], doors  # type: ignore[misc]


def vectorize(wall_mask: np.ndarray, origin: tuple[float, float], resolution: float) -> Plan:
    """Rectify the wall mask to its dominant axis and extract wall runs + doors."""
    x0, y0 = origin
    ys, xs = np.nonzero(wall_mask)
    if len(xs) == 0:
        return Plan()

    theta = _dominant_angle(wall_mask, resolution)
    c, s = math.cos(theta), math.sin(theta)

    # rotate cell centers by -theta into the rectified frame
    wx = x0 + xs * resolution
    wy = y0 + ys * resolution
    rx = c * wx + s * wy
    ry = -s * wx + c * wy

    rx0, ry0 = rx.min() - 0.3, ry.min() - 0.3
    rw = math.ceil((rx.max() - rx0 + 0.3) / resolution)
    rh = math.ceil((ry.max() - ry0 + 0.3) / resolution)
    rmask = np.zeros((rh, rw), dtype=np.uint8)
    rmask[
        ((ry - ry0) / resolution).astype(np.int32).clip(0, rh - 1),
        ((rx - rx0) / resolution).astype(np.int32).clip(0, rw - 1),
    ] = 255
    rmask = cv2.morphologyEx(rmask, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))

    def _unrotate(px: float, py: float) -> tuple[float, float]:
        return (c * px - s * py, s * px + c * py)

    plan = Plan(dominant_angle=theta)
    klen = int(0.6 / resolution)
    bridge = max(3, int(0.25 / resolution))  # bridge sparse sampling along the wall
    axis_strips: list[np.ndarray] = []

    for axis in ("h", "v"):
        if axis == "h":
            close_k = cv2.getStructuringElement(cv2.MORPH_RECT, (bridge, 1))
            open_k = cv2.getStructuringElement(cv2.MORPH_RECT, (klen, 1))
        else:
            close_k = cv2.getStructuringElement(cv2.MORPH_RECT, (1, bridge))
            open_k = cv2.getStructuringElement(cv2.MORPH_RECT, (1, klen))
        # close along the wall direction first: far walls are sparsely sampled
        # (short of the 0.25 m door-gap floor, so openings survive)
        bridged = cv2.morphologyEx(rmask, cv2.MORPH_CLOSE, close_k)
        strips = cv2.morphologyEx(bridged, cv2.MORPH_OPEN, open_k)
        axis_strips.append(strips)
        n, labels = cv2.connectedComponents(strips)

        # collect strip segments, then group collinear ones by offset proximity
        segments: list[tuple[float, float, float, float]] = []  # offset, lo, hi, thickness
        for i in range(1, n):
            sy, sx = np.nonzero(labels == i)
            if axis == "h":
                offset = ry0 + sy.mean() * resolution
                lo, hi = rx0 + sx.min() * resolution, rx0 + sx.max() * resolution
                thickness = (sy.max() - sy.min() + 1) * resolution
            else:
                offset = rx0 + sx.mean() * resolution
                lo, hi = ry0 + sy.min() * resolution, ry0 + sy.max() * resolution
                thickness = (sx.max() - sx.min() + 1) * resolution
            if hi - lo < MIN_WALL_RUN:
                continue
            segments.append((offset, lo, hi, thickness))

        segments.sort()
        groups: list[list[tuple[float, float, float, float]]] = []
        for seg in segments:
            if groups and seg[0] - groups[-1][-1][0] <= 0.18:
                groups[-1].append(seg)
            else:
                groups.append([seg])

        for group in groups:
            lengths = [hi - lo for _, lo, hi, _ in group]
            offset = sum(o * ln for (o, _, _, _), ln in zip(group, lengths, strict=False)) / sum(
                lengths
            )
            runs, gaps = _merge_intervals([(lo, hi, t) for _, lo, hi, t in group])
            for lo, hi, thickness in runs:
                thickness = min(max(thickness, 0.10), 0.40)
                if axis == "h":
                    p1, p2 = _unrotate(lo, offset), _unrotate(hi, offset)
                else:
                    p1, p2 = _unrotate(offset, lo), _unrotate(offset, hi)
                plan.walls.append(Wall(p1, p2, thickness))
            for g_lo, g_hi in gaps:
                if axis == "h":
                    hinge, jamb = _unrotate(g_lo, offset), _unrotate(g_hi, offset)
                    normal = _unrotate(0, 1)
                else:
                    hinge, jamb = _unrotate(offset, g_lo), _unrotate(offset, g_hi)
                    normal = _unrotate(1, 0)
                plan.doors.append(Door(hinge, jamb, normal))

    # off-axis walls: whatever the axis-aligned pass didn't claim
    claimed = cv2.dilate(
        cv2.bitwise_or(axis_strips[0], axis_strips[1]),
        cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5)),
    )
    leftover = cv2.bitwise_and(rmask, cv2.bitwise_not(claimed))
    lines = cv2.HoughLinesP(
        leftover,
        rho=1,
        theta=np.pi / 180,
        threshold=25,
        minLineLength=int(MIN_WALL_RUN / resolution),
        maxLineGap=int(0.3 / resolution),
    )
    if lines is not None:
        candidates = sorted(
            (
                (
                    float(np.hypot(c2 - c1, r2 - r1)) * resolution,
                    (rx0 + c1 * resolution, ry0 + r1 * resolution),
                    (rx0 + c2 * resolution, ry0 + r2 * resolution),
                )
                for c1, r1, c2, r2 in lines[:, 0]
            ),
            reverse=True,
        )
        accepted: list[tuple[tuple[float, float], tuple[float, float]]] = []
        for _length, q1, q2 in candidates:
            mid = ((q1[0] + q2[0]) / 2, (q1[1] + q2[1]) / 2)
            dup = False
            for a1, a2 in accepted:
                amid = ((a1[0] + a2[0]) / 2, (a1[1] + a2[1]) / 2)
                if math.hypot(mid[0] - amid[0], mid[1] - amid[1]) < 0.5:
                    dup = True
                    break
            if dup:
                continue
            accepted.append((q1, q2))
            plan.walls.append(Wall(_unrotate(*q1), _unrotate(*q2)))

    return plan


# --------------------------------------------------------------------------- #
# OpenAI review passes (classification, surfaces, final plan critique)
# --------------------------------------------------------------------------- #

# Numbered flagship generations, newest first. "max" prefers the -pro variant
# (deep reasoning, Responses-API only); "fast" uses the plain flagship for the
# quicker vision checks. Specialized variants (codex/mini/nano/chat/search)
# are never picked.
_GENERATIONS = ("gpt-5.5", "gpt-5.4", "gpt-5.2", "gpt-5.1", "gpt-5", "gpt-4.1", "gpt-4o")
_model_cache: dict[str, str] = {}
_model_cache_lock = threading.Lock()  # levels/sheets now query this concurrently


def _best_model(client: Any, tier: str = "fast") -> str:
    """Most capable vision model this key can access for the given tier."""
    if tier in _model_cache:
        return _model_cache[tier]
    with _model_cache_lock:
        if tier in _model_cache:  # another thread populated it while we waited
            return _model_cache[tier]
        choice = "gpt-4o"
        try:
            available = {m.id for m in client.models.list().data}

            def _pick(suffix: str) -> str | None:
                for gen in _GENERATIONS:
                    if gen + suffix in available:
                        return gen + suffix
                return None

            if tier == "max":
                choice = _pick("-pro") or _pick("") or choice
            else:
                choice = _pick("") or choice
        except Exception:
            pass
        _model_cache[tier] = choice
        logger.info(f"OpenAI model ({tier}): {choice}")
        return choice


def _openai_json(
    prompt: str,
    image_b64: str | list[str] | None = None,
    *,
    tier: str = "fast",
    timeout: float = 120.0,
    mime: str = "jpeg",
) -> dict[str, Any] | None:
    """One JSON-answering multimodal call; Responses API with chat fallback.

    image_b64 may be a single image or a list — batching several images into
    one call (e.g. multiple wall-glazing crops) trades a few extra prompt
    tokens for N-1 fewer round trips.
    """
    try:
        from openai import OpenAI
    except ImportError:
        logger.warning("openai package not installed — skipping AI pass")
        return None

    images = [image_b64] if isinstance(image_b64, str) else (image_b64 or [])

    client = OpenAI(max_retries=0)  # a slow deep-reasoning call must not triple itself
    blocks: list[dict[str, Any]] = [{"type": "input_text", "text": prompt}]
    for img in images:
        blocks.append({"type": "input_image", "image_url": f"data:image/{mime};base64,{img}"})

    # pro-tier models think for minutes; if the bounded attempt times out,
    # degrade to the plain flagship rather than stalling the whole pipeline
    models = [_best_model(client, tier)]
    if tier == "max" and _best_model(client, "fast") not in models:
        models.append(_best_model(client, "fast"))

    text: str | None = None
    errors: list[str] = []
    for model in models:
        try:
            response = client.responses.create(
                model=model, input=[{"role": "user", "content": blocks}], timeout=timeout  # type: ignore[misc, list-item]
            )
            text = response.output_text
            break
        except Exception as e:
            errors.append(f"{model}: {e}")
    if text is None:
        try:  # legacy chat endpoint as the last resort
            content: list[dict[str, Any]] = [{"type": "text", "text": prompt}]
            for img in images:
                content.append(
                    {
                        "type": "image_url",
                        "image_url": {"url": f"data:image/{mime};base64,{img}"},
                    }
                )
            response = client.chat.completions.create(  # type: ignore[call-overload]
                model=models[-1],
                response_format={"type": "json_object"},
                timeout=timeout,
                messages=[{"role": "user", "content": content}],
            )
            text = response.choices[0].message.content
        except Exception as e:
            errors.append(f"chat: {e}")
            logger.warning(f"OpenAI call failed: {'; '.join(str(x)[:120] for x in errors)}")
            return None

    if not text:
        return None
    # models sometimes wrap JSON in a code fence or prose — extract the object
    start, end = text.find("{"), text.rfind("}")
    if start == -1 or end <= start:
        return None
    try:
        return dict(json.loads(text[start : end + 1]))
    except json.JSONDecodeError:
        return None


def render_debug_image(
    grids: Grids, walls: list[Cluster], movables: list[Cluster], path: Path
) -> None:
    h, w = grids.all_mask.shape
    img: np.ndarray = np.full((h, w, 3), 40, dtype=np.uint8)
    for cluster in walls:
        img[cluster.mask > 0] = (60, 60, 230)  # red-ish (BGR)
    for cluster in movables:
        img[cluster.mask > 0] = (230, 200, 60)  # cyan-ish
    img = cv2.flip(img, 0)  # world +y up
    scale = max(1, int(900 / max(h, w)))
    img = cv2.resize(img, (w * scale, h * scale), interpolation=cv2.INTER_NEAREST)
    res = grids.resolution
    x0, y0 = grids.origin
    for cluster in walls + movables:
        cx = int((cluster.centroid[0] - x0) / res) * scale
        cy = (h - 1 - int((cluster.centroid[1] - y0) / res)) * scale
        cv2.putText(img, cluster.label, (cx, cy), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    cv2.imwrite(str(path), img)


def ai_review(
    grids: Grids, walls: list[Cluster], movables: list[Cluster], debug_path: Path
) -> dict[str, list[str]]:
    """Vision-model sanity check of the wall/movable split. Returns label flips."""
    render_debug_image(grids, walls, movables, debug_path)
    b64 = base64.b64encode(debug_path.read_bytes()).decode()

    def _describe(clusters: list[Cluster]) -> str:
        return "\n".join(
            f"  {cl.label}: {cl.extent[0]:.2f}m x {cl.extent[1]:.2f}m, "
            f"tall-return fraction {cl.high_frac:.2f}"
            for cl in clusters
        )

    prompt = (
        "This is a top-down lidar occupancy map of an indoor space from a ground robot. "
        "RED clusters (labels W*) were classified as PERMANENT STRUCTURE (walls, columns) "
        "because the lidar saw tall returns there. CYAN clusters (labels M*) were classified "
        "as MOVABLE (furniture, people, clutter) and will be REMOVED from an architectural "
        "floor plan.\n\nCluster stats:\nSTRUCTURE:\n"
        + _describe(walls)
        + "\nMOVABLE:\n"
        + _describe(movables)
        + "\n\nReview the image. Long straight runs at room boundaries are walls; compact "
        "blobs in open space are furniture; human-sized blobs (~0.5 m) are likely people. "
        'Furniture stays on the plan as thin outlines; anything listed in "remove" '
        "(people, transient noise) is deleted entirely. Reply with JSON only: "
        '{"make_wall": ["M2", ...], "make_movable": ["W1", ...], "remove": ["M5", ...], '
        '"reason": "<one sentence>"}. Use empty lists if the classification looks right.'
    )

    verdict = _openai_json(prompt, b64, tier="fast", timeout=120, mime="png")
    if verdict is None:
        logger.warning("AI review unavailable — keeping heuristic classification")
        return {}
    logger.info(
        "AI review",
        make_wall=verdict.get("make_wall", []),
        make_movable=verdict.get("make_movable", []),
        remove=verdict.get("remove", []),
        reason=verdict.get("reason", ""),
    )
    return {
        "make_wall": list(verdict.get("make_wall", [])),
        "make_movable": list(verdict.get("make_movable", [])),
        "remove": list(verdict.get("remove", [])),
    }


def apply_flips(
    grids: Grids, walls: list[Cluster], movables: list[Cluster], flips: dict[str, list[str]]
) -> None:
    by_label = {cl.label: cl for cl in walls + movables}
    for label in flips.get("make_wall", []):
        if label in by_label:
            grids.high_mask = cv2.bitwise_or(grids.high_mask, by_label[label].mask)
    for label in flips.get("make_movable", []) + flips.get("remove", []):
        if label in by_label:
            grids.high_mask = cv2.bitwise_and(
                grids.high_mask, cv2.bitwise_not(by_label[label].mask)
            )


# --------------------------------------------------------------------------- #
# Rendering
# --------------------------------------------------------------------------- #


def _dimension_targets(plan: Plan, max_dims: int = 4) -> list[Wall]:
    return sorted(plan.walls, key=lambda wall: -wall.length)[:max_dims]


def _dim_offset_side(wall: Wall, plan_centroid: tuple[float, float]) -> tuple[float, float]:
    """Unit normal pointing away from the plan centroid (dimension outside the plan)."""
    dx, dy = wall.p2[0] - wall.p1[0], wall.p2[1] - wall.p1[1]
    norm = math.hypot(dx, dy) or 1.0
    nx, ny = -dy / norm, dx / norm
    mx, my = (wall.p1[0] + wall.p2[0]) / 2, (wall.p1[1] + wall.p2[1]) / 2
    if (mx + nx - plan_centroid[0]) ** 2 + (my + ny - plan_centroid[1]) ** 2 < (
        mx - nx - plan_centroid[0]
    ) ** 2 + (my - ny - plan_centroid[1]) ** 2:
        nx, ny = -nx, -ny
    return nx, ny


@dataclass
class LevelSheet:
    level: Level
    plan: Plan
    robot: RobotMarker | None
    region: ClosedRegion | None = None  # sealed connected area from the voxel model


def write_dxf(path: Path, sheets: list[LevelSheet], args: FloorplanOptions) -> None:
    import ezdxf

    doc = ezdxf.new("R2010", setup=True)  # type: ignore[attr-defined]
    doc.header["$INSUNITS"] = 6  # meters
    layers = {
        "A-WALL": {"color": 7, "lineweight": 60},
        "A-WALL-PATT": {"color": 8, "lineweight": 13},
        "A-DOOR": {"color": 2, "lineweight": 25},
        "A-GLAZ": {"color": 4, "lineweight": 13},  # glazing: thin double line, no poché
        "A-FLOR-STRS": {"color": 5, "lineweight": 18},  # stairs: treads + direction
        "A-FLOR-LEVL": {"color": 6, "lineweight": 18},  # level changes: mezzanine/sunken
        "A-FURN": {"color": 8, "lineweight": 13},  # furniture: lightest weight
        "A-AREA": {"color": 3, "lineweight": 13},  # sealed connected-area loops
        "A-ANNO-DIMS": {"color": 3, "lineweight": 18},
        "A-ANNO-SYMB": {"color": 1, "lineweight": 25},
        "A-ANNO-TTLB": {"color": 7, "lineweight": 35},
    }
    for name, attrs in layers.items():
        doc.layers.add(name, **attrs)  # type: ignore[arg-type]
    msp = doc.modelspace()

    multi = len(sheets) > 1
    border_pad = 0.9
    gap = 3.0
    cursor_x = 0.0
    global_y_lo = min(s.plan.bounds()[1] for s in sheets) - border_pad
    global_y_hi = max(s.plan.bounds()[3] for s in sheets) + border_pad
    overall_x_hi = 0.0

    for sheet in sheets:
        x_lo, _, _, _ = sheet.plan.bounds()
        dx = cursor_x - x_lo + border_pad
        plan = sheet.plan.translated(dx, 0.0)
        robot: RobotMarker | None = None
        if sheet.robot is not None:
            robot = (sheet.robot[0] + dx, sheet.robot[1], sheet.robot[2])

        for wall in plan.walls:
            corners = wall.corners()
            if wall.glazed:
                # glazing: thin double line with a centerline, no poché
                msp.add_lwpolyline(
                    [tuple(pt) for pt in corners], close=True, dxfattribs={"layer": "A-GLAZ"}
                )
                msp.add_line(wall.p1, wall.p2, dxfattribs={"layer": "A-GLAZ"})
                continue
            msp.add_lwpolyline(
                [tuple(pt) for pt in corners], close=True, dxfattribs={"layer": "A-WALL"}
            )
            hatch = msp.add_hatch(dxfattribs={"layer": "A-WALL-PATT"})
            hatch.set_pattern_fill("ANSI31", scale=0.02)
            hatch.paths.add_polyline_path([tuple(pt) for pt in corners], is_closed=True)

        for poly, label in plan.level_changes:
            msp.add_lwpolyline(
                [(float(px), float(py)) for px, py in poly],
                close=True,
                dxfattribs={"layer": "A-FLOR-LEVL", "linetype": "DASHED"},
            )
            if label:
                cx_, cy_ = float(poly[:, 0].mean()), float(poly[:, 1].mean())
                msp.add_text(label, height=0.2, dxfattribs={"layer": "A-FLOR-LEVL"}).set_placement(
                    (cx_, cy_)
                )

        for stair in plan.stairs:
            sx, sy = stair.p1
            ex, ey = stair.p2
            run = math.hypot(ex - sx, ey - sy)
            if run < 0.5:
                continue
            ux, uy = (ex - sx) / run, (ey - sy) / run
            nx_, ny_ = -uy * stair.width / 2, ux * stair.width / 2
            msp.add_lwpolyline(
                [
                    (sx + nx_, sy + ny_),
                    (ex + nx_, ey + ny_),
                    (ex - nx_, ey - ny_),
                    (sx - nx_, sy - ny_),
                ],
                close=True,
                dxfattribs={"layer": "A-FLOR-STRS"},
            )
            for k in range(1, int(run / 0.28) + 1):
                tx, ty = sx + ux * k * 0.28, sy + uy * k * 0.28
                msp.add_line(
                    (tx + nx_, ty + ny_), (tx - nx_, ty - ny_), dxfattribs={"layer": "A-FLOR-STRS"}
                )
            msp.add_line((sx, sy), (ex, ey), dxfattribs={"layer": "A-FLOR-STRS"})
            msp.add_text(
                stair.label, height=0.22, dxfattribs={"layer": "A-FLOR-STRS"}
            ).set_placement((sx - nx_ * 1.8, sy - ny_ * 1.8))

        for poly in plan.furniture:
            msp.add_lwpolyline(
                [(float(px), float(py)) for px, py in poly],
                close=True,
                dxfattribs={"layer": "A-FURN"},
            )

        for loop in plan.rooms:
            msp.add_lwpolyline(
                [(float(px), float(py)) for px, py in loop[:-1]],
                close=True,
                dxfattribs={"layer": "A-AREA"},
            )

        for i, room in enumerate(plan.partitions):
            msp.add_lwpolyline(
                [(float(px), float(py)) for px, py in room.polygon[:-1]],
                close=True,
                dxfattribs={"layer": "A-AREA-RM"},
            )
            msp.add_text(
                f"RM {i + 1} — {room.area_m2:.1f} m^2",
                height=0.15,
                dxfattribs={"layer": "A-AREA-IDEN"},
            ).set_placement(room.centroid, align=ezdxf.enums.TextEntityAlignment.MIDDLE_CENTER)

        for door in plan.doors:
            hx, hy = door.hinge
            jx, jy = door.jamb
            nx, ny = door.normal
            width = door.width
            leaf_end = (hx + nx * width, hy + ny * width)
            leaf_layer = "A-GLAZ" if door.glazed else "A-DOOR"
            msp.add_line((hx, hy), leaf_end, dxfattribs={"layer": leaf_layer})
            a_leaf = math.degrees(math.atan2(leaf_end[1] - hy, leaf_end[0] - hx))
            a_jamb = math.degrees(math.atan2(jy - hy, jx - hx))
            start, end = sorted((a_leaf, a_jamb))
            if end - start > 180:
                start, end = end, start + 360
            msp.add_arc(
                (hx, hy),
                radius=width,
                start_angle=start,
                end_angle=end,
                dxfattribs={"layer": "A-DOOR"},
            )

        if plan.walls:
            pts = np.vstack([w.corners() for w in plan.walls])
            centroid = (float(pts[:, 0].mean()), float(pts[:, 1].mean()))
            for wall in _dimension_targets(plan):
                nx, ny = _dim_offset_side(wall, centroid)
                side = (
                    0.6
                    if (nx * (wall.p2[1] - wall.p1[1]) - ny * (wall.p2[0] - wall.p1[0])) <= 0
                    else -0.6
                )
                dim = msp.add_aligned_dim(
                    p1=wall.p1,
                    p2=wall.p2,
                    distance=side,
                    dxfattribs={"layer": "A-ANNO-DIMS"},
                    override={"dimtxt": 0.18, "dimdec": 0, "dimlfac": 1000.0},  # mm text
                )
                dim.render()

        if robot is not None:
            rx, ry, yaw = robot
            msp.add_circle((rx, ry), radius=0.18, dxfattribs={"layer": "A-ANNO-SYMB"})
            if yaw is not None:
                msp.add_line(
                    (rx, ry),
                    (rx + 0.4 * math.cos(yaw), ry + 0.4 * math.sin(yaw)),
                    dxfattribs={"layer": "A-ANNO-SYMB"},
                )

        # per-level border + label (only when there is more than one level)
        _, _, p_x_hi, _ = plan.bounds()
        bx_lo, bx_hi = cursor_x, p_x_hi + border_pad
        if multi:
            msp.add_lwpolyline(
                [
                    (bx_lo, global_y_lo),
                    (bx_hi, global_y_lo),
                    (bx_hi, global_y_hi),
                    (bx_lo, global_y_hi),
                ],
                close=True,
                dxfattribs={"layer": "A-ANNO-TTLB"},
            )
            label = f"{sheet.level.name} — FFL {sheet.level.floor_z:+.2f} m"
            msp.add_text(label, height=0.3, dxfattribs={"layer": "A-ANNO-TTLB"}).set_placement(
                (bx_lo + 0.3, global_y_hi + 0.25)
            )
        overall_x_hi = bx_hi
        cursor_x = bx_hi + gap

    # north arrow (+Y = project north), top-left of the sheet
    nax, nay = 0.5, global_y_hi + 1.3
    msp.add_circle((nax, nay), radius=0.35, dxfattribs={"layer": "A-ANNO-SYMB"})
    msp.add_lwpolyline(
        [(nax, nay - 0.28), (nax + 0.12, nay - 0.1), (nax, nay + 0.28), (nax - 0.12, nay - 0.1)],
        close=True,
        dxfattribs={"layer": "A-ANNO-SYMB"},
    )
    msp.add_text("N", height=0.2, dxfattribs={"layer": "A-ANNO-SYMB"}).set_placement(
        (nax + 0.45, nay)
    )

    # title block, bottom right
    stamp = datetime.now().strftime("%Y-%m-%d")
    tb_w, tb_h = 4.6, 1.5
    tbx, tby = overall_x_hi - tb_w, global_y_lo - tb_h - 0.4
    msp.add_lwpolyline(
        [(tbx, tby), (tbx + tb_w, tby), (tbx + tb_w, tby + tb_h), (tbx, tby + tb_h)],
        close=True,
        dxfattribs={"layer": "A-ANNO-TTLB"},
    )
    sheet_ids = "  ".join(f"A-10{s.level.index}" for s in sheets)
    rows = [
        f"PROJECT: {args.project}",
        f"DRAWING: FLOOR PLAN{'S' if multi else ''} — LIDAR SURVEY",
        f"SHEET {sheet_ids}   SCALE 1:50   DATE {stamp}",
        "DRAWN BY: DIMOS   DIMENSIONS IN MM   N = +Y",
    ]
    for i, row in enumerate(rows):
        msp.add_text(row, height=0.16, dxfattribs={"layer": "A-ANNO-TTLB"}).set_placement(
            (tbx + 0.15, tby + tb_h - 0.32 * (i + 1))
        )

    doc.saveas(path)
    logger.info(f"Wrote {path}")


def _draw_level(ax: Any, sheet: LevelSheet, span: float) -> None:
    from matplotlib.patches import Arc, Polygon
    import matplotlib.pyplot as plt
    from matplotlib.ticker import MultipleLocator

    plan = sheet.plan
    x_lo, y_lo, x_hi, y_hi = plan.bounds()
    cx, cy = (x_lo + x_hi) / 2, (y_lo + y_hi) / 2
    half = span / 2
    ax.set_xlim(cx - half, cx + half)
    ax.set_ylim(cy - half, cy + half)
    ax.set_facecolor(BG)
    ax.set_aspect("equal")

    centroid = (cx, cy)

    # walls: double line + solid poché, heaviest weight (cut elements);
    # glazing: thin double line + centerline, no poché
    for wall in plan.walls:
        if wall.glazed:
            ax.add_patch(
                Polygon(
                    wall.corners(),
                    closed=True,
                    facecolor="none",
                    edgecolor=ACCENT,
                    linewidth=0.9,
                    zorder=4,
                )
            )
            ax.plot(
                [wall.p1[0], wall.p2[0]], [wall.p1[1], wall.p2[1]], color=ACCENT, lw=0.5, zorder=4
            )
        else:
            ax.add_patch(
                Polygon(
                    wall.corners(),
                    closed=True,
                    facecolor=INK,
                    edgecolor=INK,
                    linewidth=1.8,
                    zorder=4,
                )
            )

    # level changes: dashed footprint + FFL label (mezzanine / sunken area)
    for poly, label in plan.level_changes:
        closed = np.vstack([poly, poly[:1]])
        ax.plot(closed[:, 0], closed[:, 1], color=ACCENT, lw=1.1, linestyle=(0, (5, 3)), zorder=5)
        if label:
            ax.text(
                float(poly[:, 0].mean()),
                float(poly[:, 1].mean()),
                label,
                color=ACCENT,
                fontsize=6.5,
                family="monospace",
                ha="center",
                va="center",
                zorder=6,
            )

    # stairs: outline + treads + direction arrow with UP/DN label
    for stair in plan.stairs:
        sx, sy = stair.p1
        ex, ey = stair.p2
        run = math.hypot(ex - sx, ey - sy)
        if run < 0.5:
            continue
        ux, uy = (ex - sx) / run, (ey - sy) / run
        nx_, ny_ = -uy * stair.width / 2, ux * stair.width / 2
        outline = np.array(
            [
                [sx + nx_, sy + ny_],
                [ex + nx_, ey + ny_],
                [ex - nx_, ey - ny_],
                [sx - nx_, sy - ny_],
            ]
        )
        ax.add_patch(
            Polygon(outline, closed=True, facecolor="none", edgecolor=INK, linewidth=1.0, zorder=5)
        )
        for k in range(1, int(run / 0.28) + 1):
            tx, ty = sx + ux * k * 0.28, sy + uy * k * 0.28
            ax.plot([tx + nx_, tx - nx_], [ty + ny_, ty - ny_], color=INK, lw=0.6, zorder=5)
        ax.annotate(
            "",
            xy=(ex, ey),
            xytext=(sx, sy),
            arrowprops={"arrowstyle": "-|>", "color": ACCENT, "lw": 1.1},
            zorder=6,
        )
        ax.text(
            sx - nx_ * 1.7,
            sy - ny_ * 1.7,
            stair.label,
            color=ACCENT,
            fontsize=7,
            family="monospace",
            ha="center",
            zorder=6,
        )

    # furniture: lightest line weight, outline only (non-structural by convention)
    for poly in plan.furniture:
        closed = np.vstack([poly, poly[:1]])
        ax.plot(closed[:, 0], closed[:, 1], color=ACCENT, lw=0.7, alpha=0.8, zorder=3)

    # sealed connected-area loops from the voxel model: hairline, below furniture
    for loop in plan.rooms:
        ax.plot(
            loop[:, 0],
            loop[:, 1],
            color=ACCENT,
            lw=0.5,
            alpha=0.45,
            linestyle=(0, (5, 3)),
            zorder=2,
        )

    # per-room partitions (walls + closed doors): labeled outline, below furniture
    for i, room in enumerate(plan.partitions):
        ax.plot(room.polygon[:, 0], room.polygon[:, 1], color=ACCENT, lw=0.6, alpha=0.6, zorder=2)
        ax.text(
            room.centroid[0],
            room.centroid[1],
            f"RM {i + 1}\n{room.area_m2:.1f} m²",
            color=ACCENT,
            fontsize=6,
            family="monospace",
            ha="center",
            va="center",
            alpha=0.85,
            zorder=3,
        )

    # doors: leaf + swing arc, thin lines
    for door in plan.doors:
        hx, hy = door.hinge
        jx, jy = door.jamb
        nx, ny = door.normal
        width = door.width
        leaf = (hx + nx * width, hy + ny * width)
        leaf_color, leaf_lw = (ACCENT, 0.8) if door.glazed else (INK, 1.0)
        ax.plot([hx, leaf[0]], [hy, leaf[1]], color=leaf_color, lw=leaf_lw, zorder=5)
        a_leaf = math.degrees(math.atan2(leaf[1] - hy, leaf[0] - hx))
        a_jamb = math.degrees(math.atan2(jy - hy, jx - hx))
        start, end = sorted((a_leaf, a_jamb))
        if end - start > 180:
            start, end = end, start + 360
        ax.add_patch(
            Arc(
                (hx, hy),
                2 * width,
                2 * width,
                theta1=start,
                theta2=end,
                color=ACCENT,
                lw=0.9,
                zorder=5,
            )
        )

    # dimension strings (mm) on the major runs
    for wall in _dimension_targets(plan):
        nx, ny = _dim_offset_side(wall, centroid)
        off = 0.55
        e1 = (wall.p1[0] + nx * off, wall.p1[1] + ny * off)
        e2 = (wall.p2[0] + nx * off, wall.p2[1] + ny * off)
        for pt, ext in ((wall.p1, e1), (wall.p2, e2)):
            ax.plot([pt[0], ext[0]], [pt[1], ext[1]], color=ACCENT, lw=0.7, zorder=3)
        ax.annotate(
            "",
            xy=e2,
            xytext=e1,
            arrowprops={
                "arrowstyle": "<|-|>",
                "color": ACCENT,
                "lw": 0.9,
                "shrinkA": 0,
                "shrinkB": 0,
            },
            zorder=3,
        )
        mx, my = (e1[0] + e2[0]) / 2, (e1[1] + e2[1]) / 2
        angle = math.degrees(math.atan2(wall.p2[1] - wall.p1[1], wall.p2[0] - wall.p1[0]))
        if angle > 90 or angle <= -90:
            angle += 180
        ax.text(
            mx + nx * 0.18,
            my + ny * 0.18,
            f"{wall.length * 1000:.0f}",
            color=INK,
            fontsize=7,
            family="monospace",
            ha="center",
            va="center",
            rotation=angle,
            rotation_mode="anchor",
            zorder=6,
        )

    if sheet.robot is not None:
        rx, ry, yaw = sheet.robot
        ax.plot(rx, ry, "o", ms=8, mfc="none", mec=RED, mew=1.5, zorder=6)
        if yaw is not None:
            ax.annotate(
                "",
                xy=(rx + 0.5 * math.cos(yaw), ry + 0.5 * math.sin(yaw)),
                xytext=(rx, ry),
                arrowprops={"arrowstyle": "-|>", "color": RED, "lw": 1.5},
                zorder=6,
            )

    # north arrow, top-left of this plan frame
    nax, nay = cx - half + 0.09 * span, cy + half - 0.09 * span
    r = 0.035 * span
    ax.add_patch(plt.Circle((nax, nay), r, fill=False, color=INK, lw=1.1, zorder=6))  # type: ignore[attr-defined]
    ax.add_patch(
        Polygon(
            np.array(
                [
                    [nax, nay - r * 0.75],
                    [nax + r * 0.32, nay - r * 0.28],
                    [nax, nay + r * 0.75],
                    [nax - r * 0.32, nay - r * 0.28],
                ]
            ),
            closed=True,
            facecolor=INK,
            edgecolor=INK,
            zorder=6,
        )
    )
    ax.text(
        nax + r * 1.4, nay, "N", color=INK, fontsize=9, family="monospace", va="center", zorder=6
    )

    # scale bar (1 m), bottom-left
    sb_x, sb_y = cx - half + 0.07 * span, cy - half + 0.06 * span
    ax.plot([sb_x, sb_x + 1.0], [sb_y, sb_y], color=INK, lw=3, zorder=6)
    ax.text(
        sb_x + 0.5,
        sb_y + 0.015 * span,
        "1 m",
        color=INK,
        ha="center",
        fontsize=7,
        family="monospace",
        zorder=6,
    )

    # grid + frame
    ax.xaxis.set_major_locator(MultipleLocator(5.0))
    ax.yaxis.set_major_locator(MultipleLocator(5.0))
    ax.xaxis.set_minor_locator(MultipleLocator(1.0))
    ax.yaxis.set_minor_locator(MultipleLocator(1.0))
    ax.grid(which="major", color=GRID_MAJOR, lw=0.8, alpha=0.9)
    ax.grid(which="minor", color=GRID_MINOR, lw=0.4, alpha=0.9)
    ax.tick_params(colors=ACCENT, labelsize=6)
    for spine in ax.spines.values():
        spine.set_color(INK)
        spine.set_linewidth(1.3)


def write_jpeg(path: Path, sheets: list[LevelSheet], args: FloorplanOptions) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    n = len(sheets)
    ncols = min(n, 3)
    nrows = math.ceil(n / ncols)

    # one shared scale: every subplot spans the same number of meters
    span = 0.0
    for sheet in sheets:
        x_lo, y_lo, x_hi, y_hi = sheet.plan.bounds()
        span = max(span, x_hi - x_lo, y_hi - y_lo)
    span += 3.2  # padding

    fig, axes = plt.subplots(
        nrows, ncols, figsize=(7.5 * ncols, 7.8 * nrows), facecolor=BG, squeeze=False
    )
    for ax in axes.flat[n:]:
        ax.set_visible(False)

    for ax, sheet in zip(axes.flat, sheets, strict=False):
        _draw_level(ax, sheet, span)
        if n > 1:
            ax.set_title(
                f"{sheet.level.name} — FFL {sheet.level.floor_z:+.2f} m",
                color=INK,
                family="monospace",
                fontsize=11,
                pad=10,
            )

    stamp = datetime.now().strftime("%Y-%m-%d %H:%M")
    sheet_ids = "  ".join(f"A-10{s.level.index}" for s in sheets)
    block = (
        f"PROJECT: {args.project}\n"
        f"DRAWING: FLOOR PLAN{'S' if n > 1 else ''} — LIDAR SURVEY\n"
        f"SHEET {sheet_ids}   SCALE 1:50   {stamp}\n"
        "DRAWN BY: DIMOS   DIMENSIONS IN MM   N = +Y"
    )
    fig.text(
        0.995,
        0.005,
        block,
        color=INK,
        fontsize=8,
        family="monospace",
        ha="right",
        va="bottom",
        linespacing=1.7,
        bbox={"boxstyle": "square,pad=0.6", "facecolor": BG, "edgecolor": INK, "linewidth": 1.2},
    )

    fig.tight_layout(rect=(0, 0.045, 1, 1))
    fig.savefig(path, dpi=170, facecolor=BG, format="jpeg")
    plt.close(fig)
    logger.info(f"Wrote {path}")


# --------------------------------------------------------------------------- #


def process_level(
    level: Level,
    model: SceneModel,
    robot_live: RobotMarker | None,
    out: Path,
    args: FloorplanOptions,
    use_ai: bool,
    session: SessionImagery | None = None,
    vox: VoxelOccupancy | None = None,
) -> LevelSheet | None:
    trajectory = model.trajectory
    z_min = level.floor_z + 0.15
    z_max = level.ceiling_z
    # generate_floorplan resolves wall_z before dispatching levels
    high_z = level.floor_z + (args.wall_z if args.wall_z is not None else 0.95)
    # indoor evidence: returns overhead, between this band's top and ~2.5 m above
    # (wide enough to catch tall ceilings and the next story's floor slab)
    ceil_band = (level.ceiling_z, level.ceiling_z + 2.5)
    traj_xy: np.ndarray | None = None
    if len(trajectory):
        on_level = trajectory[
            (trajectory[:, 2] >= level.floor_z - 0.4) & (trajectory[:, 2] <= level.ceiling_z)
        ]
        if len(on_level):
            traj_xy = on_level[:, :2]

    grids = build_grids(
        model, z_min, z_max, high_z, ceil_band, traj_xy, args.resolution, args.min_hits
    )
    if grids is None:
        logger.warning(f"{level.name}: too few points in z [{z_min:.2f}, {z_max:.2f}] — skipped")
        return None

    walls_c, movables_c = split_clusters(grids)
    logger.info(f"{level.name}: {len(walls_c)} structural / {len(movables_c)} movable clusters")

    removed: set[str] = set()
    reclassified: set[str] = set()
    if use_ai:
        debug = out.with_suffix(f".debug-L{level.index}.png")
        flips = ai_review(grids, walls_c, movables_c, debug)
        if any(flips.values()):
            apply_flips(grids, walls_c, movables_c, flips)
            removed = set(flips.get("remove", []))
            reclassified = set(flips.get("make_wall", []))

    plan = vectorize(grids.high_mask, grids.origin, grids.resolution)
    n_interior = len(plan.walls)
    if grids.envelope_mask is not None:
        env_walls = envelope_walls(
            grids.envelope_mask, grids.origin, grids.resolution, plan.dominant_angle
        )
        classify_glazing(env_walls, grids)
        if session is not None and use_ai:
            env_walls = visually_confirm_glazing(env_walls, session, level, out)
        n_glazed = sum(1 for w in env_walls if w.glazed)
        if n_glazed:
            logger.info(f"{level.name}: {n_glazed}/{len(env_walls)} envelope runs are glazing")
        plan.walls.extend(env_walls)

    # doors: wherever the robot drove through a wall there must be an opening.
    # Use a band reaching below the floor so the outdoor approach to an
    # entrance (ground slopes away from the slab) still crosses the envelope.
    carve_xy: np.ndarray | None = None
    if len(trajectory):
        near = trajectory[
            (trajectory[:, 2] >= level.floor_z - 1.2) & (trajectory[:, 2] <= level.ceiling_z)
        ]
        if len(near):
            carve_xy = near[:, :2]
    carved = carve_passages(plan, carve_xy)

    tidy_stats = tidy_plan(plan)
    if any(tidy_stats.values()):
        logger.info(f"{level.name} tidy", **tidy_stats)

    # voxel closed loops: seal each connected area the robot drove through
    region: ClosedRegion | None = None
    if vox is not None and traj_xy is not None and len(traj_xy):
        barrier = _indoor_barrier(vox, grids)
        region = vox.closed_region(z_min, z_max, traj_xy, min_hits=args.min_hits, barrier=barrier)
        if region is not None and not region.loops:
            # z_min excludes near-floor returns to keep furniture out of the
            # structural raster — but a low-mounted lidar (small quadrupeds)
            # can have most of its wall evidence concentrated down there, so
            # that same cutoff can starve the seal of evidence entirely. The
            # seal only needs *some* wall evidence per column, not a
            # furniture-height cut, so retry against the fuller band before
            # giving up on room loops.
            retry = vox.closed_region(
                level.floor_z + 0.02, z_max, traj_xy, min_hits=args.min_hits, barrier=barrier
            )
            if retry is not None and retry.loops:
                logger.info(
                    f"{level.name}: seal retry at floor+0.02m recovered {len(retry.loops)} "
                    "room loop(s) that the furniture-height band missed"
                )
                region = retry
    if region is not None and args.close_loops:
        plan.rooms = region.loops
        n_bridged = close_wall_gaps(plan, region)
        logger.info(
            f"{level.name}: sealed at {region.closure_m} m closing; "
            f"{len(region.loops)} room loop(s), {n_bridged} gap wall(s) bridged"
        )

    if args.furniture:
        plan.furniture = furniture_outlines(movables_c, grids, drop=removed | reclassified)

    for mezz in level.mezzanines:
        # label the dominant footprint; smaller pieces get outline only
        def _shoelace(p: np.ndarray) -> float:
            x, y = p[:, 0], p[:, 1]
            return 0.5 * abs(float(np.dot(x, np.roll(y, 1)) - np.dot(y, np.roll(x, 1))))

        for k, poly in enumerate(sorted(mezz.footprints, key=_shoelace, reverse=True)):
            plan.level_changes.append((poly, mezz.label if k == 0 else ""))

    logger.info(
        f"{level.name}: {n_interior} interior + {len(plan.walls) - n_interior} envelope "
        f"wall runs, {len(plan.doors)} door openings ({carved} from trajectory crossings), "
        f"{len(level.mezzanines)} level change(s), "
        f"axis {math.degrees(plan.dominant_angle):.1f} deg"
    )
    if not plan.walls and not plan.furniture:
        logger.warning(f"{level.name}: nothing to draw — skipped")
        return None

    robot: RobotMarker | None = robot_live
    if robot is None and len(trajectory):
        on_level = trajectory[
            (trajectory[:, 2] >= level.floor_z - 0.4) & (trajectory[:, 2] <= level.ceiling_z)
        ]
        if len(on_level):
            robot = (float(on_level[-1, 0]), float(on_level[-1, 1]), None)

    return LevelSheet(level=level, plan=plan, robot=robot, region=region)


@dataclass
class FloorplanOptions:
    """Everything the floorplan pipeline can be told to do.

    Field-per-flag mirror of the CLI; the skill container exposes the subset
    that makes sense for an agent. Paths accept str or Path.
    """

    # -- input source (exactly one) ------------------------------------------
    rrd: Path | None = None  # Rerun recording of a mapping session
    duration: float = 30.0  # live LCM collection time, s (when rrd is None)
    lidar_topic: str = "/lidar"
    spin: bool = False  # live: rotate in place while collecting
    explore: bool = False  # live: trigger the frontier explorer for coverage

    # -- optional correlated imagery -----------------------------------------
    session_dir: Path | None = None  # gtsam_odom.tum + mem2.db + intrinsics

    # -- geometry knobs --------------------------------------------------------
    levels: str | None = None  # manual floor elevations "z1,z2,..." (else auto)
    wall_z: float | None = None  # structural-evidence height (None = auto)
    resolution: float = 0.05  # plan raster cell, m
    min_hits: int = 2  # lidar hits for an occupied cell
    voxel: float = 0.1  # 3D voxel edge for the scene voxel model, m
    close_loops: bool = True  # bridge lidar gaps so area boundaries close
    furniture: bool = True  # movable objects as thin A-FURN outlines

    # -- AI passes -------------------------------------------------------------
    ai_review: bool | None = None  # None = on when OPENAI_API_KEY is set
    ai_render: str | None = None  # rendition styles "drafted,cyanotype,..."

    # -- outputs ---------------------------------------------------------------
    out: Path = Path("floorplan")
    project: str = "DIMOS SITE SURVEY"
    debug_artifacts: bool = False  # keep AI-evidence images next to the output
    save_3d_model: bool = False  # write <out>.model.rrd (sliceable 3D model)
    open_viewer: bool = False  # launch dimos-viewer on the 3D model


@dataclass
class SheetStats:
    """What one level's plan contains — the analyzable attributes of the space."""

    name: str
    ffl: float  # finished floor level, m
    walls: int
    glazed_walls: int
    doors: int
    stairs: int
    level_changes: list[str]  # mezzanine / sunken labels on this sheet
    extent_m: tuple[float, float]  # sheet bounding box, m
    room_areas_m2: list[float]  # sealed room loops, largest first (noise filtered out)


@dataclass
class FloorplanResult:
    """Outputs + measured attributes of the navigated space."""

    dxf: Path
    jpeg: Path
    sheets: list[SheetStats]
    rooms_json: Path | None = None  # structured per-room data for later querying
    model_rrd: Path | None = None
    renders: list[Path] = field(default_factory=list)
    debug_dir: Path | None = None
    warnings: list[str] = field(default_factory=list)  # data-quality caveats (e.g. odom drift)

    def summary(self) -> str:
        lines = [f"Floor plan generated: {len(self.sheets)} level(s)."]
        for w in self.warnings:
            lines.append(f"  WARNING: {w}")
        for s in self.sheets:
            glaze = f", {s.glazed_walls} glazed" if s.glazed_walls else ""
            extra = "".join(f"; {lc}" for lc in s.level_changes)
            if s.room_areas_m2:
                areas = ", ".join(f"{a:.1f}" for a in s.room_areas_m2)
                rooms = f", {len(s.room_areas_m2)} room(s) [{areas}] m^2"
            else:
                rooms = ", room boundaries not sealed (see room_areas_m2)"
            lines.append(
                f"  {s.name} (FFL {s.ffl:+.2f} m): {s.walls} wall runs{glaze}, "
                f"{s.doors} doors, {s.stairs} stairs, extent "
                f"{s.extent_m[0]:.1f} x {s.extent_m[1]:.1f} m{rooms}{extra}"
            )
        lines.append(f"CAD drawing (AIA layers): {self.dxf}")
        lines.append(f"Sheet image: {self.jpeg}")
        if self.rooms_json is not None:
            lines.append(f"Room data (query this to answer questions about the space): {self.rooms_json}")
        if self.model_rrd is not None:
            lines.append(f"Sliceable 3D model: {self.model_rrd} (open with dimos-viewer)")
        for r in self.renders:
            lines.append(f"Stylized rendition: {r}")
        if self.debug_dir is not None:
            lines.append(f"Debug artifacts kept in: {self.debug_dir}")
        return "\n".join(lines)

    def agent_encode(self) -> list[dict[str, Any]]:
        """MCP content blocks: the text summary plus one image per rendition.

        Picked up automatically by the MCP server (any skill return value with
        this method has its output forwarded as-is) and by McpClient, which
        already special-cases non-text content into the agent's visible
        history — so a rendition becomes something the agent can actually see,
        not just a file path in text.
        """
        content: list[dict[str, Any]] = [{"type": "text", "text": self.summary()}]
        for r in self.renders:
            try:
                content.extend(Image.from_file(r).agent_encode())
            except (ValueError, OSError) as e:
                logger.warning(f"Could not load rendition {r} for agent: {e}")
        return content


def generate_floorplan(opts: FloorplanOptions, model: SceneModel | None = None) -> FloorplanResult:
    """Run the full pipeline: 3D slices -> drawing -> conditional cleaning.

    `model` may be injected (tests, or an already-built SceneModel); otherwise
    it is loaded from `opts.rrd` or collected live over LCM. All intermediate
    AI-evidence images (classification debug maps, camera inspection frames,
    critique review sheets) go to a temporary directory that is removed unless
    `opts.debug_artifacts` is set.
    """
    load_dotenv()
    wall_z = opts.wall_z
    if wall_z is None:
        wall_z = 1.6 if (opts.rrd is not None or model is not None) else 0.95
    opts = replace(opts, wall_z=wall_z, out=Path(opts.out))

    robot_live: RobotMarker | None = None
    if model is None and opts.rrd is not None:
        # the 3D scene model (wishlist item 8) is the shared representation:
        # every evidence raster below is one of its plan cuts
        model = SceneModel.from_rrd(opts.rrd)
        if len(model.points) == 0:
            raise ValueError(f"No point cloud found in {opts.rrd}")
    elif model is None:
        points, pose, trajectory = collect_points(
            opts.duration, opts.lidar_topic, opts.spin, opts.explore
        )
        if len(points) == 0:
            raise RuntimeError(
                "No lidar data received. Is a dimos instance running? "
                "(e.g. `dimos --simulation run unitree-go2 --daemon`)"
            )
        # Trajectory enables dwell-based floor detection (multi-level live
        # collections were previously flattened to a single level).
        model = SceneModel(points, trajectory if len(trajectory) else None)
        if pose is not None:
            robot_live = (
                float(pose.position.x),
                float(pose.position.y),
                float(pose.orientation.to_euler().z),
            )

    warnings: list[str] = []
    if len(model.trajectory) or opts.levels:
        floors = detect_floor_elevations(model.trajectory[:, 2], opts.levels)
        if not floors:
            logger.warning("No floors detected from trajectory — treating as single level")
            floors = [float(np.percentile(model.points[:, 2], 2.0))]
        if not opts.levels:  # manual levels are the caller's own claim
            drift = check_floor_plausibility(floors)
            if drift is not None:
                logger.warning(drift)
                warnings.append(drift)
        levels = build_levels(floors)
        logger.info(
            "Detected levels: "
            + ", ".join(f"{lv.name} FFL {lv.floor_z:+.2f}m" for lv in levels)
            + f" (points above z={levels[-1].ceiling_z:.2f} treated as roof and excluded)"
        )
        # intermediate platforms — mezzanines, split-levels, sunken areas —
        # are indicated on their parent level's sheet, not separate sheets
        detect_mezzanines(model, levels)
    else:
        # live world frame: z=0 is the floor the robot stands on
        levels = [Level(index=1, floor_z=0.0, ceiling_z=STORY_CAP)]

    use_ai = opts.ai_review
    if use_ai is None:
        use_ai = bool(os.environ.get("OPENAI_API_KEY"))

    session: SessionImagery | None = None
    if opts.session_dir is not None:
        try:
            session = SessionImagery(Path(opts.session_dir).expanduser())
            logger.info(f"Session imagery available: {len(session.t)} stamped poses")
        except (FileNotFoundError, OSError, ValueError) as e:
            logger.warning(f"session imagery unavailable ({e}) — continuing without")

    out = opts.out
    out.parent.mkdir(parents=True, exist_ok=True)

    # AI-evidence artifacts (debug maps, inspection frames, review sheets) are
    # working files, not deliverables: they live in a tempdir unless asked for
    tmp_ctx: tempfile.TemporaryDirectory[str] | None = None
    if opts.debug_artifacts:
        art_base = out
        debug_dir: Path | None = out.parent
    else:
        tmp_ctx = tempfile.TemporaryDirectory(prefix="dimos-floorplan-")
        art_base = Path(tmp_ctx.name) / out.name
        debug_dir = None

    try:
        # the analytic 3D voxel model: seals room loops and feeds the 3D viewer.
        # from_rrd clips z outliers only — clip xy the same way so a few stray
        # returns don't inflate the grid to city-block size.
        vox: VoxelOccupancy | None = None
        if opts.close_loops or opts.save_3d_model or opts.open_viewer:
            lo = np.percentile(model.points, 0.5, axis=0) - 0.3
            hi = np.percentile(model.points, 99.5, axis=0) + 0.3
            try:
                vox = model.voxel_occupancy(
                    voxel=opts.voxel,
                    bounds=(lo[0], lo[1], lo[2], hi[0], hi[1], hi[2]),
                )
            except ValueError as e:
                logger.warning(f"voxel model unavailable ({e}) — continuing without")

        # levels are independent (own geometry, own AI-review calls, own debug
        # files) — run them concurrently so a 3-level survey doesn't pay 3x the
        # AI round-trip latency of a single level.
        with concurrent.futures.ThreadPoolExecutor(max_workers=max(1, len(levels))) as pool:
            results = list(
                pool.map(
                    lambda level: process_level(
                        level, model, robot_live, art_base, opts, use_ai, session, vox
                    ),
                    levels,
                )
            )
        sheets: list[LevelSheet] = [s for s in results if s is not None]

        if not sheets:
            raise ValueError("No level produced any drawable geometry.")

        # stairs connect levels: same footprint on both, UP below / DN above
        stair_map = detect_stairs(model.trajectory, levels)
        for sheet in sheets:
            sheet.plan.stairs = stair_map.get(sheet.level.index, [])
        if stair_map:
            logger.info(
                "Stairs: " + ", ".join(f"L{idx}: {len(s)}" for idx, s in sorted(stair_map.items()))
            )

        if use_ai:
            needs_cleaning: list[LevelSheet] = []
            for sheet in sheets:
                needs, issues = plan_needs_cleaning(sheet.plan)
                if needs:
                    logger.info(f"{sheet.level.name}: cleaning needed", **issues)
                    needs_cleaning.append(sheet)
                else:
                    logger.info(
                        f"{sheet.level.name}: plan already clean — skipping AI cleaning stage",
                        **issues,
                    )
            if needs_cleaning:
                # critique_plan is the "max" tier (up to 420s, deep-reasoning
                # model) — run the sheets that need it concurrently rather than
                # serially eating that latency once per sheet.
                with concurrent.futures.ThreadPoolExecutor(
                    max_workers=len(needs_cleaning)
                ) as pool:
                    list(pool.map(lambda sheet: critique_plan(sheet, art_base), needs_cleaning))

        for sheet in sheets:
            sheet.plan.partitions = segment_rooms(sheet.plan, region=sheet.region)
            logger.info(f"{sheet.level.name}: {len(sheet.plan.partitions)} room(s) segmented")

        write_dxf(out.with_suffix(".dxf"), sheets, opts)
        write_jpeg(out.with_suffix(".jpg"), sheets, opts)

        model_rrd: Path | None = None
        if (opts.save_3d_model or opts.open_viewer) and vox is not None:
            model_rrd = out.with_suffix(".model.rrd")
            model.save_rerun_sliceable(
                model_rrd, vox, regions=[s.region for s in sheets if s.region is not None]
            )
            if opts.open_viewer:
                launch_viewer(model_rrd)

        renders: list[Path] = []
        if opts.ai_render:
            if len(sheets) > 1:
                # A full-resolution sheet per level rather than a cramped
                # subplot of the combined multi-level sheet, so each level's
                # rendition actually resolves its detail. Intermediate, not a
                # deliverable -- lives next to the other AI-evidence artifacts.
                level_sources = []
                for sheet in sheets:
                    level_jpg = art_base.parent / f"{art_base.name}.L{sheet.level.index}.jpg"
                    write_jpeg(level_jpg, [sheet], opts)
                    level_sources.append((f"L{sheet.level.index}", level_jpg))
            else:
                level_sources = [("", out.with_suffix(".jpg"))]
            renders = generate_blueprint_renditions(
                level_sources, out, [s.strip() for s in opts.ai_render.split(",")]
            )

        stats = []
        for sheet in sheets:
            x_lo, y_lo, x_hi, y_hi = sheet.plan.bounds()
            stats.append(
                SheetStats(
                    name=sheet.level.name,
                    ffl=sheet.level.floor_z,
                    walls=len(sheet.plan.walls),
                    glazed_walls=sum(1 for w in sheet.plan.walls if w.glazed),
                    doors=len(sheet.plan.doors),
                    stairs=len(sheet.plan.stairs),
                    level_changes=[mz.label for mz in sheet.level.mezzanines],
                    extent_m=(x_hi - x_lo, y_hi - y_lo),
                    room_areas_m2=[r.area_m2 for r in sheet.plan.partitions],
                )
            )

        rooms_json = out.with_suffix(".rooms.json")
        rooms_json.write_text(
            json.dumps(
                {
                    "project": opts.project,
                    "levels": [
                        {
                            "name": sheet.level.name,
                            "ffl_m": sheet.level.floor_z,
                            "extent_m": list(s.extent_m),
                            "wall_runs": len(sheet.plan.walls),
                            "doors": len(sheet.plan.doors),
                            "rooms": [
                                {
                                    "id": i,
                                    "area_m2": round(r.area_m2, 2),
                                    "centroid": [round(r.centroid[0], 2), round(r.centroid[1], 2)],
                                    "polygon": [[round(x, 2), round(y, 2)] for x, y in r.polygon],
                                }
                                for i, r in enumerate(sheet.plan.partitions)
                            ],
                        }
                        for sheet, s in zip(sheets, stats, strict=True)
                    ],
                },
                indent=2,
            )
        )

        return FloorplanResult(
            dxf=out.with_suffix(".dxf"),
            jpeg=out.with_suffix(".jpg"),
            sheets=stats,
            rooms_json=rooms_json,
            model_rrd=model_rrd,
            renders=renders,
            debug_dir=debug_dir,
            warnings=warnings,
        )
    finally:
        if session is not None:
            session.close()
        if tmp_ctx is not None:
            tmp_ctx.cleanup()
