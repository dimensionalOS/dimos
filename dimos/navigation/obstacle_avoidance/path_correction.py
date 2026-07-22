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

from __future__ import annotations

import math

import numpy as np
from reactivex import operators as ops

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3, VectorLike
from dimos.msgs.nav_msgs.OccupancyGrid import OccupancyGrid
from dimos.msgs.nav_msgs.Path import Path
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


def footprint_clear(
    costmap: OccupancyGrid,
    pose: Pose,
    *,
    body: VectorLike = (0.70, 0.31, 0.0),
    cost_threshold: int = 50,
) -> bool:
    """True if no cell under the oriented footprint is at or above *cost_threshold*.

    Checks every cell whose center falls inside the body rectangle — corners
    alone would straddle walls thinner than the footprint.
    """
    size = Vector3(body)
    hx, hy = size.x / 2, size.y / 2
    res = costmap.resolution
    radius = math.hypot(hx, hy)

    col_min = max(0, int((pose.position.x - radius - costmap.origin.position.x) / res))
    col_max = min(
        costmap.width - 1, int((pose.position.x + radius - costmap.origin.position.x) / res)
    )
    row_min = max(0, int((pose.position.y - radius - costmap.origin.position.y) / res))
    row_max = min(
        costmap.height - 1, int((pose.position.y + radius - costmap.origin.position.y) / res)
    )
    if col_min > col_max or row_min > row_max:
        return True

    cols = np.arange(col_min, col_max + 1)
    rows = np.arange(row_min, row_max + 1)
    cell_x = costmap.origin.position.x + (cols + 0.5) * res - pose.position.x
    cell_y = costmap.origin.position.y + (rows + 0.5) * res - pose.position.y
    dx, dy = np.meshgrid(cell_x, cell_y)

    yaw = pose.yaw
    cos_yaw, sin_yaw = math.cos(yaw), math.sin(yaw)
    body_x = dx * cos_yaw + dy * sin_yaw
    body_y = -dx * sin_yaw + dy * cos_yaw
    inside = (np.abs(body_x) <= hx) & (np.abs(body_y) <= hy)

    sub = costmap.grid[row_min : row_max + 1, col_min : col_max + 1]
    return not bool(np.any(sub[inside] >= cost_threshold))


def _headings(pts: np.ndarray) -> np.ndarray:
    """Per-waypoint heading along the path (central difference)."""
    deltas = np.empty_like(pts)
    deltas[1:-1] = pts[2:] - pts[:-2]
    deltas[0] = pts[1] - pts[0]
    deltas[-1] = pts[-1] - pts[-2]
    return np.asarray(np.arctan2(deltas[:, 1], deltas[:, 0]))


def correct_path(
    path: Path,
    costmap: OccupancyGrid,
    *,
    influence_radius: float = 0.8,
    gain: float = 1.0,
    cost_threshold: int = 50,
    smoothing: float = 0.3,
    step: float = 0.05,
    iterations: int = 100,
    tolerance: float = 0.01,
) -> Path:
    """Push a path away from obstacles until the repulsive field relaxes.

    Elastic-band-style: each interior waypoint is displaced by the repulsive
    force at the waypoint (capped at *step* per iteration so it cannot jump
    across thin walls), plus a smoothing pull toward its neighbors' midpoint.
    Endpoints stay fixed. Waypoint orientations are set to the corrected path
    heading, so consumers know the desired body yaw at every point — check
    body feasibility with :func:`footprint_clear`.
    """
    if len(path.poses) < 3:
        return path

    pts = np.array([[p.position.x, p.position.y] for p in path.poses])

    # Extract obstacle cells once and crop to the path's neighborhood —
    # per-waypoint full-grid scans make correction O(map size) and the
    # global map grows without bound. Waypoints move at most
    # iterations * step, so pad the crop by that travel too.
    rows, cols = np.nonzero(costmap.grid >= cost_threshold)
    res = costmap.resolution
    cell_x = costmap.origin.position.x + (cols + 0.5) * res
    cell_y = costmap.origin.position.y + (rows + 0.5) * res
    pad = influence_radius + iterations * step
    near_path = (
        (cell_x >= pts[:, 0].min() - pad)
        & (cell_x <= pts[:, 0].max() + pad)
        & (cell_y >= pts[:, 1].min() - pad)
        & (cell_y <= pts[:, 1].max() + pad)
    )
    cell_x = cell_x[near_path]
    cell_y = cell_y[near_path]
    weight = costmap.grid[rows[near_path], cols[near_path]] / 100.0

    def repulsion(x: float, y: float) -> tuple[float, float]:
        dx = x - cell_x
        dy = y - cell_y
        d = np.sqrt(dx**2 + dy**2)
        near = (d > 1e-6) & (d < influence_radius)
        if not np.any(near):
            return 0.0, 0.0
        d = d[near]
        magnitude = gain * weight[near] * res**2 * (1.0 / d - 1.0 / influence_radius) / d**2
        return (
            float(np.sum(magnitude * dx[near] / d)),
            float(np.sum(magnitude * dy[near] / d)),
        )

    for _ in range(iterations):
        new_pts = pts.copy()
        for i in range(1, len(pts) - 1):
            fx, fy = repulsion(pts[i, 0], pts[i, 1])
            norm = math.hypot(fx, fy)
            if norm > step:
                fx, fy = fx * step / norm, fy * step / norm

            midpoint = (pts[i - 1] + pts[i + 1]) / 2
            new_pts[i, 0] = pts[i, 0] + fx + smoothing * (midpoint[0] - pts[i, 0])
            new_pts[i, 1] = pts[i, 1] + fy + smoothing * (midpoint[1] - pts[i, 1])

        max_move = float(np.max(np.hypot(*(new_pts - pts).T)))
        pts = new_pts
        if max_move < tolerance:
            break

    yaws = _headings(pts)
    poses = [
        PoseStamped(
            ts=path.poses[i].ts,
            frame_id=path.frame_id,
            position=(pts[i, 0], pts[i, 1], path.poses[i].position.z),
            orientation=Quaternion.from_euler(Vector3(0.0, 0.0, yaws[i])),
        )
        for i in range(len(pts))
    ]
    return Path(ts=path.ts, frame_id=path.frame_id, poses=poses)


def truncate_before_collision(
    path: Path,
    costmap: OccupancyGrid,
    *,
    body: VectorLike = (0.70, 0.31, 0.0),
    cost_threshold: int = 50,
    margin: float = 0.2,
) -> Path:
    """Cut the path *margin* meters before the first colliding waypoint.

    Walks the path checking :func:`footprint_clear` at every pose; if a
    collision is found, keeps only the prefix ending *margin* before it
    (by arc length), so a follower drives as far as safely possible and
    stops. A clear path is returned unchanged.
    """
    collision_s = None
    s = 0.0
    prev = None
    for pose in path.poses:
        if prev is not None:
            s += math.hypot(pose.position.x - prev.position.x, pose.position.y - prev.position.y)
        if not footprint_clear(costmap, pose, body=body, cost_threshold=cost_threshold):
            collision_s = s
            break
        prev = pose

    if collision_s is None:
        return path

    cut = collision_s - margin
    kept: list[PoseStamped] = []
    s = 0.0
    prev = None
    for pose in path.poses:
        if prev is not None:
            s += math.hypot(pose.position.x - prev.position.x, pose.position.y - prev.position.y)
        if s > cut:
            break
        kept.append(pose)
        prev = pose

    return Path(ts=path.ts, frame_id=path.frame_id, poses=kept)


class PathCorrectorConfig(ModuleConfig):
    influence_radius: float = 0.8
    gain: float = 1.0
    cost_threshold: int = 50
    smoothing: float = 0.3
    step: float = 0.05
    iterations: int = 100
    tolerance: float = 0.01
    body: tuple[float, float, float] = (0.70, 0.31, 0.0)
    stop_margin: float = 0.2
    # Resample the incoming plan to this waypoint spacing (m) before
    # correcting — planners emit sparse/jagged waypoints. 0 disables.
    resample_spacing: float = 0.1
    # Radians added to every stamped yaw. Debug knob to confirm the follower
    # tracks our orientations: π/2 makes the robot strafe the whole path.
    yaw_offset: float = 0.0


class PathCorrector(Module):
    """Correct each incoming plan against the latest costmap, republish it.

    Sits between a planner and a path follower: ``plan`` in, obstacle-aware
    ``path`` out (waypoints carry desired body yaw). If the corrected path
    still collides somewhere the footprint doesn't fit, only the prefix up
    to ``stop_margin`` before the collision is published.
    """

    config: PathCorrectorConfig

    plan: In[Path]
    global_costmap: In[OccupancyGrid]
    path: Out[Path]

    @rpc
    def start(self) -> None:
        super().start()

        cfg = self.config.model_dump(
            include=set(PathCorrectorConfig.model_fields) - set(ModuleConfig.model_fields)
        )
        body = cfg.pop("body")
        margin = cfg.pop("stop_margin")
        spacing = cfg.pop("resample_spacing")
        yaw_offset = cfg.pop("yaw_offset")

        def _correct(pair: tuple[Path, OccupancyGrid]) -> None:
            plan, costmap = pair
            if spacing > 0 and len(plan.poses) >= 2:
                from dimos.mapping.occupancy.path_resampling import simple_resample_path

                plan = simple_resample_path(plan, plan.poses[-1], spacing)
            corrected = correct_path(plan, costmap, **cfg)
            if yaw_offset:
                for pose in corrected.poses:
                    pose.orientation = Quaternion.from_euler(
                        Vector3(0.0, 0.0, pose.yaw + yaw_offset)
                    )
            truncated = truncate_before_collision(
                corrected,
                costmap,
                body=body,
                cost_threshold=cfg["cost_threshold"],
                margin=margin,
            )
            if len(truncated.poses) < len(corrected.poses):
                logger.warning(
                    "PathCorrector: footprint collision after correction — "
                    "publishing %d/%d waypoints (stopping %.2fm before it)",
                    len(truncated.poses),
                    len(corrected.poses),
                    margin,
                )
            self.path.publish(truncated)

        def _on_error(e: Exception) -> None:
            logger.error("PathCorrector pipeline error: %s", e, exc_info=True)

        self.register_disposable(
            self.plan.observable()  # type: ignore[no-untyped-call]
            .pipe(ops.with_latest_from(self.global_costmap.observable()))  # type: ignore[no-untyped-call]
            .subscribe(_correct, on_error=_on_error)
        )

    @rpc
    def stop(self) -> None:
        super().stop()
