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

"""Repulsive-field local planner — a wavefront *navigation function* over a
repulsive cost field.

Takes a 2D **costmap** (``nav_msgs/OccupancyGrid``) plus a **global path** and
produces a **local path of oriented poses** that follows the route and rounds
obstacles with clearance.

Why this design (and not a force-integrated potential field): the classic
Khatib artificial potential field sums an attractive and a repulsive *force* and
integrates them step by step. That is greedy — it gets trapped in local minima
and ties itself in knots/loops when an obstacle sits head-on. This planner keeps
the repulsive field but uses it the robust way: as a **cost layer** searched
**globally**. A single **Dijkstra** rooted at the robot builds the shortest-path
tree over the (clearance-weighted) free space; the local path is the tree's
optimal path to a target cell. Following parent pointers can never cycle, so:

* **No local minima, no loops/knots** — the path is an acyclic optimal path in a
  shortest-path tree; it cannot circle, double back, or stall in a pocket.
* **Clearance + obstacle avoidance** — the repulsive field is folded into the
  cell costs, so the optimal path naturally bows away from obstacles.
* **Follows the global route** — the target is the *furthest point along the
  global path* the robot can reach (the carrot), plus a distance-to-path cost
  term that keeps the local path in the corridor. So when a goal's region is a
  disconnected island in the local costmap (its only door a long loop away, not
  in the window), the planner makes progress *along the route toward the door*
  instead of beelining at the goal and stalling at the near wall.
* **Best effort** — when the goal itself is reachable the carrot is the goal;
  when it is walled off or inside an obstacle the path runs as far along the
  route as it safely can (nosing up to the blockage) rather than freezing or
  clipping. It returns empty only if the robot itself is boxed in.

The output is a list of **poses** (position *and* yaw), with two facing options:

* ``face_forward_weight`` ∈ [0, 1] — blends each pose's yaw between the travel
  tangent (1.0, "face where you're going") and the goal direction (0.0). Robots
  are assumed able to turn in place, so this is a preference, not a constraint.
* ``omnidirectional`` — holonomic mode: facing is decoupled from travel and the
  poses simply face the goal heading.

The pure ``plan_path`` and field helpers take numpy + plain tuples and have no
DimOS dependency, so the web sim's Python backend imports and runs the exact same
planner core. ``RepulsiveFieldLocalPlanner`` is the DimOS ``Module`` wrapper.
"""

from __future__ import annotations

import asyncio
from collections.abc import AsyncIterator
from dataclasses import dataclass
import heapq
import math
import threading
import os
import time
from typing import Any

import numpy as np
from scipy.ndimage import distance_transform_edt

from dimos.utils.logging_config import setup_logger

logger = setup_logger()

_SQRT2 = math.sqrt(2.0)
# (drow, dcol, geometric step length in cells) for 8-connectivity.
_NEIGHBORS = (
    (-1, 0, 1.0),
    (1, 0, 1.0),
    (0, -1, 1.0),
    (0, 1, 1.0),
    (-1, -1, _SQRT2),
    (-1, 1, _SQRT2),
    (1, -1, _SQRT2),
    (1, 1, _SQRT2),
)


# --------------------------------------------------------------------------- #
# Pure planner core (no DimOS deps — shared verbatim by the web-sim backend).  #
# --------------------------------------------------------------------------- #
@dataclass
class RepulsiveFieldParams:
    """Tunables for the navigation-function planner. Distances are metres."""

    # Costmap interpretation.
    lethal_threshold: int = 50  # cost >= this is an (impassable) obstacle
    vehicle_width: float = 0.5  # robot footprint width (m); obstacles are inflated by
    #                             half this so the body fits through gaps and the path
    #                             centreline keeps clear of walls
    safety_margin: float = 0.0  # extra hard clearance (m) kept beyond the body — a buffer
    #                             for moving obstacles. Trades tight-gap access for margin.
    influence_radius: float = 0.8  # repulsive cost ramps up within this *beyond* the body

    # Cost-field weights — shape of the wavefront.
    clearance_weight: float = 4.0  # how strongly to prefer staying clear of obstacles
    path_weight: float = 0.35  # how strongly to hug the global-path corridor
    commitment_weight: float = 0.0  # cost to stray from last tick's path — temporal
    #                                 hysteresis that stops the route flip-flopping when a
    #                                 nearby obstacle wiggles (needs previous_path passed)

    # Search window.
    dijkstra_radius: float = 0.0  # m; 0 = search the whole costmap. A positive value confines
    #                               the wavefront Dijkstra to a window of this radius around the
    #                               robot — cheap per-tick solves on a large costmap (the search
    #                               only needs to reach the carrot, a few m ahead). Keep it well
    #                               above carrot_lookahead so detours around obstacles still fit.

    # Route following.
    carrot_lookahead: float = 0.0  # cap (m of path arc-length) on how far along the global
    #                                path the carrot may sit; 0 = unbounded (farthest reachable
    #                                point anywhere). A positive cap follows the route
    #                                incrementally (pure-pursuit style) so a goal that is near
    #                                in straight-line distance but far along a *loop* (e.g. a
    #                                room reachable only via a far door) is not shortcut into by
    #                                a beeline across the wall between robot and goal.
    carrot_gap_max: float = 0.0  # m of route arc the capped carrot scan may hop over
    #                              unreachable route cells. 0 = stop at the first unreachable
    #                              point after the followable prefix (original hard break). A
    #                              small positive value tolerates single-cell reachability
    #                              flicker (slice/terrain updates) that otherwise collapses the
    #                              carrot — and with it the whole local path — to under a metre
    #                              for a tick. Real walls span well over a metre of route arc,
    #                              so they still stop the scan; and any hopped-to carrot is
    #                              still reached via the Dijkstra tree (collision-free by
    #                              construction), never a beeline.

    # Output.
    horizon: float = 0.0  # max local-path length (m); 0 = all the way to the goal
    goal_tolerance: float = 0.15  # treat within this of the goal as arrived
    smoothing_iterations: int = 12  # output low-pass passes (rounds the octile steps)

    # Facing.
    face_forward_weight: float = 0.8  # 1 = face travel dir, 0 = face goal
    omnidirectional: bool = False  # holonomic: face goal, ignore travel dir


def _unit(v: np.ndarray) -> np.ndarray:
    n = float(np.hypot(v[0], v[1]))
    if n < 1e-9:
        return np.zeros(2)
    return v / n


def _world_to_cell(
    x: float, y: float, resolution: float, origin: tuple[float, float], shape: tuple[int, int]
) -> tuple[int, int]:
    """World (x, y) -> (row, col), clamped into the grid."""
    height, width = shape
    ox, oy = origin
    col = round((x - ox) / resolution)
    row = round((y - oy) / resolution)
    return (min(max(row, 0), height - 1), min(max(col, 0), width - 1))


def _cell_center_world(
    row: int, col: int, resolution: float, origin: tuple[float, float]
) -> tuple[float, float]:
    return (origin[0] + col * resolution, origin[1] + row * resolution)


def _nearest_free_cell(free: np.ndarray, cell: tuple[int, int]) -> tuple[int, int] | None:
    """The given cell if free, else the closest free cell (so a goal/robot that
    lands on an obstacle still gets planned to/from the nearest open space)."""
    row, col = cell
    if free[row, col]:
        return cell
    idx = np.argwhere(free)
    if idx.size == 0:
        return None
    d = (idx[:, 0] - row) ** 2 + (idx[:, 1] - col) ** 2
    j = int(np.argmin(d))
    return (int(idx[j, 0]), int(idx[j, 1]))


def _obstacle_distance(
    cost: np.ndarray, resolution: float, params: RepulsiveFieldParams
) -> tuple[np.ndarray, np.ndarray]:
    """Obstacle mask and per-cell metres to the nearest obstacle (inf if none)."""
    obstacle = cost >= params.lethal_threshold
    if not obstacle.any():
        return obstacle, np.full(cost.shape, np.inf, dtype=np.float64)
    dist = distance_transform_edt(~obstacle).astype(np.float64) * resolution
    return obstacle, dist


def _clearance_penalty(
    dist: np.ndarray, robot_radius: float, influence_radius: float
) -> np.ndarray:
    """Soft repulsive cost ramping 0 -> 1 as clearance beyond the body shrinks to 0.

    Clearance is measured from the *inflated* obstacle (distance minus the robot
    radius), so the penalty rewards keeping the whole footprint — not just the
    centre point — away from walls.
    """
    penalty = np.zeros(dist.shape, dtype=np.float64)
    if influence_radius <= 0.0:
        return penalty
    clear = np.maximum(dist - robot_radius, 0.0)
    inside = clear < influence_radius
    penalty[inside] = ((influence_radius - clear[inside]) / influence_radius) ** 2
    return penalty


def repulsive_cost(cost: np.ndarray, resolution: float, params: RepulsiveFieldParams) -> np.ndarray:
    """The repulsive field as a bounded cost layer (the obstacle-proximity term).

    Built from the Euclidean distance transform, measured from the robot's body
    (obstacles inflated by half ``vehicle_width``): 0 beyond ``influence_radius``
    of the inflated obstacle, ramping to 1 at its face. This is the same Khatib
    "repulsion" intuition, expressed as a cost the wavefront pays for hugging
    obstacles rather than as a force.
    """
    _, dist = _obstacle_distance(cost, resolution, params)
    return _clearance_penalty(dist, max(0.0, params.vehicle_width * 0.5), params.influence_radius)


def _path_distance(
    global_path: np.ndarray, shape: tuple[int, int], resolution: float, origin: tuple[float, float]
) -> np.ndarray:
    """Per-cell metric distance to the global path (rasterized as a polyline)."""
    height, width = shape
    ox, oy = origin
    off_path = np.ones((height, width), dtype=bool)  # True where there is no path
    for i in range(len(global_path) - 1):
        a = global_path[i]
        b = global_path[i + 1]
        seg = float(np.hypot(b[0] - a[0], b[1] - a[1]))
        steps = max(1, int(seg / (resolution * 0.5)))
        for k in range(steps + 1):
            t = k / steps
            x = a[0] + (b[0] - a[0]) * t
            y = a[1] + (b[1] - a[1]) * t
            col = round((x - ox) / resolution)
            row = round((y - oy) / resolution)
            if 0 <= row < height and 0 <= col < width:
                off_path[row, col] = False
    if off_path.all():  # path entirely off the grid — no corridor preference
        return np.zeros((height, width), dtype=np.float64)
    return distance_transform_edt(off_path).astype(np.float64) * resolution


def _resample_arclen(path: list[tuple[float, float]], n: int) -> np.ndarray:
    """Resample a polyline to ``n`` points evenly by arc length (robust to point
    count changing between two versions of the same route)."""
    pts = np.asarray(path, dtype=np.float64)
    seg = np.hypot(np.diff(pts[:, 0]), np.diff(pts[:, 1]))
    cum = np.concatenate([[0.0], np.cumsum(seg)])
    total = float(cum[-1])
    if total == 0.0:
        return np.repeat(pts[:1], n, axis=0)
    targets = np.linspace(0.0, total, n)
    return np.stack([np.interp(targets, cum, pts[:, 0]), np.interp(targets, cum, pts[:, 1])], axis=1)


def _dijkstra_tree(
    free: np.ndarray,
    entry_cost: np.ndarray,
    resolution: float,
    start_cell: tuple[int, int],
    max_radius_cells: int = 0,
) -> tuple[np.ndarray, np.ndarray]:
    """Dijkstra shortest-path tree rooted at the robot.

    Returns ``(dist, parent)``: ``dist[r, c]`` is the minimum accumulated cost to
    reach (r, c) from the robot over free cells (8-connected, no diagonal
    corner-cutting; ``inf`` if unreachable), and ``parent[r, c]`` is the (row,
    col) of the predecessor on that shortest path (``-1`` if none). Following
    ``parent`` from any cell back to the robot is an acyclic optimal path — the
    property that makes the result loop-free, and lets us route to the best
    *reachable* cell when the goal itself is walled off.

    ``max_radius_cells`` (0 = unbounded) confines the search to a Chebyshev window
    of that half-width around the robot. The carrot is at most a few metres ahead
    along the route (``carrot_lookahead``) and the emitted plan is horizon-capped,
    so on a large costmap (e.g. the 18 m terrain slice) exploring every cell is
    wasted work; windowing keeps each per-tick solve cheap so the frequent
    re-solves stay light. The window must be generous enough to still reach the
    carrot around obstacles — set it well beyond ``carrot_lookahead``.
    """
    height, width = free.shape
    dist = np.full((height, width), np.inf, dtype=np.float64)
    parent = np.full((height, width, 2), -1, dtype=np.int32)
    sr, sc = start_cell
    dist[sr, sc] = 0.0
    heap: list[tuple[float, int, int]] = [(0.0, sr, sc)]
    while heap:
        d, r, c = heapq.heappop(heap)
        if d > dist[r, c]:
            continue
        for dr, dc, w in _NEIGHBORS:
            nr, nc = r + dr, c + dc
            if nr < 0 or nc < 0 or nr >= height or nc >= width or not free[nr, nc]:
                continue
            if max_radius_cells and (
                nr - sr > max_radius_cells
                or sr - nr > max_radius_cells
                or nc - sc > max_radius_cells
                or sc - nc > max_radius_cells
            ):
                continue  # outside the local search window
            if dr != 0 and dc != 0 and not (free[r + dr, c] and free[r, c + dc]):
                continue  # don't squeeze diagonally between two obstacles
            nd = d + w * resolution * entry_cost[nr, nc]
            if nd < dist[nr, nc]:
                dist[nr, nc] = nd
                parent[nr, nc] = (r, c)
                heapq.heappush(heap, (nd, nr, nc))
    return dist, parent


def _backtrack(
    parent: np.ndarray, start_cell: tuple[int, int], target_cell: tuple[int, int]
) -> list[tuple[int, int]]:
    """Path of cells start -> target by following parent pointers (acyclic)."""
    height, width = parent.shape[:2]
    cells: list[tuple[int, int]] = []
    cur = target_cell
    for _ in range(height * width):
        cells.append(cur)
        if cur == start_cell:
            break
        pr, pc = int(parent[cur[0], cur[1], 0]), int(parent[cur[0], cur[1], 1])
        if pr < 0:
            break
        cur = (pr, pc)
    cells.reverse()
    return cells


def _cell_blocked(
    blocked: np.ndarray, resolution: float, origin: tuple[float, float], point: np.ndarray
) -> bool:
    ox, oy = origin
    col = round((point[0] - ox) / resolution)
    row = round((point[1] - oy) / resolution)
    if row < 0 or col < 0 or row >= blocked.shape[0] or col >= blocked.shape[1]:
        return False
    return bool(blocked[row, col])


def _smooth_positions(
    points: list[np.ndarray],
    iterations: int,
    blocked: np.ndarray,
    resolution: float,
    origin: tuple[float, float],
) -> list[np.ndarray]:
    """Moving-average smooth the path, keeping endpoints and the vehicle clear.

    Rounds the 45-degree joints of the grid (octile) descent into a smooth curve.
    Endpoints are pinned, and a smoothed point is rejected (kept at its original)
    if it would land in inflated (body-blocked) space, so smoothing can never
    shave the vehicle's clearance into an obstacle.
    """
    if len(points) < 3:
        return points
    pts = [p.copy() for p in points]
    for _ in range(max(0, iterations)):
        nxt = [pts[0]]
        for i in range(1, len(pts) - 1):
            avg = 0.25 * pts[i - 1] + 0.5 * pts[i] + 0.25 * pts[i + 1]
            if not _cell_blocked(blocked, resolution, origin, avg):
                nxt.append(avg)
            else:
                nxt.append(pts[i])
        nxt.append(pts[-1])
        pts = nxt
    return pts


def _blend_heading(goal_dir: np.ndarray, travel: np.ndarray, weight: float) -> float:
    """Blend two heading directions by weight (1 = travel, 0 = goal)."""
    vec = weight * travel + (1.0 - weight) * goal_dir
    if np.hypot(vec[0], vec[1]) < 1e-9:
        vec = travel if np.hypot(*travel) > 1e-9 else goal_dir
    return math.atan2(vec[1], vec[0])


def _assign_headings(
    points: list[np.ndarray], goal: np.ndarray, params: RepulsiveFieldParams
) -> list[tuple[float, float, float]]:
    """Turn a list of positions into (x, y, yaw) poses per the facing options."""
    poses: list[tuple[float, float, float]] = []
    n = len(points)
    for i, p in enumerate(points):
        if i < n - 1:
            travel = _unit(points[i + 1] - p)
        elif i > 0:
            travel = _unit(p - points[i - 1])
        else:
            travel = np.array([1.0, 0.0])
        goal_dir = _unit(goal - p)
        if np.hypot(*goal_dir) < 1e-9:
            goal_dir = travel
        if params.omnidirectional:
            # Holonomic: facing decoupled from motion — face the goal heading.
            yaw = math.atan2(goal_dir[1], goal_dir[0])
        else:
            yaw = _blend_heading(goal_dir, travel, params.face_forward_weight)
        poses.append((float(p[0]), float(p[1]), yaw))
    return poses


def _path_target(
    path: np.ndarray,
    robot_w: tuple[float, float],
    reachable: np.ndarray,
    resolution: float,
    origin: tuple[float, float],
    carrot_lookahead: float = 0.0,
    carrot_gap_max: float = 0.0,
) -> tuple[tuple[int, int], bool]:
    """The "carrot": furthest-along-the-global-path cell the robot can reach.

    Scans the global path forward from the robot's projection and returns the
    *furthest* path point whose cell is reachable in the local costmap (plus
    whether that point is the final goal). This makes the local planner **follow
    the global route** — e.g. the long loop to a far door — instead of aiming
    straight at the goal and stalling where the goal's region is a disconnected
    island in the local window. If no path point ahead is reachable, it heads
    toward the reachable cell nearest the next path point.

    ``carrot_lookahead`` (m of path arc-length) bounds how far along the route the
    carrot may sit. This matters when the global path is a **loop** whose end (the
    goal) comes back *near* the robot in straight-line distance while remaining far
    away *along the route* — e.g. wp4, ~1.5 m south of the robot through a wall but
    ~20 m away via the only (south-door) loop. With no cap, the goal cell is
    reachable directly (the wall is missing from the local costmap), so the carrot
    jumps to it and the robot beelines into the wall and stalls. Capping the carrot
    to a few metres of arc-length makes the robot follow the route incrementally
    (pure-pursuit style) through the door instead. 0 = unbounded (original
    behaviour). When capped, the scan is also contiguous: it stops once unreachable
    route points span more than ``carrot_gap_max`` metres of arc after the followable
    prefix begins (leading unreachable points — the robot's own inflated cell — are
    skipped) so the carrot can't hop a real gap, while single-cell reachability
    flicker doesn't collapse it.
    """
    height, width = reachable.shape
    start = int(np.argmin((path[:, 0] - robot_w[0]) ** 2 + (path[:, 1] - robot_w[1]) ** 2))
    best_cell: tuple[int, int] | None = None
    best_idx = -1
    if carrot_lookahead > 0.0:
        arc = 0.0
        gap = 0.0
        started = False
        for i in range(start, len(path)):
            step = 0.0
            if i > start:
                step = float(np.hypot(path[i, 0] - path[i - 1, 0], path[i, 1] - path[i - 1, 1]))
                arc += step
            if arc > carrot_lookahead:
                break
            cell = _world_to_cell(
                float(path[i, 0]), float(path[i, 1]), resolution, origin, (height, width)
            )
            if reachable[cell]:
                best_cell = cell
                best_idx = i
                started = True
                gap = 0.0
            elif started:
                gap += step
                if gap > carrot_gap_max:
                    break
    else:
        for i in range(start, len(path)):
            cell = _world_to_cell(
                float(path[i, 0]), float(path[i, 1]), resolution, origin, (height, width)
            )
            if reachable[cell]:
                best_cell = cell
                best_idx = i
    if best_cell is not None:
        return best_cell, best_idx == len(path) - 1

    # Nothing on the route ahead is reachable — aim at the reachable cell closest
    # to the next route point so the robot at least heads onto the route.
    fr, fc = _world_to_cell(
        float(path[start, 0]), float(path[start, 1]), resolution, origin, (height, width)
    )
    rows = np.arange(height)[:, None]
    cols = np.arange(width)[None, :]
    d2 = np.where(reachable, (rows - fr) ** 2 + (cols - fc) ** 2, np.inf)
    flat = int(np.argmin(d2))
    return (flat // width, flat % width), False


def plan_path(
    cost: np.ndarray,
    resolution: float,
    origin: tuple[float, float],
    global_path: list[tuple[float, float]] | np.ndarray,
    robot: tuple[float, float, float],
    params: RepulsiveFieldParams | None = None,
    previous_path: list[tuple[float, float]] | np.ndarray | None = None,
    obstacle_dist: tuple[np.ndarray, np.ndarray] | None = None,
    carrot_extension: list[tuple[float, float]] | np.ndarray | None = None,
) -> list[tuple[float, float, float]]:
    """Plan a local path of oriented poses with a repulsive-cost navigation function.

    Args:
        cost: HxW costmap, ROS convention (0 free .. 100 lethal, <0 unknown).
        resolution: metres per cell.
        origin: world (x, y) of cell (row=0, col=0).
        global_path: world-frame (x, y) waypoints to follow (last point = goal).
        robot: world-frame (x, y, yaw) start pose.
        previous_path: last tick's local path (x, y); with ``commitment_weight``
            it biases the plan to stay on it (temporal hysteresis vs. flip-flop).
        carrot_extension: route geometry BEYOND the global path's end (the next
            legs' markers, densified). Extends the carrot scan and the goal so the
            plan holds its horizon through an intermediate leg goal — but is NOT
            part of the path-adherence cost field: its straight marker-to-marker
            segments only approximate the future route and must not attract the
            plan sideways off the real global path (hl33 recorded exactly that
            off-route pull on the wp3->wp4 leg).
        params: tunables; defaults if omitted.

    Returns:
        List of (x, y, yaw) world-frame poses from the robot toward the goal,
        bowing around obstacles with clearance. If the goal is walled off (or
        inside an obstacle), the path runs to the *reachable* cell closest to the
        goal — best effort, never into an obstacle. Empty only if the robot
        itself cannot move (boxed in) or there is no global path.
    """
    params = params or RepulsiveFieldParams()
    path = np.asarray(global_path, dtype=np.float64)
    if path.ndim != 2 or len(path) == 0:
        return []
    # The scan path (carrot + goal) may extend past the global path's end; the
    # adherence field below stays on the REAL global path only.
    scan_path = path
    if carrot_extension is not None:
        ext = np.asarray(carrot_extension, dtype=np.float64)
        if ext.ndim == 2 and len(ext) > 0:
            scan_path = np.vstack([path, ext])
    # The obstacle mask + Euclidean distance-to-obstacle depend only on the costmap.
    # The module re-plans every odometry tick (far faster than the costmap updates),
    # so it caches this per costmap and passes it in as `obstacle_dist` — the EDT was
    # ~a full CPU core recomputed identically each tick (py-spy). Without a cache
    # (tests, direct callers) it is computed here as before.
    if obstacle_dist is not None:
        obstacle, dist = obstacle_dist
        height, width = obstacle.shape
    else:
        cost = np.clip(np.nan_to_num(cost, nan=0.0), 0, 100).astype(np.float64)
        height, width = cost.shape
        if height == 0 or width == 0:
            return []
        obstacle, dist = _obstacle_distance(cost, resolution, params)
    if height == 0 or width == 0:
        return []

    # Inflate obstacles by the robot radius: a cell is traversable only if the
    # whole footprint fits, i.e. it is at least half a vehicle width from any
    # obstacle. This is what makes the vehicle fit through gaps and keeps the
    # path centreline clear — gaps narrower than the vehicle are simply blocked.
    robot_radius = max(0.0, params.vehicle_width * 0.5)
    inflate = robot_radius + max(0.0, params.safety_margin)
    free = ~obstacle & (dist >= inflate)
    blocked = ~free

    goal_w = (float(scan_path[-1, 0]), float(scan_path[-1, 1]))
    robot_w = (float(robot[0]), float(robot[1]))
    goal_arr = np.asarray(goal_w)

    if math.hypot(robot_w[0] - goal_w[0], robot_w[1] - goal_w[1]) <= params.goal_tolerance:
        return _assign_headings([np.asarray(robot_w)], goal_arr, params)

    robot_cell = _nearest_free_cell(
        free, _world_to_cell(*robot_w, resolution, origin, (height, width))
    )
    if robot_cell is None:
        return []  # the robot itself is boxed in — nowhere safe to go

    # Cost layer = base travel + repulsion (clearance) + global-path adherence.
    entry_cost = (
        1.0
        + params.clearance_weight * _clearance_penalty(dist, inflate, params.influence_radius)
        + params.path_weight * _path_distance(path, (height, width), resolution, origin)
    )
    # Temporal commitment: penalise straying from last tick's path so the route
    # doesn't flip-flop when a nearby obstacle wiggles. Cheap hysteresis.
    if params.commitment_weight > 0.0 and previous_path is not None:
        prev = np.asarray(previous_path, dtype=np.float64)
        if prev.ndim == 2 and len(prev) >= 2:
            entry_cost = entry_cost + params.commitment_weight * _path_distance(
                prev, (height, width), resolution, origin
            )
    # One Dijkstra rooted at the robot gives the reachable set and an optimal
    # (acyclic) path to every reachable cell. The target is the furthest point we
    # can reach *along the global route* (the carrot): that is the goal itself
    # when it is open and connected, and otherwise the furthest the robot can
    # follow the route toward it — so a goal whose region is a disconnected
    # island in the local costmap yields route-following progress (toward the far
    # door) instead of a beeline that stalls at the near wall.
    max_radius_cells = (
        int(math.ceil(params.dijkstra_radius / resolution)) if params.dijkstra_radius > 0.0 else 0
    )
    _, parent = _dijkstra_tree(free, entry_cost, resolution, robot_cell, max_radius_cells)
    reachable = parent[:, :, 0] >= 0
    reachable[robot_cell] = True
    target_cell, goal_reachable = _path_target(
        scan_path, robot_w, reachable, resolution, origin, params.carrot_lookahead,
        params.carrot_gap_max,
    )

    cells = _backtrack(parent, robot_cell, target_cell)
    pts = [np.asarray(_cell_center_world(r, c, resolution, origin)) for r, c in cells]
    pts[0] = np.asarray(robot_w)
    if goal_reachable:
        # Pin the exact goal (cells snap to grid centres).
        if math.hypot(pts[-1][0] - goal_w[0], pts[-1][1] - goal_w[1]) > resolution:
            pts.append(goal_arr.copy())
        else:
            pts[-1] = goal_arr.copy()
    # else: leave the endpoint at the safe target cell centre (best effort).

    if params.horizon > 0.0:
        truncated = [pts[0]]
        travelled = 0.0
        for i in range(1, len(pts)):
            travelled += float(np.hypot(*(pts[i] - pts[i - 1])))
            truncated.append(pts[i])
            if travelled >= params.horizon:
                break
        pts = truncated

    pts = _smooth_positions(pts, params.smoothing_iterations, blocked, resolution, origin)
    return _assign_headings(pts, goal_arr, params)


# --------------------------------------------------------------------------- #
# DimOS module wrapper.                                                        #
# --------------------------------------------------------------------------- #
def _densify_polyline(
    points: list[tuple[float, float]], step: float
) -> list[tuple[float, float]]:
    """Resample a sparse polyline at ~``step``-metre spacing (endpoints preserved).

    The route tail is bare marker positions many metres apart; the carrot scan and
    the path-adherence field both walk path POINTS, so a sparse segment would leap
    the whole arc budget in one stride. Densifying keeps their per-point arithmetic
    meaningful."""
    if len(points) < 2:
        return list(points)
    out = [points[0]]
    for (ax, ay), (bx, by) in zip(points, points[1:]):
        n = max(1, int(math.hypot(bx - ax, by - ay) / step))
        for k in range(1, n + 1):
            out.append((ax + (bx - ax) * k / n, ay + (by - ay) * k / n))
    return out


def yaw_of_quaternion(qx: float, qy: float, qz: float, qw: float) -> float:
    return math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))


def quaternion_of_yaw(yaw: float) -> tuple[float, float, float, float]:
    return (0.0, 0.0, math.sin(yaw * 0.5), math.cos(yaw * 0.5))


from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.nav_msgs.OccupancyGrid import OccupancyGrid
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.nav_msgs.Path import Path as NavPath


class RepulsiveFieldLocalPlannerConfig(ModuleConfig):
    """Config for :class:`RepulsiveFieldLocalPlanner` (mirrors the params)."""

    body_frame: str = "base_link"
    world_frame: str = "map"
    # When True, publish ``local_path`` in the robot's base frame (poses relative
    # to the current odometry pose) instead of the world frame. The CMU
    # ``PathFollower`` consumes a vehicle-frame route (it does not tf-transform the
    # route), so pair this planner with that follower by setting this True.
    output_base_frame: bool = False

    lethal_threshold: int = 50
    vehicle_width: float = 0.5
    # Extra hard clearance beyond the body. At full speed the robot can OUTRUN its
    # costmap into lidar-shadowed pockets: a concave notch in a wall is invisible until
    # scanned from up close, and a path grazing the mapped face let the base drive
    # nose-first into the unmapped notch and wedge (recorded twice at the cross-wall
    # notch; the following leg then started from inside it). 0.1 m keeps the path far
    # enough off mapped faces that unmapped concavities behind them stay out of reach,
    # while the narrowest course passages (0.9 m doorways, the 1.0 m stair corridor at
    # 0.35 m effective inflation) remain passable (offline: stairs ascent 0/51 blocked,
    # min clearance 0.50 m).
    safety_margin: float = 0.1
    influence_radius: float = 0.8
    clearance_weight: float = 4.0
    path_weight: float = 0.35
    # Live-robot defaults below carry the dim_city course lessons (the pure-core
    # RepulsiveFieldParams keep 0 = off so offline tools stay whole-map/unbounded):
    # commitment: temporal hysteresis so the route doesn't flip-flop between two
    # valid options as the live costmap shifts tick-to-tick (froze the robot near wp4).
    commitment_weight: float = 2.0
    # carrot cap (m of path arc-length): follow the route incrementally instead of
    # beelining to a goal that is near in straight-line but far along a loop (wp4's
    # room is only reachable via the ~40 m south-door loop; unbounded carrot drove
    # into the wall between).
    carrot_lookahead: float = 4.0
    # Tolerated arc (m) of unreachable route cells in the carrot scan: single-cell
    # reachability flicker (slice/terrain updates around stairs and doorways) was
    # collapsing the carrot — and the whole local path — to under a metre for a tick
    # (recorded as sub-metre local_path dips and hesitation stops). Real walls span
    # far more than 1 m of route arc, so they still stop the scan.
    carrot_gap_max: float = 1.0
    # search window (m) around the robot: per-tick wavefront solves must stay cheap.
    # Whole-map solves on an 18 m terrain slice took ~1.2 s; and on an OPEN costmap
    # (the level-aware stairs config) even an 8 m window grew to ~0.28 s/solve, which
    # dropped local_path to ~1.3 Hz — at the PathFollower's 0.8 s dead-man — and
    # stop-flickered the robot. 6 m (with the 4 m carrot) is ample and ~0.16 s/solve.
    dijkstra_radius: float = 6.0
    horizon: float = 3.0  # rolling horizon for the live robot (m)
    goal_tolerance: float = 0.15
    smoothing_iterations: int = 12
    face_forward_weight: float = 0.8
    omnidirectional: bool = False
    # The committed local plan is re-emitted every odometry tick and only re-solved
    # (the expensive wavefront Dijkstra) when it is invalidated. If the robot strays
    # more than this far (m) from the committed path, the plan is stale — replan.
    replan_deviation_meters: float = 0.35
    # No-progress invalidation (s): if the robot advances less than ~0.1 m along the
    # committed plan for this long, the plan AND the commitment bias are both suspect —
    # drop them and re-solve fresh. Without this, a stalled robot never trips the
    # deviation check (it isn't moving), the stale plan re-emits every tick, and
    # commitment_weight resurrects its shape on re-solves: a deadlock observed live as
    # the local path flip-flopping between a fresh forward route (bearing ~16 deg) and a
    # stale backward one (>90 deg) for 80+ s while the costmap ahead was open. 0 disables.
    no_progress_replan_s: float = 5.0
    # Speed-adaptive horizon: the carrot (and with it the search window and the
    # emitted-plan horizon) EXPANDS with the robot's live speed —
    # ``carrot = clamp(speed * carrot_lookahead_time_s, carrot_lookahead,
    # carrot_lookahead_max)`` — so at 3x cruise the planner sees and commits
    # proportionally further ahead (roughly constant preview TIME), while at low
    # speed behaviour stays as tight as today. The Dijkstra window follows the
    # carrot (+2 m margin for detours), as does the emitted horizon.
    # 4 s of preview: cruise 0.55 stays at the 4 m floor (today's behaviour), the 3x
    # cruise (1.65) expands to ~6.6 m. The cap bounds the Dijkstra window (carrot+2m)
    # so a fast open-field solve stays ~2x today's cost, not quadratically worse.
    carrot_lookahead_time_s: float = 4.0
    carrot_lookahead_max: float = 8.0
    # Route-tail reversal trim (deg): the carrot extension stops at the first tail
    # segment that turns more than this from the approach direction. Extending the
    # path THROUGH a reversal (hairpin) is unachievable with a shortest-path local
    # planner: once the carrot passes the apex, the Dijkstra beelines to the return
    # branch (it has no reason to visit the apex), so the PLAN cuts the corner
    # before the waypoint's advance circle — measured: hairpin variants scored
    # churn 51.5 and 57.8 deg/m with 19 and 52 double-backs (one run MISSED the
    # top-of-stairs marker entirely); the trimmed variant scored 27.1 with 6.
    # Corners the follower can arc (<= ~92 deg, its rotate-in-place cutoff) stay
    # below this and keep flowing; a true reversal pins briefly and the follower
    # decelerates-pivots-goes. 180 disables (documented tradeoff above).
    tail_reversal_trim_deg: float = 100.0
    # Same-goal alternative-route debounce (see handle_global_path): a reroute whose
    # deviation exceeds the threshold (m) must persist CONSISTENTLY for this long (s)
    # before the local planner follows it. The global planner flip-flops between
    # near-equal-cost routes at decision points in phases of ~2-8 s (recorded: the
    # stairs corridor <-> east terrace, and the wp4 north <-> south loop, deviations
    # up to 13 m publish-to-publish); a count-based debounce (3 publishes ~ 7 s)
    # still landed most phases. 10 s outlasts the phases; a GENUINE reroute lags by
    # the same 10 s, during which the robot keeps the old route and the local
    # blocked-replan still protects it physically.
    route_change_persist_s: float = 10.0
    route_reroute_threshold_m: float = 2.0
    # Consumption-feedback horizon boost (Jeff): if the follower nears the END of
    # the committed plan before a refresh lands (replan latency > consumption
    # time), the plan's max length is EXTENDED — boost grows by _step each such
    # event up to _max (m), and decays by _decay on every comfortable refresh.
    horizon_boost_max: float = 4.0
    horizon_boost_step: float = 0.5
    horizon_boost_decay: float = 0.9
    # Routine re-solve period (s): even a still-valid committed plan is re-solved at
    # least this often, so the local path tracks costmap/terrain changes promptly and
    # sits near its full horizon length instead of eroding to the refresh margin
    # between event-driven replans (the recorded sawtooth read as jerky on video).
    # Solves are ~0.2-0.3 s off the event loop; the odometry handler awaits each one,
    # so the effective rate self-limits near the solve cost. 0 disables (event-driven
    # replans only). 0.25 (was 0.4) after Jeff's vid26 "update a bit faster" steer.
    replan_period_s: float = 0.25
    # 60 Hz output: the wavefront solve stays at its own (odometry/costmap) cadence,
    # but the committed plan is re-emitted at ``emit_hz`` re-anchored to a
    # dead-reckoned robot pose (last odometry advanced by the smoothed velocity
    # estimate), so the follower sees a smooth, motion-tracking path stream instead
    # of a slower, churning one. Emission pauses when odometry is older than
    # ``emit_max_odom_age_s`` (the follower's own cmd watchdog then stops the base).
    emit_hz: float = 60.0
    emit_max_odom_age_s: float = 0.5

    def to_params(self, speed: float = 0.0, horizon_boost: float = 0.0) -> RepulsiveFieldParams:
        carrot = self.carrot_lookahead
        if self.carrot_lookahead_time_s > 0.0 and speed > 0.0 and carrot > 0.0:
            carrot = min(
                self.carrot_lookahead_max,
                max(carrot, speed * self.carrot_lookahead_time_s),
            )
        # Consumption feedback (Jeff): when the follower keeps nearing the end of
        # the plan before a refresh lands, the module grows horizon_boost — carrot
        # AND horizon are extended (plan length is the min of both) so replanning
        # latency stops surfacing as end-of-path braking. The boosted carrot stays
        # under the speed-cap + one boost step of headroom so the Dijkstra window
        # (carrot + 2) cannot balloon and slow the very solves the boost is
        # compensating for (hl56: an uncapped ratchet ran the feedback away).
        if carrot > 0.0 and horizon_boost > 0.0:
            carrot = min(carrot + horizon_boost, self.carrot_lookahead_max + 2.0)
        radius = self.dijkstra_radius
        if radius > 0.0:
            radius = max(radius, carrot + 2.0)
        horizon = self.horizon
        if horizon > 0.0:
            horizon = max(horizon, 0.75 * carrot)
        return RepulsiveFieldParams(
            lethal_threshold=self.lethal_threshold,
            vehicle_width=self.vehicle_width,
            safety_margin=self.safety_margin,
            influence_radius=self.influence_radius,
            clearance_weight=self.clearance_weight,
            path_weight=self.path_weight,
            commitment_weight=self.commitment_weight,
            carrot_lookahead=carrot,
            carrot_gap_max=self.carrot_gap_max,
            dijkstra_radius=radius,
            horizon=horizon,
            goal_tolerance=self.goal_tolerance,
            smoothing_iterations=self.smoothing_iterations,
            face_forward_weight=self.face_forward_weight,
            omnidirectional=self.omnidirectional,
        )


class RepulsiveFieldLocalPlanner(Module):
    """Navigation-function local planner: costmap + global_path -> local_path.

    Emits an oriented ``local_path`` (a list of poses, position *and* yaw) that
    follows the global path and bows around ``costmap`` obstacles with clearance.
    See the module docstring for the algorithm (a wavefront navigation function
    over a repulsive cost field — loop- and local-minimum-free by construction).
    """

    config: RepulsiveFieldLocalPlannerConfig

    # frame:map — 2D costmap (ROS occupancy convention)
    costmap: In[OccupancyGrid]
    # frame:map — route to follow
    global_path: In[NavPath]
    # frame:map — marker positions of the route BEYOND the current leg goal
    # (MapMemManager). Appended (densified) past the global path so the local path
    # keeps its full horizon through intermediate waypoints — its length then only
    # shrinks approaching the route's TRUE final goal. Optional: never published
    # outside marker-route runs, and an empty tail is a no-op.
    route_tail: In[NavPath]
    odometry: In[Odometry]

    # frame:map (or base_link if output_base_frame) — oriented local path
    local_path: Out[NavPath]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._cost: np.ndarray | None = None
        # Cached (obstacle mask, metres-to-obstacle) for the current costmap. The EDT
        # depends only on the costmap, so it is computed once per costmap update
        # (handle_costmap) and reused across every odometry-driven replan.
        self._obstacle_dist: tuple[np.ndarray, np.ndarray] | None = None
        self._resolution = 0.05
        self._origin = (0.0, 0.0)
        self._path_world: list[tuple[float, float]] = []
        self._route_tail: list[tuple[float, float]] = []
        self._prev_local: list[tuple[float, float]] | None = None
        # No-progress tracking for the committed plan (see no_progress_replan_s).
        self._best_remaining_arc: float = float("inf")
        self._last_progress_monotonic: float = 0.0
        # Committed local plan (world-frame oriented poses) reused across odometry
        # ticks; only re-solved on invalidation (see _plan_needs_replan). _plan_dirty
        # forces a replan on the next tick (set when the global route changes).
        self._plan: list[tuple[float, float, float]] | None = None
        self._plan_dirty = True
        self._dump_count = 0
        # 60 Hz emission state: latest odometry pose (world x, y, yaw) with its
        # monotonic receive time, and a smoothed world-frame velocity estimate
        # (vx, vy, yaw rate) finite-differenced from consecutive odometry samples.
        self._last_pose: tuple[float, float, float] | None = None
        self._last_pose_mono: float = 0.0
        self._vel_est: tuple[float, float, float] = (0.0, 0.0, 0.0)
        self._emit_thread: threading.Thread | None = None
        self._emit_stop = threading.Event()
        # Debounce state for the path-ahead-blocked invalidation (see _plan_needs_replan).
        self._costmap_generation = 0
        self._blocked_costmap_id: int | None = None
        # Monotonic time of the last adopted solve (see replan_period_s).
        self._last_solve_mono: float = 0.0
        # Same-goal alternative-route debounce state (see handle_global_path).
        self._pending_route: list[tuple[float, float]] | None = None
        self._pending_route_since: float = 0.0
        # Why the plan is dirty: "goal" (moved goal — adopt unconditionally) or
        # "refine" (same-goal shape jitter — routine, adoption-gated).
        self._plan_dirty_reason: str = "dirty"
        # Consecutive adoption-gate rejections (anti-livelock; see _adopt_solve).
        self._gate_rejections: int = 0
        # Consumption-feedback horizon boost (m); see config.horizon_boost_*.
        self._horizon_boost: float = 0.0

    async def main(self) -> AsyncIterator[None]:
        # The emitter runs on its OWN thread (like MujocoNavBase's physics loop): as an
        # asyncio task it competed with the handler queue for the event loop and only
        # reached 20-49 Hz on the bus under course load (live-probed).
        self._emit_stop.clear()
        self._emit_thread = threading.Thread(
            target=self._emit_loop, daemon=True, name="RepulsiveFieldEmitter"
        )
        self._emit_thread.start()
        yield
        self._emit_stop.set()
        if self._emit_thread is not None:
            self._emit_thread.join(timeout=2.0)

    async def handle_costmap(self, msg: OccupancyGrid) -> None:
        if msg.grid.size == 0:
            self._cost = None
            self._obstacle_dist = None
            return
        self._cost = msg.grid.astype(np.float64)
        self._costmap_generation += 1
        self._resolution = float(msg.resolution)
        self._origin = (float(msg.origin.position.x), float(msg.origin.position.y))
        # Perf: precompute the obstacle Euclidean distance transform once per costmap.
        # handle_odometry replans every tick (much faster than the costmap rate) and
        # would otherwise recompute this identical transform each time — py-spy showed
        # it pinning ~a full core. It depends only on the costmap + lethal_threshold.
        clipped = np.clip(np.nan_to_num(self._cost, nan=0.0), 0, 100).astype(np.float64)
        # Off the event loop: the EDT is 100-300 ms of scipy on course-sized costmaps,
        # and inline it stalls the 60 Hz emitter and the odometry subscriber alike.
        self._obstacle_dist = await asyncio.to_thread(
            _obstacle_distance, clipped, self._resolution, self.config.to_params()
        )

    async def handle_global_path(self, msg: NavPath) -> None:
        new_path = [(float(p.position.x), float(p.position.y)) for p in msg.poses]
        # The global planner republishes the route every tick, usually a near-identical
        # path (sub-voxel jitter / minor re-solve). Only a MEANINGFUL reroute should
        # invalidate the committed local plan — otherwise every republish would force a
        # replan and defeat the cache (py-spy: this was ~half of all replans). A route
        # counts as changed if the goal moved or the path deviates from the previous one
        # by more than a couple of cells (compared after arc-length resampling so it is
        # robust to the point count changing).
        #
        # A same-goal ALTERNATIVE route (deviation > route_reroute_threshold_m) is
        # additionally DEBOUNCED: at a near-equal-cost decision point the global
        # planner can flip-flop between two routes publish-to-publish (recorded on
        # the stairs approach: stairs corridor <-> east terrace, 4 flips in 15 s),
        # and chasing each flip drove the robot back and forth (the double-back
        # clusters in the churn metric). The alternative must persist across
        # route_change_debounce_n consecutive publishes before it is adopted; goal
        # advances and refinements stay immediate.
        if len(new_path) >= 2 and len(self._path_world) >= 2:
            goal_moved = (
                math.hypot(
                    new_path[-1][0] - self._path_world[-1][0],
                    new_path[-1][1] - self._path_world[-1][1],
                )
                > self._resolution
            )
            deviation = self._route_deviation(self._path_world, new_path)
            if not goal_moved and deviation > self.config.route_reroute_threshold_m:
                now = time.monotonic()
                if (
                    self._pending_route is not None
                    and self._route_deviation(self._pending_route, new_path) < 1.0
                ):
                    if now - self._pending_route_since < self.config.route_change_persist_s:
                        return  # not stable long enough — keep the committed route
                else:
                    self._pending_route = new_path
                    self._pending_route_since = now
                    return  # a NEW alternative: start its persistence clock
        self._pending_route = None
        self._pending_route_since = 0.0
        if self._route_changed(new_path):
            self._plan_dirty = True
            # A same-goal REFINEMENT (the global start tracks the robot, the shape
            # jitters 0.4-2.4 m publish-to-publish on the stairs) is ROUTINE — the
            # adoption gate applies. Only a moved goal is an unconditional adopt.
            goal_moved = (
                len(new_path) < 2
                or len(self._path_world) < 2
                or math.hypot(
                    new_path[-1][0] - self._path_world[-1][0],
                    new_path[-1][1] - self._path_world[-1][1],
                )
                > self._resolution
            )
            self._plan_dirty_reason = "goal" if goal_moved else "refine"
        self._path_world = new_path

    async def handle_route_tail(self, msg: NavPath) -> None:
        # No dirty flag: the tail only extends the carrot scan past the leg goal, and
        # the routine replan (replan_period_s) picks a tail change up within a period.
        self._route_tail = [(float(p.position.x), float(p.position.y)) for p in msg.poses]

    def _carrot_extension(self) -> list[tuple[float, float]]:
        """Densified route tail (markers beyond the current leg goal) for the carrot.

        With an extension, the carrot keeps advancing past an intermediate waypoint,
        so the local path holds its full horizon instead of pinning (and shrinking)
        at every leg goal; only the route's true final goal terminates it. The tail's
        straight marker-to-marker segments may cross walls — the carrot scan's
        reachability + gap cap stop it there, and any carrot it does pick is reached
        via the Dijkstra tree, so the approximation is safe until the real next-leg
        path arrives. Passed to ``plan_path`` SEPARATELY from the global path so it
        extends only the carrot/goal, never the path-adherence field.

        The tail is TRIMMED at the first sharp route REVERSAL: extending the carrot
        through a >tail_reversal_trim_deg turn makes the effective path hook back
        toward (or behind) the approaching robot, and the follower starts cutting
        toward the hook before the waypoint is reached — recorded (hl40) as repeated
        turn-around/double-back on the last few stairs before the top marker (whose
        next leg reverses 180 deg back down), and at the wp1/wp3 reversal corners.
        A reversal waypoint needs a stop-and-turn anyway, so the path pinning (and
        shrinking) into it is the physically honest shape; gentle corners still flow.
        """
        if not self._path_world or not self._route_tail:
            return []
        # Cap the tail at what the carrot could ever use (its max arc + margin).
        budget = self.config.carrot_lookahead_max + 2.0
        trim_cos = math.cos(math.radians(self.config.tail_reversal_trim_deg))
        # Approach direction at the leg's end (the last global-path segment).
        heading: tuple[float, float] | None = None
        if len(self._path_world) >= 2:
            (ax, ay), (bx, by) = self._path_world[-2], self._path_world[-1]
            norm = math.hypot(bx - ax, by - ay)
            if norm > 1e-6:
                heading = ((bx - ax) / norm, (by - ay) / norm)
        tail: list[tuple[float, float]] = [self._path_world[-1]]
        for point in self._route_tail:
            if budget <= 0.0:
                break
            dx, dy = point[0] - tail[-1][0], point[1] - tail[-1][1]
            norm = math.hypot(dx, dy)
            if norm <= 1e-6:
                continue
            if heading is not None and (dx * heading[0] + dy * heading[1]) / norm < trim_cos:
                break  # sharp reversal: pin the path at this waypoint instead
            heading = (dx / norm, dy / norm)
            budget -= norm
            tail.append(point)
        return _densify_polyline(tail, 0.25)[1:]

    def _effective_path(self) -> list[tuple[float, float]]:
        """Global path + carrot extension: the full route geometry the plan follows."""
        return self._path_world + self._carrot_extension()

    @staticmethod
    def _route_deviation(
        old: list[tuple[float, float]], new: list[tuple[float, float]]
    ) -> float:
        """Max pointwise deviation (m) between two routes after arc resampling."""
        a = _resample_arclen(new, 24)
        b = _resample_arclen(old, 24)
        return float(np.max(np.hypot(a[:, 0] - b[:, 0], a[:, 1] - b[:, 1])))

    def _route_changed(self, new_path: list[tuple[float, float]]) -> bool:
        old = self._path_world
        if len(new_path) < 2 or len(old) < 2:
            return new_path != old
        # Goal (endpoint) moved more than a cell -> a real change.
        if math.hypot(new_path[-1][0] - old[-1][0], new_path[-1][1] - old[-1][1]) > self._resolution:
            return True
        return self._route_deviation(old, new_path) > 2.0 * self._resolution

    def _update_velocity_estimate(self, robot: tuple[float, float, float]) -> None:
        """Smoothed world-frame velocity from consecutive odometry samples (EMA)."""
        now = time.monotonic()
        prev, prev_t = self._last_pose, self._last_pose_mono
        self._last_pose = robot
        self._last_pose_mono = now
        if prev is None:
            return
        dt = now - prev_t
        if dt <= 1e-4 or dt > 1.0:
            return
        vx = (robot[0] - prev[0]) / dt
        vy = (robot[1] - prev[1]) / dt
        wz = math.atan2(math.sin(robot[2] - prev[2]), math.cos(robot[2] - prev[2])) / dt
        alpha = 0.35  # EMA: smooth odometry quantization without lagging real accel much
        ex, ey, ew = self._vel_est
        self._vel_est = (
            ex + alpha * (vx - ex),
            ey + alpha * (vy - ey),
            ew + alpha * (wz - ew),
        )

    @property
    def _speed_estimate(self) -> float:
        return math.hypot(self._vel_est[0], self._vel_est[1])

    async def handle_odometry(self, msg: Odometry) -> None:
        if self._cost is None or not self._path_world:
            return
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        yaw = yaw_of_quaternion(orientation.x, orientation.y, orientation.z, orientation.w)
        robot = (float(position.x), float(position.y), yaw)
        self._update_velocity_estimate(robot)
        params = self.config.to_params(
            speed=self._speed_estimate, horizon_boost=self._horizon_boost
        )
        # The wavefront Dijkstra is the planner's cost (py-spy: ~a full core). Run it
        # only when the committed plan is invalidated — a new global route, the robot
        # straying off the path, or the live costmap now blocking the path ahead —
        # and otherwise re-emit the committed plan. Output still updates every
        # odometry tick (re-anchored to the current pose below), just without
        # re-solving an unchanged plan.
        extension = self._carrot_extension()
        effective_path = self._path_world + extension
        reason = self._plan_needs_replan(robot, params, effective_path)
        if reason is not None:
            # Solve OFF the event loop: a wavefront solve is 100-300 ms of pure
            # numpy, and running it inline froze the 60 Hz emitter for its duration
            # (recorded local_path gaps up to ~solve length). The handler awaits the
            # worker thread, so odometry ticks stay serialized; the emitter keeps
            # re-anchoring the previous committed plan meanwhile.
            poses_xyyaw = await asyncio.to_thread(
                plan_path,
                self._cost,
                self._resolution,
                self._origin,
                self._path_world,
                robot,
                params,
                previous_path=self._prev_local,
                obstacle_dist=self._obstacle_dist,
                carrot_extension=extension or None,
            )
            self._last_solve_mono = time.monotonic()
            adopted = self._adopt_solve(poses_xyyaw, robot, reason)
            # Diagnostic breadcrumb (one line per SOLVE, ~1-4 Hz): reason + adoption
            # + the solve's initial world bearing/arc. This is what lets a recorded
            # course flip be attributed to a reason offline instead of inferred.
            if len(poses_xyyaw) >= 2:
                head = poses_xyyaw[: min(6, len(poses_xyyaw))]
                bearing = math.degrees(
                    math.atan2(head[-1][1] - head[0][1], head[-1][0] - head[0][0])
                )
                arc = sum(
                    math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(poses_xyyaw, poses_xyyaw[1:])
                )
                logger.info(
                    "solve",
                    reason=reason,
                    adopted=adopted,
                    bearing=round(bearing, 1),
                    arc=round(arc, 2),
                )
            if adopted:
                self._plan = poses_xyyaw
                self._plan_dirty = False
                self._best_remaining_arc = float("inf")
                self._last_progress_monotonic = time.monotonic()
                # Remember this plan (world-frame) for next-tick temporal commitment.
                self._prev_local = (
                    [(x, y) for x, y, _ in poses_xyyaw] if len(poses_xyyaw) >= 2 else None
                )
            else:
                poses_xyyaw = self._plan or []
        else:
            poses_xyyaw = self._plan or []
        self._maybe_dump_costmap(
            rx=float(position.x), ry=float(position.y), yaw=yaw, plan=poses_xyyaw
        )
        self._publish_plan(poses_xyyaw, (float(position.x), float(position.y), yaw))

    def _publish_plan(
        self,
        poses_xyyaw: list[tuple[float, float, float]],
        robot: tuple[float, float, float],
    ) -> None:
        """Publish the (world-frame) plan, re-anchored to ``robot`` for base-frame output.

        The plan is in the world frame. The CMU PathFollower wants a vehicle-frame
        route (it steers by pure pursuit on the path as-is and never tf-transforms
        it), so optionally rotate each pose into the robot's base frame at the given
        pose before publishing. Shared by the odometry-tick path and the 60 Hz
        emitter (which passes a dead-reckoned pose between odometry samples).
        """
        to_base = self.config.output_base_frame
        out_frame = self.config.body_frame if to_base else self.config.world_frame
        rx, ry, yaw = robot
        # Emit from the robot's projection onto the plan: a committed plan keeps its
        # solve-time start, so as the robot advances the leading poses fall BEHIND it —
        # in the base frame the path then starts metres back, and the follower's
        # look-ahead (first point >= L from the origin, in path order) can select a
        # behind-point and command a turn-around (observed at 3x cruise: |p0| up to
        # 2.8 m behind, stop-and-go tracking). Trimming keeps the emitted start at the
        # robot, which is also what makes consecutive 60 Hz emissions track its motion.
        if len(poses_xyyaw) > 1:
            # Whole-plan argmin — measured, not assumed: a MONOTONIC advance-only
            # projection was trialled for hairpin plans (whose branches overlap) and
            # regressed churn — the emitter dead-reckons the pose ahead of odometry,
            # the monotonic index locked that overshoot in until the next adoption,
            # and the emitted path start rode ahead of the robot. Hairpins are gone
            # (the carrot tail trims at reversals), so the self-correcting argmin
            # is unambiguous again.
            nearest = min(
                range(len(poses_xyyaw)),
                key=lambda i: (poses_xyyaw[i][0] - rx) ** 2 + (poses_xyyaw[i][1] - ry) ** 2,
            )
            poses_xyyaw = poses_xyyaw[nearest:]
        cos_y, sin_y = math.cos(yaw), math.sin(yaw)
        poses = []
        for x, y, pose_yaw in poses_xyyaw:
            if to_base:
                dx, dy = x - rx, y - ry
                px = cos_y * dx + sin_y * dy
                py = -sin_y * dx + cos_y * dy
                p_yaw = math.atan2(math.sin(pose_yaw - yaw), math.cos(pose_yaw - yaw))
            else:
                px, py, p_yaw = x, y, pose_yaw
            qx, qy, qz, qw = quaternion_of_yaw(p_yaw)
            poses.append(
                PoseStamped(
                    frame_id=out_frame,
                    position=[px, py, 0.0],
                    orientation=[qx, qy, qz, qw],
                )
            )
        self.local_path.publish(NavPath(frame_id=out_frame, poses=poses))

    def _emit_loop(self) -> None:
        """Re-emit the committed plan at ``emit_hz``, dead-reckoned between odometry.

        The follower's control rate is capped by how often a path arrives; odometry
        (and the solve) tick far slower than 60 Hz, so between samples the robot pose
        is EXTRAPOLATED along the smoothed velocity estimate and the committed
        world-frame plan is re-anchored to that predicted pose. Consecutive emissions
        therefore track the robot's motion (the base-frame path start stays at the
        robot) instead of repeating a stale snapshot. Emission pauses when odometry
        goes stale so the follower's cmd watchdog can stop the base.

        Runs on a dedicated thread with absolute deadlines (sleep(period) drifts by
        the per-tick work; deadlines resynchronize after a stall instead of bursting
        stale catch-up emissions). Shared state (_plan/_last_pose/_vel_est) is read
        as atomic reference swaps.
        """
        period = 1.0 / self.config.emit_hz if self.config.emit_hz > 0 else 0.0
        if period <= 0.0:
            return
        next_emit = time.monotonic() + period
        while not self._emit_stop.is_set():
            delay = next_emit - time.monotonic()
            if delay > 0:
                time.sleep(delay)
            next_emit = max(next_emit + period, time.monotonic())
            plan = self._plan
            pose = self._last_pose
            if not plan or pose is None:
                continue
            age = time.monotonic() - self._last_pose_mono
            if age > self.config.emit_max_odom_age_s:
                continue  # odometry stale: stop feeding the follower fresh paths
            vx, vy, wz = self._vel_est
            x, y, yaw = pose
            # Unicycle dead-reckoning: rotate the world-frame velocity with the
            # extrapolated yaw at small angle (first order is plenty for <=0.5 s).
            x += vx * age
            y += vy * age
            yaw += wz * age
            self._publish_plan(plan, (x, y, yaw))

    def _adopt_solve(
        self,
        new_plan: list[tuple[float, float, float]],
        robot: tuple[float, float, float],
        reason: str,
    ) -> bool:
        """Whether a fresh solve should replace the committed plan.

        Event-driven replans (a MOVED goal, drift, a confirmed blockage, arrival,
        no-progress) always adopt — the committed plan is known-bad. But a ROUTINE
        refresh (periodic / rolling horizon / same-goal route refinement) re-solves
        a plan that is still valid, and two transient failure shapes must not
        replace it:

        - drastically SHORTER (carrot collapse from reachability flicker): the
          follower would get a sub-metre path — recorded as hesitation stops. The
          gate self-limits because the committed remainder erodes toward the short
          solve's length.
        - direction REVERSAL (> ~90 deg initial-bearing swing): during the stairs
          descent the band-edge flicker made every other solve point back UP while
          the global route ran stably DOWN — each was adopted via the 'refinement'
          dirty path and the robot visibly turned around mid-staircase (Jeff,
          vid28). A committed plan that is still collision-free wins over a
          contradicting routine solve; a genuinely necessary about-turn arrives as
          a "goal"/"blocked"/"no_progress" replan, which always adopts.
        """
        if reason not in ("periodic", "horizon", "refine"):
            self._gate_rejections = 0
            return True
        old = self._plan
        if old is None or len(old) < 2 or len(new_plan) < 2:
            self._gate_rejections = 0
            return True
        # Anti-livelock for LENGTH rejections only: a persistently-short solve
        # means the reachable world genuinely shrank — adopt after 3 in a row.
        # Direction reversals get NO streak override: at the stairs' band-ceiling
        # transition the solver is BISTABLE for seconds, and a blind adoption sent
        # the robot back DOWN from one riser below the top (hl49, course FAIL) —
        # oscillating up/down as each direction was adopted in turn. The release
        # valve for a truly-wrong committed plan is the no-progress replan (5 s),
        # which adopts unconditionally and is reachable now that 'refine' is the
        # lowest-priority reason.
        if self._gate_rejections >= 3:
            self._gate_rejections = 0
            return True
        pw = np.asarray([(p[0], p[1]) for p in old], dtype=np.float64)
        d2 = (pw[:, 0] - robot[0]) ** 2 + (pw[:, 1] - robot[1]) ** 2
        nearest = int(d2.argmin())
        remaining = float(np.sum(np.hypot(np.diff(pw[nearest:, 0]), np.diff(pw[nearest:, 1]))))
        nw = np.asarray([(p[0], p[1]) for p in new_plan], dtype=np.float64)
        new_arc = float(np.sum(np.hypot(np.diff(nw[:, 0]), np.diff(nw[:, 1]))))
        if new_arc < 0.5 * remaining:
            self._gate_rejections += 1
            return False  # transient carrot collapse
        old_dir = self._initial_direction(pw[nearest:])
        new_dir = self._initial_direction(nw)
        if old_dir is not None and new_dir is not None:
            # Reject only NEAR-REVERSALS (> ~120 deg): the recorded descent flips
            # were 135-173 deg, while legitimate climb corners (stairs-base entry)
            # run 90-120 deg — a 90 deg gate broke the ascent (hl48/49). No streak
            # increment: direction rejections must never force a blind adoption.
            if old_dir[0] * new_dir[0] + old_dir[1] * new_dir[1] < -0.5:
                return False  # routine solve reverses the committed direction
        self._gate_rejections = 0
        return True

    @staticmethod
    def _initial_direction(pts: np.ndarray) -> tuple[float, float] | None:
        """Unit direction of the first ~1 m of a plan (None if too short)."""
        if len(pts) < 2:
            return None
        arc = 0.0
        k = 1
        for k in range(1, len(pts)):
            arc += float(np.hypot(pts[k, 0] - pts[k - 1, 0], pts[k, 1] - pts[k - 1, 1]))
            if arc >= 1.0:
                break
        dx, dy = float(pts[k, 0] - pts[0, 0]), float(pts[k, 1] - pts[0, 1])
        norm = math.hypot(dx, dy)
        if norm < 0.3:
            return None
        return (dx / norm, dy / norm)

    def _plan_needs_replan(
        self,
        robot: tuple[float, float, float],
        params: RepulsiveFieldParams,
        effective_path: list[tuple[float, float]],
    ) -> str | None:
        """Why the committed plan is stale (None if it is still good).

        Invalidated by: no plan yet / an explicit dirty flag (a changed global route);
        the robot reaching the goal region; the robot straying more than
        ``replan_deviation_meters`` from the committed path; the plan outliving
        ``replan_period_s``; or any cell of the path ahead of the robot now being
        blocked/too-close-to-an-obstacle in the live costmap. All checks are
        O(path length) — cheap enough to run every tick. The reason string feeds
        the adoption gate (see ``_adopt_solve``).
        """
        plan = self._plan
        if plan is None or len(plan) < 2 or self._obstacle_dist is None:
            return "dirty"
        if self._plan_dirty and self._plan_dirty_reason == "goal":
            return "goal"  # moved goal: unconditional
        # A same-goal REFINEMENT ("refine") is the LOWEST-priority reason — checked
        # at the END. When it outranked the checks below, a stale committed plan
        # could live-lock: every global republish re-marked dirty-refine, that
        # returned first every tick, each (direction-gated) solve was rejected, and
        # the no-progress escape was never reached — recorded (hl48) as the robot
        # parked 120 s beside the stairs while offline solves showed a good path.
        gx, gy = effective_path[-1]
        if math.hypot(robot[0] - gx, robot[1] - gy) <= params.goal_tolerance:
            return "goal"  # at the goal — re-solve (emits the trivial at-goal path)
        pw = np.asarray([(p[0], p[1]) for p in plan], dtype=np.float64)
        d2 = (pw[:, 0] - robot[0]) ** 2 + (pw[:, 1] - robot[1]) ** 2
        nearest = int(d2.argmin())
        if math.sqrt(float(d2[nearest])) > self.config.replan_deviation_meters:
            return "deviation"  # robot drifted off the committed path
        # No-progress invalidation: see no_progress_replan_s in the config.
        if self.config.no_progress_replan_s > 0.0:
            remaining_arc = float(
                np.sum(np.hypot(np.diff(pw[nearest:, 0]), np.diff(pw[nearest:, 1])))
            )
            now_monotonic = time.monotonic()
            if remaining_arc < self._best_remaining_arc - 0.1:
                self._best_remaining_arc = remaining_arc
                self._last_progress_monotonic = now_monotonic
            elif now_monotonic - self._last_progress_monotonic > self.config.no_progress_replan_s:
                self._prev_local = None  # the commitment bias is part of the deadlock
                self._best_remaining_arc = float("inf")
                self._last_progress_monotonic = now_monotonic
                return "no_progress"
        # Refresh the rolling horizon *before* the robot consumes it. The committed plan
        # is only ~horizon metres long; without this, the robot drives to its end, runs
        # out of pure-pursuit lookahead, and stalls until it drifts far enough off the
        # spent path to trip the deviation check above — a stop-go cycle every ~horizon
        # metres that roughly halves the effective speed on long routes (e.g. the wp4
        # loop). If little of the plan remains ahead, re-solve so a fresh horizon is
        # always in front of the robot. Skipped when the plan already reaches the goal
        # (nothing more to extend — the goal-tolerance / drift checks handle arrival).
        # The margin is HALF THE HORIZON, uncapped: an earlier 1.0 m cap meant only
        # ~0.6 s of runway at 3x cruise — the follower reached the plan's end (which
        # it treats as arrival), momentum braked to a full stop, and the next solve
        # restarted it: a surge-and-stall cycle every ~horizon metres (measured
        # rms_accel 1.06 m/s^2 / 38% stop ratio on the recorded 3x course). Since the
        # horizon scales with speed, half of it is a constant-TIME runway (~1.5 s) that
        # comfortably covers a 0.2-0.3 s solve at any cruise.
        if params.horizon > 0.0:
            gxp, gyp = float(pw[-1, 0]), float(pw[-1, 1])
            plan_reaches_goal = math.hypot(gxp - gx, gyp - gy) <= params.goal_tolerance
            if not plan_reaches_goal:
                remaining = float(
                    np.sum(np.hypot(np.diff(pw[nearest:, 0]), np.diff(pw[nearest:, 1])))
                )
                if remaining < 0.5 * params.horizon:
                    # Consumption feedback (Jeff): reaching well past the refresh
                    # margin before this fired means replanning is not keeping up
                    # with consumption — extend the plan; relax when comfortable.
                    # GUARD (hl56 course FAIL): only grow when the last solve
                    # actually FILLED the horizon (plan_arc ~ horizon). On the
                    # stairs, solves are INFORMATION-limited (the band edge caps
                    # them) — every refresh looks starved, an unguarded boost
                    # ratchets to max, the carrot/window balloon, solves slow, and
                    # the feedback runs away. A longer horizon can only help when
                    # the solver could deliver it.
                    plan_arc = float(np.sum(np.hypot(np.diff(pw[:, 0]), np.diff(pw[:, 1]))))
                    if remaining < 0.25 * params.horizon and plan_arc >= 0.8 * params.horizon:
                        self._horizon_boost = min(
                            self.config.horizon_boost_max,
                            self._horizon_boost + self.config.horizon_boost_step,
                        )
                    else:
                        self._horizon_boost *= self.config.horizon_boost_decay
                    return "horizon"  # rolling horizon nearly consumed — refresh it
        # Routine refresh: see replan_period_s in the config.
        if (
            self.config.replan_period_s > 0.0
            and time.monotonic() - self._last_solve_mono > self.config.replan_period_s
        ):
            return "periodic"
        obstacle, dist = self._obstacle_dist
        height, width = obstacle.shape
        inflate = max(0.0, params.vehicle_width * 0.5) + max(0.0, params.safety_margin)
        ox, oy = self._origin
        blocked = False
        for x, y in pw[nearest:]:  # only the portion still ahead of the robot
            col = round((x - ox) / self._resolution)
            row = round((y - oy) / self._resolution)
            if 0 <= row < height and 0 <= col < width and (
                obstacle[row, col] or dist[row, col] < inflate
            ):
                blocked = True
                break
        if blocked:
            # Debounce: a SINGLE costmap showing the path blocked is often transient
            # flicker (slice/terrain updates around stairs and doorways), and reacting
            # to each one churned the plan — recorded as brief mid-course hesitation
            # stops that resume in the same direction. Require the blockage to persist
            # across two different costmaps before re-solving; a real new obstacle
            # still triggers within one costmap period (~0.6 s), well inside the
            # follower's reaction envelope at cruise (braking from 1.65 covers 0.54 m).
            costmap_id = self._costmap_generation
            if self._blocked_costmap_id is not None and self._blocked_costmap_id != costmap_id:
                self._blocked_costmap_id = None
                return "blocked"  # blocked in two consecutive costmaps: genuinely blocked
            self._blocked_costmap_id = costmap_id
            return None if not self._plan_dirty else "refine"
        self._blocked_costmap_id = None
        return "refine" if self._plan_dirty else None

    def _maybe_dump_costmap(
        self, rx: float, ry: float, yaw: float, plan: list[tuple[float, float, float]]
    ) -> None:
        """Diagnostic: when ``JNAV_DUMP_STALL_COSTMAP=<dir>`` is set, save the
        costmap + route + robot + planned path whenever the plan ends well short
        of the goal (the wp4-style "problem" moment), so the real costmap can be
        rendered offline. Off by default; no effect on planning."""
        out_dir = os.environ.get("JNAV_DUMP_STALL_COSTMAP")
        if not out_dir or self._cost is None or not self._path_world or self._dump_count >= 16:
            return
        gx, gy = self._path_world[-1]
        ex, ey = (plan[-1][0], plan[-1][1]) if plan else (rx, ry)
        short = math.hypot(ex - gx, ey - gy)
        if short <= 1.0:
            return
        os.makedirs(out_dir, exist_ok=True)
        np.savez(
            os.path.join(out_dir, f"stall_{self._dump_count:03d}.npz"),
            cost=self._cost,
            resolution=self._resolution,
            origin=np.asarray(self._origin),
            global_path=np.asarray(self._path_world, dtype=np.float64),
            robot=np.asarray([rx, ry, yaw]),
            local_path=np.asarray([[p[0], p[1]] for p in plan], dtype=np.float64)
            if plan
            else np.zeros((0, 2)),
            goal=np.asarray([gx, gy]),
            short=short,
        )
        self._dump_count += 1
