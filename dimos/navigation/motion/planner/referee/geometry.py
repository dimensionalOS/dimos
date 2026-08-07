# Copyright 2025-2026 Dimensional Inc.
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

"""Body-collision geometry the referee scores with.

VENDORED from ``dimos/navigation/motion/obstacle.py`` at commit
d7c1b7c88 (the referee epoch this benchmark's scores are pinned to) —
only the pure-geometry slice the judge needs, with ``dimos.msgs`` types
swapped for :mod:`.types`. The math is verbatim; scores produced through
this module are compared bit-exactly against that commit.

If ``obstacle.py``'s scoring functions ever change on the robot side,
re-vendor deliberately and re-baseline — silently diverging copies of
the scoring math are worse than either copy.
"""

from __future__ import annotations

import math
from typing import Any

import numpy as np

from .types import (
    BaseConfig,
    Path as PathT,
    PointCloud2,
    Pose,
    PoseStamped,
    Quaternion,
    SolidPrimitive,
    Vector3,
)


def angle_diff(a: float, b: float) -> float:
    """Shortest signed angular difference a - b, wrapped to [-pi, pi)."""
    return (a - b + math.pi) % (2 * math.pi) - math.pi


def _pose_matrix(pose: Pose) -> np.ndarray:
    """4x4 homogeneous matrix of a pose (verbatim Transform.to_matrix formula)."""
    x, y, z, w = pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w
    rotation_matrix = np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )
    matrix = np.eye(4)
    matrix[:3, :3] = rotation_matrix
    matrix[:3, 3] = [pose.position.x, pose.position.y, pose.position.z]
    return matrix


# ---------------------------------------------------------------- collider --


class CollisionShape(BaseConfig):
    """One collider volume, posed relative to the robot body origin."""

    primitive: SolidPrimitive
    pose: Pose = Pose()

    def at(self, pose: Pose) -> CollisionShape:
        """This collider placed at a path pose, keeping its body-relative offset."""
        return self.model_copy(update={"pose": pose + self.pose})

    @property
    def bounding_radius(self) -> float:
        """Radius of the sphere bounding the primitive."""
        d = self.primitive.dimensions
        if self.primitive.type == SolidPrimitive.BOX:
            return float(np.linalg.norm(d)) / 2
        if self.primitive.type == SolidPrimitive.SPHERE:
            return float(d[SolidPrimitive.SPHERE_RADIUS])
        # cylinder / cone: height, radius
        return float(np.hypot(d[0] / 2, d[1]))

    def distance(self, points: np.ndarray) -> np.ndarray:
        """Signed distance from world points to the collider surface (negative = inside).

        Exact for box, sphere and cylinder; a cone is bounded by its cylinder.
        """
        mat = _pose_matrix(self.pose)
        local = (points - mat[:3, 3]) @ mat[:3, :3]  # rows: R^T (p - t)
        d = self.primitive.dimensions
        kind = self.primitive.type
        if kind == SolidPrimitive.SPHERE:
            return np.asarray(np.linalg.norm(local, axis=1) - d[SolidPrimitive.SPHERE_RADIUS])
        if kind == SolidPrimitive.BOX:
            q = np.abs(local) - np.asarray(d) / 2
        else:  # cylinder / cone: (height, radius), axis along z
            q = np.stack(
                [np.hypot(local[:, 0], local[:, 1]) - d[1], np.abs(local[:, 2]) - d[0] / 2],
                axis=1,
            )
        outside = np.linalg.norm(np.maximum(q, 0.0), axis=1)
        inside = np.minimum(np.max(q, axis=1), 0.0)
        return np.asarray(outside + inside)


# Unitree Go2 moving-body envelope, measured in the fitted MuJoCo sim (union
# of all robot geometry over stand/fwd/reverse/strafe/spin/arc/crab commands,
# yaw-aligned base frame): 0.852 x 0.495, centre offset -0.009. The swinging
# legs, not the 0.31 m trunk of the official spec, set the width — strafe and
# spin splay them to ~0.50 m. Planner paths are ground-level (z=0), so the box
# is lifted half its height to sit on the ground rather than straddle the
# waypoint.
GO2_BODY = CollisionShape(
    primitive=SolidPrimitive.box(0.85, 0.50, 0.40),
    pose=Pose(-0.01, 0.0, 0.20),
)


class AvoidanceConfig(BaseConfig):
    """Avoidance/scoring parameters (the referee-relevant subset is what the
    judge reads; the rest ride along so the defaults stay one block)."""

    shape: CollisionShape = GO2_BODY
    rate: float = 10.0
    # waypoint spacing after resampling (m); coarse-to-fine starts higher
    resolution: float = 0.1
    # deformation aims for this much room around the body (m); more is not rewarded
    clearance: float = 0.5
    # deform/score only this much path ahead (m, arc length); the rest is the
    # global planner's business — and usually beyond the local map anyway
    horizon: float = 10.0
    # speed governor: creep at min_speed when the body is touching its
    # clearance target's worth of trouble ahead, cruise at max_speed in the
    # open; judged over the next speed_lookahead metres of path
    max_speed: float = 0.5
    min_speed: float = 0.2
    speed_lookahead: float = 2.0
    # clearance (m) at which the governor grants full speed — deliberately
    # smaller than the scoring `clearance` target: deform keeps aiming for
    # 0.5 m of room, but indoors 0.35 m of room is already full-speed room
    speed_clearance: float = 0.35
    # points within this height of a waypoint's own z are walkable surface,
    # not obstacles (m, relative — the floor may climb along the path)
    ground_eps: float = 0.05
    # yaw jump between scored waypoints (rad) beyond which the waypoint is
    # scored as a rotation in place — the box's circumscribing cylinder
    # instead of the yaw-oriented box (the swept volume of the turn)
    turn_yaw_eps: float = 0.5
    # yaw change between scored waypoints (rad) beyond which the waypoint's
    # box is scored swept over the yaw interval (min SDF at interpolated
    # yaws): between samples the rotating rear corner swings wider than
    # either sampled box (field 20:27, contact at -0.19 while rotating)
    sweep_yaw_step: float = 0.15
    # homotopy switch hysteresis (m): the mirrored-side candidate must beat
    # the incumbent side's min clearance by this much to take over — the
    # incumbent is whichever side the seed/anchor already runs, so a switch
    # flips incumbency and cannot oscillate
    side_gain: float = 0.08
    # commanded yaw may step by at most this between consecutive waypoints
    # (rad); a larger jump becomes a fan — a run of waypoints with
    # incrementally stepped yaw, each scored as its own oriented box
    max_yaw_step: float = 0.3
    # arc spacing between fan waypoints (m). Must keep dyaw/ds trackable at the
    # follower's speed floor: spacing >= max_yaw_step * v_floor / wz_max
    # (0.3 * 0.25 / 1.4 ~= 0.054 for the Go2 artifact) — tighter spacing
    # demands more yaw rate than the clamp allows and the turn never completes
    turn_spacing: float = 0.06
    # a near-180 turn keeps its previous rotation direction unless the shortest
    # direction is better by more than this (rad); prevents frame-to-frame
    # sign flips (yaw noise around +-pi) that cancel the rotation
    turn_dir_hysteresis: float = 1.5
    # how far along the path (m) a blocked fan may walk, heading held, to a
    # spot that admits the rotation
    turn_search: float = 2.5
    # "gradient" (elastic band, deterministic, warm-startable) or "sampling"
    # (CEM-lite; the zeroth-order baseline the gradient is measured against)
    method: str = "gradient"
    # hard cap on deviation from the plan (m). Generous on purpose: the
    # global plan is a hint about topology, not geometry — deform owns the
    # line and may take any berth the map affords (Ivan's call)
    max_lateral: float = 1.5
    # a scored waypoint below this clearance (m) is a wall: the published
    # path truncates veto_margin short of it and the robot holds there while
    # replanning continues. The margin must cover the body's forward extent
    # (0.4 m) plus coasting allowance — at 0.25 the nose stopped 0.13 m from
    # the box and one load-stall coast closed the gap (field 17:35)
    veto_clearance: float = -0.05
    veto_margin: float = 0.6
    # veto release hysteresis (m): once a spot fires, it stays a wall until it
    # scores above veto_clearance + veto_release — a borderline obstacle
    # flipping across the fire threshold alternated stop-stubs with full
    # paths through the box, and the robot averaged them (field 17:42)
    veto_release: float = 0.1
    # re-seed: drop the previous-solution anchor when the plan's near field
    # diverges from it by more than this mean xy distance (m)
    reseed_divergence: float = 0.3
    # re-seed: drop the anchor when the previous solution dips below this
    # clearance (m) and the plan clears better; the box SDF floors at minus
    # its smallest half-dimension
    reseed_blocked: float = -0.1
    # anchor tapers back to the plan over this much path before the window
    # exit (m), blending into the spliced remainder
    anchor_taper: float = 1.5
    # skip republishing a path within this near-field mean xy diff of the
    # last published one (m)
    publish_eps: float = 0.01
    # governor output slew (m/s per second, stepped at rate): per-frame
    # clearance noise at the lerp edge must not surge the commanded speed;
    # collision emergencies (negative clearance) drop instantly
    speed_slew: float = 0.25
    # gradient: descent iteration cap and step size. The loop breaks at the
    # first stall; the cap must be roomy enough that a cold (unseeded) frame
    # converges in ONE call — a truncated descent resumes on the next seeded
    # frame and the published path creeps (sim2d: 12 left box_on_path moving
    # 5 cm/frame for 3 frames)
    grad_iterations: int = 40
    step: float = 0.4
    # sampling: candidates per round, refinement rounds, initial amplitude (m)
    samples: int = 16
    iterations: int = 3
    sigma: float = 0.3
    seed: int = 0


# --------------------------------------------------------------- pure core --


class DistanceField:
    """Nearest-obstacle distance queries over one pointcloud snapshot.

    Built once per step, then every candidate body placement is a lookup.
    KD-tree for now; swap for a voxelized ESDF (with gradients) when the
    optimizer needs d-field derivatives rather than finite differences.
    """

    def __init__(self, cloud: PointCloud2, ground_eps: float | None = None) -> None:
        from scipy.spatial import cKDTree

        pts = cloud.points_f32()
        # Optional absolute floor cut (flat-ground convenience). On uneven
        # terrain leave it None and filter per waypoint instead (see deform).
        self.points = pts[pts[:, 2] > ground_eps] if ground_eps is not None else pts
        self._tree = cKDTree(self.points) if len(self.points) else None

    def local_points_batch(self, centers: np.ndarray, radius: float) -> list[np.ndarray]:
        """Obstacle points within radius of each center, one array per center."""
        if self._tree is None:
            empty = np.empty((0, 3), dtype=np.float32)
            return [empty] * len(centers)
        return [self.points[i] for i in self._tree.query_ball_point(centers, radius)]

    def exclude(self, shape: CollisionShape) -> None:
        """Drop points inside the collider — e.g. the robot's own scan returns.

        The lidar maps the robot's own body (a pitched-down mid360 sees the
        front legs); those points sit at the path start and no deformation can
        dodge them. Points strictly inside the body volume are self, not world.
        """
        from scipy.spatial import cKDTree

        if self._tree is None:
            return
        keep = shape.distance(self.points) >= 0.0
        if keep.all():
            return
        self.points = self.points[keep]
        self._tree = cKDTree(self.points) if len(self.points) else None

    def distance(self, centers: np.ndarray) -> np.ndarray:
        """Distance from each (N,3) query point to the nearest obstacle point."""
        if self._tree is None:
            return np.full(len(centers), np.inf)
        d, _ = self._tree.query(centers)
        return np.asarray(d)

    def local_points(self, center: np.ndarray, radius: float) -> np.ndarray:
        """Obstacle points within radius of a center; (0, 3) when there are none."""
        if self._tree is None:
            return np.empty((0, 3), dtype=self.points.dtype if self.points.size else np.float32)
        idx = self._tree.query_ball_point(np.asarray(center), radius)
        return np.asarray(self.points[idx])


def clearance(
    field: DistanceField,
    swept: list[CollisionShape],
    exact_within: float = 0.5,
    ground_eps: float | None = None,
    turn: np.ndarray | None = None,
) -> np.ndarray:
    """Per-waypoint clearance: signed distance from the collider surface (negative = hit).

    Exact (oriented, per-primitive) for obstacles within ``exact_within`` of
    the body; beyond that a bounding-sphere lower bound is returned — those
    values are only ever compared against targets <= ``exact_within``.

    ``ground_eps``: drop points within this height of each collider's *bottom*
    (walkable surface, not obstacle). Use when the field wasn't ground-filtered
    globally — i.e. on terrain where the floor height varies along the path.

    ``turn``: per-waypoint mask (see :func:`turn_mask`); masked boxes are
    scored as their rotation cylinder — xy half-diagonal radius, same height.
    """
    out = np.empty(len(swept))
    for i, s in enumerate(swept):
        if turn is not None and turn[i] and s.primitive.type == SolidPrimitive.BOX:
            d = s.primitive.dimensions
            s = CollisionShape(
                primitive=SolidPrimitive.cylinder(d[2], float(np.hypot(d[0], d[1]) / 2)),
                pose=s.pose,
            )
        center = np.array([s.pose.position.x, s.pose.position.y, s.pose.position.z])
        near = field.local_points(center, s.bounding_radius + exact_within)
        had_near = len(near) > 0
        if ground_eps is not None and had_near:
            d = s.primitive.dimensions
            if s.primitive.type == SolidPrimitive.BOX:
                bottom = center[2] - d[2] / 2
            elif s.primitive.type == SolidPrimitive.CYLINDER:
                bottom = center[2] - d[0] / 2  # dimensions = (height, radius)
            else:
                bottom = center[2] - s.bounding_radius
            near = near[near[:, 2] > bottom + ground_eps]
        if len(near):
            out[i] = float(np.min(s.distance(near)))
        elif had_near:
            # Everything nearby was walkable surface; any real obstacle is
            # beyond the exact band. (The raw nearest-point fallback would
            # count the ground and report a false hit here.)
            out[i] = exact_within
        else:
            out[i] = float(field.distance(center[None])[0]) - s.bounding_radius
    return out


def scored_clearance(
    field: DistanceField,
    shape: CollisionShape,
    poses: list[PoseStamped],
    cfg: AvoidanceConfig,
    turn: np.ndarray | None = None,
    prev_yaw: float | None = None,
) -> tuple[list[CollisionShape], np.ndarray]:
    """:func:`clearance` along poses, with rotation-swept box scoring.

    A pose entered through a yaw change > ``cfg.sweep_yaw_step`` scores as
    the min over the box at its own yaw plus interpolated yaws back to the
    previous pose's (see :func:`sweep_samples`); turn poses keep the
    rotation cylinder. Returns (base swept shapes for markers, clearances).
    """
    swept = [shape.at(p) for p in poses]
    yaws = np.array([p.orientation.euler[2] for p in poses])
    samples = sweep_samples(yaws, cfg.sweep_yaw_step, prev_yaw=prev_yaw, skip=turn)
    flat = list(swept)
    flat_turn = list(turn) if turn is not None else [False] * len(swept)
    owner = list(range(len(swept)))
    for i, extras in enumerate(samples):
        for y in extras:
            pose = Pose(
                position=poses[i].position,
                orientation=Quaternion.from_euler(Vector3(0.0, 0.0, y)),
            )
            flat.append(shape.at(pose))
            flat_turn.append(False)
            owner.append(i)
    clear_flat = clearance(
        field,
        flat,
        cfg.clearance,
        cfg.ground_eps,
        turn=np.asarray(flat_turn) if turn is not None else None,
    )
    clear = np.full(len(swept), np.inf)
    np.minimum.at(clear, np.asarray(owner, dtype=np.intp), clear_flat)
    return swept, clear


# The search scores the body box at this spacing along the path, not at every
# waypoint: a 0.70 m box at 0.30 m stride still overlaps itself by 0.4 m, so
# no obstacle can slip between scored boxes. Reporting stays full-resolution.
SCORE_STRIDE_M = 0.3
# "Near field" for re-seed divergence and publish gating: the first stretch of
# path (m) the follower is actually tracking.
NEAR_FIELD_M = 3.0


def _wrap(a: Any) -> Any:
    """Wrap angle(s) to [-pi, pi)."""
    return (a + np.pi) % (2 * np.pi) - np.pi


def turn_mask(
    yaws: np.ndarray, eps: float | np.ndarray, prev_yaw: float | None = None
) -> np.ndarray:
    """Waypoints entered through a > eps yaw jump — score those as a rotation.

    ``eps`` may be a per-segment array (len n-1) for variably spaced samples.
    ``prev_yaw``: the robot's current heading; when given, the first waypoint
    is a turn if reaching the path-start tangent takes a > eps rotation.
    """
    mask = np.zeros(len(yaws), dtype=bool)
    if len(yaws) > 1:
        mask[1:] = np.abs(_wrap(np.diff(yaws))) > eps
    if prev_yaw is not None and len(yaws):
        mask[0] = bool(abs(_wrap(float(yaws[0]) - prev_yaw)) > eps)
    return mask


def sweep_samples(
    yaws: np.ndarray,
    step: float,
    prev_yaw: float | None = None,
    skip: np.ndarray | None = None,
) -> list[list[float]]:
    """Extra yaw samples per waypoint covering the rotation into it.

    A waypoint entered through a yaw change > ``step`` (from the previous
    sample, or ``prev_yaw`` for the first) is scored as the box swept over
    the turn: min SDF over these interpolated yaws plus the waypoint's own —
    between samples the rotating rear corner swings wider than either
    sampled box. ``skip`` marks waypoints already scored as the full
    rotation cylinder (a superset of any sweep).
    """
    out: list[list[float]] = [[] for _ in yaws]
    for i in range(len(yaws)):
        if skip is not None and skip[i]:
            continue
        if i == 0:
            if prev_yaw is None:
                continue
            prev = float(prev_yaw)
        else:
            prev = float(yaws[i - 1])
        d = float(_wrap(float(yaws[i]) - prev))
        if abs(d) <= step:
            continue
        k = int(np.ceil(abs(d) / step))
        out[i] = [float(_wrap(prev + d * m / k)) for m in range(k)]
    return out


def near_field_diff(a: PathT, b: PathT, n: int = 30) -> float:
    """Mean xy distance from a's first n waypoints to b's polyline.

    Geometric, not index-wise: the robot advancing along an unchanged path
    reads as ~0 — only actual shape change (the wobble a follower chases)
    counts.
    """
    if len(a) == 0 or len(b) == 0:
        return float("inf")
    pa = np.array([[p.position.x, p.position.y] for p in a.poses[:n]])
    pb = np.array([[p.position.x, p.position.y] for p in b])
    if len(pb) == 1:
        return float(np.mean(np.hypot(*(pa - pb[0]).T)))
    seg = pb[1:] - pb[:-1]  # (M, 2)
    d = pa[:, None, :] - pb[None, :-1, :]  # (N, M, 2)
    t = np.clip((d * seg).sum(-1) / np.maximum((seg**2).sum(-1), 1e-12), 0.0, 1.0)
    off = d - t[..., None] * seg
    return float(np.mean(np.sqrt((off**2).sum(-1)).min(axis=1)))


def _path_arcs(xy: np.ndarray) -> np.ndarray:
    """Cumulative xy arc length per waypoint."""
    if len(xy) < 2:
        return np.zeros(len(xy))
    return np.concatenate(([0.0], np.cumsum(np.hypot(*np.diff(xy, axis=0).T))))


def _pose_at_arc(path: PathT, arcs: np.ndarray, s: float) -> PoseStamped:
    """Pose interpolated on the polyline at arc ``s`` (position and yaw lerp)."""
    i = min(max(int(np.searchsorted(arcs, s, side="right") - 1), 0), len(arcs) - 2)
    ds = float(arcs[i + 1] - arcs[i])
    t = (s - float(arcs[i])) / ds if ds > 1e-12 else 0.0
    a, b = path.poses[i], path.poses[i + 1]
    yaw = float(a.orientation.euler[2])
    yaw = float(_wrap(yaw + t * float(_wrap(float(b.orientation.euler[2]) - yaw))))
    return PoseStamped(
        frame_id=path.frame_id,
        position=[
            a.position.x + t * (b.position.x - a.position.x),
            a.position.y + t * (b.position.y - a.position.y),
            a.position.z + t * (b.position.z - a.position.z),
        ],
        orientation=Quaternion.from_euler(Vector3(0.0, 0.0, yaw)),
    )


def station_poses(
    path: PathT,
    arcs: np.ndarray,
    idx: list[int],
    stride_m: float = SCORE_STRIDE_M,
    end: int | None = None,
) -> tuple[list[PoseStamped], np.ndarray, np.ndarray]:
    """Scoring stations: the picked indices plus interpolated arc-gap fill.

    Index stride assumes uniform waypoint spacing, but a deformed path can
    carry metre-long segments (mirror blends): a thin wall crossed inside one
    was never scored (sim2d boxed_in: wall-crossing path scored +0.03). Fill
    poses sit ON the polyline so consecutive stations end up <= ``stride_m``
    apart in arc; uniformly spaced paths gain none. ``end`` extends the fill
    (without becoming a station itself) past the last pick. Returns (poses,
    arc per station, owner path index — -1 for fill stations).
    """
    poses: list[PoseStamped] = []
    s_arc: list[float] = []
    owner: list[int] = []

    def fill(a0: float, a1: float) -> None:
        m = int(np.ceil((a1 - a0) / stride_m - 1e-9))
        for j in range(1, m):
            s = a0 + (a1 - a0) * j / m
            poses.append(_pose_at_arc(path, arcs, s))
            s_arc.append(s)
            owner.append(-1)

    prev: int | None = None
    for i in idx:
        if prev is not None:
            fill(float(arcs[prev]), float(arcs[i]))
        poses.append(path.poses[i])
        s_arc.append(float(arcs[i]))
        owner.append(i)
        prev = i
    if end is not None and prev is not None and end > prev:
        fill(float(arcs[prev]), float(arcs[end]))
    return poses, np.asarray(s_arc), np.asarray(owner, dtype=np.intp)
