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

"""Observation builder: exactly what ``update()`` gets, as a flat vector.

THIS MODULE IS THE SEARCH SURFACE. The arc marks, the scalars, the frame, the
scales -- all candidate-internal, all fair game for the loop. Everything
downstream keys off :data:`FORMAT`, so a changed builder invalidates old
checkpoints loudly instead of silently mismatching.

The inputs are the controller seam and nothing else: pose, path, tick, and the
room hint. No embodiment vector, deliberately -- the body under the controller
is always the go2 (the referee's sim has one robot; a scenario's ``emb`` shapes
the PLAN, not the plant), so conditioning on it would be conditioning on
something that never moves. That also keeps the net portable: what it reads is
what a rust port reads off the same seam, with no extra plumbing.

Both tracks produce the same vector. ``hinted`` is handed live clearance;
``blind`` decodes the profile the planner stamped into the path
(:mod:`~...control.profile`), which is the same thing ``laws/blind.py`` does.
One slot, one net, and :data:`HINTED`/:data:`BLIND` says which source it came
from -- the net is told whether the room number is measured or inferred.

Ego frame: origin at the pose point, +x along pose yaw (forward), +y left.
"""

from __future__ import annotations

import math
from typing import TYPE_CHECKING

import numpy as np

from dimos.navigation.motion.control.profile import ceilings_to_clearance, decode_ceilings

if TYPE_CHECKING:
    from dimos.msgs.nav_msgs.Path import Path

# Arc distances ahead of the projected foot, in metres. Dense near the body
# (where tracking is decided) and sparse out to the governor's own
# speed_lookahead of 2.0 m plus a little, so braking has something to brake for.
ARC_MARKS: tuple[float, ...] = (0.0, 0.15, 0.35, 0.60, 1.00, 1.50, 2.20, 3.00)
MARK_DIM = 5  # ego dx, dy, cos dyaw, sin dyaw, room
MARK_TOTAL = len(ARC_MARKS) * MARK_DIM

# Scalar slots, named rather than counted: the scale vector, the tests and the
# eventual rust port all index this block, and an off-by-one here is a silently
# mis-scaled input rather than a crash.
S_CROSS = 0  # signed cross-track, +left (m)
S_TAN_COS = 1  # heading error to the path tangent
S_TAN_SIN = 2
S_REMAIN = 3  # arc left to the path end (m)
S_END_X = 4  # ego vector to the path end (m)
S_END_Y = 5
S_VEL_X = 6  # measured body velocity, ego frame (m/s)
S_VEL_Y = 7
S_VEL_YAW = 8  # rad/s
S_PREV_VX = 9  # previous normalised action
S_PREV_VY = 10
S_PREV_WZ = 11
S_PREV2_VX = 12  # the one before that
S_PREV2_VY = 13
S_PREV2_WZ = 14
S_TRACK = 15  # hinted (room measured) or blind (room decoded)
S_AGE = 16  # how old the active plan is (s)
SCALAR_DIM = 17

DIM = MARK_TOTAL + SCALAR_DIM

FORMAT = f"tc-m{len(ARC_MARKS)}-s{SCALAR_DIM}-v2"

HINTED, BLIND = 1.0, 0.0

# Room to report when the path carries no profile at all -- an unstamped path
# from a producer that does not speak the dialect. Cruise-permitting, so an
# un-hinted path is not silently read as a pinch.
NO_HINT_ROOM = 0.35

# Room is CLIPPED, and that is load-bearing rather than tidy: an obstacle-free
# world has infinite clearance (``world.path_clearance`` and the judge both say
# so), and an inf reaching the net makes every output NaN, the twist NaN and
# the sim unstable. Past the governor's own cruise threshold more room changes
# no decision anyway, so the ceiling costs nothing.
MAX_ROOM = 1.0

# Fixed scales so the net sees O(1) inputs while the vector itself stays in
# real units (metres, m/s, rad/s) and reads like a debug dump. Applied by
# policy.py, never here -- same split as the planner-side lab.
MARK_SCALE = np.array([0.5, 2.0, 1.0, 1.0, 3.0], dtype=np.float32)
SCALAR_SCALE = np.zeros(SCALAR_DIM, dtype=np.float32)
SCALAR_SCALE[S_CROSS] = 5.0
SCALAR_SCALE[[S_TAN_COS, S_TAN_SIN]] = 1.0
SCALAR_SCALE[S_REMAIN] = 0.2
SCALAR_SCALE[[S_END_X, S_END_Y]] = 0.2, 0.5
SCALAR_SCALE[[S_VEL_X, S_VEL_Y, S_VEL_YAW]] = 2.0, 2.0, 1.0
SCALAR_SCALE[[S_PREV_VX, S_PREV_VY, S_PREV_WZ]] = 1.0
SCALAR_SCALE[[S_PREV2_VX, S_PREV2_VY, S_PREV2_WZ]] = 1.0
SCALAR_SCALE[S_TRACK] = 1.0
SCALAR_SCALE[S_AGE] = 5.0


def path_arrays(path: Path) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """The path as (xy (n,2), yaws (n,), cumulative arc (n,))."""
    xy = np.array([[p.position.x, p.position.y] for p in path.poses]).reshape(-1, 2)
    yaws = np.array([p.yaw for p in path.poses])
    if len(xy) < 2:
        return xy, yaws, np.zeros(len(xy))
    seg = np.linalg.norm(np.diff(xy, axis=0), axis=1)
    return xy, yaws, np.concatenate([[0.0], np.cumsum(seg)])


def project(xy: np.ndarray, arcs: np.ndarray, px: float, py: float) -> tuple[float, float]:
    """Foot of the perpendicular onto the polyline: (arc length, signed offset).

    Where on the LINE the body is, not which waypoint it is nearest -- the same
    quantity ``judge.cross_track`` measures and ``laws/hinted.py`` projects for.
    Sign is +left of the direction of travel. First minimum wins, so a plan that
    doubles back cannot snap the foot forward.
    """
    best, s_star, side = math.inf, 0.0, 0.0
    for m in range(len(xy) - 1):
        ax, ay = float(xy[m][0]), float(xy[m][1])
        dx, dy = float(xy[m + 1][0]) - ax, float(xy[m + 1][1]) - ay
        dd = dx * dx + dy * dy
        u = min(max(((px - ax) * dx + (py - ay) * dy) / dd, 0.0), 1.0) if dd > 1e-12 else 0.0
        qx, qy = ax + u * dx, ay + u * dy
        d = float(np.hypot(px - qx, py - qy))
        if d < best:
            best = d
            s_star = float(arcs[m]) + u * (float(arcs[m + 1]) - float(arcs[m]))
            # cross product of the segment direction with the offset: +ve left
            side = math.copysign(d, dx * (py - ay) - dy * (px - ax))
    return s_star, side


def room_profile(path: Path, clearance: np.ndarray | None) -> tuple[np.ndarray, float]:
    """Per-waypoint room (m) and which track it came from.

    Hinted hands it over. Blind recovers it from the stamps, exactly as
    ``laws/blind.py`` does -- so the same net reads the same slot on both
    tracks and the track bit is the only difference.
    """
    n = len(path.poses)
    if clearance is not None and len(clearance) == n:
        room = np.asarray(clearance, dtype=float)
        # nan_to_num before the clip: an inf survives np.clip, a nan does not.
        return np.clip(np.nan_to_num(room, nan=MAX_ROOM, posinf=MAX_ROOM), 0.0, MAX_ROOM), HINTED
    ceilings = decode_ceilings(path)
    if ceilings is not None and len(ceilings) == n:
        return np.clip(ceilings_to_clearance(ceilings), 0.0, MAX_ROOM), BLIND
    return np.full(n, NO_HINT_ROOM), BLIND


class Velocity:
    """Body velocity off a zero-order-held pose.

    The controller ticks at 50 Hz on a pose sampled at 29 Hz, so a one-tick
    finite difference is exactly zero on about four ticks in ten -- and a net
    handed that alternating zero learns that its own speed is noise. This
    differences against the last pose that actually MOVED, and holds the last
    estimate across the repeats, which is what the hold means.

    A pure function of what ``update()`` already receives, so nothing new
    crosses the seam and the rust port stays a ring buffer.
    """

    def __init__(self, min_dt: float = 0.03) -> None:
        self._min_dt = min_dt  # ~ one 29 Hz period
        self.reset()

    def reset(self) -> None:
        self._t: float | None = None
        self._pose: tuple[float, float, float] | None = None
        self._vel = np.zeros(3)

    def update(self, pose: tuple[float, float, float], t: float) -> np.ndarray:
        """-> (vx, vy, vyaw) in the CURRENT ego frame."""
        if self._pose is None or self._t is None:
            self._t, self._pose = t, pose
            return self._vel
        dt = t - self._t
        moved = (pose[0] - self._pose[0], pose[1] - self._pose[1])
        turned = math.remainder(pose[2] - self._pose[2], math.tau)
        if dt >= self._min_dt and (abs(moved[0]) + abs(moved[1]) + abs(turned)) > 1e-9:
            c, s = math.cos(-pose[2]), math.sin(-pose[2])
            self._vel = np.array(
                [
                    (c * moved[0] - s * moved[1]) / dt,
                    (s * moved[0] + c * moved[1]) / dt,
                    turned / dt,
                ]
            )
            self._t, self._pose = t, pose
        return self._vel


def build(
    pose: tuple[float, float, float],
    path: Path,
    t: float,
    clearance: np.ndarray | None,
    prev_cmd: np.ndarray,
    prev2_cmd: np.ndarray,
    vel: np.ndarray,
) -> np.ndarray:
    """The observation, shape ``(DIM,)`` float32, in real units.

    A path with fewer than two poses is a veto stub -- the caller holds
    position and never asks for one of these.
    """
    xy, yaws, arcs = path_arrays(path)
    px, py, pyaw = pose
    room, track = room_profile(path, clearance)

    s_star, side = project(xy, arcs, px, py)
    total = float(arcs[-1])
    c, s = math.cos(-pyaw), math.sin(-pyaw)

    marks = np.zeros((len(ARC_MARKS), MARK_DIM), dtype=np.float32)
    for k, ahead in enumerate(ARC_MARKS):
        s_k = min(s_star + ahead, total)
        mx = float(np.interp(s_k, arcs, xy[:, 0]))
        my = float(np.interp(s_k, arcs, xy[:, 1]))
        # unwrapped before interpolation or a pi/-pi crossing averages backwards
        myaw = float(np.interp(s_k, arcs, np.unwrap(yaws)))
        dx, dy = mx - px, my - py
        dyaw = myaw - pyaw
        marks[k] = (
            c * dx - s * dy,
            s * dx + c * dy,
            math.cos(dyaw),
            math.sin(dyaw),
            float(np.interp(s_k, arcs, room)),
        )

    tangent = float(np.interp(min(s_star, total), arcs, np.unwrap(yaws))) - pyaw
    ex, ey = float(xy[-1][0]) - px, float(xy[-1][1]) - py
    scalars = np.zeros(SCALAR_DIM, dtype=np.float32)
    scalars[S_CROSS] = side
    scalars[S_TAN_COS] = math.cos(tangent)
    scalars[S_TAN_SIN] = math.sin(tangent)
    scalars[S_REMAIN] = max(0.0, total - s_star)
    scalars[S_END_X] = c * ex - s * ey
    scalars[S_END_Y] = s * ex + c * ey
    scalars[[S_VEL_X, S_VEL_Y, S_VEL_YAW]] = vel[:3]
    scalars[[S_PREV_VX, S_PREV_VY, S_PREV_WZ]] = prev_cmd[:3]
    scalars[[S_PREV2_VX, S_PREV2_VY, S_PREV2_WZ]] = prev2_cmd[:3]
    scalars[S_TRACK] = track
    # staleness: the plan's own t0 against now. On the robot this is the
    # difference between a fresh plan and one the link stopped feeding.
    scalars[S_AGE] = max(0.0, t - float(path.poses[0].ts))
    return np.concatenate([marks.ravel(), scalars])
