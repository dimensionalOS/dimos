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

"""The follower's law: seed pursuit plus the gait slip inverse (:func:`walk_command`),
constant time headway, and the stamped precision profile as the governor's input
(:mod:`~...local_planner.profile`). Held apart from the frozen :mod:`~...laws.seed`.
"""

from __future__ import annotations

import math
from typing import Any

import numpy as np
from numpy.typing import NDArray

from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.nav_msgs.Path import Path
from dimos.navigation.embodiment.base import Embodiment
from dimos.navigation.embodiment.go2 import GO2
from dimos.navigation.local_planner.profile import decode_ceilings
from dimos.navigation.trajectory_follower.fancy.controller import (
    ControllerConfig,
    angle_diff,
    load_extension,
    path_xy_yaw,
)


def make(emb: Embodiment = GO2) -> HintedController:
    return HintedController(emb)


def make_rust(emb: Embodiment = GO2) -> RustHintedController:
    return RustHintedController(emb)


def walk_command(want: float, gain: float, slip: float, ramp: float) -> float:
    """Invert the gait's affine under-delivery (ground ~= ``gain * cmd - slip``, measured
    open loop) so it walks ``want`` m/s; identity at 0 and full inverse from ``ramp`` up.
    """
    if want <= 0.0:
        return 0.0
    correction = (gain * want + slip) - want
    return want + correction * min(want / ramp, 1.0)


class HintedController:
    """Pursuit governed by the stamped precision profile, at gait-true speeds. An explicit
    ``clearance`` array (the lab's channel) outranks the stamps."""

    config: ControllerConfig

    def __init__(self, emb: Embodiment = GO2) -> None:
        self.config = emb.control
        self.emb = emb
        self.reset()

    def reset(self) -> None:
        pass

    def update(
        self, pose: PoseStamped, path: Path, t: float, clearance: NDArray[np.float64] | None = None
    ) -> Twist:
        cfg, emb = self.config, self.emb
        if len(path) < 2:
            # empty path or single-pose veto stub: the planner says stop
            return Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))
        xy = np.array([[p.position.x, p.position.y] for p in path.poses])
        yaws = np.array([p.yaw for p in path.poses])
        px, py, pyaw = pose.position.x, pose.position.y, pose.yaw
        n = len(xy)

        seg = np.linalg.norm(np.diff(xy, axis=0), axis=1) if n > 1 else np.zeros(1)
        arcs = np.concatenate([[0.0], np.cumsum(seg)])

        # inside a fan the waypoints are coincident, so advance by yaw progress
        i = int(np.argmin(np.linalg.norm(xy - (px, py), axis=1)))
        while (
            i + 1 < n
            and float(arcs[i + 1] - arcs[i]) < 1e-6
            and abs(angle_diff(float(yaws[i + 1]), pyaw)) < abs(angle_diff(float(yaws[i]), pyaw))
        ):
            i += 1

        # governor first: the lookahead is derived from vmax (constant time headway)
        vmax = emb.max_speed
        governed = False
        if clearance is not None and len(clearance) == n:
            governed = True
            ahead = clearance[(arcs >= arcs[i]) & (arcs <= arcs[i] + cfg.speed_lookahead)]
            room = float(np.min(ahead)) if len(ahead) else float(clearance[i])
            frac = (room - emb.precision) / max(emb.speed_clearance - emb.precision, 1e-6)
            vmax = emb.min_speed + (emb.max_speed - emb.min_speed) * min(max(frac, 0.0), 1.0)

        # the same governor off the stamped profile (local_planner/profile.py); the two
        # channels are alternatives, not layers
        if not governed:
            ceilings = decode_ceilings(path, emb.min_speed, emb.max_speed)
            if ceilings is not None:
                # from i + 1: ceilings[k] belongs to the segment ending at k, so this is
                # exactly the clearance branch's window
                hi = arcs[i] + cfg.speed_lookahead
                window = ceilings[i + 1 :][arcs[i + 1 :] <= hi]
                # the next segment always counts, even if speed_lookahead would exclude it
                nxt = float(ceilings[min(i + 1, n - 1)])
                vmax = min(nxt, float(np.min(window))) if len(window) else nxt

        # constant time headway, not distance: a fixed carrot chords the turn toward the
        # obstacle the planner curved around; the floor keeps the command saturating
        headway = cfg.lookahead / max(emb.max_speed, 1e-6)
        look = max(vmax * headway, vmax / max(abs(cfg.k_pos), 1e-6))

        # fan: yaw stepping at (near-)zero displacement means rotate here
        j = min(i + 1, n - 1)
        ds = float(arcs[j] - arcs[i])
        dyaw = abs(angle_diff(float(yaws[j]), float(yaws[i])))
        in_fan = j > i and dyaw > 1e-6 and dyaw / max(ds, 1e-6) > cfg.fan_yaw_per_m
        if in_fan and abs(angle_diff(float(yaws[j]), pyaw)) > cfg.fan_yaw_done:
            target_xy = xy[i]
            target_yaw = float(yaws[j])
        else:
            target_xy, target_yaw = _carrot_lerp(xy, yaws, arcs, i, look)

        ex, ey = target_xy[0] - px, target_xy[1] - py
        c, s_ = math.cos(-pyaw), math.sin(-pyaw)
        bx, by = c * ex - s_ * ey, s_ * ex + c * ey
        vx, vy = cfg.k_pos * bx, cfg.k_pos * by
        # np.hypot, not math.hypot: CPython's is correctly rounded, rust's is libm, and
        # the ulp reaches the twist through the divide (test_rust_parity)
        speed = float(np.hypot(vx, vy))
        if speed > 1e-12:
            want = min(speed, vmax)
            cmd = walk_command(want, emb.walk_gain, emb.walk_slip, emb.walk_slip_ramp)
            vx, vy = vx / speed * cmd, vy / speed * cmd
        wz = float(
            np.clip(cfg.k_yaw * angle_diff(target_yaw, pyaw), -emb.max_yaw_rate, emb.max_yaw_rate)
        )
        return Twist(Vector3(vx, vy, 0.0), Vector3(0.0, 0.0, wz))


def _carrot_lerp(
    xy: NDArray[np.float64],
    yaws: NDArray[np.float64],
    arcs: NDArray[np.float64],
    i: int,
    look: float,
) -> tuple[NDArray[np.float64], float]:
    """The point at exactly ``arcs[i] + look``, interpolated within its segment: snapping
    to a waypoint would make a 0.14 m carrot chatter on the 0.1 m discretisation."""
    n = len(xy)
    s = float(arcs[i]) + look
    k = int(np.searchsorted(arcs, s))
    if k == 0 or k >= n:
        k = min(k, n - 1)
        return xy[k], float(yaws[k])
    a0, a1 = float(arcs[k - 1]), float(arcs[k])
    d = a1 - a0
    if d <= 1e-9:
        return xy[k], float(yaws[k])
    u = min(max((s - a0) / d, 0.0), 1.0)
    point = xy[k - 1] + u * (xy[k] - xy[k - 1])
    # yaw the short way round, so a wrap across +-pi does not spin the carrot
    yaw = float(yaws[k - 1]) + u * angle_diff(float(yaws[k]), float(yaws[k - 1]))
    return point, yaw


class RustHintedController:
    """``dimos_trajectory_follower.update_hinted`` behind the controller protocol."""

    config: ControllerConfig

    def __init__(self, emb: Embodiment = GO2) -> None:
        self._mod: Any = load_extension()
        self.config = emb.control
        self._emb = emb.to_json()
        self.reset()

    def reset(self) -> None:
        pass

    def update(
        self, pose: PoseStamped, path: Path, t: float, clearance: NDArray[np.float64] | None = None
    ) -> Twist:
        clr = None if clearance is None else np.ascontiguousarray(clearance, dtype=np.float64)
        # the law reads only stamp deltas: a precision profile, not a schedule
        ts = np.ascontiguousarray(
            np.array([p.ts for p in path.poses], dtype=np.float64).reshape(-1)
        )
        vx, vy, wz = self._mod.update_hinted(
            (float(pose.position.x), float(pose.position.y), float(pose.yaw)),
            path_xy_yaw(path),
            clr,
            ts,
            self._emb,
        )
        return Twist(Vector3(vx, vy, 0.0), Vector3(0.0, 0.0, wz))
