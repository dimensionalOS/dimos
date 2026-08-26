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

"""The reference pursuit law: holonomic, clearance-governed, fixed lookahead.

The permanent baseline. Every track's A/B is against this law and every
later law starts from it, so it does NOT absorb their results — a
moving baseline is not a baseline. Fold those into the track's own law.
"""

from __future__ import annotations

import math
from typing import Any

import numpy as np

from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.nav_msgs.Path import Path
from dimos.navigation.motion.control.controller import (
    ControllerConfig,
    angle_diff,
    emb_json,
    load_extension,
    path_xy_yaw,
)
from dimos.navigation.motion.embodiment.base import Embodiment
from dimos.navigation.motion.embodiment.go2 import GO2


def make(emb: Embodiment = GO2) -> PursuitController:
    return PursuitController(emb)


def make_rust(emb: Embodiment = GO2) -> RustPursuitController:
    return RustPursuitController(emb)


class PursuitController:
    """Holonomic pursuit: project, look ahead, P-law in the body frame.

    The Go2 can crab, so position error maps straight to (vx, vy) — no
    car-style steering. Yaw tracks the path's own yaw (the planner encodes
    side-stepping and fans there), and fan segments become rotate-in-place:
    position holds the fan waypoint while yaw converges.
    """

    config: ControllerConfig

    def __init__(self, emb: Embodiment = GO2) -> None:
        self.config = emb.control
        self.emb = emb
        self.reset()

    def reset(self) -> None:
        self._goal_reached = False

    def update(
        self, pose: PoseStamped, path: Path, t: float, clearance: np.ndarray | None = None
    ) -> Twist:
        cfg, emb = self.config, self.emb
        if len(path) < 2:
            # empty path or a single-pose veto stub: there is nothing to
            # follow -- hold position (the planner is saying "stop")
            return Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))
        xy = np.array([[p.position.x, p.position.y] for p in path.poses])
        yaws = np.array([p.yaw for p in path.poses])
        px, py, pyaw = pose.position.x, pose.position.y, pose.yaw

        seg = np.linalg.norm(np.diff(xy, axis=0), axis=1) if len(xy) > 1 else np.zeros(1)
        arcs = np.concatenate([[0.0], np.cumsum(seg)])

        # closest waypoint = progress along the path; inside a fan the
        # waypoints are coincident, so advance by yaw progress instead of
        # re-rotating from the fan's first pose
        i = int(np.argmin(np.linalg.norm(xy - (px, py), axis=1)))
        while (
            i + 1 < len(xy)
            and float(arcs[i + 1] - arcs[i]) < 1e-6
            and abs(angle_diff(float(yaws[i + 1]), pyaw)) < abs(angle_diff(float(yaws[i]), pyaw))
        ):
            i += 1

        # fan detection at the current position: yaw stepping with (near-)zero
        # displacement means the planner commands a rotation here
        j = min(i + 1, len(xy) - 1)
        ds = float(arcs[j] - arcs[i])
        dyaw = abs(angle_diff(float(yaws[j]), float(yaws[i])))
        in_fan = j > i and dyaw > 1e-6 and dyaw / max(ds, 1e-6) > cfg.fan_yaw_per_m
        if in_fan and abs(angle_diff(float(yaws[j]), pyaw)) > cfg.fan_yaw_done:
            target_xy = xy[i]
            target_yaw = float(yaws[j])
        else:
            s = float(arcs[i]) + cfg.lookahead
            k = min(int(np.searchsorted(arcs, s)), len(xy) - 1)
            target_xy = xy[k]
            target_yaw = float(yaws[k])

        # speed governor: cap cruise by the room ahead, when we know it
        vmax = emb.max_speed
        if clearance is not None and len(clearance) == len(xy):
            ahead = clearance[(arcs >= arcs[i]) & (arcs <= arcs[i] + cfg.speed_lookahead)]
            room = float(np.min(ahead)) if len(ahead) else float(clearance[i])
            frac = (room - emb.precision) / max(emb.speed_clearance - emb.precision, 1e-6)
            vmax = emb.min_speed + (emb.max_speed - emb.min_speed) * min(max(frac, 0.0), 1.0)

        # body-frame error -> velocity
        ex, ey = target_xy[0] - px, target_xy[1] - py
        c, s_ = math.cos(-pyaw), math.sin(-pyaw)
        bx, by = c * ex - s_ * ey, s_ * ex + c * ey
        vx, vy = cfg.k_pos * bx, cfg.k_pos * by
        # math.hypot is CPython's own correctly-rounded one; rust's f64::hypot
        # is libm, and they differ by an ulp on some inputs. It only reaches the
        # twist here when the command clamps, and the sweep in test_rust_parity
        # is bit-exact, so this stays as it is -- the baseline does not move for
        # a latent ulp. A NEW law should use np.hypot, as laws/blind.py does.
        speed = math.hypot(vx, vy)
        if speed > vmax:
            vx, vy = vx / speed * vmax, vy / speed * vmax
        wz = float(
            np.clip(cfg.k_yaw * angle_diff(target_yaw, pyaw), -emb.max_yaw_rate, emb.max_yaw_rate)
        )
        return Twist(Vector3(vx, vy, 0.0), Vector3(0.0, 0.0, wz))


class RustPursuitController:
    """``dimos_motion2_tc.update_seed`` behind the controller protocol."""

    config: ControllerConfig

    def __init__(self, emb: Embodiment = GO2) -> None:
        self._mod: Any = load_extension()
        self.config = emb.control
        self._emb = emb_json(emb)
        self.reset()

    def reset(self) -> None:
        pass

    def update(
        self, pose: PoseStamped, path: Path, t: float, clearance: np.ndarray | None = None
    ) -> Twist:
        clr = None if clearance is None else np.ascontiguousarray(clearance, dtype=np.float64)
        vx, vy, wz = self._mod.update_seed(
            (float(pose.position.x), float(pose.position.y), float(pose.yaw)),
            path_xy_yaw(path),
            clr,
            self._emb,
        )
        return Twist(Vector3(vx, vy, 0.0), Vector3(0.0, 0.0, wz))
