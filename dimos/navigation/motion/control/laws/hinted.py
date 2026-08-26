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

"""The follower's law: follow the path the gait can actually walk.

Three mechanisms over :mod:`~...laws.seed`, in order of worth:

1. :func:`walk_command`, the gait slip inverse. The twist is a REQUEST to a
   learned policy that under-delivers it, and the governor's creep rung sits
   inside the gait's dead-stall band, so asking for it stops the robot dead.
2. Constant time headway: the carrot distance scales with commanded speed
   instead of staying fixed, shrinking the pursuit chord where the plan is
   tight — and the chord always falls toward the obstacle the planner curved
   around.
3. The stamped precision profile as the governor's input: the follower holds
   no map of its own, so the room the planner priced arrives in the path's
   own timestamps (:mod:`~...control.profile`).

(1) and (2) are held here rather than shared with :mod:`~...laws.seed`, which
is the frozen baseline.
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
from dimos.navigation.motion.control.controller import (
    ControllerConfig,
    angle_diff,
    load_extension,
    path_xy_yaw,
)
from dimos.navigation.motion.control.profile import decode_ceilings
from dimos.navigation.motion.embodiment.base import Embodiment
from dimos.navigation.motion.embodiment.go2 import GO2


def make(emb: Embodiment = GO2) -> HintedController:
    return HintedController(emb)


def make_rust(emb: Embodiment = GO2) -> RustHintedController:
    return RustHintedController(emb)


def walk_command(want: float, gain: float, slip: float, ramp: float) -> float:
    """The command that asks the gait for a ground speed of ``want`` m/s.

    The twist is a request to a learning walking policy that under-delivers:
    ground speed ~= ``gain * cmd - slip`` above the stall band, nothing at all
    below it (the embodiment's ``walk_*``, measured open loop). This inverts
    that affine map so the intended ground speed stays exactly the one the
    governor chose — an actuator inverse, not a flat offset, which would
    overshoot at cruise. Identity at ``want = 0``, the full inverse once
    ``want >= ramp``, since a stop request has to remain a stop.
    """
    if want <= 0.0:
        return 0.0
    correction = (gain * want + slip) - want
    return want + correction * min(want / ramp, 1.0)


class HintedController:
    """Pursuit governed by the stamped precision profile, at gait-true speeds.

    ``clearance`` is the lab's channel: an explicit per-waypoint room array
    outranks the stamps when one is handed in. The follower never hands one.
    """

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
            # empty path or a single-pose veto stub: there is nothing to
            # follow -- hold position (the planner is saying "stop")
            return Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))
        xy = np.array([[p.position.x, p.position.y] for p in path.poses])
        yaws = np.array([p.yaw for p in path.poses])
        px, py, pyaw = pose.position.x, pose.position.y, pose.yaw
        n = len(xy)

        seg = np.linalg.norm(np.diff(xy, axis=0), axis=1) if n > 1 else np.zeros(1)
        arcs = np.concatenate([[0.0], np.cumsum(seg)])

        # closest waypoint = progress along the path; inside a fan the
        # waypoints are coincident, so advance by yaw progress instead of
        # re-rotating from the fan's first pose
        i = int(np.argmin(np.linalg.norm(xy - (px, py), axis=1)))
        while (
            i + 1 < n
            and float(arcs[i + 1] - arcs[i]) < 1e-6
            and abs(angle_diff(float(yaws[i + 1]), pyaw)) < abs(angle_diff(float(yaws[i]), pyaw))
        ):
            i += 1

        # Speed governor, hoisted above target selection because the lookahead
        # distance is derived from it (constant time headway, below).
        vmax = emb.max_speed
        governed = False
        if clearance is not None and len(clearance) == n:
            governed = True
            ahead = clearance[(arcs >= arcs[i]) & (arcs <= arcs[i] + cfg.speed_lookahead)]
            room = float(np.min(ahead)) if len(ahead) else float(clearance[i])
            frac = (room - emb.precision) / max(emb.speed_clearance - emb.precision, 1e-6)
            vmax = emb.min_speed + (emb.max_speed - emb.min_speed) * min(max(frac, 0.0), 1.0)

        # THE SAME GOVERNOR, OFF THE STAMPS. When no clearance array arrives
        # the room ahead has not been withheld, only re-encoded: the planner
        # stamps the required-precision profile into the path's own timestamps
        # on every replan (control/profile.py). Decoding it
        # puts a speed ceiling back under this law at exactly the point the
        # clearance branch occupies, so the two channels are alternatives
        # rather than layers.
        if not governed:
            ceilings = decode_ceilings(path, emb.min_speed, emb.max_speed)
            if ceilings is not None:
                # Read from i + 1, not i. Not an off-by-one: a decoded ceiling
                # is a property of the SEGMENT ending at its waypoint, so
                # ceilings[k] already carries clr[k-1]. Scanning [i+1 ..]
                # reproduces the clearance branch's window exactly, whereas
                # starting at i would drag in the waypoint behind the robot.
                hi = arcs[i] + cfg.speed_lookahead
                window = ceilings[i + 1 :][arcs[i + 1 :] <= hi]
                # the segment about to be traversed always counts, even if a
                # degenerate speed_lookahead would exclude it; at the end of
                # the plan there is no next segment and the last one stands
                nxt = float(ceilings[min(i + 1, n - 1)])
                vmax = min(nxt, float(np.min(window))) if len(window) else nxt

        # CONSTANT TIME HEADWAY, not a constant distance. Steering at a carrot
        # a fixed distance away chords the plan's curvature, and the chord
        # always falls to the INSIDE of the turn -- toward the obstacle the
        # planner curved around -- with an inset that grows as the square of
        # the carrot distance. Holding the TIME headway constant instead
        # (lookahead / max_speed) leaves full cruise untouched and shortens the
        # carrot where the governor has slowed for a pinch.
        #
        # It costs no speed: the command magnitude is min(k_pos * L, vmax), so
        # any L >= vmax / k_pos still saturates, and the floor below enforces
        # that so an odd config cannot turn a shorter carrot into a slower
        # robot.
        headway = cfg.lookahead / max(emb.max_speed, 1e-6)
        look = max(vmax * headway, vmax / max(abs(cfg.k_pos), 1e-6))

        # fan detection at the current position: yaw stepping with (near-)zero
        # displacement means the planner commands a rotation here
        j = min(i + 1, n - 1)
        ds = float(arcs[j] - arcs[i])
        dyaw = abs(angle_diff(float(yaws[j]), float(yaws[i])))
        in_fan = j > i and dyaw > 1e-6 and dyaw / max(ds, 1e-6) > cfg.fan_yaw_per_m
        if in_fan and abs(angle_diff(float(yaws[j]), pyaw)) > cfg.fan_yaw_done:
            target_xy = xy[i]
            target_yaw = float(yaws[j])
        else:
            target_xy, target_yaw = _carrot_lerp(xy, yaws, arcs, i, look)

        # body-frame error -> velocity
        ex, ey = target_xy[0] - px, target_xy[1] - py
        c, s_ = math.cos(-pyaw), math.sin(-pyaw)
        bx, by = c * ex - s_ * ey, s_ * ex + c * ey
        vx, vy = cfg.k_pos * bx, cfg.k_pos * by
        # np.hypot, not math.hypot: CPython implements its own correctly-rounded
        # hypot, rust's f64::hypot is libm, and the two differ by an ulp. This
        # law divides by `speed` on EVERY tick (the seed only did so when
        # clamping), so that ulp reaches the twist -- np.hypot is the same libm
        # call the rust makes. See test_rust_parity.
        speed = float(np.hypot(vx, vy))
        if speed > 1e-12:
            # `want` is the intended GROUND speed -- the pursuit gain, capped
            # by the governor. Unchanged from the seed.
            want = min(speed, vmax)
            # ...and this is what the gait has to be asked for to deliver it.
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
    """The point at exactly ``arcs[i] + look``, interpolated within its segment.

    The plan is discretised at 0.1 m — fine noise against a 0.35 m carrot, but
    70% of a 0.14 m one. Without this the shortened headway would make the
    carrot distance, and so the commanded heading, chatter waypoint to
    waypoint.
    """
    n = len(xy)
    s = float(arcs[i]) + look
    k = int(np.searchsorted(arcs, s))
    if k == 0 or k >= n:
        # s is at or beyond an endpoint: pursue the endpoint itself
        k = min(k, n - 1)
        return xy[k], float(yaws[k])
    a0, a1 = float(arcs[k - 1]), float(arcs[k])
    d = a1 - a0
    if d <= 1e-9:
        return xy[k], float(yaws[k])
    u = min(max((s - a0) / d, 0.0), 1.0)
    point = xy[k - 1] + u * (xy[k] - xy[k - 1])
    # interpolate yaw the short way round, not linearly in the raw angle, so a
    # wrap across +-pi does not spin the carrot
    yaw = float(yaws[k - 1]) + u * angle_diff(float(yaws[k]), float(yaws[k - 1]))
    return point, yaw


class RustHintedController:
    """``dimos_motion2_tc.update_hinted`` behind the controller protocol."""

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
        # The path's own per-waypoint stamps. The law reads only the deltas,
        # never the absolute times: the stamps are a precision profile and not
        # a schedule.
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
