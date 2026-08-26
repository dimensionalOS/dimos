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

"""The hinted track's law: clear the plant's dead zone, brake late, ramp itself.

Four mechanisms over :mod:`~...laws.seed`:

1. WALK-THRESHOLD ENVELOPE (the embodiment's ``gait_band``). ``min_speed`` 0.2 sits inside the
   walking policy's dead zone, so the governor's creep was a stand-still.
2. TANGENT FEEDFORWARD + FOOT CORRECTION replacing the aim-at-the-carrot term,
   whose chord cut corners toward the obstacle the plan curved around.
3. BRAKE-FEASIBLE PREVIEW: a previewed waypoint imposes only what it can, given
   the body may brake on the way there.
4. SELF-RATE-LIMITED COMMAND: the law ramps its own output at the plant's slew,
   so the request it signs its name to is the one the robot executes. This is
   the only law here that keeps state, and ``reset()`` clears it.

Blind cancels the same dead zone with an actuator inverse instead of by moving
the envelope.
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
    load_extension,
    path_xy_yaw,
)
from dimos.navigation.motion.embodiment.base import Embodiment
from dimos.navigation.motion.embodiment.go2 import GO2

# Tick period assumed when there is no previous tick to difference against, and
# the cap on the period the limiter will integrate over: a longer gap means the
# caller stalled, and banking that time hands back the unbounded step the
# limiter exists to prevent.
NOMINAL_TICK = 0.02
MAX_TICK = 0.10


def config(emb: Embodiment = GO2) -> ControllerConfig:
    """The shared config, driving inside the body's own gait band.

    The band is a plant measurement, not a preference. Gait initiation is a
    BIFURCATION, not a ramp: below it the policy stands, so the governor's 0.2
    floor is a stand-still wherever the clearance sits near it. The ceiling is
    not about pace either -- a body that crosses a tight gap fast is out the
    far side before its tracking drift has eaten the margin -- and the go2's
    0.95 stops short of the 1.0 that hands its policy to a dedicated expert.
    """
    lo, hi = emb.gait_band
    return ControllerConfig(min_speed=lo, max_speed=hi)


def make(cfg: ControllerConfig | None = None, emb: Embodiment = GO2) -> HintedController:
    return HintedController(cfg, emb)


def make_rust(cfg: ControllerConfig | None = None, emb: Embodiment = GO2) -> RustHintedController:
    return RustHintedController(cfg, emb)


def _project_onto(
    xy: np.ndarray, arcs: np.ndarray, px: float, py: float
) -> tuple[np.ndarray, float]:
    """Foot of the perpendicular onto the polyline, and its arc length.

    Where on the LINE the body is, rather than which waypoint it is nearest —
    the quantity the judge measures (``judge.cross_track`` projects onto
    segments too) and the one the seed never computed.
    """
    best, foot, s_star = math.inf, xy[0], 0.0
    for m in range(len(xy) - 1):
        ax, ay = float(xy[m][0]), float(xy[m][1])
        dx, dy = float(xy[m + 1][0]) - ax, float(xy[m + 1][1]) - ay
        dd = dx * dx + dy * dy
        u = min(max(((px - ax) * dx + (py - ay) * dy) / dd, 0.0), 1.0) if dd > 1e-12 else 0.0
        qx, qy = ax + u * dx, ay + u * dy
        d = float(np.hypot(px - qx, py - qy))
        # strict `<` keeps the FIRST minimum, the same tie-break the seed's
        # argmin has -- a plan that doubles back must not snap forward
        if d < best:
            best = d
            foot = np.array([qx, qy])
            s_star = float(arcs[m]) + u * (float(arcs[m + 1]) - float(arcs[m]))
    return foot, s_star


def _point_at(xy: np.ndarray, arcs: np.ndarray, s: float) -> np.ndarray:
    """Point on the polyline at arc ``s``, clamped to its ends."""
    n = len(xy)
    if s <= 0.0:
        return np.asarray(xy[0])
    if s >= float(arcs[n - 1]):
        return np.asarray(xy[n - 1])
    k = min(max(int(np.searchsorted(arcs, s)), 1), n - 1)
    a0, a1 = float(arcs[k - 1]), float(arcs[k])
    u = (s - a0) / (a1 - a0) if a1 > a0 else 0.0
    return np.asarray(xy[k - 1] + u * (xy[k] - xy[k - 1]))


def _tangent_at(xy: np.ndarray, arcs: np.ndarray, s: float, preview: float) -> tuple[float, float]:
    """Unit direction of travel at ``s``: a chord across a CENTRED window.

    Centred, not forward-looking, and that is the whole design of it. A forward
    chord from ``s`` to ``s + preview`` is rotated toward the inside of a curve
    by ``preview / (2R)``, so the feedforward would push the body inward until
    the correction balanced it — reintroducing the seed's chord bug at small
    scale. A centred chord is the average tangent over the window, which on
    constant curvature is the tangent at ``s`` exactly: the bias is zero and
    only the anticipation survives.
    """
    total = float(arcs[len(xy) - 1])
    # widen if the window is degenerate (a plan made of fans has no arc to
    # difference across); the last resort is the whole plan's own direction
    for h in (preview * 0.5, preview * 1.5, total):
        a = _point_at(xy, arcs, max(s - h, 0.0))
        b = _point_at(xy, arcs, min(s + h, total))
        dx, dy = float(b[0]) - float(a[0]), float(b[1]) - float(a[1])
        length = float(np.hypot(dx, dy))
        if length > 1e-9:
            return dx / length, dy / length
    return 0.0, 0.0


def update(
    pose: PoseStamped, path: Path, cfg: ControllerConfig, clearance: np.ndarray | None = None
) -> tuple[float, float, float]:
    """The pure law, before the rate limiter."""
    if len(path) < 2:
        # empty path or a single-pose veto stub: there is nothing to
        # follow -- hold position (the planner is saying "stop")
        return (0.0, 0.0, 0.0)
    xy = np.array([[p.position.x, p.position.y] for p in path.poses])
    yaws = np.array([p.yaw for p in path.poses])
    px, py, pyaw = pose.position.x, pose.position.y, pose.yaw
    n = len(xy)

    seg = np.linalg.norm(np.diff(xy, axis=0), axis=1) if n > 1 else np.zeros(1)
    arcs = np.concatenate([[0.0], np.cumsum(seg)])

    i = int(np.argmin(np.linalg.norm(xy - (px, py), axis=1)))
    while (
        i + 1 < n
        and float(arcs[i + 1] - arcs[i]) < 1e-6
        and abs(angle_diff(float(yaws[i + 1]), pyaw)) < abs(angle_diff(float(yaws[i]), pyaw))
    ):
        i += 1

    j = min(i + 1, n - 1)
    ds = float(arcs[j] - arcs[i])
    dyaw = abs(angle_diff(float(yaws[j]), float(yaws[i])))
    in_fan = j > i and dyaw > 1e-6 and dyaw / max(ds, 1e-6) > cfg.fan_yaw_per_m
    rotating = in_fan and abs(angle_diff(float(yaws[j]), pyaw)) > cfg.fan_yaw_done

    foot, s_star = _project_onto(xy, arcs, px, py)

    # `yaw_ff` is the plan's turn rate per metre over the window the body is
    # about to cross; times the commanded speed it is the open-loop yaw rate
    # that holds the heading error at zero.
    rotation_in_window = False
    if rotating:
        target_yaw, yaw_ff = float(yaws[j]), 0.0
    else:
        s = float(arcs[i]) + cfg.lookahead
        k = min(int(np.searchsorted(arcs, s)), n - 1)
        # POSITION leads by `lookahead`; ORIENTATION does not. A body held at
        # the next segment's heading while still inside this one sweeps its
        # corners across a corridor it has not left yet, so aim at the heading
        # the plan wants HERE and cancel the P-loop's standing error with
        # feedforward instead of with lead. Bounded by `fan_yaw_per_m`, past
        # which the law has decided the plan rotates rather than curves;
        # `rotation_in_window` carries that the clamp FIRED down to the command
        # site, where the number is the threshold, not the plan's curvature.
        span = float(arcs[k] - arcs[i])
        raw_rate = angle_diff(float(yaws[k]), float(yaws[i])) / span if span > 0.05 else 0.0
        rotation_in_window = abs(raw_rate) > cfg.fan_yaw_per_m
        target_yaw = float(yaws[i])
        yaw_ff = min(max(raw_rate, -cfg.fan_yaw_per_m), cfg.fan_yaw_per_m)

    # speed governor: cap cruise by the room ahead, when we know it
    vmax = cfg.max_speed
    # Does the ramp find full cruise over its WHOLE window? The separator the
    # yaw feedforward's silence condition is written against; it has to keep
    # meaning "there is room everywhere ahead" rather than "the braking profile
    # happens to permit cruise here". Blind ticks leave it true, which is what
    # `vmax == max_speed` did for them.
    ramp_unlimited = True
    if clearance is not None and len(clearance) == n:
        # THE PREVIEW IS A BRAKING PROFILE, NOT A MINIMUM: a flat min asks the
        # body to already be at a pinch's speed metres before reaching it. Each
        # previewed waypoint imposes only what it can given the body may brake
        # on the way there; at d <= brake_margin that is the flat value, so the
        # waypoint the body stands on binds as hard as ever. Still a MIN over
        # the same window, so brake_accel = 0 is the seed's governor bit for
        # bit and nothing here can make it SLOWER.
        window = (arcs >= arcs[i]) & (arcs <= arcs[i] + cfg.speed_lookahead)
        denom = max(cfg.speed_clearance - cfg.speed_floor_clearance, 1e-6)
        cap = cfg.max_speed
        room = math.inf
        for k in range(n):
            if not window[k]:
                continue
            ck = float(clearance[k])
            room = min(room, ck)
            frac_k = min(max((ck - cfg.speed_floor_clearance) / denom, 0.0), 1.0)
            v_k = cfg.min_speed + (cfg.max_speed - cfg.min_speed) * frac_k
            d_k = max(float(arcs[k]) - float(arcs[i]) - cfg.brake_margin, 0.0)
            reachable = math.sqrt(v_k * v_k + 2.0 * cfg.brake_accel * d_k)
            if reachable < cap:
                cap = reachable
        # arcs[i] always passes its own mask, so the window is never empty
        if room == math.inf:
            room = float(clearance[i])
        # the flat-min ramp, kept only as the feedforward's separator:
        # `vmax == max_speed` under the seed's governor was exactly this
        ramp_unlimited = room >= cfg.speed_clearance
        vmax = min(max(cap, cfg.min_speed), cfg.max_speed)

        # PINCH ESCAPE: the lower anchor is not a constant. With room to spare
        # a low anchor is what buys precision; with the room gone it is a loss,
        # because what kills in a gap is DWELL -- the cross-track offset is a
        # fixed distance, so time inside decides whether it becomes a contact.
        # Taken as a `max` against the ramp, so the join is continuous and the
        # leg can never SLOW anything, and read over `escape_preview` rather
        # than the ramp's window, which would lift the cap while the robot is
        # still in open space short of the gap.
        hi_here = float(arcs[i]) + cfg.escape_preview
        room_here = float(clearance[i])
        for k in range(n):
            if arcs[i] <= arcs[k] <= hi_here and float(clearance[k]) < room_here:
                room_here = float(clearance[k])
        if cfg.escape_speed > cfg.min_speed and room_here < cfg.escape_clearance:
            e = min(
                max((cfg.escape_clearance - room_here) / max(cfg.escape_clearance, 1e-6), 0.0), 1.0
            )
            top = min(cfg.escape_speed, cfg.max_speed)
            vmax = max(vmax, cfg.min_speed + (top - cfg.min_speed) * e)

    # Aiming the whole velocity at the carrot drives down a CHORD, which cuts
    # the inside of every corner by about L^2/(8R) -- a distance, not a lag.
    # Splitting the jobs removes it: a FEEDFORWARD along the plan's tangent
    # carries the body at the governor's speed, and a proportional CORRECTION
    # toward the foot is the only term that answers to being off the line, so
    # the error decays to zero instead of settling at the chord's sagitta. The
    # carrot survives only as the YAW reference.
    if rotating:
        # the fan branch is the seed's, gain included
        wx = cfg.k_pos * (float(xy[i][0]) - px)
        wy = cfg.k_pos * (float(xy[i][1]) - py)
    else:
        tx, ty = _tangent_at(xy, arcs, s_star, cfg.tangent_preview)
        # terminal deceleration: the seed got this free from its carrot pinning
        # to the last waypoint, so reproduce it explicitly
        v_ff = min(vmax, cfg.k_pos * max(float(arcs[n - 1]) - s_star, 0.0))
        wx = v_ff * tx + cfg.k_pos * (float(foot[0]) - px)
        wy = v_ff * ty + cfg.k_pos * (float(foot[1]) - py)

    c, s_ = math.cos(-pyaw), math.sin(-pyaw)
    vx, vy = c * wx - s_ * wy, s_ * wx + c * wy
    speed = float(np.hypot(vx, vy))
    if speed > vmax:
        vx, vy = vx / speed * vmax, vy / speed * vmax

    # Feedforward: at `cmd_speed` along a plan turning `yaw_ff` rad per metre
    # the heading must slew at `yaw_ff * cmd_speed` just to stay aligned.
    # Supplying it open loop is what lets the proportional term settle at zero
    # error. Added inside the clamp, so the envelope holds.
    #
    # SILENT where it is an artefact: past `fan_yaw_per_m` -- a curve-vs-rotate
    # CLASSIFIER, not a yaw-authority bound -- the value fed forward is the
    # threshold itself, which can demand more rate than the ceiling allows; the
    # sum clamp then goes FLAT in the heading error, deleting the proportional
    # term rather than attenuating it, and the heading loop runs open. Gated on
    # `ramp_unlimited`, not on `vmax >= max_speed`: the braking profile reaches
    # max_speed on the APPROACH to a pinch too. It cannot be zeroed outright --
    # the same feedforward that is a hazard in open room is load-bearing in a
    # gap.
    cmd_speed = float(np.hypot(vx, vy))
    ff = 0.0 if (rotation_in_window and ramp_unlimited) else yaw_ff * cmd_speed
    wz = float(
        np.clip(cfg.k_yaw * angle_diff(target_yaw, pyaw) + ff, -cfg.max_yaw_rate, cfg.max_yaw_rate)
    )
    return (vx, vy, wz)


class HintedController:
    """The hinted law plus the only state it keeps: its own previous command.

    Rate limiting and a pure transport delay COMMUTE, and the plant's limiter is
    idempotent on a stream that already satisfies it, so feeding it a
    slew-compliant stream leaves what the policy sees unchanged to one ulp. What
    this buys is not a quieter loop but an honest one: the request the law signs
    its name to is the request the robot executes. It is not anti-churn
    reasoning — churn was measured to ANTI-predict failure here (rank-AUC 0.373)
    — and nothing treats a fast command as dangerous. The claim is narrower: a
    command the plant cannot deliver is not a command.
    """

    config: ControllerConfig

    def __init__(self, cfg: ControllerConfig | None = None, emb: Embodiment = GO2) -> None:
        self.config = cfg or config(emb)
        self._slew = emb.command_slew
        self.reset()

    def reset(self) -> None:
        self._last_cmd: tuple[float, float, float] | None = None
        self._last_t: float | None = None

    def update(
        self, pose: PoseStamped, path: Path, t: float, clearance: np.ndarray | None = None
    ) -> Twist:
        cfg = self.config
        raw = update(pose, path, cfg, clearance)

        # THE VETO IS NOT RATE LIMITED. A path shorter than two poses is the
        # planner saying "stop", and the contract is that the law obeys it by
        # commanding zero -- not by ramping toward zero over the ~12 ticks the
        # slew would take. "Hold position" is a claim about what the law ASKS
        # for, and the limiter must not quietly turn it into "coast to a halt".
        if len(path) < 2:
            self._last_cmd = (0.0, 0.0, 0.0)
            self._last_t = t if math.isfinite(t) else None
            return Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))

        # Elapsed since the command currently standing at the plant's ramp. A
        # non-finite, zero or backwards `t` carries no information about how far
        # the ramp advanced, so fall back to the nominal period rather than to
        # an unbounded (or negative) step.
        if self._last_t is not None and math.isfinite(t) and t > self._last_t:
            dt = min(t - self._last_t, MAX_TICK)
        else:
            dt = NOMINAL_TICK
        pvx, pvy, pwz = self._last_cmd or (0.0, 0.0, 0.0)
        sx, sy, sw = (self._slew[0] * dt, self._slew[1] * dt, self._slew[2] * dt)
        vx = pvx + min(max(raw[0] - pvx, -sx), sx)
        vy = pvy + min(max(raw[1] - pvy, -sy), sy)
        wz = min(max(pwz + min(max(raw[2] - pwz, -sw), sw), -cfg.max_yaw_rate), cfg.max_yaw_rate)

        # THE DECLARED ENVELOPE IS A DISC AND THE SLEW IS A BOX, so a step taken
        # toward a point inside the disc can still leave it -- the rectangle
        # between the previous command and the request has corners the disc does
        # not contain. Measured largest overshoot anywhere: |v| = 0.9509 against
        # max_speed 0.95, always a REDUCTION, but far past the parity tolerance,
        # so dropping this would put the law outside its own declared envelope.
        speed = float(np.hypot(vx, vy))
        if speed > cfg.max_speed and speed > 0.0:
            vx, vy = vx / speed * cfg.max_speed, vy / speed * cfg.max_speed

        if math.isfinite(vx) and math.isfinite(vy) and math.isfinite(wz):
            self._last_cmd = (vx, vy, wz)
            self._last_t = t
        return Twist(Vector3(vx, vy, 0.0), Vector3(0.0, 0.0, wz))


class RustHintedController:
    """``dimos_motion2_tc.HintedLaw`` behind the controller protocol."""

    config: ControllerConfig

    def __init__(self, cfg: ControllerConfig | None = None, emb: Embodiment = GO2) -> None:
        mod: Any = load_extension()
        self.config = cfg or config(emb)
        self._law = mod.HintedLaw()
        self._params = self.config.law_params
        self._hinted = self.config.hinted_params
        self._slew = emb.command_slew

    def reset(self) -> None:
        self._law.reset()

    def update(
        self, pose: PoseStamped, path: Path, t: float, clearance: np.ndarray | None = None
    ) -> Twist:
        clr = None if clearance is None else np.ascontiguousarray(clearance, dtype=np.float64)
        vx, vy, wz = self._law.step(
            (float(pose.position.x), float(pose.position.y), float(pose.yaw)),
            path_xy_yaw(path),
            clr,
            float(t),
            self._params,
            self._hinted,
            self._slew,
        )
        return Twist(Vector3(vx, vy, 0.0), Vector3(0.0, 0.0, wz))
