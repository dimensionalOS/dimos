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

"""Quasi-static pair gait: turns a (vx, wz) command into slab joint targets.

The slabs work as two legs: A = outer slabs (1, 4), B = inner slabs (2, 3). Each pair
spans both sides of the hub, so a pair standing vertical on its flat feet is statically
stable on its own. One cycle:

    swing(A)  B holds the hub on flat feet; A retracts, swings, lands `step` ahead
    shift     all four feet planted; hub glides forward over A
    swing(B)  mirror of the above
    shift

During a shift the robot rests on B's toe edge and A's heel edge, which are only
(step - foot depth) apart, and the center of mass has to stay between them. That makes
short steps unstable, so the step length is fixed and speed is set by cadence instead.

Every slab only pitches about the hub axis, so the robot cannot yaw without a foot
slipping. Turning is a "twist": while the hub sits over pair B, pair A (the wide pair)
stays planted and pushes its left and right feet in opposite directions, which spins
the hub while B's feet skid. A then swings as usual.
"""

from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np

from tars_sdk import scaling as sc
from tars_sdk.kinematics import SlabGeometry, foot_fk, foot_ik, wrap_angle
from tars_sdk.types import N_SLABS, JointTargets, Measurement

GROUP_A = (0, 3)  # outer slabs
GROUP_B = (1, 2)  # inner slabs
G = 9.81


@dataclass
class GaitParams:
    hub_height: float = 1.45  # walking hinge-axis height (m); sit height is pivot_to_bottom
    step_length: float = 0.65  # foot placement ahead of the hub per half cycle (m); keep >= 0.45
    swing_time: float = 0.45
    min_shift_time: float = 0.3  # shift slows down (longer) for commands below max_vx
    rise_time: float = 1.0
    swing_press: float = 0.003  # extra slide after touchdown to load the foot (m)
    touchdown_force: float = 40.0  # N, foot counts as landed above this
    touchdown_reach: float = 0.03  # swing slide overshoot while searching for ground (m)
    force_filter_time: float = 0.05  # s, foot-load low-pass
    shift_gain: float = 0.0  # slide correction per meter of hub position error
    shift_tolerance: float = 0.04  # shift ends when the hub is this close over the landed pair (m)
    load_feedforward: float = (
        0.7  # blend of measured foot load vs equal share for slide gravity comp
    )
    twist_max: float = 0.15  # max heading change per twist (rad)
    twist_torque: float = 220.0  # pusher hinge torque at full yaw error (Nm)
    twist_timeout: float = 1.5
    twist_preload: float = 0.01  # pusher slide over-extension to load its feet (m)
    vx_deadband: float = 0.01
    wz_deadband: float = 0.03
    hinge_kp: float = 900.0
    hinge_kd: float = 40.0
    slide_kp: float = 40000.0
    slide_kd: float = 1200.0

    @property
    def max_vx(self) -> float:
        return self.step_length / (self.swing_time + self.min_shift_time)

    def shift_time(self, vx: float) -> float:
        """Shift duration so that one half cycle averages |vx|."""
        if abs(vx) < 1e-6:
            return self.min_shift_time
        return max(self.min_shift_time, self.step_length / abs(vx) - self.swing_time)

    def scaled(self, s: float) -> GaitParams:
        """Froude-scaled copy for a robot `s` times the reference size."""
        return sc.froude(
            self,
            s,
            {
                **dict.fromkeys(
                    (
                        "hub_height",
                        "step_length",
                        "swing_press",
                        "shift_tolerance",
                        "twist_preload",
                    ),
                    sc.LENGTH,
                ),
                **dict.fromkeys(
                    (
                        "swing_time",
                        "min_shift_time",
                        "rise_time",
                        "twist_timeout",
                        "force_filter_time",
                    ),
                    sc.TIME,
                ),
                "touchdown_force": sc.FORCE,
                "twist_torque": sc.TORQUE,
                "vx_deadband": sc.SPEED,
                "wz_deadband": sc.ANG_VEL,
                "hinge_kp": sc.ROT_STIFF,
                "hinge_kd": sc.ROT_DAMP,
                "slide_kp": sc.LIN_STIFF,
                "slide_kd": sc.LIN_DAMP,
            },
        )


def smooth(u: float) -> float:
    u = min(max(u, 0.0), 1.0)
    return u * u * (3 - 2 * u)


def _other(group: tuple[int, int]) -> tuple[int, int]:
    return GROUP_B if group == GROUP_A else GROUP_A


class PairGait:
    def __init__(
        self, geom: SlabGeometry, params: GaitParams, total_mass: float, lower_mass: float
    ) -> None:
        self.g = geom
        self.p = params
        self.weight = total_mass * G
        self.lower_weight = lower_mass * G

        self.phase = "sit"
        self.t = 0.0
        self.h = geom.pivot_to_bottom  # current commanded hinge-axis height
        self.rel = [0.0] * N_SLABS  # foot flat-center x relative to hub, per slab
        self.cmd = (0.0, 0.0)
        self._want_stand = False
        self._want_sit = False

        # phase-local state
        self._group: tuple[int, int] = GROUP_A  # swinging / pushing pair
        self._land = 0.0  # swing landing rel x
        self._swing_start: dict[int, tuple[float, float]] = {}  # slab -> (theta, slide)
        self._shift_d = 0.0
        self._shift_time = params.min_shift_time
        self._shift_rel0 = [0.0] * N_SLABS
        self._yaw_goal = 0.0
        self._force = np.zeros(N_SLABS)  # filtered foot loads
        self._touched = [False] * N_SLABS
        self._touch_slide = [0.0] * N_SLABS
        self._rel_meas = [0.0] * N_SLABS

    # ------------------------------------------------------------ commands
    def request_stand(self) -> None:
        self._want_stand, self._want_sit = True, False

    def reset_seated(self) -> None:
        """All slabs vertical at sit height (e.g. after folding out of roll mode)."""
        self.h = self.g.pivot_to_bottom
        self.rel = [0.0] * N_SLABS
        self._want_sit = False
        self._enter("sit")

    def request_sit(self) -> None:
        self._want_sit, self._want_stand = True, False

    def set_command(self, vx: float, wz: float) -> None:
        self.cmd = (vx, wz)

    @property
    def walking(self) -> bool:
        return self.phase in ("swing", "shift", "twist", "ready")

    @property
    def planned_hub_speed(self) -> float:
        """Forward hub speed the gait intends right now (m/s, hub frame)."""
        if self.phase != "shift":
            return 0.0
        u = min(max(self.t / self._shift_time, 0.0), 1.0)
        return self._shift_d * 6 * u * (1 - u) / self._shift_time

    @property
    def hub_lag(self) -> float:
        """How far the hub trails the gait's plan along x (m), from the planted feet; positive = behind."""
        if self.phase in ("shift", "ready", "twist"):
            planted = list(range(N_SLABS))
        elif self.phase == "swing":
            planted = [i for i in range(N_SLABS) if i not in self._group]
        else:
            return 0.0
        return float(np.mean([self._rel_meas[i] - self.rel[i] for i in planted]))

    # ------------------------------------------------------------ helpers
    def _step(self) -> float:
        vx = self.cmd[0]
        if abs(vx) < self.p.vx_deadband:
            return 0.0
        return math.copysign(self.p.step_length, vx)

    def _turning(self) -> bool:
        return abs(self.cmd[1]) >= self.p.wz_deadband

    def _enter(self, phase: str) -> None:
        self.phase, self.t = phase, 0.0

    def _start_swing(
        self, group: tuple[int, int], land: float, meas: Measurement, pitch: float
    ) -> None:
        self._group, self._land = group, land
        self._swing_start = {
            i: (pitch + meas.joint_q[2 * i], meas.joint_q[2 * i + 1]) for i in group
        }
        for i in group:
            self._touched[i] = False
        self._enter("swing")

    def _start_shift(self, d: float) -> None:
        self._shift_d, self._shift_rel0 = d, list(self.rel)
        self._shift_time = self.p.shift_time(self.cmd[0])
        self._enter("shift")

    def _start_twist(self, yaw: float) -> None:
        cycle = self.p.twist_timeout + self.p.swing_time
        dpsi = float(np.clip(self.cmd[1] * cycle, -self.p.twist_max, self.p.twist_max))
        self._yaw_goal = wrap_angle(yaw + dpsi)
        self._group = GROUP_A
        self._enter("twist")

    def _under_hub(self, group: tuple[int, int]) -> bool:
        return all(abs(self.rel[i]) < 1e-6 for i in group)

    def _next_after_planted(self, meas: Measurement, yaw: float, pitch: float) -> None:
        """Decide what to do when all feet are planted (ready or end of a shift)."""
        behind = GROUP_A if self._under_hub(GROUP_B) else GROUP_B
        if self._want_sit and all(abs(r) < 1e-6 for r in self.rel):
            self._enter("lower")
        elif self._turning() and behind == GROUP_A and not self._want_sit:
            self._start_twist(yaw)
        elif self._want_sit:
            self._start_swing(behind, 0.0, meas, pitch)
        elif self._step() != 0.0 or not all(abs(r) < 1e-6 for r in self.rel):
            self._start_swing(behind, self._step(), meas, pitch)
        elif self._turning():  # everything under the hub: twist with A
            self._start_twist(yaw)
        else:
            self._enter("ready")

    # ------------------------------------------------------------ update
    def measured_rel(self, meas: Measurement, pitch: float) -> list[float]:
        """Foot flat-center x relative to the hub, per slab, from encoders + IMU pitch."""
        return [
            foot_fk(self.g, pitch + float(meas.joint_q[2 * i]), float(meas.joint_q[2 * i + 1]))[2]
            for i in range(N_SLABS)
        ]

    def update(self, dt: float, meas: Measurement, yaw: float, pitch: float) -> JointTargets:
        p = self.p
        self.t += dt
        alpha = min(1.0, dt / p.force_filter_time)
        self._force += alpha * (meas.foot_force - self._force)
        rel_meas = self.measured_rel(meas, pitch)
        self._rel_meas = rel_meas
        stance = list(range(N_SLABS))

        if self.phase == "sit":
            self.h = self.g.pivot_to_bottom
            if self._want_stand:
                self._want_stand = False
                self._enter("rise")
        elif self.phase == "rise":
            u = smooth(self.t / p.rise_time)
            self.h = self.g.pivot_to_bottom + u * (p.hub_height - self.g.pivot_to_bottom)
            if self.t >= p.rise_time:
                self._enter("ready")
        elif self.phase == "lower":
            u = smooth(self.t / p.rise_time)
            self.h = p.hub_height + u * (self.g.pivot_to_bottom - p.hub_height)
            if self.t >= p.rise_time:
                self._want_sit = False
                self._enter("sit")
        elif self.phase == "ready":
            self._next_after_planted(meas, yaw, pitch)
        elif self.phase == "shift":
            u = smooth(self.t / self._shift_time)
            self.rel = [r0 - self._shift_d * u for r0 in self._shift_rel0]
            landed = max(abs(rel_meas[i]) for i in self._group)
            if (
                self.t >= self._shift_time and landed < p.shift_tolerance
            ) or self.t >= 2 * self._shift_time:
                self.rel = [0.0 if i in self._group else rel_meas[i] for i in range(N_SLABS)]
                self._next_after_planted(meas, yaw, pitch)
        elif self.phase == "swing":
            stance = [i for i in range(N_SLABS) if i not in self._group]
            if self.t >= p.swing_time and all(self._touched[i] for i in self._group):
                for i in self._group:
                    self.rel[i] = rel_meas[i]
                if abs(self._land) > 1e-6:
                    self._start_shift(float(np.mean([rel_meas[i] for i in self._group])))
                else:
                    self.rel = [0.0] * N_SLABS
                    self._next_after_planted(meas, yaw, pitch)
                stance = list(range(N_SLABS))
        elif self.phase == "twist":
            err = wrap_angle(self._yaw_goal - yaw)
            if abs(err) < 0.01 or self.t >= p.twist_timeout:
                self._start_swing(GROUP_A, self._step(), meas, pitch)
                stance = [i for i in range(N_SLABS) if i not in GROUP_A]

        out = JointTargets()
        self._fill_targets(out, stance, meas, pitch, yaw, rel_meas)
        return out

    def _fill_targets(
        self,
        out: JointTargets,
        stance: list[int],
        meas: Measurement,
        pitch: float,
        yaw: float,
        rel_meas: list[float],
    ) -> None:
        p = self.p
        n_stance = max(len(stance), 1)

        # Hub position feedback during a shift: move the hub forward by lengthening the
        # trailing pair and shortening the leading one (the only way to push the hub).
        dslide = [0.0] * N_SLABS
        if self.phase == "shift":
            err = float(np.mean([rel_meas[i] - self.rel[i] for i in stance]))
            mid = float(np.mean(self.rel))
            for i in stance:
                lead = 1.0 if self.rel[i] > mid else -1.0
                dslide[i] = float(
                    np.clip(-p.shift_gain * err * lead, -p.touchdown_reach, p.touchdown_reach)
                )

        for i in range(N_SLABS):
            hi, si = 2 * i, 2 * i + 1
            out.kp[hi], out.kd[hi] = p.hinge_kp, p.hinge_kd
            out.kp[si], out.kd[si] = p.slide_kp, p.slide_kd
            if i in stance:
                # stance hinges are relative to the hub: the ground fixes their world angle
                theta, slide = foot_ik(self.g, self.rel[i], self.h)
                out.q[hi] = theta
                out.q[si] = slide + dslide[i]
                share = self.weight / n_stance
                out.tau[si] = (
                    p.load_feedforward * float(self._force[i]) + (1 - p.load_feedforward) * share
                )
            else:
                # swing hinges track a world angle (IMU pitch) so hub drift doesn't move the landing
                phi0, slide0 = self._swing_start[i]
                phi_land, slide_land = foot_ik(self.g, self._land, self.h)
                u = self.t / p.swing_time
                if u < 0.25:
                    phi, slide = phi0, slide0 * (1 - smooth(u / 0.25))
                elif u < 0.75:
                    phi, slide = phi0 + (phi_land - phi0) * smooth((u - 0.25) / 0.5), 0.0
                else:
                    phi = phi_land
                    if self._touched[i] or meas.foot_force[i] > p.touchdown_force:
                        if not self._touched[i]:
                            self._touched[i] = True
                            self._touch_slide[i] = float(meas.joint_q[si]) + p.swing_press
                        slide = self._touch_slide[i]
                    else:  # keep extending past the planned length until the foot finds ground
                        slide = (slide_land + p.touchdown_reach) * smooth((u - 0.75) / 0.25)
                out.q[hi], out.q[si] = phi - pitch, slide
                out.tau[si] = -self.lower_weight * math.cos(phi)
            out.q[si] = float(np.clip(out.q[si], 0.0, self.g.slide_travel))

        if self.phase == "twist":
            err = wrap_angle(self._yaw_goal - yaw)
            push = p.twist_torque * float(np.clip(err / 0.05, -1.0, 1.0))
            for i in GROUP_A:
                hi, si = 2 * i, 2 * i + 1
                left = self.g.slab_y[i] > 0
                out.q[hi] = meas.joint_q[hi]  # hinge in torque mode, damped
                out.kp[hi] = 0.0
                out.tau[hi] = -push if left else push
                out.q[si] += p.twist_preload
