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

"""Roll mode: the four slabs lock 90 degrees apart as spokes and TARS rolls like a wheel.

Spoke order around the wheel alternates sides (slab 1 L-outer, 3 R-inner, 2 L-inner,
4 R-outer) so consecutive ground contacts switch between left and right. A bare 4-spoke
rimless wheel loses nearly all its speed at every spoke landing (spokes are 90 deg
apart), so each loaded spoke telescopes to h/cos(angle): the hub glides level over a
virtual rim and two spokes share the load at each hand-off. That needs ~0.57 m of slide
travel. The hub is locked to the wheel and turns with it.

Phases: to_roll -> roll -> brake -> gather -> done (hand back to the walking gait).
"""

from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np

from tars_sdk import scaling as sc
from tars_sdk.gait import smooth
from tars_sdk.kinematics import SlabGeometry, wrap_angle
from tars_sdk.types import N_SLABS, JointTargets, Measurement

# World angle of each slab (0 = pointing down, + = foot toward -x) with the wheel at rest.
# Forward rolling increases every spoke's angle, so the next spoke to land is at -pi/2.
WHEEL_ANGLE = (0.0, math.pi, -math.pi / 2, math.pi / 2)  # slabs 1, 2, 3, 4
HUB_PITCH = -math.pi / 4  # keeps every hinge within +/-3pi/4 of its joint limit


@dataclass
class RollParams:
    max_speed: float = 1.2  # m/s
    transition_time: float = 2.0
    gather_time: float = 1.5
    brake_speed: float = 0.1  # m/s (smoothed), below this the wheel may fold up
    brake_hold: float = 0.5  # s the wheel must stay that slow before folding
    contact_press: float = 0.01  # extra slide on the loaded spoke (m)
    hinge_kp: float = 10000.0
    hinge_kd: float = 300.0
    slide_kp: float = 40000.0
    slide_kd: float = 1200.0

    def scaled(self, s: float) -> RollParams:
        """Froude-scaled copy for a robot `s` times the reference size."""
        return sc.froude(
            self,
            s,
            {
                **dict.fromkeys(("max_speed", "brake_speed"), sc.SPEED),
                **dict.fromkeys(("transition_time", "gather_time", "brake_hold"), sc.TIME),
                "contact_press": sc.LENGTH,
                "hinge_kp": sc.ROT_STIFF,
                "hinge_kd": sc.ROT_DAMP,
                "slide_kp": sc.LIN_STIFF,
                "slide_kd": sc.LIN_DAMP,
            },
        )


class RollGait:
    def __init__(self, geom: SlabGeometry, params: RollParams, total_mass: float) -> None:
        self.g = geom
        self.p = params
        self.weight = total_mass * 9.81
        self.phase = "done"
        self.t = 0.0
        self.cmd = (0.0, 0.0)
        self._exit = False
        self._q0 = np.zeros(N_SLABS)
        self._q_goal = np.zeros(N_SLABS)
        self._anchor_slide = 0.0
        self._slow_for = 0.0
        self._speed_avg = 0.0
        self.foot_offset = 0.0  # load-weighted (planted foot x - hub x) along the heading (m)

    @property
    def active(self) -> bool:
        return self.phase != "done"

    @property
    def speed_command(self) -> float:
        """Forward speed the assist should drive (m/s)."""
        return self.cmd[0] if self.phase == "roll" else 0.0

    def set_command(self, vx: float, wz: float) -> None:
        self.cmd = (vx, wz)

    def start(self, meas: Measurement) -> None:
        self._q0 = np.array([meas.joint_q[2 * i] for i in range(N_SLABS)])
        self._anchor_slide = float(
            meas.joint_q[1]
        )  # slab 1 carries the robot while the others lift
        self._q_goal = np.array([wrap_angle(w - HUB_PITCH) for w in WHEEL_ANGLE])
        self._exit = False
        self._enter("to_roll")

    def request_exit(self) -> None:
        self._exit = True

    def _enter(self, phase: str) -> None:
        self.phase, self.t = phase, 0.0

    def _spoke_slide(self, b: float, others_landed: bool) -> float:
        """Slide for a spoke at angle b from straight down, signed along the roll direction.

        Within reach (|b| <= b_max) the spoke is exactly as long as needed to keep the hub at
        its rolling height (a virtual rim, so hand-offs happen without impact). Spokes about
        to land pre-extend; the trailing spoke only retracts once the next one carries load.
        """
        h = self.g.pivot_to_bottom
        travel = self.g.slide_travel
        b_max = math.acos(h / (h + travel))  # widest angle the spoke can still reach the rim
        if abs(b) <= b_max:
            return min(h / math.cos(b) - h + self.p.contact_press, travel)
        if -0.45 * math.pi <= b < -b_max:
            return travel
        if b_max < b <= 0.6 * math.pi:
            if not others_landed:
                return travel
            return travel * (1 - smooth((b - b_max) / (0.6 * math.pi - b_max)))
        return 0.0

    def update(self, dt: float, meas: Measurement, pitch: float, speed: float) -> JointTargets:
        p = self.p
        self.t += dt
        q = np.array([meas.joint_q[2 * i] for i in range(N_SLABS)])
        world = [wrap_angle(pitch + q[i]) for i in range(N_SLABS)]
        out = JointTargets()
        hinge = self._q_goal.copy()
        direction = 1.0 if (self.cmd[0] if abs(self.cmd[0]) > 0.05 else speed) >= 0 else -1.0
        signed = [wrap_angle(a) * direction for a in world]
        h = self.g.pivot_to_bottom
        b_max = math.acos(h / (h + self.g.slide_travel))
        leading_loaded = any(
            -b_max <= signed[j] < 0 and meas.foot_force[j] > 0.085 * self.weight
            for j in range(N_SLABS)
        )
        slides = np.array([self._spoke_slide(signed[i], leading_loaded) for i in range(N_SLABS)])

        if self.phase == "to_roll":
            u = smooth(self.t / p.transition_time)
            hinge = self._q0 + (self._q_goal - self._q0) * u
            slides = np.zeros(N_SLABS)
            slides[0] = self._anchor_slide * (1 - smooth((self.t / p.transition_time - 0.6) / 0.4))
            if self.t >= p.transition_time:
                self._enter("roll")
        elif self.phase == "roll":
            if self._exit:
                self._slow_for, self._speed_avg = 0.0, speed
                self._enter("brake")
        elif self.phase == "brake":
            # smoothed speed: a wheel parked on one spoke rocks a little, fold once it's settled
            self._speed_avg += min(1.0, dt / p.brake_hold) * (speed - self._speed_avg)
            self._slow_for = self._slow_for + dt if abs(self._speed_avg) < p.brake_speed else 0.0
            if self._slow_for >= p.brake_hold or self.t >= 6 * p.brake_hold:
                anchor = int(np.argmin([abs(a) for a in world]))
                self._q0 = q.copy()
                self._q_goal = np.full(N_SLABS, q[anchor] - world[anchor])  # all straight down
                self._enter("gather")
        elif self.phase == "gather":
            u = smooth(self.t / p.gather_time)
            hinge = self._q0 + (self._q_goal - self._q0) * u
            slides = np.zeros(N_SLABS)
            if self.t >= p.gather_time:
                self._enter("done")

        loaded = [i for i in range(N_SLABS) if meas.foot_force[i] > 0.05 * self.weight]
        f = np.array([meas.foot_force[i] for i in loaded])
        if len(loaded) and f.sum() > 0:
            offs = [-(h + meas.joint_q[2 * i + 1]) * math.sin(world[i]) for i in loaded]
            self.foot_offset = float(np.dot(f, offs) / f.sum())
        else:
            self.foot_offset = 0.0
        for i in range(N_SLABS):
            hi, si = 2 * i, 2 * i + 1
            out.q[hi], out.kp[hi], out.kd[hi] = hinge[i], p.hinge_kp, p.hinge_kd
            out.q[si], out.kp[si], out.kd[si] = slides[i], p.slide_kp, p.slide_kd
            if i in loaded:
                out.tau[si] = float(meas.foot_force[i]) * 0.8
        return out
