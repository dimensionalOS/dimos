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

"""Sim-only locomotion assist: a bounded virtual wrench on the hub.

The stock TARS design (pitch-only hinges, flat feet) cannot walk quasi-statically: every
leg force pushes the hub *away* from its foot, so nothing can push the hub forward over
the trailing pair until the center of mass is already past its toe, and nothing can yaw
it without slipping. Until the design or the controller solves that (ankles, a dynamic
gait, RL), the assist supplies what is missing:

- tilt:    PD on hub roll/pitch keeps the robot from toppling
- forward: pushes the hub along the gait's planned trajectory during a shift
- yaw:     tracks the commanded heading (feet skid, as they would when twisting)

Everything else is real physics: contacts, joint torques, sensors, odometry.
Disable with TarsClient(assist=None) to work on unassisted locomotion.
"""

from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np
from numpy.typing import NDArray

from tars_sdk import scaling as sc
from tars_sdk.kinematics import heading, quat_to_mat, wrap_angle


@dataclass
class AssistParams:
    tilt_kp: float = 1500.0  # Nm/rad
    tilt_kd: float = 150.0  # Nm/(rad/s)
    max_tilt_torque: float = 300.0
    push_kp: float = 1800.0  # N/m of hub lag
    push_kv: float = 900.0  # N/(m/s) of hub speed error
    max_force: float = 750.0  # ~1.3x robot weight: walking speed is mostly assist
    yaw_kp: float = 600.0  # Nm/rad
    yaw_kd: float = 150.0
    max_yaw_torque: float = 400.0
    roll_push_kv: float = 300.0  # N per m/s of speed error in roll mode
    max_roll_force: float = 700.0

    def scaled(self, s: float) -> AssistParams:
        """Froude-scaled copy for a robot `s` times the reference size."""
        return sc.froude(
            self,
            s,
            {
                **dict.fromkeys(
                    ("tilt_kp", "yaw_kp", "max_tilt_torque", "max_yaw_torque"), sc.TORQUE
                ),
                **dict.fromkeys(("tilt_kd", "yaw_kd"), sc.ROT_DAMP),
                **dict.fromkeys(("max_force", "max_roll_force"), sc.FORCE),
                "push_kp": sc.LIN_STIFF,
                **dict.fromkeys(("push_kv", "roll_push_kv"), sc.LIN_DAMP),
            },
        )


class Assist:
    def __init__(self, params: AssistParams) -> None:
        self.p = params
        self.heading: float | None = None

    def reset_heading(self, yaw: float) -> None:
        self.heading = yaw

    def roll_push(
        self,
        foot_offset: float,
        hub_height: float,
        weight: float,
        v_fwd: float,
        v_cmd: float,
        hold: bool = False,
    ) -> float:
        """Roll mode forward force (N) along the heading.

        Rolling over a planted spoke at constant hub height is an inverted pendulum: with
        the hub behind the foot, gravity pulls it back by weight * offset / height. The
        push cancels that and tracks the commanded speed.
        """
        p = self.p
        gain = 2.0 if hold else 1.0  # hold: also pull the hub over the planted spoke to stop
        f = gain * weight * foot_offset / max(hub_height, 1e-3) + p.roll_push_kv * (v_cmd - v_fwd)
        return float(np.clip(f, -p.max_roll_force, p.max_roll_force))

    def wrench(
        self,
        dt: float,
        quat: NDArray[np.float64],
        lin_vel_world: NDArray[np.float64],
        ang_vel_world: NDArray[np.float64],
        wz_cmd: float,
        planned_speed: float,
        hub_lag: float,
        active: bool,
        rolling: bool = False,
        roll_force: float = 0.0,
    ) -> NDArray[np.float64]:
        """World-frame [fx, fy, fz, tx, ty, tz] to apply at the hub.

        Rolling: the hub turns with the wheel, so only sideways lean is corrected and the
        forward push tracks `planned_speed` directly.
        """
        p = self.p
        R = quat_to_mat(quat)
        yaw = heading(R)
        if self.heading is None:
            self.heading = yaw
        out = np.zeros(6)
        cy, sy = math.cos(yaw), math.sin(yaw)
        fwd = np.array([cy, sy, 0.0])

        if rolling:  # lean only: tip the axle back to horizontal about the heading axis
            lean = math.asin(max(-1.0, min(1.0, float(R[2, 1]))))
            tau = (-p.tilt_kp * lean - p.tilt_kd * float(fwd @ ang_vel_world)) * fwd
        else:  # tilt: rotate the body z-axis back toward world z
            z_axis = R[:, 2]
            tilt_axis = np.cross(z_axis, np.array([0.0, 0.0, 1.0]))  # |.| = sin(tilt)
            w_tilt = ang_vel_world.copy()
            w_tilt[2] = 0.0
            tau = p.tilt_kp * tilt_axis - p.tilt_kd * w_tilt
        n = np.linalg.norm(tau)
        if n > p.max_tilt_torque:
            tau *= p.max_tilt_torque / n
        out[3:5] = tau[:2]

        if not active:  # sitting / rising: only keep it upright
            self.heading = yaw
            return out

        # heading
        self.heading = wrap_angle(self.heading + wz_cmd * dt)
        tz = p.yaw_kp * wrap_angle(self.heading - yaw) + p.yaw_kd * (wz_cmd - ang_vel_world[2])
        out[5] = float(np.clip(tz, -p.max_yaw_torque, p.max_yaw_torque))

        # forward push along the gait plan; damp sideways drift
        v_fwd = cy * lin_vel_world[0] + sy * lin_vel_world[1]
        v_side = -sy * lin_vel_world[0] + cy * lin_vel_world[1]
        if rolling:
            f_fwd = roll_force
        else:
            f_fwd = p.push_kp * hub_lag + p.push_kv * (planned_speed - v_fwd)
        f_side = -p.push_kv * v_side
        if not rolling:
            f_fwd = float(np.clip(f_fwd, -p.max_force, p.max_force))
        f_side = float(np.clip(f_side, -p.max_force, p.max_force))
        out[0] = cy * f_fwd - sy * f_side
        out[1] = sy * f_fwd + cy * f_side
        return out
