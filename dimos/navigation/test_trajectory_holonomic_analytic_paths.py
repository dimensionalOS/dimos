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

"""Analytic path closed-loop tests (921 P6-1 / T-18).

Known references (constant-speed line, constant-curvature arc) against
``IntegratedHolonomicPlant`` (ideal integration) and ``ActuatedHolonomicPlant``
(deterministic lag and accel limits). Each test name states the finite horizon
and the asserted error bound so failures read as spec regressions, not magic
numbers buried only in assertions.
"""

from __future__ import annotations

import math
from collections.abc import Callable
from dataclasses import dataclass

from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.navigation.trajectory_command_limits import HolonomicCommandLimits, clamp_holonomic_cmd_vel
from dimos.navigation.trajectory_holonomic_plant import ActuatedHolonomicPlant, IntegratedHolonomicPlant
from dimos.navigation.trajectory_holonomic_tracking_controller import HolonomicTrackingController
from dimos.navigation.trajectory_metrics import planar_position_divergence, pose_errors_vs_reference
from dimos.navigation.trajectory_types import TrajectoryMeasuredSample, TrajectoryReferenceSample


def _pose_xy_yaw(x: float, y: float, yaw: float) -> Pose:
    return Pose(
        x,
        y,
        0.0,
        0.0,
        0.0,
        math.sin(yaw / 2.0),
        math.cos(yaw / 2.0),
    )


def _ref(time_s: float, pose: Pose, twist: Twist) -> TrajectoryReferenceSample:
    return TrajectoryReferenceSample(time_s=time_s, pose_plan=pose, twist_body=twist)


@dataclass(frozen=True)
class _LineRef:
    v: float

    def sample(self, t: float) -> tuple[Pose, Twist]:
        p = _pose_xy_yaw(self.v * t, 0.0, 0.0)
        tw = Twist(linear=Vector3(self.v, 0.0, 0.0), angular=Vector3(0.0, 0.0, 0.0))
        return p, tw


@dataclass(frozen=True)
class _ArcRef:
    """CCW circular path radius ``R`` (m), yaw rate ``omega`` (rad/s), body twist v=R*omega."""

    R: float
    omega: float

    def sample(self, t: float) -> tuple[Pose, Twist]:
        w = self.omega
        R = self.R
        phi = w * t
        x = R * math.cos(phi)
        y = R * math.sin(phi)
        yaw = phi + 0.5 * math.pi
        p = _pose_xy_yaw(x, y, yaw)
        v = abs(R * w)
        tw = Twist(linear=Vector3(v, 0.0, 0.0), angular=Vector3(0.0, 0.0, w))
        return p, tw


@dataclass
class _TrackStats:
    end_planar_div_m: float
    end_heading_err_rad: float
    max_planar_div_m: float
    max_abs_heading_err_rad: float


def _simulate_closed_loop(
    *,
    dt: float,
    t_end: float,
    ref: Callable[[float], tuple[Pose, Twist]],
    plant_step: Callable[[Twist, float], None],
    plant_sample: Callable[[float, Twist], TrajectoryMeasuredSample],
    plant_xy_yaw: Callable[[], tuple[float, float, float]],
    ctrl: HolonomicTrackingController,
    limits: HolonomicCommandLimits,
    prev_cmd: Twist | None = None,
) -> _TrackStats:
    n = int(math.ceil(t_end / dt))
    prev = Twist() if prev_cmd is None else prev_cmd
    max_div = 0.0
    max_psi = 0.0
    end_div = 0.0
    end_psi = 0.0
    for i in range(n):
        t = i * dt
        pose_r, twist_r = ref(t)
        meas = plant_sample(t, prev)
        raw = ctrl.control(_ref(t, pose_r, twist_r), meas)
        prev = clamp_holonomic_cmd_vel(prev, raw, limits, dt)
        plant_step(prev, dt)
        xf, yf, yawf = plant_xy_yaw()
        e_at, e_ct, e_psi = pose_errors_vs_reference(
            xf,
            yf,
            yawf,
            float(pose_r.position.x),
            float(pose_r.position.y),
            float(pose_r.orientation.euler.z),
        )
        div = planar_position_divergence(e_at, e_ct)
        max_div = max(max_div, div)
        max_psi = max(max_psi, abs(e_psi))
        if i == n - 1:
            end_div = div
            end_psi = e_psi
    return _TrackStats(
        end_planar_div_m=end_div,
        end_heading_err_rad=end_psi,
        max_planar_div_m=max_div,
        max_abs_heading_err_rad=max_psi,
    )


def _integrated_wrappers(
    plant: IntegratedHolonomicPlant,
) -> tuple[Callable[[Twist, float], None], Callable[[float, Twist], TrajectoryMeasuredSample], Callable[[], tuple[float, float, float]]]:
    def step(cmd: Twist, dt_s: float) -> None:
        plant.step(cmd, dt_s)

    def sample(t: float, cmd: Twist) -> TrajectoryMeasuredSample:
        return plant.measured_sample(t, cmd)

    def xyw() -> tuple[float, float, float]:
        return plant.x, plant.y, plant.yaw_rad

    return step, sample, xyw


def _actuated_wrappers(
    plant: ActuatedHolonomicPlant,
) -> tuple[Callable[[Twist, float], None], Callable[[float, Twist], TrajectoryMeasuredSample], Callable[[], tuple[float, float, float]]]:
    def step(cmd: Twist, dt_s: float) -> None:
        plant.step(cmd, dt_s)

    def sample(t: float, cmd: Twist) -> TrajectoryMeasuredSample:
        return plant.measured_sample(t, cmd)

    def xyw() -> tuple[float, float, float]:
        return plant.x, plant.y, plant.yaw_rad

    return step, sample, xyw


def test_p6_1_analytic_line_integrated_plant_end_planar_div_lt_85mm_abs_heading_lt_125mrad_6s_horizon() -> None:
    """Lateral offset 0.35 m onto +x line at 0.35 m/s; end-state vs moving reference (name bounds)."""
    dt = 0.05
    t_end = 6.0
    limits = HolonomicCommandLimits(
        max_planar_speed_m_s=1.2,
        max_yaw_rate_rad_s=1.5,
        max_planar_linear_accel_m_s2=4.0,
        max_yaw_accel_rad_s2=4.0,
    )
    ctrl = HolonomicTrackingController(k_position_per_s=2.2, k_yaw_per_s=2.5)
    ctrl.configure(limits)
    plant = IntegratedHolonomicPlant(x=0.0, y=0.35, yaw_rad=0.0)
    line = _LineRef(v=0.35)
    step, sample, xyw = _integrated_wrappers(plant)

    def ref_fn(t: float) -> tuple[Pose, Twist]:
        return line.sample(t)

    st = _simulate_closed_loop(
        dt=dt,
        t_end=t_end,
        ref=ref_fn,
        plant_step=step,
        plant_sample=sample,
        plant_xy_yaw=xyw,
        ctrl=ctrl,
        limits=limits,
    )
    assert st.end_planar_div_m < 0.085
    assert abs(st.end_heading_err_rad) < 0.125


def test_p6_1_analytic_line_actuated_plant_end_planar_div_lt_160mm_abs_heading_lt_200mrad_8s_horizon() -> None:
    """Same geometry as the integrated line case with mild lag and slew (name bounds)."""
    dt = 0.05
    t_end = 8.0
    limits = HolonomicCommandLimits(
        max_planar_speed_m_s=1.2,
        max_yaw_rate_rad_s=1.5,
        max_planar_linear_accel_m_s2=4.0,
        max_yaw_accel_rad_s2=4.0,
    )
    ctrl = HolonomicTrackingController(k_position_per_s=2.2, k_yaw_per_s=2.5)
    ctrl.configure(limits)
    plant = ActuatedHolonomicPlant(
        x=0.0,
        y=0.35,
        yaw_rad=0.0,
        linear_lag_time_constant_s=0.08,
        yaw_lag_time_constant_s=0.08,
        max_linear_accel_m_s2=4.0,
        max_yaw_accel_rad_s2=4.0,
    )
    line = _LineRef(v=0.35)
    step, sample, xyw = _actuated_wrappers(plant)

    def ref_fn(t: float) -> tuple[Pose, Twist]:
        return line.sample(t)

    st = _simulate_closed_loop(
        dt=dt,
        t_end=t_end,
        ref=ref_fn,
        plant_step=step,
        plant_sample=sample,
        plant_xy_yaw=xyw,
        ctrl=ctrl,
        limits=limits,
    )
    assert st.end_planar_div_m < 0.16
    assert abs(st.end_heading_err_rad) < 0.20


def test_p6_1_analytic_arc_integrated_plant_max_planar_div_lt_55mm_max_abs_heading_lt_80mrad_one_rev_horizon() -> None:
    """CCW unit circle at 0.35 rad/s; start on reference; worst-case pose error over one period (name bounds)."""
    dt = 0.05
    omega = 0.35
    R = 1.0
    period = 2.0 * math.pi / abs(omega)
    limits = HolonomicCommandLimits(
        max_planar_speed_m_s=1.2,
        max_yaw_rate_rad_s=1.5,
        max_planar_linear_accel_m_s2=4.0,
        max_yaw_accel_rad_s2=4.0,
    )
    ctrl = HolonomicTrackingController(k_position_per_s=2.5, k_yaw_per_s=3.0)
    ctrl.configure(limits)
    phi0 = 0.0
    x0 = R * math.cos(phi0)
    y0 = R * math.sin(phi0)
    yaw0 = phi0 + 0.5 * math.pi
    plant = IntegratedHolonomicPlant(x=x0, y=y0, yaw_rad=yaw0)
    arc = _ArcRef(R=R, omega=omega)
    step, sample, xyw = _integrated_wrappers(plant)

    def ref_fn(t: float) -> tuple[Pose, Twist]:
        return arc.sample(t)

    st = _simulate_closed_loop(
        dt=dt,
        t_end=period,
        ref=ref_fn,
        plant_step=step,
        plant_sample=sample,
        plant_xy_yaw=xyw,
        ctrl=ctrl,
        limits=limits,
    )
    assert st.max_planar_div_m < 0.055
    assert st.max_abs_heading_err_rad < 0.08


def test_p6_1_analytic_arc_integrated_plant_max_planar_div_lt_120mm_max_abs_heading_lt_150mrad_one_rev_with_80mm_radial_offset() -> None:
    """Arc tracking from slightly inside the reference circle (name bounds)."""
    dt = 0.05
    omega = 0.3
    R = 1.0
    period = 2.0 * math.pi / abs(omega)
    limits = HolonomicCommandLimits(
        max_planar_speed_m_s=1.2,
        max_yaw_rate_rad_s=1.5,
        max_planar_linear_accel_m_s2=4.0,
        max_yaw_accel_rad_s2=4.0,
    )
    ctrl = HolonomicTrackingController(k_position_per_s=2.5, k_yaw_per_s=3.0)
    ctrl.configure(limits)
    phi0 = 0.0
    r_meas = R - 0.08
    x0 = r_meas * math.cos(phi0)
    y0 = r_meas * math.sin(phi0)
    yaw0 = phi0 + 0.5 * math.pi
    plant = IntegratedHolonomicPlant(x=x0, y=y0, yaw_rad=yaw0)
    arc = _ArcRef(R=R, omega=omega)
    step, sample, xyw = _integrated_wrappers(plant)

    def ref_fn(t: float) -> tuple[Pose, Twist]:
        return arc.sample(t)

    st = _simulate_closed_loop(
        dt=dt,
        t_end=period,
        ref=ref_fn,
        plant_step=step,
        plant_sample=sample,
        plant_xy_yaw=xyw,
        ctrl=ctrl,
        limits=limits,
    )
    assert st.max_planar_div_m < 0.12
    assert st.max_abs_heading_err_rad < 0.15
