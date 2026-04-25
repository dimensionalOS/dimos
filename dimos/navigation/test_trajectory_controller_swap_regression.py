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

"""Controller swap regression (921 P6-2 / T-19).

The closed-loop harness must accept any ``TrajectoryController``. A checked-in
YAML fixture pins one scenario; the same plant, limits, and reference run
under ``HolonomicTrackingController`` and under a feedforward-only stub with
different classes and no shared concrete base beyond the protocol.
"""

from __future__ import annotations

from pathlib import Path
from typing import Any, cast

import yaml

from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.navigation.test_trajectory_holonomic_analytic_paths import (
    _LineRef,
    _integrated_wrappers,
    _simulate_closed_loop,
)
from dimos.navigation.trajectory_command_limits import HolonomicCommandLimits
from dimos.navigation.trajectory_controller import TrajectoryController
from dimos.navigation.trajectory_holonomic_plant import IntegratedHolonomicPlant
from dimos.navigation.trajectory_holonomic_tracking_controller import HolonomicTrackingController
from dimos.navigation.trajectory_types import TrajectoryMeasuredSample, TrajectoryReferenceSample

_FIXTURE = Path(__file__).resolve().parent / "fixtures" / "trajectory_controller_swap_regression_scenario_v1.yaml"


class _FeedforwardOnlyTrajectoryController:
    """Protocol-shaped stub: reference body twist only, ignores measurement (no feedback)."""

    def reset(self) -> None:
        pass

    def control(
        self,
        reference: TrajectoryReferenceSample,
        measurement: TrajectoryMeasuredSample,
    ) -> Twist:
        _ = measurement
        return Twist(reference.twist_body)


def _load_swap_scenario_v1() -> dict[str, Any]:
    data = yaml.safe_load(_FIXTURE.read_text(encoding="utf-8"))
    if not isinstance(data, dict):
        raise ValueError("swap regression fixture must be a mapping")
    if data.get("schema") != "trajectory_controller_swap_regression":
        raise ValueError("unsupported swap regression fixture schema")
    if int(data.get("version", -1)) != 1:
        raise ValueError("unsupported swap regression fixture version")
    return cast(dict[str, Any], data)


def test_p6_2_fixture_same_scenario_two_controllers_tracking_beats_feedforward_stub() -> None:
    """YAML fixture drives identical scenario; harness is not hard-wired to HolonomicTrackingController."""
    spec = _load_swap_scenario_v1()
    dt = float(spec["dt_s"])
    t_end = float(spec["t_end_s"])
    pl = cast(dict[str, Any], spec["plant"])
    ref_spec = cast(dict[str, Any], spec["reference"])
    lim_spec = cast(dict[str, Any], spec["limits"])
    gains = cast(dict[str, Any], spec["holonomic_tracking_gains"])

    limits = HolonomicCommandLimits(
        max_planar_speed_m_s=float(lim_spec["max_planar_speed_m_s"]),
        max_yaw_rate_rad_s=float(lim_spec["max_yaw_rate_rad_s"]),
        max_planar_linear_accel_m_s2=float(lim_spec["max_planar_linear_accel_m_s2"]),
        max_yaw_accel_rad_s2=float(lim_spec["max_yaw_accel_rad_s2"]),
    )
    line = _LineRef(v=float(ref_spec["v_m_s"]))

    def run_with_controller(ctrl: TrajectoryController) -> float:
        plant = IntegratedHolonomicPlant(
            x=float(pl["x_m"]),
            y=float(pl["y_m"]),
            yaw_rad=float(pl["yaw_rad"]),
        )
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
        return float(st.end_planar_div_m)

    track = HolonomicTrackingController(
        k_position_per_s=float(gains["k_position_per_s"]),
        k_yaw_per_s=float(gains["k_yaw_per_s"]),
    )
    track.configure(limits)
    div_track = run_with_controller(track)

    div_ff = run_with_controller(_FeedforwardOnlyTrajectoryController())

    assert div_ff > 0.12
    assert div_track < div_ff
    assert div_track < 0.09
