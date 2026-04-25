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

"""Bounded holonomic calibration scenarios and v1 params output (921 P5-1, P5-4).

Plant-only path: a step along body ``+x`` (or ``+y``) with a fixed open-loop
command, a zero-command dwell, then a symmetric return segment of equal step
count. The run records pose and body twist, estimates a first-order ``vx``
time constant on the outbound hold when dynamics are lag-dominant, and writes a
versioned YAML artifact. Replay-backed calibration can reuse the same
``HolonomicCalibrationParamsV1`` schema later.
"""

from __future__ import annotations

import math
from dataclasses import asdict, dataclass, field
from datetime import datetime, timezone
from pathlib import Path
from typing import IO, Any, Literal

import yaml

from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.navigation.trajectory_holonomic_plant import (
    ActuatedHolonomicPlant,
    IntegratedHolonomicPlant,
)

# Schema for written YAML; bump when fields change incompatibly.
HOLONOMIC_TRAJECTORY_CALIBRATION_SCHEMA = "holonomic_trajectory_calibration"
HOLONOMIC_TRAJECTORY_CALIBRATION_VERSION = 1

HolonomicBodyAxis = Literal["x", "y"]


def _zero_twist() -> Twist:
    return Twist(linear=Vector3(0.0, 0.0, 0.0), angular=Vector3(0.0, 0.0, 0.0))


def _twist_axis(peak: float, axis: HolonomicBodyAxis) -> Twist:
    if axis == "x":
        return Twist(linear=Vector3(peak, 0.0, 0.0), angular=Vector3(0.0, 0.0, 0.0))
    return Twist(linear=Vector3(0.0, peak, 0.0), angular=Vector3(0.0, 0.0, 0.0))


def _plan_displacement(plant: IntegratedHolonomicPlant | ActuatedHolonomicPlant, x0: float, y0: float, axis: HolonomicBodyAxis) -> float:
    if axis == "x":
        return float(plant.x - x0)
    return float(plant.y - y0)


@dataclass(frozen=True)
class HolonomicStepDwellReturnConfig:
    """Open-loop step, dwell, and symmetric return, bounded in plan displacement.

    ``step_displacement_m`` is a target motion along the chosen body axis, expressed
    as change in the corresponding plan coordinate (body and plan are aligned
    if yaw is zero). The maneuver never targets more than
    ``max_displacement_m`` per leg.
    """

    max_displacement_m: float
    step_displacement_m: float
    peak_body_speed_m_s: float
    dwell_s: float
    dt_s: float
    body_axis: HolonomicBodyAxis = "x"
    # Optional safety cap: leave None for default from geometry.
    max_simulation_steps: int | None = None

    def __post_init__(self) -> None:
        if not math.isfinite(self.max_displacement_m) or self.max_displacement_m <= 0.0:
            raise ValueError("max_displacement_m must be a positive finite float")
        if not math.isfinite(self.step_displacement_m) or self.step_displacement_m <= 0.0:
            raise ValueError("step_displacement_m must be a positive finite float")
        if self.step_displacement_m > self.max_displacement_m + 1e-9:
            raise ValueError("step_displacement_m must be <= max_displacement_m")
        if not math.isfinite(self.peak_body_speed_m_s) or self.peak_body_speed_m_s <= 0.0:
            raise ValueError("peak_body_speed_m_s must be a positive finite float")
        if not math.isfinite(self.dwell_s) or self.dwell_s < 0.0:
            raise ValueError("dwell_s must be a non-negative finite float")
        if not (math.isfinite(self.dt_s) and self.dt_s > 0.0):
            raise ValueError("dt_s must be a positive finite float")
        if self.max_simulation_steps is not None and (
            not isinstance(self.max_simulation_steps, int) or self.max_simulation_steps <= 0
        ):
            raise ValueError("max_simulation_steps must be a positive int when set")
        t_min = self.step_displacement_m / self.peak_body_speed_m_s
        if t_min * self.peak_body_speed_m_s < self.step_displacement_m * 0.5:
            # Avoid absurdly long searches if misconfigured; still allow any positive dt.
            pass


@dataclass(frozen=True)
class HolonomicPlantCalibrationRecord:
    """One plant tick during an open-loop calibration run."""

    time_s: float
    phase: Literal["outbound", "dwell", "return"]
    cmd_linear_x: float
    cmd_linear_y: float
    cmd_angular_z: float
    meas_x_m: float
    meas_y_m: float
    meas_yaw_rad: float
    meas_vx: float
    meas_vy: float
    meas_wz: float


@dataclass
class HolonomicPlantCalibrationResult:
    """Result of a plant-only step-dwell-return run."""

    config: HolonomicStepDwellReturnConfig
    records: list[HolonomicPlantCalibrationRecord] = field(default_factory=list)
    outbound_step_count: int = 0
    dwell_step_count: int = 0
    return_step_count: int = 0
    estimated_linear_lag_s: float | None = None
    return_plan_error_m: float = 0.0
    suggested_k_position_per_s: float = 0.0
    suggested_k_yaw_per_s: float = 0.0


@dataclass(frozen=True)
class HolonomicCalibrationParamsV1:
    """Versioned artifact (YAML on disk) for holonomic tracking calibration.

    This is the persistence shape for T-16; not intended for hand-edits in
    normal operation. Load with :func:`load_holonomic_calibration_params_yaml`.
    """

    schema: str
    version: int
    kind: Literal["plant", "replay"]
    generated_utc: str
    scenario: dict[str, Any]
    results: dict[str, Any]
    suggested_holonomic_gains: dict[str, float]

    def __post_init__(self) -> None:
        if self.schema != HOLONOMIC_TRAJECTORY_CALIBRATION_SCHEMA:
            raise ValueError("unsupported calibration schema id")
        if int(self.version) != HOLONOMIC_TRAJECTORY_CALIBRATION_VERSION:
            raise ValueError("unsupported calibration version")


def _default_max_steps(cfg: HolonomicStepDwellReturnConfig) -> int:
    if cfg.max_simulation_steps is not None:
        return int(cfg.max_simulation_steps)
    t_move = float(cfg.max_displacement_m) / max(cfg.peak_body_speed_m_s, 1e-9)
    t_total = 4.0 * t_move + float(cfg.dwell_s) + 10.0
    n = int(math.ceil(t_total / cfg.dt_s)) + 1000
    return min(max(n, 20_000), 5_000_000)


def _estimate_first_order_tau_s(dt_s: float, u_cmd: float, vx_samples: list[float]) -> float | None:
    """``tau`` as time to 63% of step ``u`` on the axis velocity (first-order heuristic)."""
    u = abs(float(u_cmd))
    if u < 1e-9 or not vx_samples:
        return None
    if float(vx_samples[0]) >= 0.999 * u - 1e-9:
        return None
    target = 0.632 * u
    for i, vx in enumerate(vx_samples):
        if float(vx) >= target - 1e-9:
            return float((i + 1) * dt_s)
    return None


def _suggested_gains_from_tau(
    lag_s: float | None, *, k_yaw_scale: float = 0.75, max_kp: float = 2.0, min_kp: float = 0.2
) -> tuple[float, float]:
    if lag_s is None or lag_s <= 0.0 or not math.isfinite(lag_s):
        return 1.0, 0.75
    k_p = 1.0 / max(3.0 * float(lag_s), 0.01)
    k_p = min(max_kp, max(min_kp, k_p))
    k_y = min(max_kp, max(min_kp * k_yaw_scale, 0.5 * k_p))
    return (k_p, k_y)


def run_step_dwell_return_on_plant(
    plant: IntegratedHolonomicPlant | ActuatedHolonomicPlant,
    config: HolonomicStepDwellReturnConfig,
) -> HolonomicPlantCalibrationResult:
    """Run open-loop body-axis step, dwell, symmetric return. Plant starts at the caller pose.

    The displacement target uses plan-frame``x`` or``y`` depending on
    ``body_axis``, matching a zero-yaw or ``map``-aligned start.
    """
    x0 = float(plant.x)
    y0 = float(plant.y)
    cmd_hold = _twist_axis(config.peak_body_speed_m_s, config.body_axis)
    cmd_back = _twist_axis(-config.peak_body_speed_m_s, config.body_axis)
    t = 0.0
    records: list[HolonomicPlantCalibrationRecord] = []
    cap = _default_max_steps(config)
    n_out = 0
    n_dwell = int(max(0, round(config.dwell_s / config.dt_s)))

    pos_tol = 1.0e-3 * min(config.max_displacement_m, 0.1) + 1.0e-6
    d = 0.0

    while n_out < cap:
        t += config.dt_s
        plant.step(cmd_hold, config.dt_s)
        m = plant.measured_sample(t, cmd_hold)
        n_out += 1
        records.append(
            HolonomicPlantCalibrationRecord(
                time_s=t,
                phase="outbound",
                cmd_linear_x=float(cmd_hold.linear.x),
                cmd_linear_y=float(cmd_hold.linear.y),
                cmd_angular_z=float(cmd_hold.angular.z),
                meas_x_m=float(m.pose_plan.position.x),
                meas_y_m=float(m.pose_plan.position.y),
                meas_yaw_rad=float(m.pose_plan.orientation.euler.z),
                meas_vx=float(m.twist_body.linear.x),
                meas_vy=float(m.twist_body.linear.y),
                meas_wz=float(m.twist_body.angular.z),
            )
        )
        d = _plan_displacement(plant, x0, y0, config.body_axis)
        if d + pos_tol >= config.step_displacement_m:
            break

    if d + pos_tol < config.step_displacement_m and n_out >= cap:
        raise RuntimeError("outbound phase hit max_simulation_steps before reaching target displacement")

    z = _zero_twist()
    for _ in range(n_dwell):
        t += config.dt_s
        plant.step(z, config.dt_s)
        m = plant.measured_sample(t, z)
        records.append(
            HolonomicPlantCalibrationRecord(
                time_s=t,
                phase="dwell",
                cmd_linear_x=0.0,
                cmd_linear_y=0.0,
                cmd_angular_z=0.0,
                meas_x_m=float(m.pose_plan.position.x),
                meas_y_m=float(m.pose_plan.position.y),
                meas_yaw_rad=float(m.pose_plan.orientation.euler.z),
                meas_vx=float(m.twist_body.linear.x),
                meas_vy=float(m.twist_body.linear.y),
                meas_wz=float(m.twist_body.angular.z),
            )
        )

    n_ret = 0
    while n_ret < n_out and n_ret < cap:
        t += config.dt_s
        plant.step(cmd_back, config.dt_s)
        m = plant.measured_sample(t, cmd_back)
        n_ret += 1
        records.append(
            HolonomicPlantCalibrationRecord(
                time_s=t,
                phase="return",
                cmd_linear_x=float(cmd_back.linear.x),
                cmd_linear_y=float(cmd_back.linear.y),
                cmd_angular_z=float(cmd_back.angular.z),
                meas_x_m=float(m.pose_plan.position.x),
                meas_y_m=float(m.pose_plan.position.y),
                meas_yaw_rad=float(m.pose_plan.orientation.euler.z),
                meas_vx=float(m.twist_body.linear.x),
                meas_vy=float(m.twist_body.linear.y),
                meas_wz=float(m.twist_body.angular.z),
            )
        )

    out_rec = [r for r in records if r.phase == "outbound"]
    if config.body_axis == "x":
        v_series = [r.meas_vx for r in out_rec]
    else:
        v_series = [r.meas_vy for r in out_rec]
    tau = _estimate_first_order_tau_s(config.dt_s, config.peak_body_speed_m_s, v_series)

    err_x = float(plant.x - x0)
    err_y = float(plant.y - y0)
    r_err = math.hypot(err_x, err_y)
    k_p, k_y = _suggested_gains_from_tau(tau)

    out = HolonomicPlantCalibrationResult(
        config=config,
        records=records,
        outbound_step_count=n_out,
        dwell_step_count=n_dwell,
        return_step_count=n_ret,
        estimated_linear_lag_s=tau,
        return_plan_error_m=r_err,
        suggested_k_position_per_s=k_p,
        suggested_k_yaw_per_s=k_y,
    )
    return out


def plant_result_to_params_v1(result: HolonomicPlantCalibrationResult) -> HolonomicCalibrationParamsV1:
    sc = asdict(result.config)
    if "max_simulation_steps" in sc and sc["max_simulation_steps"] is None:
        del sc["max_simulation_steps"]
    return HolonomicCalibrationParamsV1(
        schema=HOLONOMIC_TRAJECTORY_CALIBRATION_SCHEMA,
        version=HOLONOMIC_TRAJECTORY_CALIBRATION_VERSION,
        kind="plant",
        generated_utc=datetime.now(timezone.utc).isoformat(),
        scenario=sc,
        results={
            "outbound_step_count": result.outbound_step_count,
            "dwell_step_count": result.dwell_step_count,
            "return_step_count": result.return_step_count,
            "estimated_linear_lag_s": result.estimated_linear_lag_s,
            "return_plan_error_m": result.return_plan_error_m,
        },
        suggested_holonomic_gains={
            "k_position_per_s": result.suggested_k_position_per_s,
            "k_yaw_per_s": result.suggested_k_yaw_per_s,
        },
    )


def write_holonomic_calibration_params_yaml(params: HolonomicCalibrationParamsV1, target: str | Path | IO[str]) -> None:
    block: dict[str, Any] = {
        "schema": params.schema,
        "version": params.version,
        "kind": params.kind,
        "generated_utc": params.generated_utc,
        "scenario": params.scenario,
        "results": params.results,
        "suggested_holonomic_gains": params.suggested_holonomic_gains,
    }
    text = yaml.safe_dump(
        block,
        sort_keys=False,
        allow_unicode=True,
    )
    if isinstance(target, (str, Path)):
        Path(target).write_text(text, encoding="utf-8")
    else:
        target.write(text)
        if hasattr(target, "flush"):
            target.flush()


def read_holonomic_calibration_params_yaml(source: str | Path | IO[str]) -> HolonomicCalibrationParamsV1:
    if isinstance(source, (str, Path)):
        data = yaml.safe_load(Path(source).read_text(encoding="utf-8"))
    else:
        r = source.read()
        if not isinstance(r, str):
            r = r.decode("utf-8")
        data = yaml.safe_load(r)
    if not isinstance(data, dict):
        raise ValueError("calibration YAML must be a mapping at top level")
    return HolonomicCalibrationParamsV1(
        schema=str(data["schema"]),
        version=int(data["version"]),
        kind=data["kind"],
        generated_utc=str(data["generated_utc"]),
        scenario=dict(data.get("scenario", {})),
        results=dict(data.get("results", {})),
        suggested_holonomic_gains={k: float(v) for k, v in dict(data.get("suggested_holonomic_gains", {})).items()},
    )


__all__ = [
    "HOLONOMIC_TRAJECTORY_CALIBRATION_SCHEMA",
    "HOLONOMIC_TRAJECTORY_CALIBRATION_VERSION",
    "HolonomicBodyAxis",
    "HolonomicCalibrationParamsV1",
    "HolonomicPlantCalibrationRecord",
    "HolonomicPlantCalibrationResult",
    "HolonomicStepDwellReturnConfig",
    "plant_result_to_params_v1",
    "read_holonomic_calibration_params_yaml",
    "run_step_dwell_return_on_plant",
    "write_holonomic_calibration_params_yaml",
]
