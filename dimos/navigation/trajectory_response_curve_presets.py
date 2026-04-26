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

"""Versioned response-curve presets for trajectory simulation runs.

The presets are artifact contracts first: every simulation run can serialize the
exact plant response parameters that shaped the result. ``actuated_plant_kwargs``
adapts presets that fit the current symmetric ``ActuatedHolonomicPlant`` API.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass
import math
from typing import Any, Literal

ResponseCurveSource = Literal["ideal", "synthetic", "fitted_from_go2_log", "unknown"]
ResponseCurveAxis = Literal["linear_x", "linear_y", "yaw"]
ResponseCurveDirection = Literal["positive", "negative", "bidirectional"]

RESPONSE_CURVE_SCHEMA = "holonomic_response_curve"
RESPONSE_CURVE_VERSION = 1

PR_GATE_ALLOWED_RESPONSE_CURVE_SOURCES: frozenset[ResponseCurveSource] = frozenset(
    {"ideal", "synthetic", "fitted_from_go2_log"}
)


@dataclass(frozen=True)
class ResponseCurveAxisConfig:
    """One command-to-velocity response curve for an axis and direction."""

    axis: ResponseCurveAxis
    direction: ResponseCurveDirection
    tau_s: float
    max_accel: float
    command_gain: float
    deadband: float
    saturation: float
    noise_max: float
    latency_s: float
    notes: str = ""

    def __post_init__(self) -> None:
        for name, value in (
            ("tau_s", self.tau_s),
            ("max_accel", self.max_accel),
            ("command_gain", self.command_gain),
            ("deadband", self.deadband),
            ("saturation", self.saturation),
            ("noise_max", self.noise_max),
            ("latency_s", self.latency_s),
        ):
            if not math.isfinite(value):
                raise ValueError(f"{name} must be finite")
        if self.tau_s < 0.0:
            raise ValueError("tau_s must be non-negative")
        if self.max_accel <= 0.0:
            raise ValueError("max_accel must be positive")
        if self.command_gain <= 0.0:
            raise ValueError("command_gain must be positive")
        if self.deadband < 0.0:
            raise ValueError("deadband must be non-negative")
        if self.saturation <= 0.0:
            raise ValueError("saturation must be positive")
        if self.noise_max < 0.0:
            raise ValueError("noise_max must be non-negative")
        if self.latency_s < 0.0:
            raise ValueError("latency_s must be non-negative")

    def as_parameters(self) -> dict[str, float | str]:
        return asdict(self)


@dataclass(frozen=True)
class ResponseCurvePreset:
    """Stable preset identity plus all response-curve parameters."""

    name: str
    response_curve_id: str
    response_curve_version: int
    source: ResponseCurveSource
    plant_model: str
    curves: tuple[ResponseCurveAxisConfig, ...]
    notes: str

    def __post_init__(self) -> None:
        if self.response_curve_version != RESPONSE_CURVE_VERSION:
            raise ValueError("unsupported response_curve_version")
        if not self.curves:
            raise ValueError("curves must not be empty")
        axes = {curve.axis for curve in self.curves}
        missing_axes = {"linear_x", "linear_y", "yaw"} - axes
        if missing_axes:
            missing = ", ".join(sorted(missing_axes))
            raise ValueError(f"preset {self.name!r} is missing required axes: {missing}")

    def validate_for_pr_gate(self) -> None:
        if self.source not in PR_GATE_ALLOWED_RESPONSE_CURVE_SOURCES:
            raise ValueError(f"response_curve_source {self.source!r} is not allowed for PR-gate runs")

    def to_plant_config(self) -> dict[str, Any]:
        return {
            "model": self.plant_model,
            "preset": self.name,
            "response_curve_source": self.source,
            "response_curve_id": self.response_curve_id,
            "response_curve_version": self.response_curve_version,
            "parameters": response_curve_parameters(self),
            "notes": self.notes,
        }

    def actuated_plant_kwargs(self) -> dict[str, float]:
        """Return kwargs for ``ActuatedHolonomicPlant`` when the preset is representable.

        The current plant has symmetric linear x/y dynamics, one yaw channel,
        no latency, no saturation, no deadband, and unit command gain.
        """
        try:
            linear_x = _single_curve(self, "linear_x")
            linear_y = _single_curve(self, "linear_y")
            yaw = _single_curve(self, "yaw")
        except ValueError as exc:
            raise ValueError(
                f"preset {self.name!r} is not representable by ActuatedHolonomicPlant: {exc}"
            ) from exc
        unsupported: list[str] = []
        for curve in self.curves:
            if curve.direction != "bidirectional":
                unsupported.append(f"{curve.axis}.{curve.direction}")
            if curve.command_gain != 1.0:
                unsupported.append(f"{curve.axis}.command_gain")
            if curve.deadband != 0.0:
                unsupported.append(f"{curve.axis}.deadband")
            if curve.latency_s != 0.0:
                unsupported.append(f"{curve.axis}.latency_s")
        if linear_x.tau_s != linear_y.tau_s:
            unsupported.append("linear_x/linear_y.tau_s")
        if linear_x.max_accel != linear_y.max_accel:
            unsupported.append("linear_x/linear_y.max_accel")
        if linear_x.noise_max != linear_y.noise_max:
            unsupported.append("linear_x/linear_y.noise_max")
        if unsupported:
            details = ", ".join(unsupported)
            raise ValueError(f"preset {self.name!r} is not representable by ActuatedHolonomicPlant: {details}")
        return {
            "linear_lag_time_constant_s": linear_x.tau_s,
            "yaw_lag_time_constant_s": yaw.tau_s,
            "max_linear_accel_m_s2": linear_x.max_accel,
            "max_yaw_accel_rad_s2": yaw.max_accel,
            "noise_linear_max_m_s": linear_x.noise_max,
            "noise_yaw_max_rad_s": yaw.noise_max,
        }


def _curve(
    axis: ResponseCurveAxis,
    *,
    tau_s: float,
    max_accel: float,
    saturation: float,
    noise_max: float = 0.0,
    direction: ResponseCurveDirection = "bidirectional",
    command_gain: float = 1.0,
    deadband: float = 0.0,
    latency_s: float = 0.0,
    notes: str = "",
) -> ResponseCurveAxisConfig:
    return ResponseCurveAxisConfig(
        axis=axis,
        direction=direction,
        tau_s=tau_s,
        max_accel=max_accel,
        command_gain=command_gain,
        deadband=deadband,
        saturation=saturation,
        noise_max=noise_max,
        latency_s=latency_s,
        notes=notes,
    )


def _preset(
    name: str,
    *,
    source: ResponseCurveSource,
    curves: tuple[ResponseCurveAxisConfig, ...],
    notes: str,
) -> ResponseCurvePreset:
    return ResponseCurvePreset(
        name=name,
        response_curve_id=f"{name}_v{RESPONSE_CURVE_VERSION}",
        response_curve_version=RESPONSE_CURVE_VERSION,
        source=source,
        plant_model="actuated_holonomic",
        curves=curves,
        notes=notes,
    )


def _single_curve(preset: ResponseCurvePreset, axis: ResponseCurveAxis) -> ResponseCurveAxisConfig:
    matches = [curve for curve in preset.curves if curve.axis == axis and curve.direction == "bidirectional"]
    if len(matches) != 1:
        raise ValueError(f"preset {preset.name!r} must have exactly one bidirectional {axis} curve")
    return matches[0]


RESPONSE_CURVE_PRESETS: dict[str, ResponseCurvePreset] = {
    "ideal": _preset(
        "ideal",
        source="ideal",
        curves=(
            _curve("linear_x", tau_s=0.0, max_accel=1.0e6, saturation=10.0),
            _curve("linear_y", tau_s=0.0, max_accel=1.0e6, saturation=10.0),
            _curve("yaw", tau_s=0.0, max_accel=1.0e6, saturation=10.0),
        ),
        notes="Mathematical integration baseline with effectively instant response.",
    ),
    "synthetic_nominal": _preset(
        "synthetic_nominal",
        source="synthetic",
        curves=(
            _curve("linear_x", tau_s=0.18, max_accel=1.5, saturation=2.0, deadband=0.02),
            _curve("linear_y", tau_s=0.18, max_accel=1.5, saturation=2.0, deadband=0.02),
            _curve("yaw", tau_s=0.16, max_accel=2.0, saturation=1.5, deadband=0.02),
        ),
        notes="Synthetic envelope, not a Go2 hardware claim.",
    ),
    "synthetic_sluggish": _preset(
        "synthetic_sluggish",
        source="synthetic",
        curves=(
            _curve("linear_x", tau_s=0.35, max_accel=0.75, saturation=1.4, deadband=0.04),
            _curve("linear_y", tau_s=0.35, max_accel=0.75, saturation=1.4, deadband=0.04),
            _curve("yaw", tau_s=0.30, max_accel=1.0, saturation=1.0, deadband=0.03),
        ),
        notes="Slow synthetic response for controller margin checks.",
    ),
    "synthetic_asymmetric": _preset(
        "synthetic_asymmetric",
        source="synthetic",
        curves=(
            _curve("linear_x", tau_s=0.16, max_accel=1.8, saturation=2.0, direction="positive"),
            _curve("linear_x", tau_s=0.24, max_accel=1.2, saturation=1.6, direction="negative"),
            _curve("linear_y", tau_s=0.22, max_accel=1.1, saturation=1.5, direction="positive"),
            _curve("linear_y", tau_s=0.18, max_accel=1.4, saturation=1.8, direction="negative"),
            _curve("yaw", tau_s=0.14, max_accel=2.2, saturation=1.5, direction="positive"),
            _curve("yaw", tau_s=0.20, max_accel=1.4, saturation=1.1, direction="negative"),
        ),
        notes="Synthetic directional asymmetry; requires a runner that supports direction-specific curves.",
    ),
    "synthetic_noisy": _preset(
        "synthetic_noisy",
        source="synthetic",
        curves=(
            _curve("linear_x", tau_s=0.20, max_accel=1.3, saturation=1.8, deadband=0.03, noise_max=0.04),
            _curve("linear_y", tau_s=0.20, max_accel=1.3, saturation=1.8, deadband=0.03, noise_max=0.04),
            _curve("yaw", tau_s=0.18, max_accel=1.6, saturation=1.2, deadband=0.03, noise_max=0.03),
        ),
        notes="Synthetic bounded command noise; record the run seed with this preset.",
    ),
    "synthetic_conservative_timing_stress": _preset(
        "synthetic_conservative_timing_stress",
        source="synthetic",
        curves=(
            _curve("linear_x", tau_s=0.28, max_accel=0.9, saturation=1.2, deadband=0.05, latency_s=0.08),
            _curve("linear_y", tau_s=0.28, max_accel=0.9, saturation=1.2, deadband=0.05, latency_s=0.08),
            _curve("yaw", tau_s=0.24, max_accel=1.2, saturation=0.9, deadband=0.04, latency_s=0.08),
        ),
        notes="Synthetic timing stress with fixed command latency; intended for runner timing validation.",
    ),
}


def get_response_curve_preset(name: str) -> ResponseCurvePreset:
    try:
        return RESPONSE_CURVE_PRESETS[name]
    except KeyError as exc:
        known = ", ".join(sorted(RESPONSE_CURVE_PRESETS))
        raise KeyError(f"unknown response-curve preset {name!r}; known presets: {known}") from exc


def response_curve_parameters(preset: ResponseCurvePreset) -> dict[str, Any]:
    return {
        "schema": RESPONSE_CURVE_SCHEMA,
        "version": RESPONSE_CURVE_VERSION,
        "curves": [curve.as_parameters() for curve in preset.curves],
    }


__all__ = [
    "PR_GATE_ALLOWED_RESPONSE_CURVE_SOURCES",
    "RESPONSE_CURVE_PRESETS",
    "RESPONSE_CURVE_SCHEMA",
    "RESPONSE_CURVE_VERSION",
    "ResponseCurveAxisConfig",
    "ResponseCurvePreset",
    "get_response_curve_preset",
    "response_curve_parameters",
]
