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

"""Go2 tuning config artifact + the DERIVE step (model -> config).

This is the contract the two tuning tools share:

* :func:`derive_config` is the **pure** DERIVE step — a fitted FOPDT
  plant model in, a fully-populated controller config out. No file or
  robot I/O, so it is unit-tested in isolation
  (``test_go2_tuning.py``).
* :class:`Go2TuningConfig` is the versioned artifact. It owns the JSON
  (de)serialization (``to_json`` / ``from_json``) and the
  runtime-config converters the benchmark tool consumes.
* :func:`invert_tolerance` is the pure tolerance -> max-safe-speed
  inversion the benchmark tool fills section 5 with (also unit-tested).

Why these numbers (the settled characterization findings, not re-derived
here — see ``reports/go2_tuning_README.md``): the Go2 base is FOPDT per
axis; at a given speed the tracking error is the plant floor
``(tau + L) * v``; reactive controllers have ~zero headroom over that
floor; the dominant lever is speed vs path curvature; the simple
production baseline P-controller is the recommended controller.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass, field
import json
from pathlib import Path
import subprocess

from dimos.control.tasks.feedforward_gain_compensator import FeedforwardGainConfig
from dimos.utils.benchmarking.plant import Go2PlantParams
from dimos.utils.benchmarking.velocity_profile import (
    GO2_VX_MAX,
    GO2_WZ_MAX,
    VelocityProfileConfig,
)

SCHEMA_VERSION = 1

# --- DERIVE tunable constants (documented; single source of truth) -------

# Cross-track headroom margin on the measured angular-rate ceiling. The
# baseline P-controller adds a cross-track correction term on top of the
# nominal turn rate; if the profile lets wz ride at the saturation
# ceiling there is no authority left for that correction and corners get
# cut (the oscillation/cut-corner failure mode). Reserve 15%.
WZ_HEADROOM_MARGIN = 0.15

# Lateral (centripetal) comfort acceleration cap for the curvature
# profile, m/s^2. Constant, not derived: it is a ride-quality / stability
# choice, not a plant property. 1.0 matches the shipped VelocityProfiler
# default and is conservative for a ~15 kg quadruped — it keeps the
# corner-speed cap inside the regime the curvature-profile R&D validated.
A_LAT_MAX = 1.0

# Braking authority exceeds forward-accel authority: a robot can decel
# harder than it can accel. Mirrors the shipped VelocityProfileConfig
# 1.0 / 2.0 accel/decel ratio.
DECEL_ACCEL_RATIO = 2.0

RECOMMENDED_CONTROLLER_EVIDENCE = (
    "Baseline P-controller, hardcoded. The Go2 base is FOPDT per axis; at "
    "a given speed the tracking error equals the plant floor (tau + L) * "
    "v, which no reactive control law can beat (~zero headroom over the "
    "floor — validated controller bake-off). The only effective lever is "
    "speed vs path curvature, which the derived velocity profile + "
    "feedforward already apply. See reports/go2_tuning_README.md and the "
    "characterization findings."
)


# --- Artifact schema -----------------------------------------------------


@dataclass
class Provenance:
    """Where/when this model was measured — defines its validity scope."""

    robot_id: str = "unknown"
    surface: str = "unknown"
    mode: str = "default"
    date: str = "unknown"
    git_sha: str = "unknown"
    sim_or_hw: str = "sim"
    characterization_session_dir: str = ""


@dataclass
class FopdtChannelDC:
    K: float
    tau: float
    L: float


@dataclass
class PlantModelDC:
    vx: FopdtChannelDC
    vy: FopdtChannelDC
    wz: FopdtChannelDC


@dataclass
class FeedforwardDC:
    K_vx: float
    K_vy: float
    K_wz: float
    output_min_vx: float = -GO2_VX_MAX
    output_max_vx: float = GO2_VX_MAX
    output_min_vy: float = -GO2_VX_MAX
    output_max_vy: float = GO2_VX_MAX
    output_min_wz: float = -GO2_WZ_MAX
    output_max_wz: float = GO2_WZ_MAX

    def to_runtime(self) -> FeedforwardGainConfig:
        """Build the live :class:`FeedforwardGainConfig` the controller
        consumes (the benchmark tool's single mapping point)."""
        return FeedforwardGainConfig(
            K_vx=self.K_vx,
            K_vy=self.K_vy,
            K_wz=self.K_wz,
            output_min_vx=self.output_min_vx,
            output_max_vx=self.output_max_vx,
            output_min_vy=self.output_min_vy,
            output_max_vy=self.output_max_vy,
            output_min_wz=self.output_min_wz,
            output_max_wz=self.output_max_wz,
        )


@dataclass
class VelocityProfileDC:
    max_linear_speed: float
    max_angular_speed: float
    max_centripetal_accel: float
    max_linear_accel: float
    max_linear_decel: float
    min_speed: float = 0.05
    lookahead_pts: int = 8

    def to_runtime(self, max_linear_speed: float | None = None) -> VelocityProfileConfig:
        """Build the live :class:`VelocityProfileConfig`. The benchmark
        tool overrides ``max_linear_speed`` per speed-ladder rung."""
        return VelocityProfileConfig(
            max_linear_speed=(
                self.max_linear_speed if max_linear_speed is None else max_linear_speed
            ),
            max_angular_speed=self.max_angular_speed,
            max_centripetal_accel=self.max_centripetal_accel,
            max_linear_accel=self.max_linear_accel,
            max_linear_decel=self.max_linear_decel,
            min_speed=self.min_speed,
            lookahead_pts=self.lookahead_pts,
        )


@dataclass
class RecommendedControllerDC:
    name: str = "baseline"
    params: dict = field(default_factory=lambda: {"k_angular": 0.5})
    evidence: str = RECOMMENDED_CONTROLLER_EVIDENCE


@dataclass
class OperatingPoint:
    path: str
    speed: float
    cte_max: float
    cte_rms: float
    arrived: bool


@dataclass
class ToleranceRow:
    tol_cm: float
    max_speed: float | None  # None = no tested speed meets the tolerance
    binding_path: str | None


@dataclass
class OperatingPointMap:
    speeds: list[float]
    points: list[OperatingPoint]
    tolerance_inversion: list[ToleranceRow]


@dataclass
class Go2TuningConfig:
    provenance: Provenance
    plant: PlantModelDC
    feedforward: FeedforwardDC
    velocity_profile: VelocityProfileDC
    recommended_controller: RecommendedControllerDC
    caveats: list[str] = field(default_factory=list)
    operating_point_map: OperatingPointMap | None = None
    schema_version: int = SCHEMA_VERSION

    # --- serialization ---

    def to_json(self, path: str | Path) -> Path:
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(asdict(self), indent=2, sort_keys=False))
        return path

    @classmethod
    def from_json(cls, path: str | Path) -> Go2TuningConfig:
        data = json.loads(Path(path).read_text())
        return cls.from_dict(data)

    @classmethod
    def from_dict(cls, data: dict) -> Go2TuningConfig:
        sv = data.get("schema_version")
        if sv != SCHEMA_VERSION:
            raise ValueError(
                f"unsupported go2 tuning artifact schema_version={sv!r} "
                f"(this build understands {SCHEMA_VERSION})"
            )

        def _chan(d: dict) -> FopdtChannelDC:
            return FopdtChannelDC(K=d["K"], tau=d["tau"], L=d["L"])

        opm = None
        if data.get("operating_point_map") is not None:
            m = data["operating_point_map"]
            opm = OperatingPointMap(
                speeds=list(m["speeds"]),
                points=[OperatingPoint(**p) for p in m["points"]],
                tolerance_inversion=[ToleranceRow(**t) for t in m["tolerance_inversion"]],
            )
        return cls(
            provenance=Provenance(**data["provenance"]),
            plant=PlantModelDC(
                vx=_chan(data["plant"]["vx"]),
                vy=_chan(data["plant"]["vy"]),
                wz=_chan(data["plant"]["wz"]),
            ),
            feedforward=FeedforwardDC(**data["feedforward"]),
            velocity_profile=VelocityProfileDC(**data["velocity_profile"]),
            recommended_controller=RecommendedControllerDC(**data["recommended_controller"]),
            caveats=list(data.get("caveats", [])),
            operating_point_map=opm,
            schema_version=sv,
        )


# --- helpers -------------------------------------------------------------


def git_sha() -> str:
    """Short HEAD sha, best-effort (``unknown`` off a repo)."""
    try:
        return (
            subprocess.run(
                ["git", "rev-parse", "--short", "HEAD"],
                capture_output=True,
                text=True,
                check=True,
            ).stdout.strip()
            or "unknown"
        )
    except Exception:
        return "unknown"


def _safe_inv_gain(K: float) -> float:
    """1/K with a guard for a degenerate (near-zero) fitted gain."""
    if abs(K) < 1e-6:
        return 1.0
    return 1.0 / K


def _channel_ceiling(per_amplitude: dict | None, channel: str, fallback: float) -> float:
    """Measured steady-state magnitude ceiling for a channel:
    ``max(K_at_amp * |amplitude|)`` over the swept amplitudes. Falls back
    to the Go2 saturation envelope when per-amplitude data is missing or
    too sparse to be trustworthy."""
    if not per_amplitude:
        return fallback
    entries = per_amplitude.get(channel) or []
    vals: list[float] = []
    for e in entries:
        K = e.get("K")
        amp = e.get("amplitude")
        if K is None or amp is None:
            continue
        try:
            vals.append(abs(float(K) * float(amp)))
        except (TypeError, ValueError):
            continue
    if not vals:
        return fallback
    return max(vals)


# --- DERIVE: pure model -> config ---------------------------------------


def derive_config(
    plant: Go2PlantParams,
    provenance: Provenance,
    *,
    per_amplitude: dict | None = None,
) -> Go2TuningConfig:
    """Derive the full controller config from a fitted FOPDT plant model.

    Pure: model + provenance in, :class:`Go2TuningConfig` out. No I/O.

    - Feedforward gain per axis = ``1 / K`` (the compensator divides the
      controller command by the plant gain so commanded == achieved).
    - ``max_angular_speed`` = measured wz ceiling minus the cross-track
      headroom margin.
    - ``max_centripetal_accel`` = the lateral comfort constant.
    - ``max_linear_accel`` ~= ``vx_ceiling / tau_vx`` (first-order rise:
      a step of size v settles in ~tau, so the achievable mean accel is
      ~v/tau); decel = ``DECEL_ACCEL_RATIO x`` that.
    - recommended controller = baseline, hardcoded, with cited evidence.

    ``per_amplitude`` (optional) is the fitter's per-amplitude table
    ``{channel: [{amplitude, K, ...}, ...]}``; when absent the Go2
    saturation envelope is used for the ceilings.
    """
    # Clamp the measured ceiling to the Go2 saturation envelope: an
    # un-saturated FOPDT fit extrapolates linearly past what the platform
    # can physically deliver, so the envelope is a hard upper bound.
    vx_ceiling = min(_channel_ceiling(per_amplitude, "vx", GO2_VX_MAX), GO2_VX_MAX)
    wz_ceiling = min(_channel_ceiling(per_amplitude, "wz", GO2_WZ_MAX), GO2_WZ_MAX)

    feedforward = FeedforwardDC(
        K_vx=_safe_inv_gain(plant.vx.K),
        K_vy=_safe_inv_gain(plant.vy.K),
        K_wz=_safe_inv_gain(plant.wz.K),
    )

    max_linear_accel = vx_ceiling / plant.vx.tau if plant.vx.tau > 1e-6 else GO2_VX_MAX
    velocity_profile = VelocityProfileDC(
        max_linear_speed=vx_ceiling,
        max_angular_speed=wz_ceiling * (1.0 - WZ_HEADROOM_MARGIN),
        max_centripetal_accel=A_LAT_MAX,
        max_linear_accel=max_linear_accel,
        max_linear_decel=max_linear_accel * DECEL_ACCEL_RATIO,
    )

    caveats = [
        f"Valid only for surface={provenance.surface!r}, "
        f"mode={provenance.mode!r}, {provenance.sim_or_hw}. Re-run "
        f"go2-characterization on any surface or gait-mode change.",
        f"Plant fitted from {provenance.characterization_session_dir or 'n/a'} "
        f"on {provenance.date} (git {provenance.git_sha}).",
    ]
    if provenance.sim_or_hw == "sim":
        caveats.append(
            "Derived from the FOPDT sim plant (self-test): validates the "
            "pipeline, not absolute on-robot numbers — re-run --mode hw "
            "for hardware-valid gains."
        )

    return Go2TuningConfig(
        provenance=provenance,
        plant=PlantModelDC(
            vx=FopdtChannelDC(plant.vx.K, plant.vx.tau, plant.vx.L),
            vy=FopdtChannelDC(plant.vy.K, plant.vy.tau, plant.vy.L),
            wz=FopdtChannelDC(plant.wz.K, plant.wz.tau, plant.wz.L),
        ),
        feedforward=feedforward,
        velocity_profile=velocity_profile,
        recommended_controller=RecommendedControllerDC(),
        caveats=caveats,
        operating_point_map=None,
    )


# --- tolerance -> max-safe-speed inversion (pure) ------------------------


def invert_tolerance(
    points: list[OperatingPoint], tolerances_cm: list[float]
) -> list[ToleranceRow]:
    """For each tolerance, the fastest speed that keeps every path within
    ``cte_max <= tol`` *and* arrives.

    Per path: the max speed whose run satisfies the tolerance and
    arrived. The recommendation is the *binding* (minimum across paths)
    such speed — the slowest path's limit gates the fleet. Speeds where a
    path fails the tolerance or did not arrive are excluded; if no speed
    satisfies a path, that tolerance yields ``max_speed=None``.
    """
    paths = sorted({p.path for p in points})
    rows: list[ToleranceRow] = []
    for tol in tolerances_cm:
        tol_m = tol / 100.0
        per_path_best: dict[str, float] = {}
        feasible = True
        binding_path: str | None = None
        binding_speed = float("inf")
        for path in paths:
            ok_speeds = [
                p.speed for p in points if p.path == path and p.arrived and p.cte_max <= tol_m
            ]
            if not ok_speeds:
                feasible = False
                break
            best = max(ok_speeds)
            per_path_best[path] = best
            if best < binding_speed:
                binding_speed = best
                binding_path = path
        if feasible and per_path_best:
            rows.append(
                ToleranceRow(tol_cm=tol, max_speed=binding_speed, binding_path=binding_path)
            )
        else:
            rows.append(ToleranceRow(tol_cm=tol, max_speed=None, binding_path=None))
    return rows


__all__ = [
    "SCHEMA_VERSION",
    "FeedforwardDC",
    "FopdtChannelDC",
    "Go2TuningConfig",
    "OperatingPoint",
    "OperatingPointMap",
    "PlantModelDC",
    "Provenance",
    "RecommendedControllerDC",
    "ToleranceRow",
    "VelocityProfileDC",
    "derive_config",
    "git_sha",
    "invert_tolerance",
]
