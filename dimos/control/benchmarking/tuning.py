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

"""Twist-base tuning config artifact + the DERIVE step (model -> config).

Robot-agnostic. This is the contract the two tuning tools share:

* :func:`derive_config` is the **pure** DERIVE step — a fitted FOPDT
  plant model in, a fully-populated controller config out. No file or
  robot I/O, so it is unit-tested in isolation (``test_tuning.py``).
* :class:`TuningConfig` is the versioned artifact. It owns the JSON
  (de)serialization (``to_json`` / ``from_json``) and the
  runtime-config converters the benchmark tool consumes.
* :func:`invert_tolerance` is the pure tolerance -> max-safe-speed
  inversion the benchmark tool fills section 5 with (also unit-tested).

Why these numbers (the settled characterization findings, not re-derived
here — see ``reports/tuning_README.md``): a velocity-commanded base is
FOPDT per axis; at a given speed the tracking error is the plant floor
``(tau + L) * v``; reactive controllers have ~zero headroom over that
floor; the dominant lever is speed vs path curvature; the simple
production baseline P-controller is the recommended controller.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass, field
import json
from pathlib import Path
import subprocess
from typing import TYPE_CHECKING, Any

from dimos.control.benchmarking.plant import TwistBasePlantParams
from dimos.control.benchmarking.velocity_profile import (
    GO2_VX_MAX,
    GO2_WZ_MAX,
    VelocityProfileConfig,
)
from dimos.control.tasks.feedforward_gain_compensator import FeedforwardGainConfig

if TYPE_CHECKING:
    from dimos.control.benchmarking.plant import FopdtChannelParams

SCHEMA_VERSION = 1
# SCHEMA_VERSION       = breaking field/type change.
# METHODOLOGY_VERSION  = how data was collected (additive).
#
# Methodology log (append; don't edit prior):
#   v1 — sparse sweep (3 amps), one FOPDT/channel.
#   v2 — dense sweep + floor/ceiling probes.
METHODOLOGY_VERSION = 2

# DERIVE tunable constants (documented; single source of truth)

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
    "feedforward already apply. See reports/tuning_README.md and the "
    "characterization findings (this evidence string cites the Go2 "
    "result; a different robot's headroom is TBD until characterized)."
)


# Artifact schema


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
    methodology_version: int = METHODOLOGY_VERSION


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
    params: dict[str, Any] = field(default_factory=lambda: {"k_angular": 0.5})
    evidence: str = RECOMMENDED_CONTROLLER_EVIDENCE


@dataclass
class OperatingPoint:
    path: str
    speed: float
    cte_max: float
    cte_rms: float
    arrived: bool
    # Pose-tracking accuracy vs the path's commanded yaw (rad). Defaulted so
    # pre-existing serialized maps load unchanged.
    heading_err_rms: float = 0.0
    heading_err_max: float = 0.0


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


# methodology v2: floor/ceiling envelope + per-amplitude tables


@dataclass
class ChannelEnvelopeDC:
    """Measured floor + ceiling for one velocity channel (serialized form)."""

    floor: float
    ceiling: float
    floor_not_found: bool = False
    ceiling_not_found: bool = False
    saturating_at_amp: float | None = None


@dataclass
class VelocityEnvelopeDC:
    """Section 5: per-channel velocity envelope. m/s for vx/vy, rad/s for wz."""

    vx: ChannelEnvelopeDC
    vy: ChannelEnvelopeDC
    wz: ChannelEnvelopeDC


@dataclass
class AmplitudeFitDC:
    """One FOPDT fit at a specific amplitude (sweep or ceiling probe)."""

    amp: float
    K: float
    tau: float
    L: float
    r2: float
    source: str = "sweep"  # "sweep" | "ceiling_probe"


@dataclass
class FloorProbeResultDC:
    """One floor-probe row (D3 AND-test pass/fail at one amplitude)."""

    amp: float
    motion_detected: bool
    sustained_samples: int
    net_displacement: float = 0.0  # signed body-frame displacement in cmd dir


@dataclass
class DynamicsByAmplitude:
    """Section 7: full per-amplitude K/τ/L table across regular sweep +
    ceiling probe (forensics + future lookup-based RG)."""

    vx: list[AmplitudeFitDC] = field(default_factory=list)
    vy: list[AmplitudeFitDC] = field(default_factory=list)
    wz: list[AmplitudeFitDC] = field(default_factory=list)


@dataclass
class FloorProbeResults:
    """Sibling forensic record of the floor-probe AND-test outcomes per
    amplitude (not FOPDT-fit — only pass/fail)."""

    vx: list[FloorProbeResultDC] = field(default_factory=list)
    vy: list[FloorProbeResultDC] = field(default_factory=list)
    wz: list[FloorProbeResultDC] = field(default_factory=list)


@dataclass
class TuningConfig:
    provenance: Provenance
    plant: PlantModelDC
    feedforward: FeedforwardDC
    velocity_profile: VelocityProfileDC
    recommended_controller: RecommendedControllerDC
    caveats: list[str] = field(default_factory=list)
    operating_point_map: OperatingPointMap | None = None
    velocity_envelope: VelocityEnvelopeDC | None = None
    dynamics_by_amplitude: DynamicsByAmplitude | None = None
    floor_probe_results: FloorProbeResults | None = None
    # False = a sim/self-test plumbing check, NOT measured on the robot.
    # Operators must never tune from an artifact with this False.
    valid_for_tuning: bool = True
    schema_version: int = SCHEMA_VERSION

    # serialization

    def to_json(self, path: str | Path) -> Path:
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(asdict(self), indent=2, sort_keys=False))
        return path

    @classmethod
    def from_json(cls, path: str | Path) -> TuningConfig:
        data = json.loads(Path(path).read_text())
        return cls.from_dict(data)

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> TuningConfig:
        sv = data.get("schema_version")
        if sv != SCHEMA_VERSION:
            raise ValueError(
                f"unsupported go2 tuning artifact schema_version={sv!r} "
                f"(this build understands {SCHEMA_VERSION})"
            )

        def _chan(d: dict[str, Any]) -> FopdtChannelDC:
            return FopdtChannelDC(K=d["K"], tau=d["tau"], L=d["L"])

        opm = None
        if data.get("operating_point_map") is not None:
            m = data["operating_point_map"]
            opm = OperatingPointMap(
                speeds=list(m["speeds"]),
                points=[OperatingPoint(**p) for p in m["points"]],
                tolerance_inversion=[ToleranceRow(**t) for t in m["tolerance_inversion"]],
            )

        env = None
        if data.get("velocity_envelope") is not None:
            e = data["velocity_envelope"]
            env = VelocityEnvelopeDC(
                vx=ChannelEnvelopeDC(**e["vx"]),
                vy=ChannelEnvelopeDC(**e["vy"]),
                wz=ChannelEnvelopeDC(**e["wz"]),
            )

        dba = None
        if data.get("dynamics_by_amplitude") is not None:
            d = data["dynamics_by_amplitude"]
            dba = DynamicsByAmplitude(
                vx=[AmplitudeFitDC(**a) for a in d.get("vx", [])],
                vy=[AmplitudeFitDC(**a) for a in d.get("vy", [])],
                wz=[AmplitudeFitDC(**a) for a in d.get("wz", [])],
            )

        fpr = None
        if data.get("floor_probe_results") is not None:
            f = data["floor_probe_results"]
            fpr = FloorProbeResults(
                vx=[FloorProbeResultDC(**r) for r in f.get("vx", [])],
                vy=[FloorProbeResultDC(**r) for r in f.get("vy", [])],
                wz=[FloorProbeResultDC(**r) for r in f.get("wz", [])],
            )

        # Tolerate v1 provenance (no methodology_version field).
        prov_data = dict(data["provenance"])
        prov_data.setdefault("methodology_version", 1)
        return cls(
            provenance=Provenance(**prov_data),
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
            velocity_envelope=env,
            dynamics_by_amplitude=dba,
            floor_probe_results=fpr,
            valid_for_tuning=bool(data.get("valid_for_tuning", True)),
            schema_version=sv,
        )


# helpers


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


def _output_ceiling(fits: list[dict[str, Any] | AmplitudeFitDC], cap: float) -> tuple[float, bool]:
    """Operational ceiling for one channel: ``min(max(|K·amp|), cap)``.

    Falls back to ``cap`` and ``not_found=True`` when no fits are given."""
    vals: list[float] = []
    for f in fits:
        K = getattr(f, "K", None) if not isinstance(f, dict) else f.get("K")
        amp = (
            getattr(f, "amp", None)
            if not isinstance(f, dict)
            else (f.get("amp", f.get("amplitude")))
        )
        if K is None or amp is None:
            continue
        try:
            vals.append(abs(float(K) * float(amp)))
        except (TypeError, ValueError):
            continue
    if not vals:
        return cap, True
    # `not_found` is reserved for "no data". Clamping to the platform
    # cap is silent — it just means the robot can output more than the
    # profile says is safe; that's not a failure of the measurement.
    return min(max(vals), cap), False


def _saturating_at_amp(
    fits: list[dict[str, Any] | AmplitudeFitDC], K_linear: float, sag_threshold: float
) -> float | None:
    """Forensic-only: the lowest amp where |K| drops below
    ``(1 - sag) · |K_linear|``. ``None`` if no fit saturates."""
    if not fits or K_linear == 0.0:
        return None
    threshold = (1.0 - sag_threshold) * abs(K_linear)
    saturating: list[float] = []
    for f in fits:
        K = getattr(f, "K", None) if not isinstance(f, dict) else f.get("K")
        amp = (
            getattr(f, "amp", None)
            if not isinstance(f, dict)
            else (f.get("amp", f.get("amplitude")))
        )
        if K is None or amp is None:
            continue
        try:
            if abs(float(K)) < threshold:
                saturating.append(float(amp))
        except (TypeError, ValueError):
            continue
    return min(saturating) if saturating else None


def _floor_from_probe(probe_rows: list[Any], fallback_amps: list[float]) -> tuple[float, bool]:
    """Floor = lowest amp where D3 ``motion_detected`` is true. Falls back
    to max probed amp when nothing passes."""
    passing: list[float] = []
    for r in probe_rows:
        amp = getattr(r, "amp", None) if not isinstance(r, dict) else r.get("amp")
        det = (
            getattr(r, "motion_detected", None)
            if not isinstance(r, dict)
            else (r.get("motion_detected"))
        )
        if amp is None or not det:
            continue
        try:
            passing.append(float(amp))
        except (TypeError, ValueError):
            continue
    if passing:
        return min(passing), False
    return (max(fallback_amps) if fallback_amps else 0.0), True


def compute_envelope(
    floor_probe_results: FloorProbeResults | None,
    dynamics_by_amplitude: DynamicsByAmplitude | None,
    *,
    vx_cap: float,
    wz_cap: float,
    floor_probe_amplitudes: dict[str, list[float]] | None = None,
    K_linear: dict[str, float] | None = None,
    sag_threshold: float = 0.15,
) -> VelocityEnvelopeDC:
    """Pure reducer: per-channel floor + ceiling from the densification
    data. Used by both the live characterization run and the post-hoc
    ``re-derive`` mode (where the user re-applies the current logic to an
    existing artifact's stored sweep without re-running on hardware).

    Floor = lowest amp where ``motion_detected`` is true in the floor
    probe; falls back to max probe amp with ``floor_not_found=True``.

    Ceiling = ``min(max(|K(amp)·amp|), {vx,wz}_cap)`` across the FULL
    sweep + ceiling-probe table. Robust to noisy K because the OUTPUT
    magnitude is what matters for RG (max achievable v_actual).
    ``ceiling_not_found=True`` only when no fits are available."""
    caps = {"vx": vx_cap, "vy": vx_cap, "wz": wz_cap}
    fpa = floor_probe_amplitudes or {}
    Kl = K_linear or {}
    out: dict[str, ChannelEnvelopeDC] = {}
    for ch in ("vx", "vy", "wz"):
        probe_rows = getattr(floor_probe_results, ch, []) if floor_probe_results is not None else []
        floor, floor_nf = _floor_from_probe(list(probe_rows), fpa.get(ch, []))
        fits = getattr(dynamics_by_amplitude, ch, []) if dynamics_by_amplitude is not None else []
        ceiling, ceiling_nf = _output_ceiling(list(fits), caps[ch])
        sat = _saturating_at_amp(list(fits), Kl.get(ch, 0.0), sag_threshold) if Kl else None
        out[ch] = ChannelEnvelopeDC(
            floor=floor,
            ceiling=ceiling,
            floor_not_found=floor_nf,
            ceiling_not_found=ceiling_nf,
            saturating_at_amp=sat,
        )
    return VelocityEnvelopeDC(vx=out["vx"], vy=out["vy"], wz=out["wz"])


def _channel_ceiling(per_amplitude: dict[str, Any] | None, channel: str, fallback: float) -> float:
    """Measured steady-state magnitude ceiling for a channel:
    ``max(K_at_amp * |amplitude|)`` over the swept amplitudes. Falls back
    to the robot's saturation envelope when per-amplitude data is missing
    or too sparse to be trustworthy."""
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


# DERIVE: pure model -> config


def derive_config(
    plant: TwistBasePlantParams,
    provenance: Provenance,
    *,
    per_amplitude: dict[str, Any] | None = None,
    vx_max: float = GO2_VX_MAX,
    wz_max: float = GO2_WZ_MAX,
    velocity_envelope: VelocityEnvelopeDC | None = None,
    dynamics_by_amplitude: DynamicsByAmplitude | None = None,
    floor_probe_results: FloorProbeResults | None = None,
    min_speed_floor: float = 0.0,
) -> TuningConfig:
    """Derive the full controller config from a fitted FOPDT plant model.

    Pure: model + provenance in, :class:`TuningConfig` out. No I/O.

    - Feedforward gain per axis = ``1 / K`` (the compensator divides the
      controller command by the plant gain so commanded == achieved).
      ``plant`` is the **canonical** (linear-regime, methodology v2) fit.
    - ``max_linear_speed`` / ``max_angular_speed`` = the measured ceilings
      from ``velocity_envelope`` when present (clamped to ``vx_max``/
      ``wz_max``); otherwise fall back to the deprecated per-amplitude
      ``K*amp`` estimator, then the saturation envelope.
    - ``min_speed`` (RG floor) = measured ``velocity_envelope.vx.floor``
      when present, otherwise the legacy default (``VelocityProfileDC``
      class default 0.05). A profile-supplied ``min_speed_floor > 0``
      hard-clamps the result from below.
    - ``max_centripetal_accel`` = the lateral comfort constant.
    - ``max_linear_accel`` ~= ``vx_ceiling / tau_vx`` (first-order rise);
      decel = ``DECEL_ACCEL_RATIO x`` that.
    - recommended controller = baseline, hardcoded, with cited evidence.
    """
    caveats: list[str] = []

    # Ceilings. Prefer the measured envelope (methodology v2). Fall back
    # to per-amplitude K*amp (legacy) then to the saturation envelope.
    if velocity_envelope is not None:
        vx_ceiling = min(velocity_envelope.vx.ceiling, vx_max)
        wz_ceiling = min(velocity_envelope.wz.ceiling, wz_max)
        if velocity_envelope.vx.ceiling_not_found:
            caveats.append(
                "vx ceiling probe did not saturate within the safe sweep; "
                "DERIVE used the highest probed amplitude. True ceiling "
                "may be higher — re-probe with a wider range if needed."
            )
            vx_ceiling = vx_max
        if velocity_envelope.wz.ceiling_not_found:
            caveats.append(
                "wz ceiling probe did not saturate within the safe sweep; "
                "DERIVE used the wz_max envelope as a fallback."
            )
            wz_ceiling = wz_max
    else:
        vx_ceiling = min(_channel_ceiling(per_amplitude, "vx", vx_max), vx_max)
        wz_ceiling = min(_channel_ceiling(per_amplitude, "wz", wz_max), wz_max)

    # RG min_speed: prefer measured floor, then profile hard-clamp.
    legacy_min_speed = VelocityProfileDC.min_speed
    if velocity_envelope is not None and not velocity_envelope.vx.floor_not_found:
        min_speed = max(velocity_envelope.vx.floor, min_speed_floor)
    else:
        if velocity_envelope is not None and velocity_envelope.vx.floor_not_found:
            caveats.append(
                "vx floor probe did not detect motion within the probe "
                "ladder; DERIVE fell back to the legacy min_speed default "
                f"({legacy_min_speed:g} m/s)."
            )
        min_speed = max(legacy_min_speed, min_speed_floor)

    feedforward = FeedforwardDC(
        K_vx=_safe_inv_gain(plant.vx.K),
        K_vy=_safe_inv_gain(plant.vy.K),
        K_wz=_safe_inv_gain(plant.wz.K),
    )

    max_linear_accel = vx_ceiling / plant.vx.tau if plant.vx.tau > 1e-6 else vx_max
    velocity_profile = VelocityProfileDC(
        max_linear_speed=vx_ceiling,
        max_angular_speed=wz_ceiling * (1.0 - WZ_HEADROOM_MARGIN),
        max_centripetal_accel=A_LAT_MAX,
        max_linear_accel=max_linear_accel,
        max_linear_decel=max_linear_accel * DECEL_ACCEL_RATIO,
        min_speed=min_speed,
    )

    caveats.extend(
        [
            f"Valid only for surface={provenance.surface!r}, "
            f"mode={provenance.mode!r}, {provenance.sim_or_hw}. Re-run "
            f"characterization on any surface or gait-mode change.",
            f"Plant fitted from {provenance.characterization_session_dir or 'n/a'} "
            f"on {provenance.date} (git {provenance.git_sha}).",
        ]
    )
    valid_for_tuning = provenance.sim_or_hw == "hw"
    if not valid_for_tuning:
        caveats.insert(
            0,
            "*** PIPELINE CHECK ONLY — NOT ROBOT-VALID — DO NOT TUNE FROM "
            "THIS *** Derived from the in-process FOPDT sim plant "
            "(self-test): it only proves the measure->fit->derive plumbing "
            "runs and re-recovers its own injected model. Re-run "
            "`characterization --mode hw` on the real robot for a "
            "tuning-valid artifact.",
        )

    return TuningConfig(
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
        velocity_envelope=velocity_envelope,
        dynamics_by_amplitude=dynamics_by_amplitude,
        floor_probe_results=floor_probe_results,
        valid_for_tuning=valid_for_tuning,
    )


def re_derive_config(
    artifact: TuningConfig,
    *,
    vx_max: float = GO2_VX_MAX,
    wz_max: float = GO2_WZ_MAX,
    floor_probe_amplitudes: dict[str, list[float]] | None = None,
    min_speed_floor: float = 0.0,
    sag_threshold: float = 0.15,
) -> TuningConfig:
    """Post-hoc apply the current envelope + DERIVE logic to an existing
    artifact. Uses the stored ``dynamics_by_amplitude`` +
    ``floor_probe_results`` — no re-run on hardware needed.

    Useful after a methodology bugfix (the K-sag ceiling was too
    conservative on noisy fits; switched to ``max(K·amp)`` for
    operational use): pass the artifact through here and you get a
    corrected JSON without re-collecting data.

    Plant, FF (the canonical FOPDT) and provenance are passed through
    unchanged — this only recomputes envelope + velocity_profile +
    caveats."""
    K_linear = {
        "vx": artifact.plant.vx.K,
        "vy": artifact.plant.vy.K,
        "wz": artifact.plant.wz.K,
    }
    env = compute_envelope(
        artifact.floor_probe_results,
        artifact.dynamics_by_amplitude,
        vx_cap=vx_max,
        wz_cap=wz_max,
        floor_probe_amplitudes=floor_probe_amplitudes,
        K_linear=K_linear,
        sag_threshold=sag_threshold,
    )
    plant = TwistBasePlantParams(
        vx=FopdtChannelParamsLike(artifact.plant.vx),
        vy=FopdtChannelParamsLike(artifact.plant.vy),
        wz=FopdtChannelParamsLike(artifact.plant.wz),
    )
    return derive_config(
        plant,
        artifact.provenance,
        vx_max=vx_max,
        wz_max=wz_max,
        velocity_envelope=env,
        dynamics_by_amplitude=artifact.dynamics_by_amplitude,
        floor_probe_results=artifact.floor_probe_results,
        min_speed_floor=min_speed_floor,
    )


def FopdtChannelParamsLike(dc: FopdtChannelDC) -> FopdtChannelParams:
    """Lightweight adapter: derive_config wants a TwistBasePlantParams
    (made of FopdtChannelParams), but the artifact stores them as
    FopdtChannelDC. Return a duck-typed object with .K, .tau, .L."""
    from dimos.control.benchmarking.plant import FopdtChannelParams

    return FopdtChannelParams(K=dc.K, tau=dc.tau, L=dc.L)


# tolerance -> max-safe-speed inversion (pure)


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
