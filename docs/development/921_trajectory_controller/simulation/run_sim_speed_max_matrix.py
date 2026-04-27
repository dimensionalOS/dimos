#!/usr/bin/env python3
"""Run the speed-max trajectory simulation matrix and write a report.

This matrix tests whether the issue 921 controller stack is suitable for driving
the robot at the running-speed target (default 3.7 m/s). It uses a separate
geometry from SIM-04 so the path speed profile is the speed limiter, not the
arbitrary tight curvature of the original tight-circle/right-angle matrix.

It adds:

- Long straight line (40 m) and a stop-distance straight (30 m) where the path
  speed profile can reach the running-speed target without a geometry cap.
- A wide arc (R = 25 m, 90 degree sweep) where the geometry cap depends on
  ``local_planner_max_normal_accel_m_s2`` and target speed.
- A wide S-curve (40 m length, 2.0 m amplitude, R_min ~ 20 m) that adds gentle
  lateral motion at running speed.
- A speed-max right-angle turn (20 m straight legs with a 1.5 m fillet radius)
  to confirm the path-speed profile slows the reference at a sharp corner
  using the local fillet curvature, and the controller follows. The original
  4 m ``right_angle_turn`` is *not* used here because for legs >= 12 m the
  path-speed profile's circumscribed-circle curvature cap at a bare polyline
  vertex exceeds the running target and provides no slowdown.

It enables synthetic plants to actually reach running speed by passing
``--response-saturation-scale`` (synthetic presets cap at 1.4-2.0 m/s by
default). The wrapper is recorded in ``config.yaml`` so the run is auditable
as a high-speed envelope rather than a hidden tuning change.

It computes additional speed-max metrics from each ``ticks.jsonl``:
``achieved_peak_planar_speed_m_s``, ``achieved_p95_planar_speed_m_s``,
``achieved_mean_planar_speed_m_s``, ``max_observed_lateral_accel_m_s2``,
``max_observed_yaw_rate_rad_s``, and ``max_observed_command_jerk_m_s3``.

It applies speed-scaled and hardware safety gates on top of the runner's
existing pass/fail verdict:

- ``max_planar_position_divergence_m <= 0.30 + 0.15 * target_speed``.
- ``max_observed_lateral_accel_m_s2 <= 1.5`` (hardware bound).
- ``max_observed_command_jerk_m_s3 <= 30.0`` (smoothness bound).
- On geometry-feasible scenarios, ``achieved_peak_planar_speed_m_s`` is
  expected to be at least 85% of the geometry-bounded expectation.
- On ``speed_max_right_angle_turn``, the measured planar speed inside a 2 m
  window around the fillet midpoint must drop to within 0.4 m/s of the
  geometry-bounded corner cap ``sqrt(normal_accel * fillet_radius)``.
"""

from __future__ import annotations

from collections import Counter, defaultdict
import argparse
from collections.abc import Iterable
from dataclasses import dataclass
from datetime import UTC, datetime
import json
import math
from pathlib import Path
import subprocess
import sys
from typing import Any


SPEEDS_M_S: tuple[float, ...] = (2.0, 2.5, 3.0, 3.5, 3.7)
RATES_HZ: tuple[float, ...] = (10.0, 20.0)
PLANT_PRESETS: tuple[str, ...] = (
    "ideal",
    "synthetic_nominal",
    "synthetic_sluggish",
    "synthetic_asymmetric",
    "synthetic_noisy",
)
SCENARIOS: tuple[str, ...] = (
    "speed_max_line",
    "speed_max_arc",
    "speed_max_s_curve",
    "speed_max_stop_distance",
    "speed_max_right_angle_turn",
)
SEED = 7

# Path-speed-profile caps applied to every run. The running envelope is set so
# the wide scenarios can reach the running-speed target while respecting the
# realistic plant acceleration ceilings (synthetic_sluggish max_accel ~= 0.75
# m/s^2). Goal deceleration matches tangent acceleration to keep stop distance
# symmetric and predictable.
DEFAULT_TANGENT_ACCEL_M_S2 = 0.7
DEFAULT_NORMAL_ACCEL_M_S2 = 1.0
DEFAULT_GOAL_DECEL_M_S2 = 0.7

# Synthetic presets cap at 1.4-2.0 m/s saturation by default. To physically
# reach the running-speed target we widen saturation by this scale and record
# it in config/summary as a documented high-speed envelope wrapper.
DEFAULT_RESPONSE_SATURATION_SCALE = 3.0

# Default controller command limits at running speed. The runner's defaults
# (max_yaw_rate=1.5 rad/s, max_planar_accel=3.0 m/s^2) are kept; we only widen
# the planar speed cap to match the target speed automatically inside the
# runner via --max-linear-speed when needed.
DEFAULT_MAX_YAW_RATE_RAD_S = 1.5

# Speed-scaled and hardware safety gate constants.
DIVERGENCE_BASE_M = 0.30
DIVERGENCE_PER_M_S = 0.15
HARDWARE_MAX_LATERAL_ACCEL_M_S2 = 1.5
HARDWARE_MAX_COMMAND_JERK_M_S3 = 30.0
ACHIEVED_PEAK_RATIO_MIN = 0.85
# The corner gate accepts the measured speed in the corner window if it falls
# within this absolute margin of the geometry-bounded corner cap
# sqrt(normal_accel * fillet_radius). A 0.4 m/s margin tolerates plant lag
# and yaw smoothing while still rejecting "did not slow down at all" cases.
RIGHT_ANGLE_CORNER_TOLERANCE_M_S = 0.4

# Geometry constants used to determine expected per-scenario peak speed.
SPEED_MAX_ARC_RADIUS_M = 25.0
SPEED_MAX_S_CURVE_LENGTH_M = 40.0
SPEED_MAX_S_CURVE_AMPLITUDE_M = 2.0
# R_min for y = A*sin(2 pi x / L) is L^2 / ((2 pi)^2 A).
SPEED_MAX_S_CURVE_MIN_RADIUS_M = (SPEED_MAX_S_CURVE_LENGTH_M / (2.0 * math.pi)) ** 2 / SPEED_MAX_S_CURVE_AMPLITUDE_M
SPEED_MAX_RIGHT_ANGLE_FILLET_RADIUS_M = 1.5
SPEED_MAX_RIGHT_ANGLE_STRAIGHT_LEG_M = 20.0
SPEED_MAX_RIGHT_ANGLE_CORNER_X_M = SPEED_MAX_RIGHT_ANGLE_STRAIGHT_LEG_M + SPEED_MAX_RIGHT_ANGLE_FILLET_RADIUS_M / math.sqrt(2.0)
SPEED_MAX_RIGHT_ANGLE_CORNER_Y_M = SPEED_MAX_RIGHT_ANGLE_FILLET_RADIUS_M - SPEED_MAX_RIGHT_ANGLE_FILLET_RADIUS_M / math.sqrt(2.0)
SPEED_MAX_RIGHT_ANGLE_CORNER_WINDOW_M = 2.0


def _repo_root() -> Path:
    """DimOS repo root (directory that contains ``pyproject.toml`` and ``dimos/`` package)."""
    p = Path(__file__).resolve()
    for parent in p.parents:
        if (parent / "pyproject.toml").is_file() and (parent / "dimos").is_dir():
            return parent
    raise RuntimeError(f"DimOS repository root not found above {p}")


def _simulation_dir() -> Path:
    return Path(__file__).resolve().parent


def _token(value: float) -> str:
    return f"{value:g}".replace(".", "p")


def _run_dir(
    root: Path,
    plant: str,
    scenario: str,
    speed: float,
    rate: float,
) -> Path:
    return root / f"{plant}__{scenario}__{_token(speed)}mps__{_token(rate)}hz"


def _parse_float_list(text: str, *, name: str) -> tuple[float, ...]:
    values = tuple(float(part.strip()) for part in text.split(",") if part.strip())
    if not values:
        raise argparse.ArgumentTypeError(f"{name} must contain at least one value")
    return values


def _parse_choice_list(text: str, *, name: str, choices: tuple[str, ...]) -> tuple[str, ...]:
    values = tuple(part.strip() for part in text.split(",") if part.strip())
    if not values:
        raise argparse.ArgumentTypeError(f"{name} must contain at least one value")
    unknown = sorted(set(values) - set(choices))
    if unknown:
        raise argparse.ArgumentTypeError(f"{name} contains unknown values: {', '.join(unknown)}")
    return values


def _ordered_present(
    candidates: tuple[str, ...] | tuple[float, ...],
    present: set[str] | set[float],
) -> list[Any]:
    return [item for item in candidates if item in present]


def _load_summary(path: Path) -> dict[str, Any] | None:
    if not path.is_file():
        return None
    return json.loads(path.read_text(encoding="utf-8"))


def _expected_peak_speed_m_s(scenario: str, target: float, normal_accel_m_s2: float) -> float:
    if scenario == "speed_max_arc":
        cap = math.sqrt(normal_accel_m_s2 * SPEED_MAX_ARC_RADIUS_M)
        return min(target, cap)
    if scenario == "speed_max_s_curve":
        cap = math.sqrt(normal_accel_m_s2 * SPEED_MAX_S_CURVE_MIN_RADIUS_M)
        return min(target, cap)
    if scenario in {"speed_max_line", "speed_max_stop_distance"}:
        return target
    if scenario == "speed_max_right_angle_turn":
        return target  # leg is sized to reach target; corner slows it via fillet
    if scenario == "right_angle_turn":
        return target  # legacy 4 m polyline; not used for a peak gate
    return target


@dataclass(frozen=True)
class SpeedMaxMetrics:
    achieved_peak_planar_speed_m_s: float
    achieved_p95_planar_speed_m_s: float
    achieved_mean_planar_speed_m_s: float
    max_observed_lateral_accel_m_s2: float
    max_observed_yaw_rate_rad_s: float
    max_observed_command_jerk_m_s3: float
    min_speed_during_corner_window_m_s: float | None
    sample_count: int

    def asdict(self) -> dict[str, Any]:
        return {
            "achieved_peak_planar_speed_m_s": self.achieved_peak_planar_speed_m_s,
            "achieved_p95_planar_speed_m_s": self.achieved_p95_planar_speed_m_s,
            "achieved_mean_planar_speed_m_s": self.achieved_mean_planar_speed_m_s,
            "max_observed_lateral_accel_m_s2": self.max_observed_lateral_accel_m_s2,
            "max_observed_yaw_rate_rad_s": self.max_observed_yaw_rate_rad_s,
            "max_observed_command_jerk_m_s3": self.max_observed_command_jerk_m_s3,
            "min_speed_during_corner_window_m_s": self.min_speed_during_corner_window_m_s,
            "sample_count": self.sample_count,
        }


def _percentile(values: list[float], percentile: float) -> float:
    if not values:
        return 0.0
    ordered = sorted(values)
    pos = (len(ordered) - 1) * percentile
    lo = math.floor(pos)
    hi = math.ceil(pos)
    if lo == hi:
        return ordered[int(pos)]
    frac = pos - lo
    return ordered[lo] * (1.0 - frac) + ordered[hi] * frac


def _compute_speed_max_metrics(ticks_path: Path, scenario: str) -> SpeedMaxMetrics | None:
    if not ticks_path.is_file():
        return None
    measured_planar_speed: list[float] = []
    measured_yaw_rate: list[float] = []
    commanded_planar_speed: list[float] = []
    sim_time: list[float] = []
    measured_x: list[float] = []
    measured_y: list[float] = []
    with ticks_path.open("r", encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            row = json.loads(line)
            vx = float(row.get("meas_twist_linear_x_m_s", 0.0))
            vy = float(row.get("meas_twist_linear_y_m_s", 0.0))
            measured_planar_speed.append(math.hypot(vx, vy))
            measured_yaw_rate.append(abs(float(row.get("meas_twist_angular_z_rad_s", 0.0))))
            commanded_planar_speed.append(float(row.get("commanded_planar_speed_m_s", 0.0)))
            sim_time.append(float(row.get("sim_time_s") or row.get("meas_time_s") or 0.0))
            measured_x.append(float(row.get("meas_x_m", 0.0)))
            measured_y.append(float(row.get("meas_y_m", 0.0)))
    if not measured_planar_speed:
        return None

    achieved_peak = max(measured_planar_speed)
    achieved_p95 = _percentile(measured_planar_speed, 0.95)
    achieved_mean = sum(measured_planar_speed) / len(measured_planar_speed)
    lateral_accel: list[float] = [
        speed * yaw_rate
        for speed, yaw_rate in zip(measured_planar_speed, measured_yaw_rate, strict=True)
    ]
    max_lat = max(lateral_accel) if lateral_accel else 0.0
    max_yaw = max(measured_yaw_rate) if measured_yaw_rate else 0.0
    jerk_values: list[float] = []
    for i in range(1, len(commanded_planar_speed)):
        dt = sim_time[i] - sim_time[i - 1]
        if dt > 1e-9:
            jerk_values.append(abs(commanded_planar_speed[i] - commanded_planar_speed[i - 1]) / dt)
    max_jerk = max(jerk_values) if jerk_values else 0.0

    min_during_corner: float | None = None
    if scenario == "speed_max_right_angle_turn":
        # The fillet midpoint is the geometric corner of the filleted polyline.
        # A 2 m window is wide enough to capture the slowdown plateau while
        # excluding the surrounding straight-leg cruise.
        corner_speeds = [
            speed
            for x, y, speed in zip(measured_x, measured_y, measured_planar_speed, strict=True)
            if math.hypot(x - SPEED_MAX_RIGHT_ANGLE_CORNER_X_M, y - SPEED_MAX_RIGHT_ANGLE_CORNER_Y_M)
            <= SPEED_MAX_RIGHT_ANGLE_CORNER_WINDOW_M
        ]
        min_during_corner = min(corner_speeds) if corner_speeds else None
    elif scenario == "right_angle_turn":
        # Legacy 4 m polyline corner at (4, 0); kept for SIM-04 compatibility
        # if the matrix ever runs with --scenarios including it.
        corner_speeds = [
            speed
            for x, y, speed in zip(measured_x, measured_y, measured_planar_speed, strict=True)
            if math.hypot(x - 4.0, y - 0.0) <= 1.0
        ]
        min_during_corner = min(corner_speeds) if corner_speeds else None

    return SpeedMaxMetrics(
        achieved_peak_planar_speed_m_s=achieved_peak,
        achieved_p95_planar_speed_m_s=achieved_p95,
        achieved_mean_planar_speed_m_s=achieved_mean,
        max_observed_lateral_accel_m_s2=max_lat,
        max_observed_yaw_rate_rad_s=max_yaw,
        max_observed_command_jerk_m_s3=max_jerk,
        min_speed_during_corner_window_m_s=min_during_corner,
        sample_count=len(measured_planar_speed),
    )


@dataclass(frozen=True)
class SpeedMaxGate:
    name: str
    threshold: float | None
    actual: float | None
    passed: bool


def _evaluate_speed_max_gates(
    *,
    scenario: str,
    target_speed: float,
    normal_accel_m_s2: float,
    metrics: SpeedMaxMetrics,
) -> list[SpeedMaxGate]:
    gates: list[SpeedMaxGate] = []
    expected_peak = _expected_peak_speed_m_s(scenario, target_speed, normal_accel_m_s2)

    gates.append(
        SpeedMaxGate(
            name="hardware_max_lateral_accel_m_s2",
            threshold=HARDWARE_MAX_LATERAL_ACCEL_M_S2,
            actual=metrics.max_observed_lateral_accel_m_s2,
            passed=metrics.max_observed_lateral_accel_m_s2 <= HARDWARE_MAX_LATERAL_ACCEL_M_S2,
        )
    )
    gates.append(
        SpeedMaxGate(
            name="hardware_max_command_jerk_m_s3",
            threshold=HARDWARE_MAX_COMMAND_JERK_M_S3,
            actual=metrics.max_observed_command_jerk_m_s3,
            passed=metrics.max_observed_command_jerk_m_s3 <= HARDWARE_MAX_COMMAND_JERK_M_S3,
        )
    )

    if scenario in {"speed_max_line", "speed_max_arc", "speed_max_s_curve", "speed_max_stop_distance"}:
        threshold = ACHIEVED_PEAK_RATIO_MIN * expected_peak
        gates.append(
            SpeedMaxGate(
                name="achieved_peak_speed_m_s_min",
                threshold=threshold,
                actual=metrics.achieved_peak_planar_speed_m_s,
                passed=metrics.achieved_peak_planar_speed_m_s >= threshold,
            )
        )

    if scenario == "speed_max_right_angle_turn":
        corner_cap = math.sqrt(
            normal_accel_m_s2 * SPEED_MAX_RIGHT_ANGLE_FILLET_RADIUS_M
        )
        threshold = corner_cap + RIGHT_ANGLE_CORNER_TOLERANCE_M_S
        actual = metrics.min_speed_during_corner_window_m_s
        passed = actual is not None and actual <= threshold
        gates.append(
            SpeedMaxGate(
                name="speed_max_right_angle_must_slow_down_m_s",
                threshold=threshold,
                actual=actual,
                passed=passed,
            )
        )
    elif scenario == "right_angle_turn":
        # Legacy gate retained for compatibility if invoked with the SIM-04
        # 4 m polyline corner.
        threshold = 0.5
        actual = metrics.min_speed_during_corner_window_m_s
        passed = actual is not None and actual <= threshold
        gates.append(
            SpeedMaxGate(
                name="right_angle_must_slow_down_m_s",
                threshold=threshold,
                actual=actual,
                passed=passed,
            )
        )

    return gates


def _classify(
    summary: dict[str, Any] | None,
    speed_max_metrics: SpeedMaxMetrics | None,
    speed_max_gates: list[SpeedMaxGate],
) -> tuple[str, list[str]]:
    if summary is None or speed_max_metrics is None:
        return "invalid", ["missing_summary_or_ticks"]
    failed: list[str] = []
    runner_status = summary["verdict"]["status"]
    if runner_status == "fail":
        failed.extend(summary["verdict"]["failed_gates"])
    if runner_status == "invalid":
        return "invalid", summary["verdict"].get("invalid_reasons", [])
    speed_scaled_div_threshold = DIVERGENCE_BASE_M + DIVERGENCE_PER_M_S * float(
        summary["scenario"]["target_speed_m_s"]
    )
    actual_div = float(summary["metrics"]["max_planar_position_divergence_m"])
    if actual_div > speed_scaled_div_threshold:
        failed.append(f"speed_scaled_max_planar_position_divergence_m({actual_div:.3f}>{speed_scaled_div_threshold:.3f})")
    failed.extend(g.name for g in speed_max_gates if not g.passed)
    if failed:
        return "fail", failed
    # Borderline classification only considers max-type gates: speed-scaled
    # divergence, lateral acceleration, and command jerk. Min-type gates (e.g.,
    # achieved peak speed) are intentionally excluded because passing them by
    # a wide margin should not be flagged as borderline.
    borderline = False
    if speed_scaled_div_threshold > 0.0 and actual_div >= 0.8 * speed_scaled_div_threshold:
        borderline = True
    if not borderline and speed_max_metrics is not None:
        if (
            HARDWARE_MAX_LATERAL_ACCEL_M_S2 > 0.0
            and speed_max_metrics.max_observed_lateral_accel_m_s2
            >= 0.8 * HARDWARE_MAX_LATERAL_ACCEL_M_S2
        ):
            borderline = True
        elif (
            HARDWARE_MAX_COMMAND_JERK_M_S3 > 0.0
            and speed_max_metrics.max_observed_command_jerk_m_s3
            >= 0.8 * HARDWARE_MAX_COMMAND_JERK_M_S3
        ):
            borderline = True
    return ("borderline", []) if borderline else ("pass", [])


def _rel(path: Path, root: Path) -> str:
    try:
        return path.relative_to(root).as_posix()
    except ValueError:
        return path.as_posix()


def _write_json(path: Path, payload: Any) -> None:
    path.write_text(json.dumps(payload, indent=2, sort_keys=False, allow_nan=False) + "\n", encoding="utf-8")


def _format_optional(value: float | None, fmt: str) -> str:
    return "n/a" if value is None else format(value, fmt)


def _write_report(
    run_root: Path,
    results: list[dict[str, Any]],
    *,
    started_at: str,
    finished_at: str,
    args: argparse.Namespace,
) -> None:
    repo = _repo_root()
    counts = Counter(r["classification"] for r in results)
    by_speed: dict[float, Counter[str]] = defaultdict(Counter)
    by_plant: dict[str, Counter[str]] = defaultdict(Counter)
    by_scenario: dict[str, Counter[str]] = defaultdict(Counter)
    for r in results:
        by_speed[r["speed_m_s"]][r["classification"]] += 1
        by_plant[r["plant"]][r["classification"]] += 1
        by_scenario[r["scenario"]][r["classification"]] += 1

    failures = [r for r in results if r["classification"] == "fail"]
    invalids = [r for r in results if r["classification"] == "invalid"]
    borderline = [r for r in results if r["classification"] == "borderline"]

    speeds = _ordered_present(SPEEDS_M_S, {r["speed_m_s"] for r in results})
    rates = _ordered_present(RATES_HZ, {r["rate_hz"] for r in results})
    plants = _ordered_present(PLANT_PRESETS, {r["plant"] for r in results})
    scenarios = _ordered_present(SCENARIOS, {r["scenario"] for r in results})

    lines: list[str] = [
        "# Speed-Max Trajectory Simulation Matrix",
        "",
        f"Started: `{started_at}`",
        f"Finished: `{finished_at}`",
        f"Run root: `{_rel(run_root, repo)}`",
        "",
        "## Scope",
        "",
        f"- Target speeds: `{', '.join(str(s) for s in speeds)}` m/s.",
        f"- Rates: `{', '.join(str(r) for r in rates)}` Hz.",
        f"- Plant presets: `{', '.join(plants)}`.",
        f"- Scenarios: `{', '.join(scenarios)}`.",
        f"- Reference mode: `path_speed_profile`.",
        f"- Path speed caps: tangent `{args.local_planner_max_tangent_accel_m_s2:g}` m/s^2, "
        f"normal `{args.local_planner_max_normal_accel_m_s2:g}` m/s^2, "
        f"goal decel `{args.local_planner_goal_decel_m_s2:g}` m/s^2.",
        f"- Response saturation scale: `{args.response_saturation_scale:g}` "
        "(synthetic plants need widened saturation to physically reach running speed; "
        "the wrapper is recorded in each run's config.yaml).",
        f"- Total runs: `{len(results)}`.",
        "",
        "## Implementation Vs Tuning",
        "",
        "- Implementation under test: speed-max scenario geometries plus the existing path-speed-profile reference "
        "generation, command limits, and live holonomic command acceleration/yaw config.",
        "- Tuning under test: running-envelope speed caps and saturation scale.",
        "- Hardware safety bounds tested (in addition to the runner's existing pass/fail gates):",
        f"  - max observed lateral acceleration `<= {HARDWARE_MAX_LATERAL_ACCEL_M_S2:g}` m/s^2.",
        f"  - max observed command jerk `<= {HARDWARE_MAX_COMMAND_JERK_M_S3:g}` m/s^3.",
        f"  - achieved peak speed must be `>= {ACHIEVED_PEAK_RATIO_MIN * 100.0:g}%` of the geometry-bounded expected "
        "peak speed (only on the four `speed_max_*` scenarios).",
        f"  - on `speed_max_right_angle_turn`, planar speed must drop to within "
        f"`{RIGHT_ANGLE_CORNER_TOLERANCE_M_S:g}` m/s of the geometry-bounded corner cap "
        f"`sqrt(normal_accel * fillet_radius)` (with `fillet_radius = "
        f"{SPEED_MAX_RIGHT_ANGLE_FILLET_RADIUS_M:g}` m) inside a "
        f"`{SPEED_MAX_RIGHT_ANGLE_CORNER_WINDOW_M:g}` m window around the fillet midpoint "
        "(the path-speed profile must slow the robot at sharp turns).",
        f"  - speed-scaled max planar divergence: `{DIVERGENCE_BASE_M:g} + {DIVERGENCE_PER_M_S:g} * target_speed_m_s`.",
        "",
        "## Verdict Counts",
        "",
        f"- pass `{counts['pass']}`, borderline `{counts['borderline']}`, fail `{counts['fail']}`, invalid `{counts['invalid']}`.",
        "",
        "A run is borderline when the runner verdict and all hardware gates pass but any numeric gate is at or above "
        "80% of its threshold.",
        "",
        "## Counts By Speed",
        "",
        "| Speed m/s | Pass | Borderline | Fail | Invalid |",
        "| ---: | ---: | ---: | ---: | ---: |",
    ]
    for speed in speeds:
        bucket = by_speed[speed]
        lines.append(
            f"| {speed:g} | {bucket['pass']} | {bucket['borderline']} | {bucket['fail']} | {bucket['invalid']} |"
        )

    lines.extend(
        [
            "",
            "## Counts By Plant",
            "",
            "| Plant | Pass | Borderline | Fail | Invalid |",
            "| --- | ---: | ---: | ---: | ---: |",
        ]
    )
    for plant in plants:
        bucket = by_plant[plant]
        lines.append(
            f"| `{plant}` | {bucket['pass']} | {bucket['borderline']} | {bucket['fail']} | {bucket['invalid']} |"
        )

    lines.extend(
        [
            "",
            "## Counts By Scenario",
            "",
            "| Scenario | Pass | Borderline | Fail | Invalid |",
            "| --- | ---: | ---: | ---: | ---: |",
        ]
    )
    for scenario in scenarios:
        bucket = by_scenario[scenario]
        lines.append(
            f"| `{scenario}` | {bucket['pass']} | {bucket['borderline']} | {bucket['fail']} | {bucket['invalid']} |"
        )

    lines.extend(
        [
            "",
            "## Achieved Peak Speed By Plant And Scenario At The Highest Tested Target Speed",
            "",
            f"Filtered to `target_speed_m_s = {speeds[-1]:g}` m/s and `rate_hz = {rates[-1]:g}` Hz.",
            "",
            "| Plant | Scenario | Expected peak m/s | Achieved peak m/s | Achieved p95 m/s | Max planar div m | Max lateral accel m/s^2 | Verdict |",
            "| --- | --- | ---: | ---: | ---: | ---: | ---: | --- |",
        ]
    )
    top_speed = speeds[-1]
    top_rate = rates[-1]
    for plant in plants:
        for scenario in scenarios:
            match = next(
                (
                    r
                    for r in results
                    if r["plant"] == plant
                    and r["scenario"] == scenario
                    and r["speed_m_s"] == top_speed
                    and r["rate_hz"] == top_rate
                ),
                None,
            )
            if match is None or match["summary"] is None or match["speed_max_metrics"] is None:
                lines.append(f"| `{plant}` | `{scenario}` | n/a | n/a | n/a | n/a | n/a | `{match['classification'] if match else 'missing'}` |")
                continue
            metrics = match["summary"]["metrics"]
            sm = match["speed_max_metrics"]
            expected_peak = _expected_peak_speed_m_s(
                scenario, top_speed, args.local_planner_max_normal_accel_m_s2
            )
            lines.append(
                f"| `{plant}` | `{scenario}` | {expected_peak:.2f} | "
                f"{sm['achieved_peak_planar_speed_m_s']:.2f} | "
                f"{sm['achieved_p95_planar_speed_m_s']:.2f} | "
                f"{metrics['max_planar_position_divergence_m']:.3f} | "
                f"{sm['max_observed_lateral_accel_m_s2']:.3f} | "
                f"`{match['classification']}` |"
            )

    lines.extend(["", "## Fail Cases", ""])
    if failures:
        for r in failures[:60]:
            lines.append(
                f"- `{_rel(Path(r['run_dir']), repo)}` - {', '.join(r['failed_gates']) or 'n/a'}."
            )
        if len(failures) > 60:
            lines.append(f"- Plus `{len(failures) - 60}` more fail cases.")
    else:
        lines.append("- None.")

    lines.extend(["", "## Invalid Cases", ""])
    if invalids:
        for r in invalids[:60]:
            lines.append(
                f"- `{_rel(Path(r['run_dir']), repo)}` - {', '.join(r['failed_gates']) or 'invalid_run'}."
            )
        if len(invalids) > 60:
            lines.append(f"- Plus `{len(invalids) - 60}` more invalid cases.")
    else:
        lines.append("- None.")

    lines.extend(["", "## Borderline Cases", ""])
    if borderline:
        for r in borderline[:60]:
            sm = r["speed_max_metrics"] or {}
            metrics = (r["summary"] or {}).get("metrics", {})
            lines.append(
                f"- `{_rel(Path(r['run_dir']), repo)}` - "
                f"max div `{metrics.get('max_planar_position_divergence_m', float('nan')):.3f}` m, "
                f"achieved peak `{sm.get('achieved_peak_planar_speed_m_s', float('nan')):.2f}` m/s, "
                f"max lateral accel `{sm.get('max_observed_lateral_accel_m_s2', float('nan')):.3f}` m/s^2."
            )
        if len(borderline) > 60:
            lines.append(f"- Plus `{len(borderline) - 60}` more borderline cases.")
    else:
        lines.append("- None.")

    lines.extend(
        [
            "",
            "## Worst Speed-Scaled Divergence Runs",
            "",
            "| Run | Verdict | Target m/s | Speed-scaled threshold m | Max divergence m | Achieved peak m/s |",
            "| --- | --- | ---: | ---: | ---: | ---: |",
        ]
    )
    rated = [
        r for r in results if r["summary"] is not None and r["speed_max_metrics"] is not None
    ]
    rated.sort(
        key=lambda r: float(r["summary"]["metrics"]["max_planar_position_divergence_m"]),
        reverse=True,
    )
    for r in rated[:20]:
        target = float(r["summary"]["scenario"]["target_speed_m_s"])
        threshold = DIVERGENCE_BASE_M + DIVERGENCE_PER_M_S * target
        actual_div = float(r["summary"]["metrics"]["max_planar_position_divergence_m"])
        peak = float(r["speed_max_metrics"]["achieved_peak_planar_speed_m_s"])
        lines.append(
            f"| `{_rel(Path(r['run_dir']), repo)}` | `{r['classification']}` | "
            f"{target:g} | {threshold:.3f} | {actual_div:.3f} | {peak:.2f} |"
        )

    lines.extend(
        [
            "",
            "## Machine-Readable Index",
            "",
            "- `matrix_results.json` contains one record per run with command status, classification, runner summary, "
            "speed-max metrics, and the speed-max gate evaluations.",
        ]
    )

    (run_root / "matrix_summary.md").write_text("\n".join(lines) + "\n", encoding="utf-8")


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Run the speed-max trajectory simulation matrix.")
    parser.add_argument("--run-root", type=Path, default=None)
    parser.add_argument(
        "--speeds",
        type=lambda text: _parse_float_list(text, name="--speeds"),
        default=SPEEDS_M_S,
    )
    parser.add_argument(
        "--rates",
        type=lambda text: _parse_float_list(text, name="--rates"),
        default=RATES_HZ,
    )
    parser.add_argument(
        "--plants",
        type=lambda text: _parse_choice_list(text, name="--plants", choices=PLANT_PRESETS),
        default=PLANT_PRESETS,
    )
    parser.add_argument(
        "--scenarios",
        type=lambda text: _parse_choice_list(text, name="--scenarios", choices=SCENARIOS),
        default=SCENARIOS,
    )
    parser.add_argument(
        "--local-planner-max-tangent-accel-m-s2",
        type=float,
        default=DEFAULT_TANGENT_ACCEL_M_S2,
    )
    parser.add_argument(
        "--local-planner-max-normal-accel-m-s2",
        type=float,
        default=DEFAULT_NORMAL_ACCEL_M_S2,
    )
    parser.add_argument(
        "--local-planner-goal-decel-m-s2",
        type=float,
        default=DEFAULT_GOAL_DECEL_M_S2,
    )
    parser.add_argument(
        "--response-saturation-scale",
        type=float,
        default=DEFAULT_RESPONSE_SATURATION_SCALE,
    )
    parser.add_argument(
        "--max-yaw-rate",
        type=float,
        default=DEFAULT_MAX_YAW_RATE_RAD_S,
    )
    parser.add_argument(
        "--gate-max-divergence",
        type=float,
        default=2.0,
        help="Per-run pass-through gate to keep runner verdict pass while the matrix applies its own speed-scaled gate.",
    )
    parser.add_argument(
        "--no-plot",
        action="store_true",
        help="Pass --no-plot to the simulator runs (faster matrix, no PNGs).",
    )
    parser.add_argument(
        "--only-missing",
        action="store_true",
    )
    parser.add_argument(
        "--summarize-only",
        action="store_true",
    )
    return parser


def main(argv: list[str] | None = None) -> int:
    args = _build_parser().parse_args(argv)
    if args.only_missing and args.summarize_only:
        raise SystemExit("--only-missing and --summarize-only cannot be combined")

    repo = _repo_root()
    sim_dir = _simulation_dir()
    timestamp = datetime.now(UTC).strftime("%Y-%m-%dT%H%M%SZ")
    run_root = (
        args.run_root.expanduser().resolve()
        if args.run_root is not None
        else sim_dir / "simulation_runs" / f"{timestamp}_sim_speed_max_matrix"
    )
    run_root.mkdir(parents=True, exist_ok=args.only_missing or args.summarize_only)
    started_at = datetime.now(UTC).isoformat()
    results: list[dict[str, Any]] = []

    for plant in args.plants:
        for scenario in args.scenarios:
            for speed in args.speeds:
                for rate in args.rates:
                    out_dir = _run_dir(run_root, plant, scenario, speed, rate)
                    summary = _load_summary(out_dir / "summary.json")
                    completed_returncode: int | None = None
                    if not args.summarize_only and (not args.only_missing or summary is None):
                        command = [
                            sys.executable,
                            str(sim_dir / "simulate_trajectory_controller.py"),
                            "--scenario",
                            scenario,
                            "--speed",
                            str(speed),
                            "--rate",
                            str(rate),
                            "--plant-preset",
                            plant,
                            "--output-dir",
                            str(out_dir),
                            "--seed",
                            str(SEED),
                            "--reference-mode",
                            "path_speed_profile",
                            "--local-planner-max-tangent-accel-m-s2",
                            str(args.local_planner_max_tangent_accel_m_s2),
                            "--local-planner-max-normal-accel-m-s2",
                            str(args.local_planner_max_normal_accel_m_s2),
                            "--local-planner-goal-decel-m-s2",
                            str(args.local_planner_goal_decel_m_s2),
                            "--response-saturation-scale",
                            str(args.response_saturation_scale),
                            "--max-yaw-rate",
                            str(args.max_yaw_rate),
                            "--gate-max-divergence",
                            str(args.gate_max_divergence),
                        ]
                        if args.no_plot:
                            command.append("--no-plot")
                        print(
                            f"Running {plant} {scenario} {speed:g} m/s {rate:g} Hz",
                            flush=True,
                        )
                        completed = subprocess.run(
                            command, cwd=repo, capture_output=True, text=True, check=False
                        )
                        completed_returncode = completed.returncode
                        out_dir.mkdir(parents=True, exist_ok=True)
                        (out_dir / "command.log").write_text(
                            "$ "
                            + " ".join(command)
                            + "\n\nexit_code="
                            + str(completed.returncode)
                            + "\n\nSTDOUT\n"
                            + completed.stdout
                            + "\nSTDERR\n"
                            + completed.stderr,
                            encoding="utf-8",
                        )
                        summary = _load_summary(out_dir / "summary.json")

                    speed_max_metrics = _compute_speed_max_metrics(
                        out_dir / "ticks.jsonl", scenario
                    )
                    speed_max_gates: list[SpeedMaxGate] = []
                    if speed_max_metrics is not None:
                        speed_max_gates = _evaluate_speed_max_gates(
                            scenario=scenario,
                            target_speed=speed,
                            normal_accel_m_s2=args.local_planner_max_normal_accel_m_s2,
                            metrics=speed_max_metrics,
                        )
                    classification, failed_gates = _classify(summary, speed_max_metrics, speed_max_gates)
                    results.append(
                        {
                            "plant": plant,
                            "scenario": scenario,
                            "speed_m_s": speed,
                            "rate_hz": rate,
                            "run_dir": str(out_dir),
                            "command_returncode": completed_returncode,
                            "classification": classification,
                            "failed_gates": failed_gates,
                            "summary": summary,
                            "speed_max_metrics": None
                            if speed_max_metrics is None
                            else speed_max_metrics.asdict(),
                            "speed_max_gates": [
                                {
                                    "name": g.name,
                                    "threshold": g.threshold,
                                    "actual": g.actual,
                                    "passed": g.passed,
                                }
                                for g in speed_max_gates
                            ],
                        }
                    )

    finished_at = datetime.now(UTC).isoformat()
    _write_json(run_root / "matrix_results.json", results)
    _write_report(
        run_root,
        results,
        started_at=started_at,
        finished_at=finished_at,
        args=args,
    )
    print(f"Wrote {run_root / 'matrix_summary.md'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
