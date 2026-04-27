#!/usr/bin/env python3
"""Run the SIM-04 initial trajectory simulation matrix and write a report."""

from __future__ import annotations

from collections import Counter, defaultdict
import argparse
from datetime import UTC, datetime
import json
from pathlib import Path
import subprocess
import sys
from typing import Any


SPEEDS_M_S = (0.55, 1.0, 1.5, 2.0)
RATES_HZ = (5.0, 10.0, 20.0, 30.0)
PLANT_PRESETS = (
    "ideal",
    "synthetic_nominal",
    "synthetic_sluggish",
    "synthetic_asymmetric",
    "synthetic_noisy",
)
SCENARIOS = ("line", "circle", "s_curve", "right_angle_turn")
SEED = 7
DEFAULT_K_POSITION_PER_S = 2.2
DEFAULT_K_YAW_PER_S = 2.5


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


def _gain_label(k_position: float, k_yaw: float) -> str:
    if k_position == DEFAULT_K_POSITION_PER_S and k_yaw == DEFAULT_K_YAW_PER_S:
        return "default_gains"
    return f"kp{_token(k_position)}__ky{_token(k_yaw)}"


def _run_dir(
    root: Path,
    plant: str,
    scenario: str,
    speed: float,
    rate: float,
    reference_mode: str,
    k_position: float,
    k_yaw: float,
) -> Path:
    reference_label = "" if reference_mode == "direct" else f"__{reference_mode}"
    return root / (
        f"{plant}__{scenario}__{_token(speed)}mps__{_token(rate)}hz"
        f"{reference_label}__{_gain_label(k_position, k_yaw)}"
    )


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


def _ordered_present(candidates: tuple[str, ...] | tuple[float, ...], present: set[str] | set[float]) -> list[str] | list[float]:
    return [item for item in candidates if item in present]


def _load_summary(path: Path) -> dict[str, Any] | None:
    if not path.is_file():
        return None
    return json.loads(path.read_text(encoding="utf-8"))


def _classify(summary: dict[str, Any] | None) -> str:
    if summary is None:
        return "invalid"
    status = summary["verdict"]["status"]
    if status != "pass":
        return status
    for gate in summary["gates"].values():
        threshold = gate["threshold"]
        actual = gate["actual"]
        if (
            isinstance(threshold, int | float)
            and not isinstance(threshold, bool)
            and isinstance(actual, int | float)
            and not isinstance(actual, bool)
        ):
            if threshold > 0.0 and actual >= 0.8 * threshold:
                return "borderline"
    return "pass"


def _strict_nominal_gate(summary: dict[str, Any]) -> tuple[str, str] | None:
    plant = summary["plant"]["preset"]
    if plant != "synthetic_nominal":
        return None
    scenario = summary["scenario"]["name"]
    speed = float(summary["scenario"]["target_speed_m_s"])
    metrics = summary["metrics"]
    if speed == 1.5 and scenario in {"line", "circle", "s_curve"}:
        actual = metrics["p95_planar_position_divergence_m"]
        status = "pass" if actual <= 0.20 else "fail"
        return status, f"1.5 m/s p95 planar divergence {actual:.3f} m vs 0.200 m"
    if speed == 2.0 and scenario != "right_angle_turn":
        actual = metrics["max_planar_position_divergence_m"]
        status = "pass" if actual <= 0.35 else "fail"
        return status, f"2.0 m/s max planar divergence {actual:.3f} m vs 0.350 m"
    return None


def _rel(path: Path, root: Path) -> str:
    return path.relative_to(root).as_posix()


def _write_json(path: Path, payload: Any) -> None:
    path.write_text(json.dumps(payload, indent=2, sort_keys=False, allow_nan=False) + "\n", encoding="utf-8")


def _write_report(run_root: Path, results: list[dict[str, Any]], started_at: str, finished_at: str) -> None:
    repo = _repo_root()
    counts = Counter(r["classification"] for r in results)
    verdict_counts = Counter((r["summary"] or {}).get("verdict", {}).get("status", "missing") for r in results)
    by_plant = defaultdict(Counter)
    by_scenario = defaultdict(Counter)
    strict_counts = Counter()
    strict_failures: list[tuple[dict[str, Any], str]] = []
    for result in results:
        summary = result["summary"]
        classification = result["classification"]
        by_plant[result["plant"]][classification] += 1
        by_scenario[result["scenario"]][classification] += 1
        if summary is not None:
            strict = _strict_nominal_gate(summary)
            if strict is not None:
                strict_counts[strict[0]] += 1
                if strict[0] != "pass":
                    strict_failures.append((result, strict[1]))

    failures = [r for r in results if r["classification"] in {"fail", "invalid"}]
    borderline = [r for r in results if r["classification"] == "borderline"]
    worst = sorted(
        (r for r in results if r["summary"] is not None),
        key=lambda r: r["summary"]["metrics"]["max_planar_position_divergence_m"],
        reverse=True,
    )[:20]
    speeds = _ordered_present(SPEEDS_M_S, {r["speed_m_s"] for r in results})
    rates = _ordered_present(RATES_HZ, {r["rate_hz"] for r in results})
    plants = _ordered_present(PLANT_PRESETS, {r["plant"] for r in results})
    scenarios = _ordered_present(SCENARIOS, {r["scenario"] for r in results})
    reference_modes = sorted({r["reference_mode"] for r in results})
    gain_labels = sorted({f"kp={r['k_position_per_s']:g}, ky={r['k_yaw_per_s']:g}" for r in results})

    representative_plots: list[dict[str, str]] = []
    for plant in plants:
        for scenario in scenarios:
            candidates = [
                r
                for r in results
                if r["plant"] == plant
                and r["scenario"] == scenario
                and r["reference_mode"] == "direct"
                and r["k_position_per_s"] == DEFAULT_K_POSITION_PER_S
                and r["k_yaw_per_s"] == DEFAULT_K_YAW_PER_S
                and r["speed_m_s"] == 1.5
                and r["rate_hz"] == 10.0
            ]
            if not candidates:
                candidates = [r for r in results if r["plant"] == plant and r["scenario"] == scenario]
            if candidates:
                plot = Path(candidates[0]["run_dir"]) / "plot.png"
                representative_plots.append(
                    {"plant": plant, "scenario": scenario, "plot": _rel(plot, repo) if plot.is_file() else "missing"}
                )

    lines = [
        "# SIM-04 Initial Trajectory Simulation Matrix",
        "",
        f"Started: `{started_at}`",
        f"Finished: `{finished_at}`",
        f"Run root: `{_rel(run_root, repo)}`",
        "",
        "## Scope",
        "",
        f"- Speeds: `{', '.join(str(s) for s in speeds)}` m/s.",
        f"- Rates: `{', '.join(str(r) for r in rates)}` Hz.",
        f"- Plant presets: `{', '.join(plants)}`.",
        f"- Paths: `{', '.join(scenarios)}`.",
        f"- Reference modes: `{', '.join(reference_modes)}`.",
        f"- Gains: `{', '.join(gain_labels)}`.",
        "- Calibration output: none loaded by this matrix driver. The sample calibration fixture is not a recorded run.",
        "- Fitted Go2 presets: none available in this workspace, so this report is a synthetic-envelope confidence run and not an empirical Go2 hardware claim.",
        f"- Total runs: `{len(results)}`.",
        "",
        "## Implementation Vs Tuning",
        "",
        "- Implementation changes under test: `path_speed_profile` reference generation and live-like speed caps when that reference mode is selected.",
        "- Tuning-only changes under test: gain values listed above; default-gain runs use no gain tuning.",
        "- Gates are unchanged from the runner defaults; this report does not relax divergence thresholds.",
        "",
        "## Verdict Counts",
        "",
        f"- Runner verdicts: pass `{verdict_counts['pass']}`, fail `{verdict_counts['fail']}`, invalid `{verdict_counts['invalid']}`, missing `{verdict_counts['missing']}`.",
        f"- Matrix classifications: pass `{counts['pass']}`, borderline `{counts['borderline']}`, fail `{counts['fail']}`, invalid `{counts['invalid']}`.",
        "",
        "A run is classified as borderline when the runner verdict passes but any numeric gate is at or above 80% of its threshold.",
        "",
        "## Counts By Plant",
        "",
        "| Plant | Pass | Borderline | Fail | Invalid |",
        "| --- | ---: | ---: | ---: | ---: |",
    ]
    for plant in plants:
        bucket = by_plant[plant]
        lines.append(f"| `{plant}` | {bucket['pass']} | {bucket['borderline']} | {bucket['fail']} | {bucket['invalid']} |")

    lines.extend(
        [
            "",
            "## Counts By Path",
            "",
            "| Path | Pass | Borderline | Fail | Invalid |",
            "| --- | ---: | ---: | ---: | ---: |",
        ]
    )
    for scenario in scenarios:
        bucket = by_scenario[scenario]
        lines.append(
            f"| `{scenario}` | {bucket['pass']} | {bucket['borderline']} | {bucket['fail']} | {bucket['invalid']} |"
        )

    lines.extend(["", "## PR Confidence Gate Check", ""])
    if strict_counts:
        lines.append(
            f"- Synthetic nominal PR gates checked: pass `{strict_counts['pass']}`, fail `{strict_counts['fail']}`."
        )
    else:
        lines.append("- No strict synthetic nominal PR gates were applicable.")
    if strict_failures:
        lines.append("- Strict synthetic nominal misses:")
        for result, reason in strict_failures[:20]:
            lines.append(
                f"  - `{_rel(Path(result['run_dir']), repo)}` - {reason}; runner verdict `{result['summary']['verdict']['status']}`."
            )
        if len(strict_failures) > 20:
            lines.append(f"  - Plus `{len(strict_failures) - 20}` more strict-gate misses.")
    else:
        lines.append("- No strict synthetic nominal misses.")

    lines.extend(["", "## Fail And Invalid Cases", ""])
    if failures:
        for result in failures[:40]:
            summary = result["summary"]
            failed = "missing summary" if summary is None else ", ".join(summary["verdict"]["failed_gates"])
            lines.append(f"- `{_rel(Path(result['run_dir']), repo)}` - {result['classification']} - {failed}.")
        if len(failures) > 40:
            lines.append(f"- Plus `{len(failures) - 40}` more fail or invalid cases.")
    else:
        lines.append("- None under the runner gates.")

    lines.extend(["", "## Borderline Cases", ""])
    if borderline:
        for result in borderline[:40]:
            summary = result["summary"]
            metrics = summary["metrics"]
            gates = summary["gates"]
            lines.append(
                f"- `{_rel(Path(result['run_dir']), repo)}` - max divergence `{metrics['max_planar_position_divergence_m']:.3f}` m of `{gates['max_planar_position_divergence_m']['threshold']:.3f}` m."
            )
        if len(borderline) > 40:
            lines.append(f"- Plus `{len(borderline) - 40}` more borderline cases.")
    else:
        lines.append("- None.")

    lines.extend(
        [
            "",
            "## Worst Max-Divergence Runs",
            "",
            "| Run | Verdict | Max divergence m | P95 divergence m | Final position error m |",
            "| --- | --- | ---: | ---: | ---: |",
        ]
    )
    for result in worst:
        summary = result["summary"]
        metrics = summary["metrics"]
        lines.append(
            f"| `{_rel(Path(result['run_dir']), repo)}` | `{summary['verdict']['status']}` | {metrics['max_planar_position_divergence_m']:.3f} | {metrics['p95_planar_position_divergence_m']:.3f} | {metrics['final_position_error_m']:.3f} |"
        )

    lines.extend(["", "## Representative Speed-Vs-Divergence Plots", ""])
    for item in representative_plots:
        lines.append(f"- `{item['plant']}` / `{item['scenario']}`: `{item['plot']}`")

    lines.extend(
        [
            "",
            "## Machine-Readable Index",
            "",
            f"- `matrix_results.json` contains one record per run with command status, classification, and parsed summary.",
        ]
    )
    (run_root / "matrix_summary.md").write_text("\n".join(lines) + "\n", encoding="utf-8")


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Run or resummarize the SIM-04 initial trajectory matrix.")
    parser.add_argument("--run-root", type=Path, default=None, help="Existing or new matrix run root.")
    parser.add_argument(
        "--speeds",
        type=lambda text: _parse_float_list(text, name="--speeds"),
        default=SPEEDS_M_S,
        help="Comma-separated speeds in m/s to run.",
    )
    parser.add_argument(
        "--rates",
        type=lambda text: _parse_float_list(text, name="--rates"),
        default=RATES_HZ,
        help="Comma-separated control rates in Hz to run.",
    )
    parser.add_argument(
        "--plants",
        type=lambda text: _parse_choice_list(text, name="--plants", choices=PLANT_PRESETS),
        default=PLANT_PRESETS,
        help="Comma-separated plant presets to run.",
    )
    parser.add_argument(
        "--scenarios",
        type=lambda text: _parse_choice_list(text, name="--scenarios", choices=SCENARIOS),
        default=SCENARIOS,
        help="Comma-separated scenarios to run.",
    )
    parser.add_argument(
        "--reference-mode",
        choices=["direct", "path_speed_profile"],
        default="direct",
        help="Reference mode passed to simulate_trajectory_controller.py.",
    )
    parser.add_argument(
        "--k-position-values",
        type=lambda text: _parse_float_list(text, name="--k-position-values"),
        default=(DEFAULT_K_POSITION_PER_S,),
        help="Comma-separated k_position_per_s values to sweep.",
    )
    parser.add_argument(
        "--k-yaw-values",
        type=lambda text: _parse_float_list(text, name="--k-yaw-values"),
        default=(DEFAULT_K_YAW_PER_S,),
        help="Comma-separated k_yaw_per_s values to sweep.",
    )
    parser.add_argument(
        "--local-planner-max-tangent-accel-m-s2",
        type=float,
        default=1.0,
        help="Path speed profile tangent acceleration cap passed to the runner.",
    )
    parser.add_argument(
        "--local-planner-max-normal-accel-m-s2",
        type=float,
        default=0.6,
        help="Path speed profile normal acceleration cap passed to the runner.",
    )
    parser.add_argument(
        "--local-planner-goal-decel-m-s2",
        type=float,
        default=1.0,
        help="Path speed profile goal deceleration cap passed to the runner.",
    )
    parser.add_argument(
        "--only-missing",
        action="store_true",
        help="Reuse existing summaries and run only matrix entries without summary.json.",
    )
    parser.add_argument(
        "--summarize-only",
        action="store_true",
        help="Do not run simulations; rebuild matrix_results.json and matrix_summary.md from existing summaries.",
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
        else sim_dir / "simulation_runs" / f"{timestamp}_sim04_initial_matrix"
    )
    run_root.mkdir(parents=True, exist_ok=args.only_missing or args.summarize_only)
    started_at = datetime.now(UTC).isoformat()
    results: list[dict[str, Any]] = []

    for plant in args.plants:
        for scenario in args.scenarios:
            for speed in args.speeds:
                for rate in args.rates:
                    for k_position in args.k_position_values:
                        for k_yaw in args.k_yaw_values:
                            out_dir = _run_dir(
                                run_root,
                                plant,
                                scenario,
                                speed,
                                rate,
                                args.reference_mode,
                                k_position,
                                k_yaw,
                            )
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
                                    "--k-position",
                                    str(k_position),
                                    "--k-yaw",
                                    str(k_yaw),
                                    "--reference-mode",
                                    args.reference_mode,
                                    "--local-planner-max-tangent-accel-m-s2",
                                    str(args.local_planner_max_tangent_accel_m_s2),
                                    "--local-planner-max-normal-accel-m-s2",
                                    str(args.local_planner_max_normal_accel_m_s2),
                                    "--local-planner-goal-decel-m-s2",
                                    str(args.local_planner_goal_decel_m_s2),
                                ]
                                print(
                                    "Running "
                                    f"{plant} {scenario} {speed:g} m/s {rate:g} Hz "
                                    f"{args.reference_mode} kp={k_position:g} ky={k_yaw:g}",
                                    flush=True,
                                )
                                completed = subprocess.run(command, cwd=repo, capture_output=True, text=True, check=False)
                                completed_returncode = completed.returncode
                                out_dir.mkdir(parents=True, exist_ok=True)
                                (out_dir / "command.log").write_text(
                                    "$ "
                                    + " ".join(command)
                                    + "\n\n"
                                    + "exit_code="
                                    + str(completed.returncode)
                                    + "\n\nSTDOUT\n"
                                    + completed.stdout
                                    + "\nSTDERR\n"
                                    + completed.stderr,
                                    encoding="utf-8",
                                )
                                summary = _load_summary(out_dir / "summary.json")
                            classification = _classify(summary)
                            results.append(
                                {
                                    "plant": plant,
                                    "scenario": scenario,
                                    "speed_m_s": speed,
                                    "rate_hz": rate,
                                    "reference_mode": args.reference_mode,
                                    "k_position_per_s": k_position,
                                    "k_yaw_per_s": k_yaw,
                                    "run_dir": str(out_dir),
                                    "command_returncode": completed_returncode,
                                    "classification": classification,
                                    "summary": summary,
                                }
                            )

    finished_at = datetime.now(UTC).isoformat()
    _write_json(run_root / "matrix_results.json", results)
    _write_report(run_root, results, started_at, finished_at)
    print(f"Wrote {run_root / 'matrix_summary.md'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
