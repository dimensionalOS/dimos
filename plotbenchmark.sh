#!/usr/bin/env bash
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

set -euo pipefail

readonly SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
readonly PYTHON="${SCRIPT_DIR}/.venv/bin/python"
readonly RESULTS_DIR="${1:-${SCRIPT_DIR}/benchmark_results}"
readonly CACHE_DIR="${RESULTS_DIR}/.plot-cache"

if [[ $# -gt 1 ]]; then
    echo "Usage: $0 [benchmark-results-directory]" >&2
    exit 2
fi

if [[ ! -x "${PYTHON}" ]]; then
    echo "Missing virtual environment Python: ${PYTHON}" >&2
    echo "Run 'uv sync --extra all' first." >&2
    exit 1
fi

if [[ ! -d "${RESULTS_DIR}" ]]; then
    echo "Benchmark results directory does not exist: ${RESULTS_DIR}" >&2
    exit 1
fi

mkdir -p "${CACHE_DIR}/matplotlib" "${CACHE_DIR}/xdg"

MPLCONFIGDIR="${CACHE_DIR}/matplotlib" \
    XDG_CACHE_HOME="${CACHE_DIR}/xdg" \
    "${PYTHON}" - "${RESULTS_DIR}" <<'PYTHON'
from __future__ import annotations

import json
import re
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from matplotlib.backends.backend_agg import FigureCanvasAgg
from matplotlib.figure import Figure


@dataclass(frozen=True)
class BenchmarkRun:
    path: Path
    implementation: str
    run_id: str
    workload: tuple[str, str, str, float, float]
    median_cpu_percent: float
    median_rss_mib: float


@dataclass(frozen=True)
class BenchmarkPair:
    python: BenchmarkRun
    rust: BenchmarkRun


def natural_key(value: str) -> tuple[Any, ...]:
    return tuple(
        int(part) if part.isdigit() else part.lower()
        for part in re.split(r"(\d+)", value)
    )


def load_run(path: Path) -> tuple[BenchmarkRun | None, str | None]:
    match = re.fullmatch(r"(python|rust)-(.+)", path.parent.name)
    if match is None:
        return None, f"unrecognized run directory: {path.parent.name}"

    try:
        data = json.loads(path.read_text())
        implementation = str(data["implementation"])
        workload_data = data["workload"]
        summary = data["summary"]
        samples = data["samples"]
    except (KeyError, TypeError, json.JSONDecodeError) as error:
        return None, f"invalid result JSON: {error}"

    if implementation != match.group(1):
        return None, (
            f"implementation is {implementation!r}, but directory is "
            f"{path.parent.name!r}"
        )
    if bool(data.get("interrupted", False)):
        return None, "run was interrupted"
    if not isinstance(samples, list) or not samples:
        return None, "run has no samples"

    try:
        requested_duration = float(workload_data["requested_duration_seconds"])
        observed_duration = float(data["observed_duration_seconds"])
        workload = (
            str(workload_data["blueprint"]),
            str(workload_data["replay_db"]),
            str(workload_data["transport"]),
            requested_duration,
            float(workload_data["sample_interval_seconds"]),
        )
        median_cpu_percent = float(summary["median_cpu_percent"])
        median_rss_mib = float(summary["median_rss_mib"])
    except (KeyError, TypeError, ValueError) as error:
        return None, f"invalid benchmark value: {error}"

    if requested_duration > 0 and observed_duration < requested_duration * 0.95:
        return None, (
            f"run completed only {observed_duration:.1f}s of the requested "
            f"{requested_duration:.1f}s"
        )

    return (
        BenchmarkRun(
            path=path,
            implementation=implementation,
            run_id=match.group(2),
            workload=workload,
            median_cpu_percent=median_cpu_percent,
            median_rss_mib=median_rss_mib,
        ),
        None,
    )


def percent_change(before: float, after: float) -> float | None:
    if before == 0:
        return None
    return (after / before - 1.0) * 100.0


def format_change(change: float | None) -> str:
    if change is None:
        return "n/a"
    return f"{change:+.1f}%"


def add_bar_labels(axis: Any, bars: Any, suffix: str) -> None:
    for bar in bars:
        value = float(bar.get_height())
        axis.annotate(
            f"{value:.0f}{suffix}",
            xy=(bar.get_x() + bar.get_width() / 2, value),
            xytext=(0, 4),
            textcoords="offset points",
            ha="center",
            va="bottom",
            fontsize=8,
        )


def write_chart(
    pairs: list[BenchmarkPair],
    labels: list[str],
    output: Path,
    *,
    metric: str,
    ylabel: str,
    title: str,
    suffix: str,
) -> None:
    width = max(9.0, len(pairs) * 1.6)
    figure = Figure(figsize=(width, 6.0), constrained_layout=True)
    FigureCanvasAgg(figure)
    axis = figure.subplots()
    positions = list(range(len(pairs)))
    bar_width = 0.38
    python_values = [float(getattr(pair.python, metric)) for pair in pairs]
    rust_values = [float(getattr(pair.rust, metric)) for pair in pairs]

    python_bars = axis.bar(
        [position - bar_width / 2 for position in positions],
        python_values,
        bar_width,
        label="Python",
        color="#4C78A8",
    )
    rust_bars = axis.bar(
        [position + bar_width / 2 for position in positions],
        rust_values,
        bar_width,
        label="Rust",
        color="#F58518",
    )
    axis.set_title(title)
    axis.set_ylabel(ylabel)
    axis.set_xticks(positions, labels, rotation=20, ha="right")
    axis.grid(axis="y", alpha=0.25)
    axis.legend()
    axis.set_axisbelow(True)
    if len(pairs) <= 12:
        add_bar_labels(axis, python_bars, suffix)
        add_bar_labels(axis, rust_bars, suffix)
    figure.savefig(output, dpi=180)


results_dir = Path(sys.argv[1]).resolve()
runs: dict[tuple[tuple[str, str, str, float, float], str], dict[str, BenchmarkRun]] = {}
warnings: list[str] = []

for result_path in sorted(results_dir.glob("**/result.json")):
    if "comparison" in result_path.relative_to(results_dir).parts:
        continue
    run, warning = load_run(result_path)
    if run is None:
        warnings.append(f"Skipping {result_path}: {warning}")
        continue
    key = (run.workload, run.run_id)
    implementations = runs.setdefault(key, {})
    if run.implementation in implementations:
        raise SystemExit(
            f"Duplicate {run.implementation} result for run {run.run_id}: "
            f"{implementations[run.implementation].path} and {run.path}"
        )
    implementations[run.implementation] = run

pairs: list[BenchmarkPair] = []
for (workload, run_id), implementations in runs.items():
    if "python" not in implementations or "rust" not in implementations:
        missing = "rust" if "rust" not in implementations else "python"
        warnings.append(
            f"Skipping {workload[1]} run {run_id}: missing completed {missing} result"
        )
        continue
    pairs.append(
        BenchmarkPair(
            python=implementations["python"],
            rust=implementations["rust"],
        )
    )

pairs.sort(
    key=lambda pair: (
        pair.python.workload[1],
        pair.python.workload[2],
        natural_key(pair.python.run_id),
    )
)

if not pairs:
    for warning in warnings:
        print(f"warning: {warning}", file=sys.stderr)
    raise SystemExit(f"No completed Python/Rust benchmark pairs found in {results_dir}")

table_lines = [
    "| Replay | Run | Python CPU | Rust CPU | CPU change | Python memory | Rust memory | Memory change |",
    "|---|---:|---:|---:|---:|---:|---:|---:|",
]
chart_labels: list[str] = []
for pair in pairs:
    replay_db = pair.python.workload[1]
    transport = pair.python.workload[2]
    cpu_change = percent_change(
        pair.python.median_cpu_percent, pair.rust.median_cpu_percent
    )
    memory_change = percent_change(
        pair.python.median_rss_mib, pair.rust.median_rss_mib
    )
    table_lines.append(
        "| "
        f"{replay_db} ({transport}) | {pair.python.run_id} | "
        f"{pair.python.median_cpu_percent:.1f}% | "
        f"{pair.rust.median_cpu_percent:.1f}% | {format_change(cpu_change)} | "
        f"{pair.python.median_rss_mib:.0f} MiB | "
        f"{pair.rust.median_rss_mib:.0f} MiB | {format_change(memory_change)} |"
    )
    chart_labels.append(f"{replay_db}\nrun {pair.python.run_id}")

table_lines.extend(
    [
        "",
        "CPU and memory are medians across the full DimOS process tree. "
        "100% CPU equals one logical core. Memory is RSS. Changes are Rust "
        "relative to Python; negative values are lower.",
    ]
)

report_path = results_dir / "benchmark_report.md"
report = "\n".join(table_lines) + "\n"
report_path.write_text(report)
print(report, end="")

cpu_path = results_dir / "benchmark_cpu.png"
memory_path = results_dir / "benchmark_memory.png"
write_chart(
    pairs,
    chart_labels,
    cpu_path,
    metric="median_cpu_percent",
    ylabel="Median CPU (%)",
    title="Full DimOS CPU: Python vs Rust CostMapper",
    suffix="%",
)
write_chart(
    pairs,
    chart_labels,
    memory_path,
    metric="median_rss_mib",
    ylabel="Median RSS (MiB)",
    title="Full DimOS Memory: Python vs Rust CostMapper",
    suffix="",
)

for warning in warnings:
    print(f"warning: {warning}", file=sys.stderr)
print(f"\nReport: {report_path}")
print(f"CPU graph: {cpu_path}")
print(f"Memory graph: {memory_path}")
PYTHON
