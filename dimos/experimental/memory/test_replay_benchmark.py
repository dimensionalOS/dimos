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

from pathlib import Path

import pytest

from dimos.experimental.memory.tool_replay_benchmark import (
    BenchmarkRustReplaySource,
    PythonReplaySource,
    _trial_blueprint,
    aggregate_runs,
    analyze_run,
    evaluate_gates,
    percentile,
    render_report,
)


def _trial(engine: str, *, late_ms: float = 10.0) -> dict[str, object]:
    source_times = {
        "lidar": (10.0, 10.1),
        "odom": (10.02, 10.12),
        "color_image": (10.04, 10.14),
    }
    samples: list[dict[str, object]] = []
    for stream, times in source_times.items():
        for source_ts in times:
            samples.append(
                {
                    "stream": stream,
                    "source_ts": source_ts,
                    "payload_ts": source_ts - 0.01,
                    "arrival_ns": int((source_ts - 10.0) * 1e9 + late_ms * 1e6),
                    "matched": True,
                }
            )
    return {
        "profile": "isolated:1x",
        "engine": engine,
        "repeat": 0,
        "speed": 1.0,
        "expected": dict.fromkeys(("lidar", "odom", "color_image"), 2),
        "missing": dict.fromkeys(("lidar", "odom", "color_image"), 0),
        "duplicates": dict.fromkeys(("lidar", "odom", "color_image"), 0),
        "unexpected": dict.fromkeys(("lidar", "odom", "color_image"), 0),
        "out_of_order": dict.fromkeys(("lidar", "odom", "color_image"), 0),
        "startup_ms": 100.0,
        "resource_samples": [{"cpu_pct": 20.0, "rss_mb": 30.0}],
        "samples": samples,
    }


def test_percentile_interpolates_and_handles_empty_input() -> None:
    assert percentile([], 99) is None
    assert percentile([1.0], 99) == 1.0
    assert percentile([0.0, 10.0], 50) == 5.0


def test_analyze_run_removes_only_the_constant_delivery_offset() -> None:
    run = analyze_run(_trial("python"))

    assert run["received"] == {"lidar": 2, "odom": 2, "color_image": 2}
    assert run["primary_p99_ms"] == pytest.approx(0.0, abs=0.001)
    assert run["primary_abs_drift_ms"] == pytest.approx(0.0, abs=0.001)
    assert all(sample["jitter_ms"] == pytest.approx(0.0, abs=0.001) for sample in run["samples"])
    assert run["cpu_mean_pct"] == 20.0
    assert run["rss_peak_mb"] == 30.0


def test_analyze_run_does_not_hide_a_delayed_first_observation() -> None:
    trial = _trial("python")
    first = min(trial["samples"], key=lambda sample: sample["source_ts"])  # type: ignore[arg-type,index]
    first["arrival_ns"] += 90_000_000  # type: ignore[index,operator]

    run = analyze_run(trial)
    delayed = next(sample for sample in run["samples"] if sample["source_ts"] == 10.0)

    assert delayed["schedule_error_ms"] == pytest.approx(90.0)
    assert delayed["jitter_ms"] == pytest.approx(90.0)
    assert run["timing"]["max_ms"] == pytest.approx(90.0)


def test_aggregate_gates_and_report() -> None:
    python_runs = [analyze_run(_trial("python")) for _ in range(3)]
    rust_runs = [analyze_run(_trial("rust")) for _ in range(3)]
    for run in python_runs:
        run["primary_p99_ms"] = 40.0
    for run in rust_runs:
        run["primary_p99_ms"] = 20.0

    aggregates = aggregate_runs([*python_runs, *rust_runs])
    gates = evaluate_gates(aggregates)
    report = render_report(
        aggregates,
        gates,
        {
            "dataset": "fixture",
            "seek": 5.0,
            "duration": 20.0,
            "repeats": 3,
            "transport": "zenoh",
        },
    )

    assert gates == [
        {
            "profile": "isolated:1x",
            "passed": True,
            "checks": {
                "lossless_and_ordered": True,
                "python_p99_at_most_50ms": True,
                "rust_p99_at_most_50ms": True,
                "drift_delta_at_most_10ms": True,
            },
            "python_p99_ms": 40.0,
            "rust_p99_ms": 20.0,
            "p99_improvement_pct": 50.0,
            "max_drift_delta_ms": pytest.approx(0.0),
        }
    ]
    assert "# Memory2 replay benchmark" in report
    assert "| isolated:1x | PASS |" in report


def test_python_and_rust_trial_blueprints_share_replay_config(tmp_path: Path) -> None:
    schedule: dict[str, list[tuple[float, float]]] = {
        "lidar": [],
        "odom": [],
        "color_image": [],
    }
    common = {
        "layer": "isolated",
        "path": tmp_path / "recording.db",
        "speed": 4.0,
        "seek": 5.0,
        "duration": 20.0,
        "transport": "zenoh",
        "topic_namespace": "dimos/replay-benchmark/test",
        "schedule": schedule,
        "warmup_schedule": schedule,
    }
    python = _trial_blueprint(engine="python", **common)  # type: ignore[arg-type]
    rust = _trial_blueprint(engine="rust", **common)  # type: ignore[arg-type]

    python_atom = next(
        atom for atom in python.active_blueprints if atom.module is PythonReplaySource
    )
    rust_atom = next(
        atom for atom in rust.active_blueprints if atom.module is BenchmarkRustReplaySource
    )
    for field in ("speed", "seek", "duration"):
        assert python_atom.kwargs[field] == rust_atom.kwargs[field]
    assert python_atom.kwargs["path"] == rust_atom.kwargs["store"].path
