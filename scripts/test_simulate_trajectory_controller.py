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

"""Focused tests for the issue 921 trajectory simulation runner."""

from __future__ import annotations

import contextlib
import importlib.util
import io
import json
from pathlib import Path
import sys
from types import ModuleType

import pytest

from dimos.navigation.trajectory_control_tick_export import iter_trajectory_control_tick_jsonl

_RUNNER_PATH = Path(__file__).with_name("simulate_trajectory_controller.py")


def _load_runner() -> ModuleType:
    module_name = "_simulate_trajectory_controller_under_test"
    module = sys.modules.get(module_name)
    if module is not None:
        return module

    spec = importlib.util.spec_from_file_location(module_name, _RUNNER_PATH)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"could not load {_RUNNER_PATH}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[module_name] = module
    spec.loader.exec_module(module)
    return module


def _run_runner(tmp_path: Path, *args: str) -> tuple[int, Path, dict[str, object]]:
    output_dir = tmp_path / "run"
    with contextlib.redirect_stdout(io.StringIO()):
        rc = _load_runner().main([*args, "--output-dir", str(output_dir), "--no-plot"])
    summary = json.loads((output_dir / "summary.json").read_text(encoding="utf-8"))
    return rc, output_dir, summary


@pytest.mark.parametrize("scenario", ["line", "circle"])
def test_synthetic_nominal_run_writes_jsonl_and_summary(tmp_path: Path, scenario: str) -> None:
    rc, output_dir, summary = _run_runner(
        tmp_path,
        "--scenario",
        scenario,
        "--speed",
        "0.8",
        "--rate",
        "10",
        "--plant-preset",
        "synthetic_nominal",
    )

    ticks_path = output_dir / "ticks.jsonl"
    rows = list(iter_trajectory_control_tick_jsonl(ticks_path))

    assert rc == 0
    assert ticks_path.is_file()
    assert (output_dir / "summary.json").is_file()
    assert (output_dir / "config.yaml").is_file()
    assert rows
    assert summary["verdict"]["status"] == "pass"
    assert summary["artifacts"]["ticks_jsonl"]["line_count"] == len(rows)
    assert summary["artifacts"]["plot_png"] is None


def test_low_gain_run_fails_a_gate(tmp_path: Path) -> None:
    rc, _output_dir, summary = _run_runner(
        tmp_path,
        "--scenario",
        "line",
        "--speed",
        "1.0",
        "--rate",
        "10",
        "--plant-preset",
        "synthetic_nominal",
        "--k-position",
        "0",
        "--k-yaw",
        "0",
        "--gate-max-divergence",
        "0.2",
    )

    failed_gates = summary["verdict"]["failed_gates"]

    assert rc == 1
    assert summary["verdict"]["status"] == "fail"
    assert "max_planar_position_divergence_m" in failed_gates
    assert summary["gates"]["max_planar_position_divergence_m"]["passed"] is False


def test_seeded_noisy_run_is_reproducible(tmp_path: Path) -> None:
    args = (
        "--scenario",
        "line",
        "--speed",
        "1.0",
        "--rate",
        "10",
        "--plant-preset",
        "synthetic_noisy",
        "--seed",
        "123",
    )

    rc_a, output_dir_a, summary_a = _run_runner(tmp_path / "a", *args)
    rc_b, output_dir_b, summary_b = _run_runner(tmp_path / "b", *args)

    assert rc_a == rc_b == 0
    assert (output_dir_a / "ticks.jsonl").read_text(encoding="utf-8") == (
        output_dir_b / "ticks.jsonl"
    ).read_text(encoding="utf-8")
    assert summary_a["metrics"] == summary_b["metrics"]
    assert summary_a["artifacts"]["ticks_jsonl"]["sha256"] == summary_b["artifacts"]["ticks_jsonl"]["sha256"]


def test_synthetic_asymmetric_run_writes_artifacts(tmp_path: Path) -> None:
    rc, output_dir, summary = _run_runner(
        tmp_path,
        "--scenario",
        "s_curve",
        "--speed",
        "1.0",
        "--rate",
        "10",
        "--plant-preset",
        "synthetic_asymmetric",
    )

    assert rc == 0
    assert (output_dir / "ticks.jsonl").is_file()
    assert summary["plant"]["preset"] == "synthetic_asymmetric"
    assert summary["artifacts"]["ticks_jsonl"]["line_count"] > 0
