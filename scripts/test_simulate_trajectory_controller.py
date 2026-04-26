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
import yaml

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


def test_path_speed_profile_reference_records_live_like_caps(tmp_path: Path) -> None:
    rc, output_dir, summary = _run_runner(
        tmp_path,
        "--scenario",
        "circle",
        "--speed",
        "2.0",
        "--rate",
        "10",
        "--plant-preset",
        "ideal",
        "--reference-mode",
        "path_speed_profile",
        "--local-planner-max-normal-accel-m-s2",
        "0.6",
    )

    config = yaml.safe_load((output_dir / "config.yaml").read_text(encoding="utf-8"))
    profile = config["scenario"]["path"]["speed_profile"]

    assert rc == 0
    assert summary["scenario"]["variant"] == "path_speed_profile"
    assert config["controller"]["mode"] == "path_speed_profile"
    assert profile["geometry_speed_cap_m_s"] == pytest.approx((0.6 * 1.5) ** 0.5)
    assert profile["max_profile_speed_m_s"] < 2.0


@pytest.mark.parametrize("scenario", ["s_curve", "right_angle_turn"])
@pytest.mark.parametrize("plant", ["synthetic_nominal", "synthetic_asymmetric", "synthetic_noisy"])
def test_path_speed_profile_supports_2mps_conservative_envelope(
    tmp_path: Path,
    scenario: str,
    plant: str,
) -> None:
    rc, output_dir, summary = _run_runner(
        tmp_path,
        "--scenario",
        scenario,
        "--speed",
        "2.0",
        "--rate",
        "10",
        "--plant-preset",
        plant,
        "--reference-mode",
        "path_speed_profile",
        "--local-planner-max-tangent-accel-m-s2",
        "0.5",
        "--local-planner-max-normal-accel-m-s2",
        "0.1",
        "--local-planner-goal-decel-m-s2",
        "0.5",
        "--seed",
        "7",
    )

    config = yaml.safe_load((output_dir / "config.yaml").read_text(encoding="utf-8"))
    speed_profile = config["scenario"]["path"]["speed_profile"]

    assert rc == 0
    assert summary["verdict"]["status"] == "pass"
    assert summary["scenario"]["target_speed_m_s"] == pytest.approx(2.0)
    assert speed_profile["max_profile_speed_m_s"] < 2.0
    assert summary["metrics"]["max_planar_position_divergence_m"] < 1.0
    assert summary["metrics"]["arrived"] is True


def test_config_yaml_and_cli_override_runner_options(tmp_path: Path) -> None:
    config_in = tmp_path / "runner.yaml"
    config_in.write_text(
        "\n".join(
            [
                "runner:",
                "  reference_mode: path_speed_profile",
                "  plant_preset: ideal",
                "  k_position: 3.0",
                "  k_yaw: 1.1",
                "  local_planner_max_normal_accel_m_s2: 0.6",
            ]
        )
        + "\n",
        encoding="utf-8",
    )

    rc, output_dir, _summary = _run_runner(
        tmp_path,
        "--config-yaml",
        str(config_in),
        "--scenario",
        "line",
        "--speed",
        "0.8",
        "--rate",
        "10",
        "--k-position",
        "2.0",
    )

    config = yaml.safe_load((output_dir / "config.yaml").read_text(encoding="utf-8"))

    assert rc == 0
    assert config["plant"]["preset"] == "ideal"
    assert config["controller"]["mode"] == "path_speed_profile"
    assert config["controller"]["gains"] == {"k_position_per_s": 2.0, "k_yaw_per_s": 1.1}


def test_calibration_yaml_loads_suggested_gains(tmp_path: Path) -> None:
    fixture = (
        Path(__file__).resolve().parents[1]
        / "dimos"
        / "navigation"
        / "fixtures"
        / "holonomic_calibration_params_sample_v1.yaml"
    )

    rc, output_dir, _summary = _run_runner(
        tmp_path,
        "--scenario",
        "line",
        "--speed",
        "0.8",
        "--rate",
        "10",
        "--plant-preset",
        "ideal",
        "--calibration-params-yaml",
        str(fixture),
    )

    config = yaml.safe_load((output_dir / "config.yaml").read_text(encoding="utf-8"))

    assert rc == 0
    assert config["controller"]["calibration_source"] == str(fixture)
    assert config["controller"]["gains"] == {"k_position_per_s": 1.2, "k_yaw_per_s": 0.9}


def test_measurement_delay_records_disturbance_and_fails_gate(tmp_path: Path) -> None:
    rc, output_dir, summary = _run_runner(
        tmp_path,
        "--scenario",
        "line",
        "--speed",
        "1.0",
        "--rate",
        "10",
        "--plant-preset",
        "synthetic_nominal",
        "--measurement-delay-ticks",
        "20",
        "--gate-max-divergence",
        "0.25",
    )

    config = yaml.safe_load((output_dir / "config.yaml").read_text(encoding="utf-8"))
    measurement = summary["disturbance"]["measurement"]

    assert rc == 1
    assert summary["verdict"]["status"] == "fail"
    assert "max_planar_position_divergence_m" in summary["verdict"]["failed_gates"]
    assert config["disturbance"]["measurement"]["fixed_delay_ticks"] == 20
    assert measurement["fixed_delay_s"] == pytest.approx(2.0)
    assert measurement["stats"]["delayed_samples"] > 0


def test_measurement_jitter_drop_and_stale_are_reproducible(tmp_path: Path) -> None:
    args = (
        "--scenario",
        "circle",
        "--speed",
        "0.8",
        "--rate",
        "10",
        "--plant-preset",
        "synthetic_nominal",
        "--measurement-jitter-ticks",
        "3",
        "--measurement-stale-probability",
        "0.25",
        "--measurement-drop-probability",
        "0.25",
        "--seed",
        "99",
    )

    rc_a, _output_dir_a, summary_a = _run_runner(tmp_path / "a", *args)
    rc_b, _output_dir_b, summary_b = _run_runner(tmp_path / "b", *args)

    stats = summary_a["disturbance"]["measurement"]["stats"]

    assert rc_a == rc_b
    assert summary_a["metrics"] == summary_b["metrics"]
    assert summary_a["disturbance"]["measurement"] == summary_b["disturbance"]["measurement"]
    assert stats["jittered_samples"] > 0
    assert stats["stale_reused_samples"] > 0
    assert stats["dropped_samples"] > 0


def test_slower_lateral_response_causes_controlled_degradation(tmp_path: Path) -> None:
    args = (
        "--scenario",
        "s_curve",
        "--speed",
        "1.0",
        "--rate",
        "10",
        "--plant-preset",
        "synthetic_nominal",
    )
    rc_nominal, _output_dir_nominal, nominal = _run_runner(tmp_path / "nominal", *args)
    rc_slow, output_dir_slow, slow = _run_runner(
        tmp_path / "slow",
        *args,
        "--response-linear-y-scale",
        "0.35",
        "--response-linear-y-command-gain-scale",
        "0.65",
        "--response-speed-dependent-slip-per-mps",
        "0.3",
    )

    config = yaml.safe_load((output_dir_slow / "config.yaml").read_text(encoding="utf-8"))

    assert rc_nominal == 0
    assert rc_slow in {0, 1}
    assert config["disturbance"]["response_wrappers"]["linear_y_response_scale"] == 0.35
    assert slow["plant"]["response_curve_id"].endswith("+runner_wrappers")
    assert slow["metrics"]["max_cross_track_error_m"] > nominal["metrics"]["max_cross_track_error_m"]
