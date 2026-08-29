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

import json
import subprocess
from types import SimpleNamespace

import numpy as np
import pytest

from dimos.control.tasks.g1_sonic_wbc_task import sonic_diagnostics
from dimos.control.tasks.g1_sonic_wbc_task.sonic_hardware import (
    ensure_sonic_max_performance,
)


def test_profile_counts_only_cpu_kernel_events(tmp_path) -> None:
    profile = tmp_path / "profile.json"
    profile.write_text(
        json.dumps(
            [
                {
                    "cat": "Node",
                    "name": "slice_kernel_time",
                    "args": {"provider": "CPUExecutionProvider", "op_name": "Slice"},
                },
                {
                    "cat": "Node",
                    "name": "atan_kernel_time",
                    "args": {"provider": "CPUExecutionProvider", "op_name": "Atan"},
                },
                {
                    "cat": "Node",
                    "name": "matmul_kernel_time",
                    "args": {"provider": "CUDAExecutionProvider", "op_name": "MatMul"},
                },
                {"cat": "Session", "name": "model_run", "args": {}},
            ]
        ),
        encoding="utf-8",
    )

    result = sonic_diagnostics._profile_cpu_ops(profile)

    assert result == {"Slice": 1, "Atan": 1}


def test_accuracy_gate_accepts_validated_cuda11_planner_difference() -> None:
    expected = np.zeros((4,), dtype=np.float32)
    actual = np.array([0.0026, -0.0004, 0.0, 0.0], dtype=np.float32)

    detail = sonic_diagnostics._accuracy_detail("planner", actual, expected)

    assert "max error 0.0026" in detail


def test_accuracy_gate_rejects_nonfinite_output() -> None:
    with pytest.raises(RuntimeError, match="non-finite"):
        sonic_diagnostics._accuracy_detail(
            "encoder",
            np.array([np.nan], dtype=np.float32),
            np.zeros((1,), dtype=np.float32),
        )


def test_doctor_stops_before_models_when_host_is_incompatible(mocker) -> None:
    mocker.patch.object(
        sonic_diagnostics,
        "_host_checks",
        return_value=(("architecture", lambda: sonic_diagnostics._raise("wrong host")),),
    )
    inference = mocker.patch.object(sonic_diagnostics, "_inference_checks")

    report = sonic_diagnostics.run_sonic_doctor()

    assert not report.passed
    assert report.checks == (
        sonic_diagnostics.SonicDiagnosticCheck("architecture", False, "wrong host"),
    )
    inference.assert_not_called()


def test_planner_input_matches_released_model_contract() -> None:
    inputs = sonic_diagnostics._planner_inputs()

    assert {name: (value.shape, value.dtype) for name, value in inputs.items()} == {
        "context_mujoco_qpos": ((1, 4, 36), np.dtype(np.float32)),
        "target_vel": ((1,), np.dtype(np.float32)),
        "mode": ((1,), np.dtype(np.int64)),
        "movement_direction": ((1, 3), np.dtype(np.float32)),
        "facing_direction": ((1, 3), np.dtype(np.float32)),
        "random_seed": ((1,), np.dtype(np.int64)),
        "has_specific_target": ((1, 1), np.dtype(np.int64)),
        "specific_target_positions": ((1, 4, 3), np.dtype(np.float32)),
        "specific_target_headings": ((1, 4), np.dtype(np.float32)),
        "allowed_pred_num_tokens": ((1, 11), np.dtype(np.int64)),
        "height": ((1,), np.dtype(np.float32)),
    }


def test_max_performance_check_accepts_locked_cpu_and_gpu(mocker) -> None:
    run = mocker.patch(
        "subprocess.run",
        side_effect=[
            SimpleNamespace(stdout="NV Power Mode: MAXN\n0\n"),
            SimpleNamespace(
                stdout=(
                    "cpu0: Online=1 MinFreq=2201600 MaxFreq=2201600 CurrentFreq=2201600\n"
                    "GPU MinFreq=1300500000 MaxFreq=1300500000 CurrentFreq=1300500000\n"
                )
            ),
        ],
    )

    ensure_sonic_max_performance()

    assert run.call_count == 2
    assert run.call_args_list[1].args[0] == [
        "sudo",
        "-n",
        "/usr/bin/jetson_clocks",
        "--show",
    ]


def test_max_performance_check_explains_sudo_requirement(mocker) -> None:
    mocker.patch(
        "subprocess.run",
        side_effect=[
            SimpleNamespace(stdout="NV Power Mode: MAXN\n0\n"),
            subprocess.CalledProcessError(1, ["sudo", "-n", "/usr/bin/jetson_clocks", "--show"]),
        ],
    )

    with pytest.raises(RuntimeError, match="sudo -v"):
        ensure_sonic_max_performance()


def test_max_performance_check_rejects_unlocked_clocks(mocker) -> None:
    mocker.patch(
        "subprocess.run",
        side_effect=[
            SimpleNamespace(stdout="NV Power Mode: MAXN\n0\n"),
            SimpleNamespace(
                stdout=(
                    "cpu0: Online=1 MinFreq=115200 MaxFreq=2201600 CurrentFreq=729600\n"
                    "GPU MinFreq=306000000 MaxFreq=1300500000 CurrentFreq=306000000\n"
                )
            ),
        ],
    )

    with pytest.raises(RuntimeError, match="locked Jetson clocks"):
        ensure_sonic_max_performance()
