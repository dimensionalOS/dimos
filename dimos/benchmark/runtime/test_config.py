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

"""Tests for benchmark runtime config resolution."""

from __future__ import annotations

from pathlib import Path
import sys

import pytest

REPO_ROOT = Path(__file__).resolve().parents[3]
PROTOCOL_SRC = REPO_ROOT / "packages" / "dimos-runtime-protocol" / "src"
sys.path.insert(0, str(PROTOCOL_SRC))

from dimos_runtime_protocol import (
    CommandMode,
    MotorDescription,
    ProtocolVersion,
    RobotMotorSurface,
    RuntimeDescription,
)

from dimos.benchmark.runtime.config import (
    BenchmarkEpisodeConfig,
    LiberoProBackendOptions,
    resolve_runtime_plan,
    validate_libero_pro_backend_options,
)

CONFIG_DIR = Path(__file__).parent / "configs"


def test_resolve_runtime_plan_rejects_incompatible_protocol() -> None:
    description = _description(protocol=ProtocolVersion(version="1.0", min_compatible="1.0"))

    with pytest.raises(ValueError, match="incompatible sidecar protocol"):
        resolve_runtime_plan(BenchmarkEpisodeConfig(), description)


def test_resolve_runtime_plan_rejects_robot_profile_mismatch() -> None:
    description = _description(robot_id="otherbot")

    with pytest.raises(ValueError, match="sidecar did not report robot surface"):
        resolve_runtime_plan(BenchmarkEpisodeConfig(), description)


def test_resolve_runtime_plan_rejects_missing_position_command_mode() -> None:
    description = _description(command_modes=[CommandMode.VELOCITY])

    with pytest.raises(ValueError, match="does not support position commands"):
        resolve_runtime_plan(BenchmarkEpisodeConfig(), description)


def test_existing_configs_still_parse_after_backend_options_added() -> None:
    fake = BenchmarkEpisodeConfig.model_validate_json(
        (CONFIG_DIR / "fake_runtime_smoke.json").read_text()
    )
    robosuite = BenchmarkEpisodeConfig.model_validate_json(
        (CONFIG_DIR / "robosuite_panda_lift.json").read_text()
    )

    assert fake.backend_options == {}
    assert robosuite.backend_options == {}


def test_libero_pro_config_parses_typed_backend_options() -> None:
    config = BenchmarkEpisodeConfig.model_validate_json(
        (CONFIG_DIR / "libero_pro_goal_task0.json").read_text()
    )

    options = validate_libero_pro_backend_options(config)

    assert config.backend == "libero-pro"
    assert options == LiberoProBackendOptions(
        benchmark_name="libero_goal_task",
        task_order_index=0,
        task_index=0,
        init_state_index=0,
        controller="JOINT_POSITION",
        camera_names=["agentview"],
        horizon=1000,
        bddl_root=Path("libero/libero/bddl_files"),
        init_states_root=Path("libero/libero/init_files"),
    )


def test_libero_pro_options_reject_missing_required_assets() -> None:
    config = BenchmarkEpisodeConfig(
        backend="libero-pro",
        robot_id="panda",
        dof=8,
        backend_options={
            "benchmark_name": "libero_goal_task",
            "task_index": 0,
            "init_state_index": 0,
            "controller": "JOINT_POSITION",
        },
    )

    with pytest.raises(ValueError, match="bddl_root"):
        validate_libero_pro_backend_options(config)


def test_libero_pro_options_reject_dynamic_perturbation_mode() -> None:
    config = BenchmarkEpisodeConfig(
        backend="libero-pro",
        robot_id="panda",
        dof=8,
        backend_options={
            "benchmark_name": "libero_goal_task",
            "task_order_index": 0,
            "task_index": 0,
            "init_state_index": 0,
            "controller": "JOINT_POSITION",
            "camera_names": ["agentview"],
            "horizon": 1000,
            "bddl_root": "libero/libero/bddl_files",
            "init_states_root": "libero/libero/init_files",
            "perturbation_mode": "dynamic",
        },
    )

    with pytest.raises(ValueError, match="perturbation_mode"):
        validate_libero_pro_backend_options(config)


def _description(
    *,
    robot_id: str = "fakebot",
    protocol: ProtocolVersion | None = None,
    command_modes: list[CommandMode] | None = None,
) -> RuntimeDescription:
    return RuntimeDescription(
        runtime_id="fake-runtime",
        backend="fake",
        protocol=protocol or ProtocolVersion(),
        robot_surfaces=[
            RobotMotorSurface(
                robot_id=robot_id,
                motors=[
                    MotorDescription(name=f"{robot_id}/joint{i + 1}", index=i) for i in range(3)
                ],
                supported_command_modes=command_modes or [CommandMode.POSITION],
            )
        ],
        control_step_hz=100,
    )
