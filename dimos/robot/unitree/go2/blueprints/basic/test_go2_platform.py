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

from collections.abc import Iterator
from pathlib import Path

import pytest

from dimos.control.components import HardwareComponent, HardwareType
from dimos.control.coordinator import ControlCoordinator
from dimos.control.tasks.go2_velocity_policy_task.go2_velocity_policy_task import (
    GO2_JOINT_SUFFIXES,
    GO2_POLICY_KD,
    GO2_POLICY_KP,
)
from dimos.core.coordination.blueprints import Blueprint
from dimos.core.global_config import global_config
from dimos.robot.unitree.go2.blueprints.basic import go2_platform
from dimos.simulation.providers import (
    SimulationBinding,
    SimulationFeature,
    SimulationRequest,
)


@pytest.fixture
def pimsim_go2_config(monkeypatch: pytest.MonkeyPatch) -> Iterator[None]:
    monkeypatch.setattr(global_config, "simulation", "mujoco")
    monkeypatch.setattr(global_config, "simulation_provider", "pimsim")
    monkeypatch.setattr(global_config, "scene_package", "dimsim-apartment")
    go2_platform._resolve_simulation_binding.cache_clear()
    yield
    go2_platform._resolve_simulation_binding.cache_clear()


def test_go2_platform_uses_requested_simulation_provider(
    pimsim_go2_config: None,
    mocker,
) -> None:
    del pimsim_go2_config
    backend = Blueprint(blueprints=())
    joints = [f"go2/{suffix}" for suffix in GO2_JOINT_SUFFIXES]
    binding = SimulationBinding(
        backend=backend,
        hardware=(
            HardwareComponent(
                hardware_id="go2",
                hardware_type=HardwareType.WHOLE_BODY,
                joints=joints,
                adapter_type="sim_mujoco",
                address=Path("/tmp/pimsim-go2"),
            ),
        ),
        rerun_config={"static": {"world/scene": "scene"}},
    )
    provider = mocker.Mock()
    provider.build.return_value = binding
    load_provider = mocker.patch.object(
        go2_platform,
        "load_simulation_provider",
        return_value=provider,
    )

    platform = go2_platform.resolve_go2_platform()
    coordinator = next(
        atom for atom in platform.active_blueprints if atom.module is ControlCoordinator
    )
    hardware = coordinator.kwargs["hardware"]
    tasks = coordinator.kwargs["tasks"]
    assert len(hardware) == 1
    assert hardware[0].wb_config is not None
    assert hardware[0].wb_config.kp == GO2_POLICY_KP
    assert hardware[0].wb_config.kd == GO2_POLICY_KD
    assert len(tasks) == 1
    assert tasks[0].type == "go2_velocity_policy"
    assert tasks[0].joint_names == joints
    assert platform.remapping_map[("ControlCoordinator", "twist_command")] == "cmd_vel"
    assert go2_platform.resolve_go2_rerun_config() == binding.rerun_config
    load_provider.assert_called_once_with("pimsim")
    provider.build.assert_called_once_with(
        SimulationRequest(
            robot_model="unitree_go2",
            model_path=go2_platform._GO2_MJCF_PATH,
            mesh_dir=go2_platform._GO2_MESH_DIR,
            scene_package="dimsim-apartment",
            features=frozenset(
                {
                    SimulationFeature.SENSORS,
                    SimulationFeature.EPISODE_CONTROL,
                }
            ),
        )
    )
