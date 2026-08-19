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

from dataclasses import replace
from functools import cache
from pathlib import Path

from dimos.control.components import HardwareType
from dimos.control.coordinator import ControlCoordinator, TaskConfig
from dimos.control.tasks.go2_velocity_policy_task.go2_velocity_policy_task import (
    GO2_JOINT_SUFFIXES,
    GO2_POLICY_KD,
    GO2_POLICY_KP,
)
from dimos.core.coordination.blueprints import Blueprint, autoconnect
from dimos.core.global_config import global_config
from dimos.hardware.whole_body.spec import WholeBodyConfig
from dimos.robot.unitree.go2.connection import GO2Connection
from dimos.simulation.providers import (
    SimulationBinding,
    SimulationFeature,
    SimulationRequest,
    load_simulation_provider,
)
from dimos.utils.data import LfsPath

_GO2_POLICY_MODEL = LfsPath("go2_velocity_policy/policy.onnx")
_GO2_MUJOCO_ROOT = Path(__file__).resolve().parents[2] / "assets" / "mujoco"
_GO2_MJCF_PATH = _GO2_MUJOCO_ROOT / "unitree_robots" / "go2" / "go2.pimsim.xml"
_GO2_MESH_DIR = _GO2_MUJOCO_ROOT / "unitree_robots" / "go2" / "assets"


def resolve_go2_platform() -> Blueprint:
    if global_config.simulation in ("", "dimsim"):
        return GO2Connection.blueprint()
    binding = _resolve_simulation_binding()
    return autoconnect(binding.backend, _simulation_coordinator(binding))


def resolve_go2_rerun_config() -> dict[str, object]:
    if global_config.simulation in ("", "dimsim"):
        return {}
    return _resolve_simulation_binding().rerun_config


@cache
def _resolve_simulation_binding() -> SimulationBinding:
    if global_config.simulation != "mujoco":
        raise ValueError("unitree-go2 only supports --simulation mujoco")
    if not global_config.simulation_provider:
        raise ValueError("unitree-go2 simulation requires --simulation-provider pimsim")

    provider = load_simulation_provider(global_config.simulation_provider)
    return provider.build(
        SimulationRequest(
            robot_model="unitree_go2",
            model_path=_GO2_MJCF_PATH,
            mesh_dir=_GO2_MESH_DIR,
            scene_package=global_config.scene_package,
            features=frozenset(
                {
                    SimulationFeature.SENSORS,
                    *(
                        (SimulationFeature.EPISODE_CONTROL,)
                        if global_config.scene_package is not None
                        else ()
                    ),
                }
            ),
        )
    )


def _simulation_coordinator(binding: SimulationBinding) -> Blueprint:
    if len(binding.hardware) != 1:
        raise ValueError("unitree_go2 simulation requires one hardware endpoint")
    component = binding.hardware[0]
    expected_joints = [f"go2/{suffix}" for suffix in GO2_JOINT_SUFFIXES]
    if component.hardware_id != "go2" or component.hardware_type is not HardwareType.WHOLE_BODY:
        raise ValueError("unitree_go2 simulation requires the Go2 whole-body endpoint")
    if component.joints != expected_joints:
        raise ValueError("unitree_go2 simulation hardware does not use the policy joint order")

    hardware = replace(
        component,
        wb_config=WholeBodyConfig(kp=GO2_POLICY_KP, kd=GO2_POLICY_KD),
    )
    return ControlCoordinator.blueprint(
        instance_name="ControlCoordinator",
        tick_rate=50.0,
        hardware=[hardware],
        tasks=[
            TaskConfig(
                name="go2_velocity_policy",
                type="go2_velocity_policy",
                joint_names=expected_joints,
                priority=50,
                auto_start=True,
                params={
                    "model_path": _GO2_POLICY_MODEL,
                    "hardware_id": "go2",
                    "timeout": 0.5,
                },
            )
        ],
    ).remappings([(ControlCoordinator, "twist_command", "cmd_vel")])


__all__ = ["resolve_go2_platform", "resolve_go2_rerun_config"]
