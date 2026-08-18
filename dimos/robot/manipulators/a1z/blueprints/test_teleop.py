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

from typing import Any, cast

import pytest

from dimos.control.coordinator import TaskConfig
from dimos.control.teleop_coordinator import TeleopControlCoordinator
from dimos.core.coordination.blueprints import Blueprint
from dimos.core.global_config import global_config
from dimos.robot.manipulators.a1z.blueprints.teleop import coordinator_teleop_a1z
from dimos.robot.manipulators.a1z.config import a1z_hardware
from dimos.teleop.quest.blueprints import teleop_quest_a1z


def _coordinator_kwargs(blueprint: Blueprint) -> dict[str, Any]:
    return next(
        atom.kwargs for atom in blueprint.blueprints if atom.module is TeleopControlCoordinator
    )


def test_quest_teleop_uses_mock_a1z_hardware_and_gripper_by_default() -> None:
    kwargs = _coordinator_kwargs(coordinator_teleop_a1z)
    hardware = kwargs["hardware"][0]
    tasks = cast("list[TaskConfig]", kwargs["tasks"])
    teleop = next(task for task in tasks if task.name == "teleop_a1z")

    assert hardware.adapter_type == "mock"
    assert hardware.address is None
    assert hardware.gripper_joints == ["arm/gripper"]
    assert hardware.gripper_open_position == pytest.approx(0.1)
    assert hardware.gripper_closed_position == pytest.approx(0.0)
    binding = teleop.params["bindings"][0]
    assert binding == {
        "hand": "left",
        "target_frame": "gripper_eef_link",
        "gripper_joint": "arm/gripper",
        "gripper_open_position": 1.0,
        "gripper_closed_position": 0.0,
    }
    assert teleop.params["max_joint_velocity_rad_s"] == pytest.approx(2.0)


def test_quest_left_controller_routes_to_a1z_teleop() -> None:
    assert teleop_quest_a1z.remapping_map == {
        ("armteleopmodule", "left_controller_output"): "left_cartesian_command"
    }


def test_a1z_hardware_uses_mock_adapter_in_simulation(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setattr(global_config, "can_port", "a1zcan")
    monkeypatch.setattr(global_config, "simulation", "mujoco")

    hardware = a1z_hardware("arm")

    assert hardware.adapter_type == "mock"
    assert hardware.address is None
    assert hardware.adapter_kwargs == {}
    assert hardware.gripper_joints == ["arm/gripper"]


def test_a1z_hardware_uses_real_adapter_when_can_port_is_selected(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.setattr(global_config, "can_port", "a1zcan")
    monkeypatch.setattr(global_config, "simulation", "")

    hardware = a1z_hardware("arm")

    assert hardware.adapter_type == "galaxea_a1z"
    assert hardware.address == "a1zcan"
    assert hardware.gripper_joints == ["arm/gripper"]
