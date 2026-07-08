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

from __future__ import annotations

from collections.abc import Sequence
from pathlib import Path
from typing import Any

import pytest

from dimos.control.coordinator import ControlCoordinator, TaskConfig
from dimos.core.coordination.blueprints import Blueprint
from dimos.manipulation.manipulation_module import ManipulationModule
from dimos.robot.cli.dimos import load_config_args
from dimos.robot.manipulators.openarm.blueprints import teleop
from dimos.teleop.openarm_mini.config import OPENARM_MINI_DEFAULT_BAUDRATE
from dimos.teleop.openarm_mini.teleop_module import (
    OpenArmMiniTeleopModule,
    OpenArmMiniTeleopModuleConfig,
)


def _module_kwargs(blueprint: Blueprint, module_type: type) -> dict[str, Any]:
    return next(atom.kwargs for atom in blueprint.blueprints if atom.module is module_type)


def _module_types(blueprint: Blueprint) -> list[type]:
    return [atom.module for atom in blueprint.blueprints]


def _teleop_config_after_cli_override(
    blueprint: Blueprint,
    overrides: Sequence[str],
) -> OpenArmMiniTeleopModuleConfig:
    config_args = load_config_args(
        blueprint.config(),
        overrides,
        Path("/tmp/nonexistent-dimos-config.json"),
    )
    module_kwargs = _module_kwargs(blueprint, OpenArmMiniTeleopModule).copy()
    module_kwargs.update(config_args[OpenArmMiniTeleopModule.name])
    return OpenArmMiniTeleopModuleConfig(**module_kwargs)


@pytest.mark.parametrize(
    ("blueprint", "enabled_sides", "hardware_ids"),
    [
        pytest.param(
            teleop.openarm_mini_left_teleop_viser,
            ("left",),
            ["left_arm"],
            id="left",
        ),
        pytest.param(
            teleop.openarm_mini_right_teleop_viser,
            ("right",),
            ["right_arm"],
            id="right",
        ),
        pytest.param(
            teleop.openarm_mini_dual_teleop_viser,
            ("left", "right"),
            ["left_arm", "right_arm"],
            id="dual",
        ),
    ],
)
def test_openarm_mini_viser_blueprints_use_teleop_coordinator_and_manipulation(
    blueprint: Blueprint,
    enabled_sides: tuple[str, ...],
    hardware_ids: list[str],
) -> None:
    assert _module_types(blueprint) == [
        OpenArmMiniTeleopModule,
        ControlCoordinator,
        ManipulationModule,
    ]

    teleop_kwargs = _module_kwargs(blueprint, OpenArmMiniTeleopModule)
    teleop_config = OpenArmMiniTeleopModuleConfig(**teleop_kwargs)
    assert isinstance(teleop_config, OpenArmMiniTeleopModuleConfig)
    assert teleop_config.enabled_sides == enabled_sides

    coordinator_kwargs = _module_kwargs(blueprint, ControlCoordinator)
    hardware = coordinator_kwargs["hardware"]
    assert [component.hardware_id for component in hardware] == hardware_ids
    assert [component.adapter_type for component in hardware] == ["mock"] * len(hardware_ids)

    tasks = coordinator_kwargs["tasks"]
    assert all(isinstance(task, TaskConfig) for task in tasks)
    assert [task.type for task in tasks] == ["servo"] * len(hardware_ids)
    assert [task.joint_names for task in tasks] == [component.joints for component in hardware]

    manipulation_kwargs = _module_kwargs(blueprint, ManipulationModule)
    assert manipulation_kwargs["visualization"] == {"backend": "viser"}
    assert [robot.name for robot in manipulation_kwargs["robots"]] == hardware_ids


def test_openarm_mini_real_follower_blueprint_is_not_registered() -> None:
    assert not hasattr(teleop, "openarm_mini_teleop_openarm")


def test_right_openarm_mini_cli_port_override_preserves_right_side_default() -> None:
    config = _teleop_config_after_cli_override(
        teleop.openarm_mini_right_teleop_viser,
        [
            "openarmminiteleopmodule.port_right=/dev/ttyACM0",
        ],
    )

    assert isinstance(config, OpenArmMiniTeleopModuleConfig)
    assert config.enabled_sides == ("right",)
    assert config.port_right == "/dev/ttyACM0"
    assert config.connection_baudrate() == OPENARM_MINI_DEFAULT_BAUDRATE


def test_dual_openarm_mini_cli_port_override_preserves_dual_side_default() -> None:
    config = _teleop_config_after_cli_override(
        teleop.openarm_mini_dual_teleop_viser,
        [
            "openarmminiteleopmodule.port_right=/dev/ttyACM0",
        ],
    )

    assert isinstance(config, OpenArmMiniTeleopModuleConfig)
    assert config.enabled_sides == ("left", "right")
    assert config.port_right == "/dev/ttyACM0"
    assert config.connection_baudrate() == OPENARM_MINI_DEFAULT_BAUDRATE
