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

"""Metal configuration uses explicit connection selectors and lazy geometry."""

import pytest

from dimos.control.coordinator import ControlCoordinator
from dimos.core.coordination.blueprint_config.parser import BlueprintConfigParser
from dimos.robot.manipulators.makermods_metal.blueprints.basic import coordinator_makermods_metal
from dimos.robot.manipulators.makermods_metal.config import METAL_SOURCE
from dimos.robot.manipulators.makermods_metal.connection import (
    MetalConnection,
    MetalConnectionConfig,
)


def test_connection_model_does_not_materialize_assets(mocker) -> None:
    load = mocker.patch("dimos.robot.assets.model.RobotModel.load", side_effect=AssertionError)
    config = MetalConnectionConfig(can_port="test-can")
    assert config.model is not None
    assert METAL_SOURCE.ref == "bccf3c82a21453a7b1784a94966456e16ff0f008"
    load.assert_not_called()


def test_coordinator_uses_connection_and_registers_both_tasks() -> None:
    atoms = coordinator_makermods_metal.active_blueprints
    coordinator = next(atom for atom in atoms if issubclass(atom.module, ControlCoordinator))
    assert not coordinator.kwargs.get("hardware")
    assert [task.name for task in coordinator.kwargs["tasks"]] == [
        "joint_trajectory",
        "metal_gripper",
    ]
    connection = next(atom for atom in atoms if atom.module is MetalConnection)
    assert connection.kwargs["source"] == "arm"


def test_cli_only_requires_can_selector() -> None:
    parsed = BlueprintConfigParser(coordinator_makermods_metal).parse(
        ["--can-port", "test-can"], environ={}
    )
    config = parsed.module_kwargs("metalconnection")
    assert config["can_port"] == "test-can"
    assert "gripper_opening_direction" not in config
    assert "gripper_current" not in config


@pytest.mark.parametrize("selector", ["", " "])
def test_physical_connection_rejects_empty_selector(selector: str) -> None:
    with pytest.raises(ValueError, match="must not be empty"):
        MetalConnectionConfig(can_port=selector)
