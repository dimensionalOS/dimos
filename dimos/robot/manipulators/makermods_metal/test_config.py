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

import subprocess
import sys

import pytest

from dimos.control.hardware_interface import ConnectedWholeBody
from dimos.core.coordination.blueprint_config.parser import BlueprintConfigParser
from dimos.hardware.manipulators.spec import ControlMode
from dimos.hardware.whole_body.damiao.config import DamiaoRuntimeConfig
from dimos.hardware.whole_body.registry import whole_body_adapter_registry
from dimos.robot.manipulators.makermods_metal.blueprints.basic import coordinator_makermods_metal
from dimos.robot.manipulators.makermods_metal.config import (
    METAL_JOINTS,
    METAL_SOURCE,
    metal_hardware,
)
from dimos.robot.manipulators.makermods_metal.coordinator import MetalCoordinator


def test_mock_blueprint_imports_without_motor_sdk_or_asset_checkout() -> None:
    result = subprocess.run(
        [
            sys.executable,
            "-c",
            """
import sys
sys.modules['can_motor_control'] = None
sys.modules['motorbridge'] = None
from dimos.robot.manipulators.makermods_metal.blueprints.basic import coordinator_makermods_metal
from dimos.robot.manipulators.makermods_metal.config import metal_hardware
assert metal_hardware().adapter_type == 'mock_whole_body'
""",
        ],
        capture_output=True,
        text=True,
        check=False,
    )
    assert result.returncode == 0, result.stderr


def test_mock_gripper_command_holds_arm_state() -> None:
    component = metal_hardware()
    adapter = whole_body_adapter_registry.create(component.adapter_type, dof=7)
    try:
        assert adapter.connect()
        hardware = ConnectedWholeBody(adapter, component)
        assert hardware.write_command({"joint2": -0.5, "arm/gripper": 0.75}, ControlMode.POSITION)
        assert [state.position for state in hardware.read_state().values()] == [
            0.0,
            -0.5,
            0.0,
            0.0,
            0.0,
            0.0,
            0.75,
        ]
    finally:
        adapter.disconnect()


def test_physical_component_uses_common_runtime_config() -> None:
    component = metal_hardware(
        can_port="test-can",
        gripper_opening_direction="increasing_position",
        gripper_current=0.1,
    )
    assert component.adapter_type == "makermods_metal"
    assert component.address is None
    assert component.joints == list(METAL_JOINTS)
    runtime = component.adapter_kwargs["runtime_config"]
    assert isinstance(runtime, DamiaoRuntimeConfig)
    assert runtime.bus_devices == {"metal": "test-can"}
    assert not runtime.gravity_comp
    assert METAL_SOURCE.ref == "bccf3c82a21453a7b1784a94966456e16ff0f008"


def test_physical_requires_explicit_gripper_settings() -> None:
    with pytest.raises(ValueError, match="requires gripper"):
        metal_hardware(can_port="test-can")
    with pytest.raises(ValueError, match="must not be empty"):
        metal_hardware(can_port="")


def test_coordinator_uses_mock_and_registers_both_tasks() -> None:
    parsed = BlueprintConfigParser(coordinator_makermods_metal).parse([], environ={})
    coordinator = MetalCoordinator(**parsed.module_kwargs("ControlCoordinator"))
    try:
        coordinator._setup_from_config()
        assert list(coordinator._hardware["metal"].read_state()) == list(METAL_JOINTS)
        assert list(coordinator._tasks) == ["joint_trajectory", "metal_gripper"]
    finally:
        coordinator.stop()


def test_cli_exposes_common_can_and_explicit_gripper_settings() -> None:
    parsed = BlueprintConfigParser(coordinator_makermods_metal).parse(
        [
            "--can-port",
            "test-can",
            "--gripper-opening-direction",
            "increasing_position",
            "--gripper-current",
            "0.1",
        ],
        environ={},
    )
    config = parsed.module_kwargs("ControlCoordinator")
    assert config["can_port"] == "test-can"
    assert config["gripper_opening_direction"] == "increasing_position"
    assert config["gripper_current"] == 0.1
