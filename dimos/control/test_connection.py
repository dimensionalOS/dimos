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

"""Contracts for typed connection configs and hardware resolution."""

from dataclasses import replace
import subprocess
import sys

from pydantic import ValidationError
import pytest

from dimos.control.components import HardwareComponent, HardwareType
from dimos.control.connection import (
    A1ZConnectionConfig,
    DualOpenYamConnectionConfig,
    DualXArmConnectionConfig,
    OpenArmConnectionConfig,
    OpenYamConnectionConfig,
    PiperConnectionConfig,
    XArmConnectionConfig,
)
from dimos.control.connection_factory import resolve_connection
from dimos.control.coordinator import ControlCoordinator
from dimos.control.teleop_coordinator import TeleopControlCoordinator
from dimos.core.global_config import GlobalConfig
from dimos.hardware.whole_body.damiao.config import DamiaoRuntimeConfig
from dimos.robot.manipulators.common.coordinators import ArmTwistCoordinator
from dimos.robot.manipulators.openyam.config import openyam_hardware
from dimos.robot.manipulators.piper.config import piper_hardware
from dimos.robot.manipulators.xarm.config import xarm7_hardware


@pytest.mark.parametrize(
    "coordinator_type",
    [
        ControlCoordinator,
        ArmTwistCoordinator,
        TeleopControlCoordinator,
    ],
)
def test_connection_config_composes_with_existing_control_modes(coordinator_type) -> None:
    with coordinator_type(
        connection=XArmConnectionConfig(),
        hardware=[xarm7_hardware(gripper=True)],
        publish_joint_state=False,
        g=GlobalConfig(simulation=""),
    ) as coordinator:
        hardware = coordinator._hardware["arm"]
        assert hardware.component.adapter_type == "mock"
        assert len(hardware.adapter.read_joint_positions()) == 8


def test_omitted_connection_preserves_explicit_hardware(mocker) -> None:
    setup = mocker.patch.object(ControlCoordinator, "_setup_hardware")
    component = xarm7_hardware(address="192.0.2.7")
    with ControlCoordinator(
        hardware=[component],
        publish_joint_state=False,
        g=GlobalConfig(simulation="mujoco"),
    ) as coordinator:
        assert coordinator.config.hardware == [component]
        setup.assert_called_once_with(component)


@pytest.mark.parametrize(
    "config,template",
    [
        (XArmConnectionConfig(backend="xarm_hardware", address="physical"), xarm7_hardware()),
        (PiperConnectionConfig(backend="piper_hardware", address="physical"), piper_hardware()),
    ],
)
def test_hardware_only_backends_preserve_blueprints_without_simulation(config, template) -> None:
    resolved = resolve_connection(config, [template], "mujoco")
    assert resolved[0].address == "physical"
    assert resolved[0].adapter_type == config.backend.removesuffix("_hardware")


def test_resolution_preserves_hardware_settings_and_does_not_mutate_template() -> None:
    template = xarm7_hardware(gripper=True)
    template.auto_enable = False
    template.domain_id = 9
    template.adapter_kwargs["home_joints"] = [0.1] * 8
    resolved = resolve_connection(XArmConnectionConfig(address="192.0.2.7"), [template], "")[0]
    assert resolved == replace(
        template,
        address="192.0.2.7",
        adapter_type="xarm",
        limits=None,
        adapter_kwargs={**template.adapter_kwargs, "arm_dof": 7},
    )
    assert template.address is None
    assert template.adapter_type == "mock"
    assert template.limits is not None


def test_piper_resolution_preserves_custom_adapter_options() -> None:
    template = piper_hardware()
    template.adapter_kwargs["home_joints"] = [0.2] * 7
    resolved = resolve_connection(PiperConnectionConfig(address="can0"), [template], "")[0]
    assert resolved.adapter_kwargs == template.adapter_kwargs


def test_a1z_resolution_preserves_blueprint_owned_adapter_configuration() -> None:
    template = HardwareComponent(
        hardware_id="arm",
        hardware_type=HardwareType.MANIPULATOR,
        adapter_kwargs={"config": {"gripper": None, "urdf_path": "custom.urdf"}},
    )
    resolved = resolve_connection(A1ZConnectionConfig(address="can0"), [template], "")[0]
    assert resolved.adapter_kwargs == template.adapter_kwargs
    assert resolved.adapter_type == "galaxea_a1z"
    assert resolved.address == "can0"


def test_openyam_address_override_preserves_runtime_options() -> None:
    template = openyam_hardware()
    template.adapter_kwargs["runtime_config"] = DamiaoRuntimeConfig(
        bus_devices={"openyam": "old"},
        gravity_comp=False,
        tick_deadline_us=2000,
    )
    resolved = resolve_connection(OpenYamConnectionConfig(address="can1"), [template], "")[0]
    assert resolved.adapter_kwargs["runtime_config"] == DamiaoRuntimeConfig(
        bus_devices={"openyam": "can1"},
        gravity_comp=False,
        tick_deadline_us=2000,
    )
    assert template.adapter_kwargs["runtime_config"].bus_devices == {"openyam": "old"}
    assert resolved.wb_config == template.wb_config


@pytest.mark.parametrize("config_type", [OpenArmConnectionConfig, DualOpenYamConnectionConfig])
def test_paired_can_backend_rejects_duplicate_interfaces(config_type) -> None:
    with pytest.raises(ValidationError, match="distinct"):
        config_type(left_can_port="can0", right_can_port="can0")


@pytest.mark.parametrize(
    "config,hardware",
    [
        (XArmConnectionConfig(), []),
        (DualXArmConnectionConfig(), [xarm7_hardware()]),
        (OpenArmConnectionConfig(), [xarm7_hardware(), xarm7_hardware()]),
    ],
)
def test_invalid_assembly_shape_fails_before_loading_robot_factory(
    mocker, config, hardware
) -> None:
    load = mocker.patch("dimos.control.connection_factory.import_module")
    with pytest.raises(ValueError, match="hardware description"):
        resolve_connection(config, hardware, "")
    load.assert_not_called()


def test_connection_schemas_do_not_import_robot_implementations() -> None:
    result = subprocess.run(
        [
            sys.executable,
            "-c",
            "import sys; import dimos.control.connection; "
            "print([name for name in sys.modules if name.startswith('dimos.robot.manipulators')])",
        ],
        check=True,
        capture_output=True,
        text=True,
    )
    assert result.stdout.strip() == "[]"
