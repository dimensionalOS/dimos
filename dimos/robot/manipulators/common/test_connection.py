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

"""Connection selection through real blueprints and the deployment configuration parser."""

from dataclasses import replace

import pytest

from dimos.control.coordinator import ControlCoordinator
from dimos.control.teleop_coordinator import TeleopControlCoordinator
from dimos.core.coordination.blueprint_config.errors import BlueprintConfigError
from dimos.core.coordination.blueprint_config.parser import BlueprintConfigParser
from dimos.core.coordination.blueprints import autoconnect
from dimos.core.global_config import GlobalConfig, global_config
from dimos.hardware.whole_body.damiao.config import DamiaoRuntimeConfig
from dimos.robot.manipulators.a1z.blueprints.basic import coordinator_a1z
from dimos.robot.manipulators.a750.blueprints.teleop import keyboard_teleop_a750
from dimos.robot.manipulators.common.connection import SingleArmConnectionConfig, resolve_connection
from dimos.robot.manipulators.common.coordinators import ArmTwistCoordinator
from dimos.robot.manipulators.common.mixed import coordinator_piper_xarm
from dimos.robot.manipulators.common.sim import mujoco_if_sim
from dimos.robot.manipulators.dual_openyam.blueprints.basic import coordinator_dual_openyam
from dimos.robot.manipulators.openarm.blueprints.basic import coordinator_openarm
from dimos.robot.manipulators.openyam.blueprints.basic import coordinator_openyam
from dimos.robot.manipulators.openyam.config import openyam_hardware
from dimos.robot.manipulators.piper.blueprints.basic import coordinator_piper
from dimos.robot.manipulators.piper.config import piper_hardware
from dimos.robot.manipulators.xarm.blueprints.basic import (
    coordinator_dual_xarm,
    coordinator_xarm6,
    coordinator_xarm7,
)
from dimos.robot.manipulators.xarm.config import XARM7_SIM_PATH, xarm6_hardware, xarm7_hardware
from dimos.utils.data import LfsPath


@pytest.fixture
def resolve(mocker, tmp_path):
    # Resolve real blueprint configuration without downloading assets or opening devices.
    mocker.patch.object(LfsPath, "_ensure_downloaded", return_value=tmp_path / "model")
    mocker.patch.object(ControlCoordinator, "_setup_hardware")

    def resolve_blueprint(blueprint, tokens=()):
        parsed = BlueprintConfigParser(blueprint).parse(tokens, environ={})
        atom = next(
            a for a in blueprint.active_blueprints if issubclass(a.module, ControlCoordinator)
        )
        kwargs = parsed.module_kwargs(atom.name)
        kwargs.update(tasks=[], publish_joint_state=False, g=GlobalConfig(**parsed.global_config))
        with atom.module(**kwargs) as coordinator:
            return coordinator.config.hardware

    return resolve_blueprint


@pytest.mark.parametrize(
    "blueprint,fields,adapters",
    [
        (coordinator_xarm6, ["address"], ["xarm"]),
        (coordinator_xarm7, ["address"], ["xarm"]),
        (coordinator_piper, ["address"], ["piper"]),
        (coordinator_a1z, ["address"], ["galaxea_a1z"]),
        (keyboard_teleop_a750, ["address"], ["a750"]),
        (coordinator_openyam, ["address"], ["openyam_damiao"]),
        (coordinator_dual_xarm, ["left-address", "right-address"], ["xarm", "xarm"]),
        (coordinator_piper_xarm, ["xarm-address", "piper-address"], ["xarm", "piper"]),
        (coordinator_openarm, ["left-can-port", "right-can-port"], ["openarm_damiao"]),
        (coordinator_dual_openyam, ["left-can-port", "right-can-port"], ["dual_openyam_damiao"]),
    ],
)
def test_complete_endpoints_select_physical_and_defaults_stay_mock(
    resolve, blueprint, fields, adapters
):
    tokens = [
        token
        for index, field in enumerate(fields)
        for token in (f"--connection.{field}", f"device{index}")
    ]
    physical = resolve(blueprint, tokens)
    mock = resolve(blueprint)
    assert [hw.adapter_type for hw in physical] == adapters
    assert all(hw.adapter_type in ("mock", "mock_whole_body") for hw in mock)
    assert [hw.joints for hw in physical] == [hw.joints for hw in mock]


@pytest.mark.parametrize(
    "blueprint,fields",
    [
        (coordinator_dual_xarm, ["left-address", "right-address"]),
        (coordinator_piper_xarm, ["xarm-address", "piper-address"]),
        (coordinator_openarm, ["left-can-port", "right-can-port"]),
        (coordinator_dual_openyam, ["left-can-port", "right-can-port"]),
    ],
)
def test_partial_assemblies_are_rejected(resolve, blueprint, fields):
    for field in fields:
        with pytest.raises(BlueprintConfigError, match="both"):
            resolve(blueprint, [f"--connection.{field}", "device"])


@pytest.mark.parametrize("address", ["", " ", " can0", "can0 "])
def test_invalid_address_is_not_mock(resolve, address):
    with pytest.raises(BlueprintConfigError, match="Device address"):
        resolve(coordinator_xarm7, ["--connection.address", address])


def test_address_precedence_and_instance_isolation():
    blueprint = autoconnect(
        *[
            ControlCoordinator.blueprint(
                instance_name=name, connection=SingleArmConnectionConfig(backend="xarm7")
            )
            for name in ("left", "right")
        ]
    )
    parser = BlueprintConfigParser(blueprint)
    env = {"LEFT__CONNECTION__ADDRESS": "env"}
    overrides = {"left": {"connection": {"address": "override"}}}
    assert parser.parse(environ=env).module_kwargs("left")["connection"]["address"] == "env"
    assert (
        parser.parse(environ=env, overrides=overrides).module_kwargs("left")["connection"][
            "address"
        ]
        == "override"
    )
    parsed = parser.parse(["--left.connection.address", "cli"], environ=env, overrides=overrides)
    assert parsed.module_kwargs("left")["connection"]["address"] == "cli"
    assert parsed.module_kwargs("right")["connection"]["address"] is None
    with pytest.raises(BlueprintConfigError, match="ambiguous"):
        parser.parse(["--connection.address", "device"], environ={})


@pytest.mark.parametrize(
    "blueprint,adapter",
    [
        (coordinator_xarm7, "sim_mujoco"),
        (coordinator_piper, "sim_mujoco"),
        (coordinator_a1z, "mock"),
        (coordinator_openyam, "mock_whole_body"),
    ],
)
def test_existing_simulation_selection(resolve, blueprint, adapter):
    hardware = resolve(blueprint, ["--simulation", "mujoco", "--connection.address", "physical"])
    assert hardware[0].adapter_type == adapter
    assert hardware[0].address != "physical"


def test_simulator_and_adapter_use_same_address(resolve, monkeypatch):
    monkeypatch.setattr(global_config, "simulation", "mujoco")
    simulation = mujoco_if_sim(XARM7_SIM_PATH, 7)
    hardware = resolve(autoconnect(coordinator_xarm7, *simulation), ["--simulation", "mujoco"])
    assert hardware[0].address == simulation[0].active_blueprints[0].kwargs["address"]


@pytest.mark.parametrize(
    "coordinator_type", [ControlCoordinator, ArmTwistCoordinator, TeleopControlCoordinator]
)
def test_connection_config_composes_with_control_modes(coordinator_type):
    with coordinator_type(
        connection=SingleArmConnectionConfig(backend="xarm7"),
        hardware=[xarm7_hardware(gripper=True)],
        publish_joint_state=False,
        g=GlobalConfig(simulation=""),
    ) as coordinator:
        hardware = coordinator._hardware["arm"]
        assert hardware.component.adapter_type == "mock"
        assert len(hardware.adapter.read_joint_positions()) == 8


def test_hardware_settings_are_preserved(resolve):
    template = xarm7_hardware(gripper=True)
    template.auto_enable = False
    template.adapter_kwargs["initial_positions"] = [0.1] * 8
    selected = resolve_connection(
        SingleArmConnectionConfig(backend="xarm7", address="device"), [template], ""
    )[0]
    assert selected == replace(
        template,
        adapter_type="xarm",
        address="device",
        limits=None,
        adapter_kwargs={**template.adapter_kwargs, "arm_dof": 7},
    )
    assert template.adapter_type == "mock"
    assert template.address is None
    explicit = ControlCoordinator.blueprint(hardware=[selected])
    assert resolve(explicit, ["--simulation", "mujoco"]) == [selected]


def test_can_endpoint_override_preserves_runtime_settings():
    template = openyam_hardware()
    runtime = DamiaoRuntimeConfig(
        bus_devices={"openyam": "old"}, gravity_comp=False, tick_deadline_us=2000
    )
    template.adapter_kwargs["runtime_config"] = runtime
    selected = resolve_connection(
        SingleArmConnectionConfig(backend="openyam", address="can1"), [template], ""
    )[0]
    assert selected.adapter_kwargs["runtime_config"] == replace(
        runtime, bus_devices={"openyam": "can1"}
    )
    assert template.adapter_kwargs["runtime_config"] == runtime


@pytest.mark.parametrize(
    "backend,adapter,factory",
    [("xarm6_hardware", "xarm", xarm6_hardware), ("piper_hardware", "piper", piper_hardware)],
)
def test_hardware_only_backends_ignore_simulation(backend, adapter, factory):
    selected = resolve_connection(
        SingleArmConnectionConfig(backend=backend, address="device"), [factory()], "mujoco"
    )
    assert (selected[0].adapter_type, selected[0].address) == (adapter, "device")


@pytest.mark.parametrize(
    "option",
    [
        "--address",
        "--left-address",
        "--left-can-port",
        "--xarm6-ip",
        "--xarm7-ip",
        "--can-port",
        "--device-path",
    ],
)
def test_removed_options_are_rejected(resolve, option):
    with pytest.raises(BlueprintConfigError, match="Unknown"):
        resolve(coordinator_xarm7, [option, "device"])
