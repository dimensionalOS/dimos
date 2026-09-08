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

"""Connection selection through the same parser and setup path used by deployments."""

from collections.abc import Callable, Iterator
from pathlib import Path
from typing import Any

import pytest

from dimos.control.coordinator import ControlCoordinator
from dimos.core.coordination.blueprint_config.errors import BlueprintConfigError
from dimos.core.coordination.blueprint_config.parser import BlueprintConfigParser
from dimos.core.coordination.blueprints import Blueprint, autoconnect
from dimos.core.global_config import GlobalConfig, global_config
from dimos.robot.manipulators.a1z.blueprints.basic import coordinator_a1z
from dimos.robot.manipulators.a750.blueprints.teleop import keyboard_teleop_a750
from dimos.robot.manipulators.common.mixed import coordinator_piper_xarm
from dimos.robot.manipulators.common.sim import mujoco_if_sim
from dimos.robot.manipulators.dual_openyam.blueprints.basic import coordinator_dual_openyam
from dimos.robot.manipulators.openarm.blueprints.basic import coordinator_openarm
from dimos.robot.manipulators.openyam.blueprints.basic import coordinator_openyam
from dimos.robot.manipulators.piper.blueprints.basic import coordinator_piper
from dimos.robot.manipulators.xarm.blueprints.basic import (
    coordinator_dual_xarm,
    coordinator_xarm6,
    coordinator_xarm7,
)
from dimos.robot.manipulators.xarm.config import XARM7_SIM_PATH, xarm7_hardware
from dimos.robot.manipulators.xarm.coordinator import XArm7Coordinator
from dimos.utils.data import LfsPath


@pytest.fixture(autouse=True)
def local_model_paths(mocker: Any, tmp_path: Path) -> None:
    # Connection parsing does not need robot assets. Avoid downloads while the
    # parser copies lazy model paths embedded in the real blueprint's tasks.
    mocker.patch.object(LfsPath, "_ensure_downloaded", return_value=tmp_path / "model")


@pytest.fixture
def resolve_connection(mocker: Any) -> Iterator[Callable[..., ControlCoordinator]]:
    coordinators: list[ControlCoordinator] = []
    # Resolve real configuration, but never open physical hardware or initialize IK tasks.
    mocker.patch.object(ControlCoordinator, "_setup_hardware")

    def resolve(blueprint: Blueprint, tokens: list[str], **sources: Any) -> ControlCoordinator:
        parsed = BlueprintConfigParser(blueprint).parse(tokens, **sources)
        atom = next(
            atom
            for atom in blueprint.active_blueprints
            if issubclass(atom.module, ControlCoordinator)
        )
        kwargs = parsed.module_kwargs(atom.name)
        kwargs.update(tasks=[], publish_joint_state=False, g=GlobalConfig(**parsed.global_config))
        coordinator = atom.module(**kwargs)
        coordinators.append(coordinator)
        coordinator._setup_from_config()
        return coordinator

    yield resolve

    for coordinator in coordinators:
        coordinator.stop()


@pytest.mark.parametrize(
    ("blueprint", "address", "physical_adapter", "mock_adapter"),
    [
        (coordinator_xarm6, "192.0.2.6", "xarm", "mock"),
        (coordinator_xarm7, "192.0.2.7", "xarm", "mock"),
        (coordinator_piper, "can0", "piper", "mock"),
        (coordinator_a1z, "a1zcan", "galaxea_a1z", "mock"),
        (keyboard_teleop_a750, "/dev/ttyACM0", "a750", "mock"),
        (coordinator_openyam, "can1", "openyam_damiao", "mock_whole_body"),
    ],
)
def test_local_address_selects_hardware_without_changing_blueprint_defaults(
    resolve_connection: Callable[..., ControlCoordinator],
    blueprint: Blueprint,
    address: str,
    physical_adapter: str,
    mock_adapter: str,
) -> None:
    real = resolve_connection(blueprint, ["--address", address], environ={})
    mock = resolve_connection(blueprint, [], environ={})
    assert real.config.hardware[0].adapter_type == physical_adapter
    assert mock.config.hardware[0].adapter_type == mock_adapter
    assert real.config.hardware[0].joints == mock.config.hardware[0].joints
    assert mock.config.hardware[0].address is None
    if physical_adapter == "openyam_damiao":
        assert real.config.hardware[0].adapter_kwargs["runtime_config"].bus_devices == {
            "openyam": address
        }
    else:
        assert real.config.hardware[0].address == address


@pytest.mark.parametrize(
    ("blueprint", "left_flag", "right_flag", "adapters"),
    [
        (coordinator_dual_xarm, "--left-address", "--right-address", ["xarm", "xarm"]),
        (coordinator_piper_xarm, "--xarm-address", "--piper-address", ["xarm", "piper"]),
        (coordinator_openarm, "--left-can-port", "--right-can-port", ["openarm_damiao"]),
        (coordinator_dual_openyam, "--left-can-port", "--right-can-port", ["dual_openyam_damiao"]),
    ],
)
def test_assembly_requires_all_addresses(
    resolve_connection: Callable[..., ControlCoordinator],
    blueprint: Blueprint,
    left_flag: str,
    right_flag: str,
    adapters: list[str],
) -> None:
    with pytest.raises(BlueprintConfigError, match="both"):
        BlueprintConfigParser(blueprint).parse([left_flag, "left"], environ={})
    with pytest.raises(BlueprintConfigError, match="both"):
        BlueprintConfigParser(blueprint).parse([right_flag, "right"], environ={})
    real = resolve_connection(blueprint, [left_flag, "left", right_flag, "right"], environ={})
    mock = resolve_connection(blueprint, [], environ={})
    assert [hw.adapter_type for hw in real.config.hardware] == adapters
    assert all(hw.adapter_type in ("mock", "mock_whole_body") for hw in mock.config.hardware)


@pytest.mark.parametrize("address", ["", " ", " can0", "can0 "])
def test_empty_or_padded_addresses_are_not_mock(address: str) -> None:
    with pytest.raises(BlueprintConfigError, match="Device address"):
        BlueprintConfigParser(coordinator_xarm7).parse(["--address", address], environ={})


def test_address_source_precedence_and_isolation(
    resolve_connection: Callable[..., ControlCoordinator],
) -> None:
    env = {"CONTROLCOORDINATOR__ADDRESS": "environment"}
    cli = resolve_connection(
        coordinator_xarm7,
        ["--address", "cli"],
        environ=env,
        overrides={"ControlCoordinator": {"address": "programmatic"}},
    )
    programmatic = resolve_connection(
        coordinator_xarm7,
        [],
        environ=env,
        overrides={"ControlCoordinator": {"address": "programmatic"}},
    )
    environment = resolve_connection(coordinator_xarm7, [], environ=env)
    assert [item.config.hardware[0].address for item in (cli, programmatic, environment)] == [
        "cli",
        "programmatic",
        "environment",
    ]


def test_qualified_address_disambiguates_two_module_instances() -> None:
    blueprint = autoconnect(
        XArm7Coordinator.blueprint(instance_name="left", hardware=[xarm7_hardware()]),
        XArm7Coordinator.blueprint(instance_name="right", hardware=[xarm7_hardware()]),
    )
    parser = BlueprintConfigParser(blueprint)
    with pytest.raises(BlueprintConfigError, match="[Aa]mbiguous"):
        parser.parse(["--address", "192.0.2.1"], environ={})
    parsed = parser.parse(
        ["--left.address", "192.0.2.1", "--right.address", "192.0.2.2"], environ={}
    )
    assert parsed.module_kwargs("left")["address"] == "192.0.2.1"
    assert parsed.module_kwargs("right")["address"] == "192.0.2.2"


@pytest.mark.parametrize(
    ("blueprint", "adapter"),
    [
        (coordinator_xarm7, "sim_mujoco"),
        (coordinator_piper, "sim_mujoco"),
        (coordinator_a1z, "mock"),
        (coordinator_openyam, "mock_whole_body"),
    ],
)
def test_simulation_ignores_physical_address(
    resolve_connection: Callable[..., ControlCoordinator], blueprint: Blueprint, adapter: str
) -> None:
    coordinator = resolve_connection(
        blueprint, ["--simulation", "mujoco", "--address", "physical"], environ={}
    )
    assert coordinator.config.hardware[0].adapter_type == adapter
    assert coordinator.config.hardware[0].address != "physical"


def test_xarm_simulation_uses_the_blueprint_discovery_path(
    resolve_connection: Callable[..., ControlCoordinator],
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.setattr(global_config, "simulation", "mujoco")
    simulation = mujoco_if_sim(XARM7_SIM_PATH, 7)
    blueprint = autoconnect(coordinator_xarm7, *simulation)
    coordinator = resolve_connection(
        blueprint,
        ["--simulation", "mujoco", "--controlcoordinator.address", "physical"],
        environ={},
    )
    simulator = simulation[0].active_blueprints[0]
    assert coordinator.config.hardware[0].address == simulator.kwargs["address"]
    assert coordinator.config.hardware[0].adapter_type == "sim_mujoco"


def test_default_xarm_starts_with_mock_hardware() -> None:
    with XArm7Coordinator(
        instance_name="ControlCoordinator",
        hardware=[xarm7_hardware(gripper=True)],
        publish_joint_state=False,
        g=GlobalConfig(simulation=""),
    ) as coordinator:
        hardware = coordinator._hardware["arm"]
        assert hardware.adapter.is_connected()
        assert hardware.component.adapter_type == "mock"
        assert len(hardware.adapter.read_joint_positions()) == 8


@pytest.mark.parametrize("option", ["--xarm6-ip", "--xarm7-ip", "--can-port", "--device-path"])
def test_removed_global_connection_options_are_rejected(option: str) -> None:
    with pytest.raises(BlueprintConfigError, match="Unknown"):
        BlueprintConfigParser(coordinator_xarm7).parse([option, "endpoint"], environ={})
