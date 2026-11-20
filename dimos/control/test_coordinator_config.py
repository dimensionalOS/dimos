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

from collections.abc import Callable, Iterator
import json
from pathlib import Path
import sys
from types import SimpleNamespace

from pydantic import TypeAdapter
import pytest
from typer.testing import CliRunner

from dimos.cli.dimos import main
from dimos.control.components import HardwareComponent
from dimos.control.coordinator import ControlCoordinator, ControlCoordinatorConfig
from dimos.core import run_registry
from dimos.core.coordination.blueprint_config.parser import BlueprintConfigParser
from dimos.core.coordination.module_coordinator import check_module_dependencies
from dimos.core.global_config import global_config
from dimos.hardware.drive_trains import registry as drive_trains
from dimos.hardware.manipulators import registry as manipulators
from dimos.robot.manipulators.xarm.config import make_xarm_hardware, xarm7_hardware


def test_hardware_configuration_replaces_the_full_list_with_independent_addresses(
    tmp_path: Path,
) -> None:
    hardware = [
        make_xarm_hardware("left", 6, adapter_type="xarm", address="192.0.2.10"),
        make_xarm_hardware("right", 6, adapter_type="xarm", address="192.0.2.11"),
    ]
    path = tmp_path / "hardware.json"
    path.write_text(
        json.dumps(
            {
                "controlcoordinator": {
                    "hardware": TypeAdapter(list[HardwareComponent]).dump_python(
                        hardware, mode="json"
                    )
                }
            }
        )
    )
    blueprint = ControlCoordinator.blueprint(hardware=[make_xarm_hardware("arm", 7)])

    parsed = BlueprintConfigParser(blueprint).parse(config_path=path, environ={})
    config = ControlCoordinatorConfig(**parsed.module_kwargs("controlcoordinator"))

    assert config.hardware == hardware
    assert [component.hardware_id for component in config.hardware] == ["left", "right"]


@pytest.fixture
def make_coordinator(
    monkeypatch: pytest.MonkeyPatch,
) -> Iterator[Callable[[str], ControlCoordinator]]:
    """A coordinator built in the given execution mode; modules snapshot the global config."""
    instances: list[ControlCoordinator] = []

    def make(simulation: str) -> ControlCoordinator:
        monkeypatch.setattr(global_config, "simulation", simulation)
        instance = ControlCoordinator(publish_joint_state=False, addresses={"arm": "192.0.2.10"})
        instances.append(instance)
        return instance

    yield make
    for instance in instances:
        instance.stop()


def test_addresses_select_the_physical_adapter_by_hardware_id(
    make_coordinator: Callable[[str], ControlCoordinator],
) -> None:
    coordinator = make_coordinator("")
    component = xarm7_hardware("arm", gripper=True, mock_without_address=True)
    assert (component.adapter_type, component.address) == ("xarm", None)

    resolved = coordinator._resolve_component(component)

    assert (resolved.adapter_type, resolved.address, resolved.limits) == (
        "xarm",
        "192.0.2.10",
        None,
    )
    assert resolved.adapter_kwargs["arm_dof"] == 7


def test_a_component_without_an_address_runs_its_mock_stand_in(
    make_coordinator: Callable[[str], ControlCoordinator],
) -> None:
    coordinator = make_coordinator("")
    component = xarm7_hardware("other", gripper=True, mock_without_address=True)

    resolved = coordinator._resolve_component(component)

    assert (resolved.adapter_type, resolved.address) == ("mock", None)
    assert resolved.limits == component.mock_limits
    assert resolved.limits is not None and resolved.limits.position_upper[-1] == 850.0
    # Without a stand-in the physical adapter keeps its missing address.
    needs_address = coordinator._resolve_component(xarm7_hardware("other"))
    assert (needs_address.adapter_type, needs_address.address) == ("xarm", None)


def test_simulation_keeps_the_simulated_adapter_and_takes_the_sim_modules_scene(
    make_coordinator: Callable[[str], ControlCoordinator],
) -> None:
    coordinator = make_coordinator("mujoco")
    component = xarm7_hardware("arm", gripper=True, mock_without_address=True)
    assert (component.adapter_type, component.address) == ("sim_mujoco", None)

    with pytest.raises(RuntimeError, match="no MujocoSimModule"):
        coordinator._resolve_component(component)
    coordinator._mujoco_sim = SimpleNamespace(scene_path=lambda: "/scenes/table.xml")
    resolved = coordinator._resolve_component(component)

    assert (resolved.adapter_type, resolved.address) == ("sim_mujoco", "/scenes/table.xml")
    explicit = coordinator._resolve_component(
        make_xarm_hardware("arm", 7, adapter_type="sim_mujoco", address="/own.xml")
    )
    assert explicit.address == "/own.xml"


@pytest.fixture
def missing_sdk(monkeypatch: pytest.MonkeyPatch, tmp_path: Path) -> Callable[[str, str], None]:
    """Make a real adapter import encounter a missing vendor package on any host."""
    monkeypatch.syspath_prepend(str(tmp_path))

    def hide(package: str, adapter_module: str) -> None:
        for name in list(sys.modules):
            if name.partition(".")[0] == package or name == adapter_module:
                monkeypatch.delitem(sys.modules, name)
        (tmp_path / f"{package}.py").write_text(
            f"raise ModuleNotFoundError('No module named {package}', name=__name__)"
        )

    return hide


def test_go2_keyboard_reports_adapter_sdk_before_allocating(
    missing_sdk: Callable[[str, str], None], monkeypatch: pytest.MonkeyPatch
) -> None:
    missing_sdk("unitree_sdk2py", "dimos.hardware.drive_trains.unitree_go2.adapter")
    monkeypatch.setattr(
        drive_trains, "twist_base_adapter_registry", drive_trains.TwistBaseAdapterRegistry()
    )
    monkeypatch.setattr(run_registry, "cleanup_stale", lambda: pytest.fail("allocated a run"))

    result = CliRunner().invoke(
        main, ["--viewer", "none", "--replay", "run", "unitree-go2-keyboard-teleop"]
    )

    assert result.exit_code == 1
    assert "dimos[unitree-dds]" in result.output


def test_preflight_uses_the_configured_adapter_or_its_mock(
    missing_sdk: Callable[[str, str], None], monkeypatch: pytest.MonkeyPatch
) -> None:
    missing_sdk("xarm", "dimos.hardware.manipulators.xarm.adapter")
    monkeypatch.setattr(manipulators, "adapter_registry", manipulators.AdapterRegistry())
    hardware = [make_xarm_hardware("arm", 7, adapter_type="xarm", mock_without_address=True)]
    blueprint = ControlCoordinator.blueprint(hardware=hardware, addresses={"arm": "configured"})
    parser = BlueprintConfigParser(blueprint)
    # Simulation ignores the saved hardware address and selects the SDK-free stand-in.
    check_module_dependencies(
        blueprint, parser.parse(global_overrides={"simulation": "mujoco"}, environ={})
    )
    with pytest.raises(ImportError, match=r"dimos\[control\]"):
        check_module_dependencies(
            blueprint, parser.parse(global_overrides={"simulation": ""}, environ={})
        )
