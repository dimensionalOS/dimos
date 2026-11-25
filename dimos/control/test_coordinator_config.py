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

from pydantic import TypeAdapter
import pytest

from dimos.control.components import HardwareComponent, HardwareType
from dimos.control.coordinator import ControlCoordinator, ControlCoordinatorConfig
from dimos.core.coordination.blueprint_config.parser import BlueprintConfigParser
from dimos.core.global_config import global_config
from dimos.hardware.spec import JointLimits


def _arm(
    hw_id: str,
    *,
    adapter_type: str = "piper",
    address: str | None = None,
    mock_stand_in: bool = False,
) -> HardwareComponent:
    """A seven-joint arm; with ``mock_stand_in``, a mock runs it when it has no address."""
    limits = JointLimits(
        position_lower=[-1.0] * 7, position_upper=[1.0] * 7, velocity_max=[1.0] * 7
    )
    return HardwareComponent(
        hardware_id=hw_id,
        hardware_type=HardwareType.MANIPULATOR,
        joints=[f"{hw_id}/joint{i}" for i in range(1, 8)],
        adapter_type=adapter_type,
        address=address,
        mock_limits=limits if mock_stand_in else None,
    )


def test_hardware_configuration_replaces_the_full_list_with_independent_addresses(
    tmp_path: Path,
) -> None:
    hardware = [
        _arm("left", address="192.0.2.10"),
        _arm("right", address="192.0.2.11"),
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
    blueprint = ControlCoordinator.blueprint(hardware=[_arm("arm", adapter_type="mock")])

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
    component = _arm("arm", mock_stand_in=True)

    resolved = coordinator._resolve_component(component)

    assert (resolved.adapter_type, resolved.address, resolved.limits) == (
        "piper",
        "192.0.2.10",
        None,
    )


def test_a_component_without_an_address_runs_its_mock_stand_in(
    make_coordinator: Callable[[str], ControlCoordinator],
) -> None:
    coordinator = make_coordinator("")
    component = _arm("other", mock_stand_in=True)

    resolved = coordinator._resolve_component(component)

    assert (resolved.adapter_type, resolved.address) == ("mock", None)
    assert resolved.limits == component.mock_limits
    # Without a stand-in the physical adapter keeps its missing address.
    needs_address = coordinator._resolve_component(_arm("other"))
    assert (needs_address.adapter_type, needs_address.address) == ("piper", None)
