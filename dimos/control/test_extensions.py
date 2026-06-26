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

"""Tests for ControlCoordinator extension sideloading."""

from __future__ import annotations

from collections.abc import Generator
import sys
from types import ModuleType

import pytest

from dimos.control.components import HardwareComponent, HardwareType, make_twist_base_joints
from dimos.control.coordinator import ControlCoordinator, TaskConfig
from dimos.control.extensions import register_control_task, register_hardware_adapter
from dimos.control.task import BaseControlTask, CoordinatorState, JointCommandOutput, ResourceClaim
from dimos.control.tasks.registry import control_task_registry
from dimos.hardware.drive_trains.registry import twist_base_adapter_registry
from dimos.hardware.manipulators.registry import adapter_registry
from dimos.hardware.whole_body.registry import whole_body_adapter_registry


class ExternalBaseAdapter:
    def __init__(self, dof: int = 3, **kwargs: object) -> None:
        self._dof = dof
        self._connected = False
        self._enabled = False

    def connect(self) -> bool:
        self._connected = True
        return True

    def disconnect(self) -> None:
        self._connected = False

    def is_connected(self) -> bool:
        return self._connected

    def get_dof(self) -> int:
        return self._dof

    def read_velocities(self) -> list[float]:
        return [0.0] * self._dof

    def read_odometry(self) -> list[float] | None:
        return [0.0] * self._dof

    def write_velocities(self, velocities: list[float]) -> bool:
        return True

    def write_stop(self) -> bool:
        return True

    def write_enable(self, enable: bool) -> bool:
        self._enabled = enable
        return True

    def read_enabled(self) -> bool:
        return self._enabled


class ExternalControlTask(BaseControlTask):
    def __init__(self, cfg: TaskConfig, hardware: object) -> None:
        self._name = cfg.name
        self.hardware = hardware

    @property
    def name(self) -> str:
        return self._name

    def claim(self) -> ResourceClaim:
        return ResourceClaim(joints=frozenset())

    def is_active(self) -> bool:
        return False

    def compute(self, state: CoordinatorState) -> JointCommandOutput | None:
        return None


@pytest.fixture(autouse=True)
def restore_registries() -> Generator[None, None, None]:
    manipulator_adapters = adapter_registry._adapters.copy()
    base_adapters = twist_base_adapter_registry._adapters.copy()
    whole_body_adapters = whole_body_adapter_registry._adapters.copy()
    task_paths = control_task_registry._factory_paths.copy()
    task_factories = control_task_registry._factories.copy()
    try:
        yield
    finally:
        adapter_registry._adapters = manipulator_adapters
        twist_base_adapter_registry._adapters = base_adapters
        whole_body_adapter_registry._adapters = whole_body_adapters
        control_task_registry._factory_paths = task_paths
        control_task_registry._factories = task_factories


def test_register_hardware_adapter_supports_all_hardware_types() -> None:
    def manipulator_factory(**kwargs: object) -> object:
        return object()

    def base_factory(**kwargs: object) -> object:
        return object()

    def whole_body_factory(**kwargs: object) -> object:
        return object()

    register_hardware_adapter(HardwareType.MANIPULATOR, "Ext_Manipulator", manipulator_factory)
    register_hardware_adapter(HardwareType.BASE, "Ext_Base", base_factory)
    register_hardware_adapter(HardwareType.WHOLE_BODY, "Ext_Whole_Body", whole_body_factory)

    assert adapter_registry._adapters["ext_manipulator"] is manipulator_factory
    assert twist_base_adapter_registry._adapters["ext_base"] is base_factory
    assert whole_body_adapter_registry._adapters["ext_whole_body"] is whole_body_factory


def test_hardware_adapter_create_uses_same_normalization_as_register() -> None:
    class PaddedAdapter:
        pass

    register_hardware_adapter(HardwareType.MANIPULATOR, "  Padded_Manipulator  ", PaddedAdapter)
    register_hardware_adapter(HardwareType.BASE, "  Padded_Base  ", PaddedAdapter)
    register_hardware_adapter(HardwareType.WHOLE_BODY, "  Padded_Whole_Body  ", PaddedAdapter)

    assert isinstance(adapter_registry.create("  padded_manipulator  "), PaddedAdapter)
    assert isinstance(twist_base_adapter_registry.create("  padded_base  "), PaddedAdapter)
    assert isinstance(whole_body_adapter_registry.create("  padded_whole_body  "), PaddedAdapter)


@pytest.mark.parametrize(
    ("hardware_type", "registry_name"),
    [
        (HardwareType.MANIPULATOR, "manipulator"),
        (HardwareType.BASE, "base"),
        (HardwareType.WHOLE_BODY, "whole_body"),
    ],
)
def test_register_hardware_adapter_duplicate_policy_keeps_original(
    hardware_type: HardwareType,
    registry_name: str,
) -> None:
    def original_factory(**kwargs: object) -> object:
        return object()

    def different_factory(**kwargs: object) -> object:
        return object()

    adapter_name = f"external_duplicate_{hardware_type.value}"

    register_hardware_adapter(hardware_type, adapter_name, original_factory)
    register_hardware_adapter(hardware_type, adapter_name, original_factory)

    with pytest.raises(ValueError):
        register_hardware_adapter(hardware_type, adapter_name, different_factory)

    registries = {
        "manipulator": adapter_registry._adapters,
        "base": twist_base_adapter_registry._adapters,
        "whole_body": whole_body_adapter_registry._adapters,
    }
    assert registries[registry_name][adapter_name] is original_factory


def test_register_control_task_registers_lazy_path_without_importing_target(mocker) -> None:
    import_module = mocker.patch("dimos.control.tasks.registry.importlib.import_module")

    register_control_task("External_Task", "external_pkg.tasks:make_task")

    assert control_task_registry._factory_paths["external_task"] == "external_pkg.tasks:make_task"
    import_module.assert_not_called()


def test_control_task_create_uses_same_normalization_as_register(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    module_name = "external_padded_task_module"
    module = ModuleType(module_name)
    module.__dict__["make_task"] = ExternalControlTask
    monkeypatch.setitem(sys.modules, module_name, module)

    control_task_registry.register_path("  External_Padded_Task  ", f"{module_name}:make_task")

    task = control_task_registry.create(
        "  external_padded_task  ",
        TaskConfig(name="external_padded_task", type="external_padded_task"),
    )

    assert isinstance(task, ExternalControlTask)


@pytest.mark.parametrize(
    "factory_path", ["", "external_pkg.tasks", ":make_task", "external_pkg.tasks:"]
)
def test_register_control_task_validates_path_format_only(factory_path: str) -> None:
    with pytest.raises(ValueError):
        register_control_task("external_invalid_path", factory_path)


def test_register_control_task_duplicate_policy_keeps_original() -> None:
    register_control_task("external_duplicate_task", "external_pkg.tasks:make_task")
    register_control_task("external_duplicate_task", "external_pkg.tasks:make_task")

    with pytest.raises(ValueError):
        register_control_task("external_duplicate_task", "external_pkg.other:make_task")

    assert (
        control_task_registry._factory_paths["external_duplicate_task"]
        == "external_pkg.tasks:make_task"
    )


def test_control_coordinator_resolves_external_base_adapter_without_manifest() -> None:
    adapter_name = "external_base_no_manifest"
    register_hardware_adapter(HardwareType.BASE, adapter_name, ExternalBaseAdapter)
    coordinator = ControlCoordinator(
        publish_joint_state=False,
        hardware=[
            HardwareComponent(
                hardware_id="external_base",
                hardware_type=HardwareType.BASE,
                joints=make_twist_base_joints("external_base"),
                adapter_type=adapter_name,
            )
        ],
    )

    try:
        coordinator.start()
        connected = coordinator._hardware["external_base"]
        assert isinstance(connected.adapter, ExternalBaseAdapter)
        assert "external_base/vx" in coordinator._joint_to_hardware
    finally:
        coordinator.stop()


def test_control_coordinator_resolves_external_task_from_lazy_registry_without_manifest(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    module_name = "external_control_extension_task_module"
    task_type = "external_task_no_manifest"
    module = ModuleType(module_name)
    module.__dict__["make_task"] = ExternalControlTask
    monkeypatch.setitem(sys.modules, module_name, module)
    register_control_task(task_type, f"{module_name}:make_task")
    coordinator = ControlCoordinator(
        publish_joint_state=False,
        tasks=[TaskConfig(name="external_task", type=task_type)],
    )

    try:
        coordinator.start()
        assert isinstance(coordinator.get_task("external_task"), ExternalControlTask)
    finally:
        coordinator.stop()
