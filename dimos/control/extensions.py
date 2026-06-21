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

"""Public extension surface for ControlCoordinator sideloading."""

from __future__ import annotations

from collections.abc import Callable
from typing import TYPE_CHECKING, cast

from dimos.control.components import HardwareType
from dimos.control.tasks.registry import control_task_registry
from dimos.control.tasks.registry_utils import normalize_task_name, validate_task_factory_path
from dimos.hardware.registry_utils import normalize_adapter_name

if TYPE_CHECKING:
    from dimos.hardware.drive_trains.spec import TwistBaseAdapter
    from dimos.hardware.manipulators.spec import ManipulatorAdapter
    from dimos.hardware.whole_body.spec import WholeBodyAdapter


def register_hardware_adapter(
    hardware_type: HardwareType,
    adapter_type: str,
    factory: Callable[..., object],
) -> None:
    """Register a hardware adapter factory for external packages.

    The adapter is registered into the existing DimOS registry that matches
    ``hardware_type``. Re-registering the same adapter name with the same
    factory object is idempotent; registering a different factory for an
    existing name raises from the target registry.
    """
    adapter_name = normalize_adapter_name(adapter_type)
    if not callable(factory):
        raise TypeError("Hardware adapter factory must be callable")

    match hardware_type:
        case HardwareType.MANIPULATOR:
            from dimos.hardware.manipulators.registry import adapter_registry

            adapter_registry.register(
                adapter_name, cast("Callable[..., ManipulatorAdapter]", factory)
            )
        case HardwareType.BASE:
            from dimos.hardware.drive_trains.registry import twist_base_adapter_registry

            twist_base_adapter_registry.register(
                adapter_name,
                cast("Callable[..., TwistBaseAdapter]", factory),
            )
        case HardwareType.WHOLE_BODY:
            from dimos.hardware.whole_body.registry import whole_body_adapter_registry

            whole_body_adapter_registry.register(
                adapter_name,
                cast("Callable[..., WholeBodyAdapter]", factory),
            )
        case _:
            raise ValueError(f"Unsupported hardware type: {hardware_type!r}")


def register_control_task(task_type: str, factory_path: str) -> None:
    """Register a lazy control task factory path for external packages.

    The target factory module is not imported during registration. It is
    resolved later by the control task registry when a coordinator creates a
    matching ``TaskConfig``.
    """
    task_name = normalize_task_name(task_type)
    validate_task_factory_path(factory_path, label="control task factory path")
    control_task_registry.register_path(task_name, factory_path)
