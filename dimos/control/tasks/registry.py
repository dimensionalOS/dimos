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

"""ControlTask registry with lazy factories.

The registry keeps task-specific imports out of ``ControlCoordinator``
without importing every task module at startup. Some task modules depend
on heavier packages such as Pinocchio or ONNX Runtime, so factories are
resolved only for the requested task type.

Usage:
    from dimos.control.tasks.registry import control_task_registry

    task = control_task_registry.create(cfg.type, cfg, hardware=self._hardware)
    print(control_task_registry.available())
"""

from __future__ import annotations

from collections.abc import Callable, Mapping
import importlib
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from dimos.control.coordinator import TaskConfig
    from dimos.control.hardware_interface import ConnectedHardware, ConnectedWholeBody
    from dimos.control.task import ControlTask

TaskFactory = Callable[..., "ControlTask"]


class ControlTaskRegistry:
    """Registry for control-task factories with lazy imports."""

    def __init__(self) -> None:
        self._factory_paths: dict[str, str] = {
            "trajectory": "dimos.control.tasks.trajectory_task:create_task",
            "servo": "dimos.control.tasks.servo_task:create_task",
            "velocity": "dimos.control.tasks.velocity_task:create_task",
            "cartesian_ik": "dimos.control.tasks.cartesian_ik_task:create_task",
            "teleop_ik": "dimos.control.tasks.teleop_task:create_task",
            "g1_groot_wbc": "dimos.control.tasks.g1_groot_wbc_task:create_task",
        }
        self._factories: dict[str, TaskFactory] = {}

    def create(
        self,
        name: str,
        cfg: TaskConfig,
        *,
        hardware: Mapping[str, ConnectedHardware | ConnectedWholeBody] | None = None,
    ) -> ControlTask:
        """Instantiate a task by registered name.

        Args:
            name: Registered task-type name (e.g. ``"trajectory"``).
            cfg: ``TaskConfig`` carrying name/joint_names/priority and
                whatever else this task needs.
            hardware: Coordinator's hardware map. Tasks that need an
                adapter resolve via ``cfg.hardware_id``; pass ``None``
                only if no task in this registry needs hardware.
        """
        key = name.lower()
        factory = self._resolve_factory(key)
        return factory(cfg=cfg, hardware=hardware or {})

    def available(self) -> list[str]:
        return sorted(self._factory_paths.keys())

    def _resolve_factory(self, key: str) -> TaskFactory:
        if key in self._factories:
            return self._factories[key]
        if key not in self._factory_paths:
            raise ValueError(f"Unknown task type: {key!r}. Available: {self.available()}")
        module_name, attr = self._factory_paths[key].split(":", maxsplit=1)
        module = importlib.import_module(module_name)
        factory = getattr(module, attr)
        if not callable(factory):
            raise TypeError(f"Task factory {self._factory_paths[key]!r} is not callable")
        self._factories[key] = factory
        return factory


control_task_registry = ControlTaskRegistry()

__all__ = ["ControlTaskRegistry", "TaskFactory", "control_task_registry"]
