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
import os
from typing import TYPE_CHECKING, cast

if TYPE_CHECKING:
    from dimos.control.coordinator import TaskConfig
    from dimos.control.hardware_interface import ConnectedHardware, ConnectedWholeBody
    from dimos.control.task import ControlTask

TaskFactory = Callable[..., "ControlTask"]


class ControlTaskRegistry:
    """Registry for control-task factories with lazy imports."""

    def __init__(self) -> None:
        self._factory_paths: dict[str, str] = {}
        self._factories: dict[str, TaskFactory] = {}
        self.discover()

    def discover(self) -> None:
        """Discover task registry manifests without importing task implementations."""
        tasks_pkg = importlib.import_module("dimos.control.tasks")
        for root in tasks_pkg.__path__:
            for entry in sorted(os.listdir(root)):
                if entry.startswith(("_", ".")):
                    continue
                entry_path = os.path.join(root, entry)
                if not os.path.isdir(entry_path):
                    continue

                module_name = f"dimos.control.tasks.{entry}.__registry__"
                try:
                    module = importlib.import_module(module_name)
                except ModuleNotFoundError as exc:
                    if exc.name == module_name:
                        continue
                    raise

                task_factories = getattr(module, "TASK_FACTORIES", None)
                if not isinstance(task_factories, Mapping):
                    raise TypeError(f"{module_name} must define TASK_FACTORIES")
                for name, factory_path in task_factories.items():
                    if not isinstance(name, str) or not isinstance(factory_path, str):
                        raise TypeError(f"{module_name}.TASK_FACTORIES must map strings to strings")
                    self.register_path(name, factory_path)

    def register_path(self, name: str, factory_path: str) -> None:
        """Register a lazy task factory import path."""
        if ":" not in factory_path:
            raise ValueError(f"Invalid task factory path: {factory_path!r}")
        key = name.lower()
        existing = self._factory_paths.get(key)
        if existing is not None and existing != factory_path:
            raise ValueError(f"Duplicate task type {key!r}: {existing!r} vs {factory_path!r}")
        self._factory_paths[key] = factory_path

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
            cfg: ``TaskConfig`` carrying the generic task envelope
                (name/joint_names/priority) plus task-owned ``params``.
            hardware: Coordinator's hardware map. Tasks that need an
                adapter resolve it from their typed params; pass ``None``
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
        factory_path = self._factory_paths[key]
        module_name, attr = factory_path.split(":", maxsplit=1)
        try:
            module = importlib.import_module(module_name)
        except ModuleNotFoundError as exc:
            raise ValueError(
                f"Task {key!r} is registered to missing module {module_name!r}"
            ) from exc
        factory = cast("TaskFactory", getattr(module, attr))
        if not callable(factory):
            raise TypeError(f"Task factory {factory_path!r} is not callable")
        self._factories[key] = factory
        return factory


control_task_registry = ControlTaskRegistry()
