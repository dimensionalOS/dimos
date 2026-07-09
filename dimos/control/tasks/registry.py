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

from collections.abc import Callable, Mapping, Sequence
import importlib
import os
from typing import TYPE_CHECKING, cast

from dimos.control.routing import (
    CONSUMABLE_STREAMS,
    DEFERRED_STREAMS,
    Routing,
    StreamBinding,
    TaskBindings,
)

if TYPE_CHECKING:
    from types import ModuleType

    from dimos.control.coordinator import TaskConfig
    from dimos.control.hardware_interface import ConnectedHardware, ConnectedWholeBody
    from dimos.control.task import ControlTask

TaskFactory = Callable[..., "ControlTask"]

_EMPTY_BINDINGS = TaskBindings()


class ControlTaskRegistry:
    """Registry for control-task factories with lazy imports."""

    def __init__(self) -> None:
        self._factory_paths: dict[str, str] = {}
        self._factories: dict[str, TaskFactory] = {}
        self._bindings: dict[str, TaskBindings] = {}
        self._binding_sources: dict[str, str] = {}
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

                module_name = f"dimos.control.tasks.{entry}._registry"
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
                self._load_manifest_bindings(module_name, module, task_factories)

    def register_path(self, name: str, factory_path: str) -> None:
        """Register a lazy task factory import path."""
        if ":" not in factory_path:
            raise ValueError(f"Invalid task factory path: {factory_path!r}")
        key = name.lower()
        existing = self._factory_paths.get(key)
        if existing is not None and existing != factory_path:
            raise ValueError(f"Duplicate task type {key!r}: {existing!r} vs {factory_path!r}")
        self._factory_paths[key] = factory_path

    def register_bindings(
        self,
        task_type: str,
        *,
        consumes: Mapping[str, tuple[str, str]] | None = None,
        exposes: Sequence[str] | None = None,
        source: str | None = None,
    ) -> None:
        """Register a task type's binding card; conflicting duplicates raise.

        ``consumes`` maps a coordinator input stream name to a
        ``(handler_method, routing)`` pair of strings. ``exposes`` is a
        sequence of command names the task accepts via
        ``ControlCoordinator.task_invoke``; the task method's own signature
        is the argument schema, so no separate model is declared. ``source``
        names the manifest for error messages; runtime callers can omit it.
        """
        key = task_type.lower()
        origin = source or "register_bindings()"
        bindings = TaskBindings(
            consumes=self._parse_consumes(key, consumes, origin),
            exposes=self._parse_exposes(key, exposes, origin),
        )
        existing = self._bindings.get(key)
        if existing is not None:
            if existing != bindings:
                raise ValueError(
                    f"Duplicate bindings for task type {key!r}: {existing!r} "
                    f"(from {self._binding_sources[key]}) vs {bindings!r} (from {origin})"
                )
            return
        self._bindings[key] = bindings
        self._binding_sources[key] = origin

    def bindings_for(self, task_type: str) -> TaskBindings:
        """Declared bindings for a task type; empty for unknown or card-less types."""
        return self._bindings.get(task_type.lower(), _EMPTY_BINDINGS)

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
        task = factory(cfg=cfg, hardware=hardware or {})
        self._validate_bound_handlers(key, task)
        return task

    def available(self) -> list[str]:
        return sorted(self._factory_paths.keys())

    def _load_manifest_bindings(
        self, module_name: str, module: ModuleType, task_factories: Mapping[str, str]
    ) -> None:
        consumes_by_type = getattr(module, "TASK_CONSUMES", None)
        exposes_by_type = getattr(module, "TASK_EXPOSES", None)
        declared_types = {name.lower() for name in task_factories}
        for label, per_type in (
            ("TASK_CONSUMES", consumes_by_type),
            ("TASK_EXPOSES", exposes_by_type),
        ):
            if per_type is None:
                continue
            if not isinstance(per_type, Mapping):
                raise TypeError(f"{module_name}.{label} must be a mapping keyed by task type")
            seen: set[str] = set()
            for name in per_type:
                if not isinstance(name, str):
                    raise TypeError(f"{module_name}.{label} keys must be task-type strings")
                if name.lower() in seen:
                    raise ValueError(
                        f"{module_name}.{label} declares task type {name!r} more than once"
                    )
                seen.add(name.lower())
                if name.lower() not in declared_types:
                    raise ValueError(
                        f"{module_name}.{label} declares task type {name!r} "
                        "not present in the same manifest's TASK_FACTORIES"
                    )
        consumes_by_key = {name.lower(): card for name, card in (consumes_by_type or {}).items()}
        exposes_by_key = {name.lower(): card for name, card in (exposes_by_type or {}).items()}
        for name in sorted({*consumes_by_key, *exposes_by_key}):
            self.register_bindings(
                name,
                consumes=consumes_by_key.get(name),
                exposes=exposes_by_key.get(name),
                source=module_name,
            )

    def _parse_consumes(
        self,
        task_type: str,
        consumes: Mapping[str, tuple[str, str]] | None,
        source: str,
    ) -> tuple[StreamBinding, ...]:
        if consumes is None:
            return ()
        if not isinstance(consumes, Mapping):
            raise TypeError(f"{source}: consumes for task type {task_type!r} must be a mapping")
        bindings = []
        for stream, spec in consumes.items():
            where = f"{source}: task type {task_type!r}, stream {stream!r}"
            if not isinstance(stream, str):
                raise TypeError(f"{where}: stream name must be a string")
            if stream in DEFERRED_STREAMS:
                raise ValueError(
                    f"{where}: not card-routable yet; the twist path is deliberately "
                    "deferred to a later PR, so this restriction is temporary"
                )
            if stream not in CONSUMABLE_STREAMS:
                raise ValueError(f"{where}: unknown stream; allowed: {sorted(CONSUMABLE_STREAMS)}")
            if (
                not isinstance(spec, Sequence)
                or isinstance(spec, str)
                or len(spec) != 2
                or not all(isinstance(part, str) for part in spec)
            ):
                raise TypeError(f"{where}: binding must be a (handler, routing) pair of strings")
            handler, routing_value = spec
            try:
                routing = Routing(routing_value)
            except ValueError:
                raise ValueError(
                    f"{where}: unknown routing {routing_value!r}; "
                    f"valid: {[r.value for r in Routing]}"
                ) from None
            bindings.append(StreamBinding(stream=stream, handler=handler, routing=routing))
        return tuple(sorted(bindings, key=lambda binding: binding.stream))

    def _parse_exposes(
        self,
        task_type: str,
        exposes: Sequence[str] | None,
        source: str,
    ) -> frozenset[str]:
        if exposes is None:
            return frozenset()
        if isinstance(exposes, str) or not isinstance(exposes, Sequence):
            raise TypeError(
                f"{source}: exposes for task type {task_type!r} must be a sequence of "
                "command-name strings"
            )
        commands: set[str] = set()
        for command in exposes:
            where = f"{source}: task type {task_type!r}"
            if not isinstance(command, str) or not command:
                raise TypeError(f"{where}: command names must be non-empty strings")
            if command.startswith("_"):
                raise ValueError(
                    f"{where}: command {command!r} is private; underscore-prefixed "
                    "methods are not wire-callable"
                )
            if command in commands:
                raise ValueError(f"{where}: command {command!r} declared more than once")
            commands.add(command)
        return frozenset(commands)

    def _validate_bound_handlers(self, key: str, task: ControlTask) -> None:
        bindings = self._bindings.get(key)
        if bindings is None:
            return
        source = self._binding_sources.get(key, "register_bindings()")
        for binding in bindings.consumes:
            handler = getattr(task, binding.handler, None)
            if not callable(handler):
                raise TypeError(
                    f"Task type {key!r} binds stream {binding.stream!r} to handler "
                    f"{binding.handler!r} (declared in {source}), but the created "
                    f"{type(task).__name__} has no callable {binding.handler!r}"
                )

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
