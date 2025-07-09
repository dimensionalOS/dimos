# Copyright 2025 Dimensional Inc.
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

"""Utility helpers for capability-oriented robot plug-ins.

This module defines ``robot_capability`` – a decorator that augments a
Robot subclass so that listed *module classes* are instantiated and
attached automatically during robot construction, removing the need for
explicit ``robot.add_module(module)`` calls.

Each module class exposes:

* ``name`` – unique registry key (defaults to class name in lowercase)
* ``REQUIRES`` – tuple of capability ``Protocol`` objects that the
  robot’s *conn* must satisfy (e.g. ``(Video, Odometry)``)
* ``attach(self, robot)`` – method called with the newly created robot
  instance to establish backlinks, start streams, etc.
"""

from __future__ import annotations

import inspect
from functools import wraps
from typing import Any, Callable, Type

import yaml
from pathlib import Path

from dimos.robot.capabilities import has_capability  # runtime helper

__all__ = ["robot_capability", "robot_module"]


class DependencyError(Exception):
    """Raised when a module dependency cannot be resolved."""

    pass


def robot_module(cls):
    """Lightweight decorator to mark any class as a robot plug-in.

    Extended to support dependency injection through DEPENDS_ON attribute.
    """

    # Default lifecycle hook stores robot backlink
    def setup(self, robot):
        self.robot = robot

    # Provide default setup/attach aliases depending on what the class declares
    if not hasattr(cls, "setup"):
        cls.setup = setup
    # Back-compat alias: ensure `attach` points to the same function
    if not hasattr(cls, "attach"):
        cls.attach = cls.setup

    cls.name = getattr(cls, "name", cls.__name__.lower())

    # Add dependency tracking
    cls.DEPENDS_ON = getattr(cls, "DEPENDS_ON", tuple())

    return cls


def _instantiate_module(mod_cls: Type[Any], ctor_kwargs: dict[str, Any]):
    """Create *mod_cls* passing only arguments that its __init__ accepts."""

    sig = inspect.signature(mod_cls)
    filtered_kwargs = {k: v for k, v in ctor_kwargs.items() if k in sig.parameters}
    return mod_cls(**filtered_kwargs)  # type: ignore[arg-type]


def _resolve_dependencies(
    modules: dict[str, Any], module_types: list[Type[Any]]
) -> list[Type[Any]]:
    """Resolve module dependencies and return ordered list for initialization."""
    # Build dependency graph
    dependencies = {}
    for mod_cls in module_types:
        depends_on = getattr(mod_cls, "DEPENDS_ON", tuple())
        dependencies[mod_cls] = depends_on

    # Topological sort for initialization order
    ordered = []
    visited = set()
    temp_visited = set()

    def visit(mod_cls):
        if mod_cls in temp_visited:
            raise DependencyError(f"Circular dependency detected involving {mod_cls.__name__}")
        if mod_cls in visited:
            return

        temp_visited.add(mod_cls)
        for dep in dependencies.get(mod_cls, []):
            if dep in [m.__class__ for m in modules.values()]:
                continue  # Already instantiated
            if dep in dependencies:
                visit(dep)
        temp_visited.remove(mod_cls)
        visited.add(mod_cls)
        ordered.append(mod_cls)

    for mod_cls in module_types:
        visit(mod_cls)

    return ordered


def _inject_dependencies(module_instance: Any, modules: dict[str, Any]):
    """Inject dependencies into a module instance."""
    depends_on = getattr(module_instance.__class__, "DEPENDS_ON", tuple())

    for dep_cls in depends_on:
        # Find instance of dependency
        dep_instance = None
        for name, instance in modules.items():
            if isinstance(instance, dep_cls):
                dep_instance = instance
                break

        if dep_instance is None:
            raise DependencyError(
                f"{module_instance.__class__.__name__} depends on {dep_cls.__name__}, "
                "but no instance was found"
            )

        # Inject as attribute (e.g., self.astar_planner for AstarPlanner)
        attr_name = _get_dependency_attr_name(dep_cls)
        setattr(module_instance, attr_name, dep_instance)


def _get_dependency_attr_name(cls: Type) -> str:
    """Convert class name to snake_case attribute name for dependency injection."""
    import re

    name = cls.__name__
    # Convert CamelCase to snake_case
    name = re.sub("(.)([A-Z][a-z]+)", r"\1_\2", name)
    name = re.sub("([a-z0-9])([A-Z])", r"\1_\2", name).lower()
    return name


def robot_capability(*module_types: Type[Any]) -> Callable[[Type[Any]], Type[Any]]:
    """Class decorator for *Robot* subclasses.

    Example::

        @robot_capability(SpatialMemory, PersonTrackingStream)
        class UnitreeGo2(Robot):
            ...
    """

    def decorator(robot_cls: Type[Any]) -> Type[Any]:
        original_init = robot_cls.__init__

        @wraps(original_init)
        def _new_init(self, *args: Any, **kwargs: Any):
            # 1. run the original constructor first
            original_init(self, *args, **kwargs)

            if not hasattr(self, "_modules"):
                self._modules: dict[str, Any] = {}

            # 2. Load module config if available
            module_config = {}
            if hasattr(self, "module_config_file"):
                config_path = Path(self.module_config_file)
                if config_path.exists():
                    with open(config_path, "r") as f:
                        loaded_config = yaml.safe_load(f) or {}
                        module_config = loaded_config.get("modules", {})
            elif hasattr(self, "module_config"):
                module_config = self.module_config

            # Resolve initialization order based on dependencies
            module_list = list(module_types)  # Convert tuple to list
            ordered_modules = _resolve_dependencies({}, module_list)

            instantiated: list[Any] = []
            for mod_cls in ordered_modules:
                # Capability check
                requires = getattr(mod_cls, "REQUIRES", tuple())
                for proto in requires:
                    conn = getattr(self, "conn", None)
                    if conn is None or not has_capability(conn, proto):
                        raise RuntimeError(
                            f"{mod_cls.__name__} requires capability {proto.__name__}, "
                            "but the robot's connection does not provide it."
                        )

                # Get module-specific config
                module_name_lower = mod_cls.__name__.lower()
                config = module_config.get(module_name_lower, {})

                # Merge with kwargs for backwards compatibility
                merged_config = {**kwargs, **config}

                # Create module with filtered config
                module_instance = _instantiate_module(mod_cls, merged_config)
                name = getattr(module_instance, "name", module_name_lower)
                self._modules[name] = module_instance
                instantiated.append(module_instance)

            # After all modules are instantiated, inject dependencies
            for module_instance in instantiated:
                _inject_dependencies(module_instance, self._modules)

            # Finally, run setup methods
            for module_instance in instantiated:
                setup_fn = getattr(module_instance, "setup", None)

                if callable(setup_fn):
                    setup_fn(self)

        robot_cls.__init__ = _new_init
        return robot_cls

    return decorator
