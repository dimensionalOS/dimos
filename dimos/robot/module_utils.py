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

from dimos.robot.capabilities import has_capability  # runtime helper

__all__ = ["robot_capability", "robot_module"]


def robot_module(cls):
    """Lightweight decorator to mark any class as a robot plug-in.

    It injects a single ``attach(robot)`` method that stores a back-reference
    (``self.robot``) so the module can access robot data later.  Additional
    initialisation (e.g., starting streams) should be done by the module’s own
    code or inside ``attach`` overrides if the class already defines one.
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
    return cls


def _instantiate_module(mod_cls: Type[Any], ctor_kwargs: dict[str, Any]):
    """Create *mod_cls* passing only arguments that its __init__ accepts."""

    sig = inspect.signature(mod_cls)
    filtered_kwargs = {k: v for k, v in ctor_kwargs.items() if k in sig.parameters}
    return mod_cls(**filtered_kwargs)  # type: ignore[arg-type]


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

            instantiated: list[Any] = []
            for mod_cls in module_types:
                # Capability check
                requires = getattr(mod_cls, "REQUIRES", tuple())
                for proto in requires:
                    conn = getattr(self, "conn", None)
                    if conn is None or not has_capability(conn, proto):
                        raise RuntimeError(
                            f"{mod_cls.__name__} requires capability {proto.__name__}, "
                            "but the robot's connection does not provide it."
                        )

                module_instance = _instantiate_module(mod_cls, kwargs)
                name = getattr(module_instance, "name", mod_cls.__name__.lower())
                self._modules[name] = module_instance
                instantiated.append(module_instance)

            for module_instance in instantiated:
                setup_fn = getattr(module_instance, "setup", None)

                if callable(setup_fn):
                    setup_fn(self)

        robot_cls.__init__ = _new_init
        return robot_cls

    return decorator
