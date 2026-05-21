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

"""Registry for per-(robot, backend) connection modules.

A connection module declares its identity with the `@connection` decorator:

    @connection(robot="go2", backend="webrtc")
    class Go2WebRtcConnection(Module): ...

    @connection(robot="go2", backend="mujoco")
    class Go2MujocoConnection(Module): ...

`Blueprint.with_backend("mujoco")` walks a blueprint's atoms and, for each
tagged atom whose backend differs from the requested one, looks up the
same-robot module for the requested backend in `_REGISTRY` and substitutes it.
"""

from collections.abc import Callable
from typing import TypeVar

from dimos.core.module import ConnectionTag, ModuleBase

T = TypeVar("T", bound=type[ModuleBase])


_REGISTRY: dict[tuple[str, str], type[ModuleBase]] = {}


def connection(*, robot: str, backend: str) -> Callable[[T], T]:
    """Class decorator that tags a Module as the (robot, backend) connection."""

    def deco(cls: T) -> T:
        tag = ConnectionTag(robot=robot, backend=backend)
        existing = _REGISTRY.get((robot, backend))
        if existing is not None and existing is not cls:
            raise ValueError(
                f"Duplicate connection registration for ({robot!r}, {backend!r}): "
                f"{existing.__name__} and {cls.__name__}"
            )
        cls._connection_tag = tag
        _REGISTRY[(robot, backend)] = cls
        return cls

    return deco


def get_connection(robot: str, backend: str) -> type[ModuleBase] | None:
    return _REGISTRY.get((robot, backend))


def backends_for(robot: str) -> set[str]:
    return {backend for (r, backend) in _REGISTRY if r == robot}
