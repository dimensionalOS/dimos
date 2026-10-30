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

"""Static info one module hands another after deploy, before either builds or starts.

class Sensor(Module):
    @shared_config(Simulation)
    def thing(self) -> dict[str, Any]:
        return {"fov": self.config.fov}

class Simulation(SharedConfigModule):
    def config_receive(self, config_senders: tuple[tuple[str, dict[str, Any]], ...]) -> None:
        ...  # ("sensor", {"fov": 60}), one per @shared_config method aimed at Simulation
"""

from collections.abc import Callable, Iterator
from typing import Any, TypeVar, cast

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleBase

F = TypeVar("F", bound=Callable[..., Any])

# (sender instance name, what its @shared_config method returned)
ConfigSenders = tuple[tuple[str, dict[str, Any]], ...]


def shared_config(receiver: type[ModuleBase]) -> Callable[[F], F]:
    """Mark a method whose return value is handed to every deployed `receiver` (or subclass)."""

    def mark(fn: F) -> F:
        fn.__shared_config_receiver__ = receiver  # type: ignore[attr-defined]
        return cast("F", rpc(fn))

    return mark


def shared_config_methods(cls: type) -> Iterator[tuple[str, type[ModuleBase]]]:
    """(method name, receiver class) for each @shared_config method on cls."""
    seen: set[str] = set()
    for klass in cls.__mro__:
        for name, attr in vars(klass).items():
            if name in seen:
                continue
            seen.add(name)
            receiver = getattr(attr, "__shared_config_receiver__", None)
            if receiver is not None:
                yield name, receiver


class SharedConfigModule(Module):
    """A module that gets every @shared_config(ThisClass) value before its build()."""

    def config_receive(self, config_senders: ConfigSenders) -> None:
        """Override to take the values; a plain method, not an RPC."""

    @rpc
    def deliver_shared_config(self, config_senders: ConfigSenders) -> None:
        # The coordinator's way in, so config_receive itself is never an RPC.
        self.config_receive(config_senders)
