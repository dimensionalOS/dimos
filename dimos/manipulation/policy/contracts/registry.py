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

"""Registry of concrete `RobotContract` implementations keyed by robot name."""

from __future__ import annotations

from collections.abc import Callable, Sequence

from dimos.manipulation.policy.contract import RobotContract
from dimos.manipulation.policy.contracts.piper import PiperRobotContract

_registry: dict[str, Callable[[], RobotContract]] = {}


def register_contract(name: str, factory: Callable[[], RobotContract]) -> None:
    _registry[name] = factory


def get_contract(name: str) -> RobotContract:
    factory = _registry.get(name)
    if factory is None:
        available = sorted(_registry)
        raise KeyError(f"Unknown robot contract '{name}'. Available contracts: {available}")
    return factory()


def available_contracts() -> Sequence[str]:
    return sorted(_registry)


register_contract("piper", PiperRobotContract)


__all__ = [
    "available_contracts",
    "get_contract",
    "register_contract",
]
