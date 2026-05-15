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

"""In-repo registry of `PolicyBackend` factories keyed by name.

Built-in entries — currently ``"test"`` and ``"lerobot"`` — are registered
lazily by `dimos.manipulation.policy.backends`. Importing this module on
its own does NOT import any backend implementations or their optional
dependencies (notably `lerobot`).
"""

from __future__ import annotations

from collections.abc import Callable, Sequence
from typing import Any

from dimos.manipulation.policy.backend import PolicyBackend

BackendFactory = Callable[..., PolicyBackend]

_registry: dict[str, BackendFactory] = {}


def register_backend(name: str, factory: BackendFactory) -> None:
    """Register `factory` under `name`. Re-registration overwrites."""
    _registry[name] = factory


def create_backend(name: str, **kwargs: Any) -> PolicyBackend:
    """Instantiate the backend registered under `name`.

    Raises:
        KeyError: If `name` is not registered. The error message lists the
            backends currently available so callers can spot typos.
    """
    factory = _registry.get(name)
    if factory is None:
        available = sorted(_registry)
        raise KeyError(f"Unknown policy backend '{name}'. Available backends: {available}")
    return factory(**kwargs)


def available_backends() -> Sequence[str]:
    """Return the sorted list of registered backend names."""
    return sorted(_registry)


def is_registered(name: str) -> bool:
    return name in _registry


__all__ = [
    "BackendFactory",
    "available_backends",
    "create_backend",
    "is_registered",
    "register_backend",
]
