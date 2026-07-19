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

"""Legacy bridge: deploy a PureModule exactly like a legacy module (T8b).

Spec: ``dimos/pure/tasks/t8-rim.md`` §11. ``legacy_actor(PureCls)`` generates
a cached ``dimos.core.module.Module`` subclass — the module's deployment face:
coordinator-managed, autoconnect-wired, blueprint-instantiated, one per worker
process (``dedicated_worker``), plus the conventionally-named ``health`` topic
(spec §12). Parity by inheritance (matrix §2.2); lifecycle delegates to the
rim (``build``→``warmup``, ``start``→bind+start, ``stop``→drain+stop).

This file is the ONE sanctioned ``dimos.pure`` → ``dimos.core`` edge (spec
§1): a leaf, never imported by ``dimos/pure/__init__.py``; it sunsets with the
legacy system. Nothing in ``dimos.core`` or the engine imports it back.
"""

from __future__ import annotations

from typing import TYPE_CHECKING, Any, Final

from dimos.core.module import Module, ModuleConfig  # the bridge edge (spec §1)
from dimos.pure.module import PureModule

if TYPE_CHECKING:
    from dimos.core.coordination.blueprints import Blueprint

__all__ = [
    "legacy_actor",
    "legacy_blueprint",
]

_HEALTH_PAYLOAD: type = object
"""Health stream payload; T9 replaces with ``dimos.pure.health.Health`` (spec §12, Q3)."""

_HEALTH_STREAM: Final[str] = "health"
"""The conventionally-named extra topic (spec §12, D12): merged per-namespace."""

_MODULE_SURFACE: Final[frozenset[str]] = frozenset(
    {
        "ref",
        "config",
        "rpc",
        "tf",
        "io",
        "inputs",
        "outputs",
        "rpcs",
        "name",
        "blueprint",
        "module_info",
        _HEALTH_STREAM,
    }
)
"""Legacy Module attribute names a pure field may not shadow (spec §11.2)."""

_RESERVED_CONFIG: Final[frozenset[str]] = frozenset(ModuleConfig.model_fields)
"""ModuleConfig field names a pure config field may not collide with (spec §11.2)."""

_ACTOR_CACHE: Final[dict[type[PureModule], type[Module]]] = {}
"""One generated actor class per PureModule class (spec §11.1); pickle-stable."""


def legacy_actor(cls: type[PureModule], /) -> type[Module]:
    """Generate (cached) the legacy ``Module`` subclass deploying ``cls`` (spec §11)."""
    raise NotImplementedError


def legacy_blueprint(cls: type[PureModule], /, **kwargs: Any) -> Blueprint:
    """``legacy_actor(cls).blueprint(**kwargs)`` — the blueprint spelling (spec §11.1)."""
    raise NotImplementedError


def _validate_bridgeable(cls: type[PureModule]) -> None:
    """Reject In∩Out overlap and surface/config name collisions (spec §11.2, D13)."""
    raise NotImplementedError


def _actor_annotations(cls: type[PureModule]) -> dict[str, Any]:
    """Synthesize ``In[T]``/``Out[T]``/``health``/``config`` annotations (spec §11.1, P5)."""
    raise NotImplementedError


def _actor_config_model(cls: type[PureModule]) -> type[ModuleConfig]:
    """ModuleConfig subclass carrying the pure fields with pure defaults (P6)."""
    raise NotImplementedError


def _strip_optional(annotation: Any) -> Any:
    """Drop a single ``| None`` for autoconnect payload typing (spec §11.1)."""
    raise NotImplementedError
