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

import copyreg
from typing import TYPE_CHECKING, Any, Final

from dimos.core.module import Module, ModuleConfig  # the bridge edge (spec §1)
from dimos.pure.module import PureModule

if TYPE_CHECKING:
    from dimos.core.coordination.blueprints import Blueprint

__all__ = [
    "HealthPlaceholder",
    "legacy_actor",
    "legacy_blueprint",
]


class HealthPlaceholder:
    """Named placeholder health payload; T9 swaps the TYPE, not the mechanism.

    Autoconnect keys the shared health topic on ``("health", <payload>)``
    (spec §12, D12): a NAMED type keeps that key legible (never ``("health",
    object)``) and unlikely to collide with a legacy ``Out[object]`` (AD2,
    resolved by Ivan 2026-07-20). T9 replaces it with ``dimos.pure.health.Health``.
    """


_HEALTH_PAYLOAD: type = HealthPlaceholder
"""Health stream payload type; T9 replaces with ``dimos.pure.health.Health`` (spec §12)."""

_HEALTH_STREAM: Final[str] = "health"
"""The conventionally-named extra topic (spec §12, D12): merged per-namespace."""

_MODULE_SURFACE: Final[frozenset[str]] = frozenset(dir(Module)) | {
    "config",
    "ref",
    "rpc",
    "tf",
}
"""Legacy Module surface a pure field may not shadow — COMPUTED from ``dir(Module)``
(drift-proof: catches ``set_transport``/``peek_stream``/``stop``/``start``/``build``/
``main``/``set_module_ref``, all of which a static list misses) plus the four
instance attributes set in ``Module.__init__`` that never appear on the class ``dir`` (spec
§11.2, G2 — re-verified against ``dir(Module)`` in this graft)."""

_RESERVED_CONFIG: Final[frozenset[str]] = frozenset(ModuleConfig.model_fields)
"""ModuleConfig field names a pure config field may not collide with (spec §11.2)."""

_ACTOR_CACHE: Final[dict[tuple[type[PureModule], str | None], type[Module]]] = {}
"""One generated actor class per ``(PureModule class, name)`` (spec §11.1); pickle-stable."""


class _ActorMeta(type(Module)):  # type: ignore[misc]  # ABCMeta subclass (spec §11.1, G1)
    """Metaclass of generated actor classes; registered with ``copyreg`` (spec §11.1).

    Subclasses ``type(Module)`` (``ABCMeta``) — a plain-``type`` metaclass raises
    ``metaclass conflict`` at synthesis. A metaclass ``__reduce__`` on a class
    object is IGNORED by pickle (it takes the ``issubclass(t, type)`` →
    ``save_global`` branch, which fails on a class importable from nowhere);
    ``copyreg.pickle`` registers the reducer on the dispatch table pickle
    consults BEFORE that fallback, so the actor crosses the deploy pipe by
    recipe (P1). Both facts re-verified against ``ForkingPickler`` in this graft.
    """

    __pure_class__: type[PureModule]
    __actor_name__: str | None


def _rebuild_actor(pure_cls: type[PureModule], name: str | None) -> type[Module]:
    """copyreg recipe: re-derive the cached actor class in the worker (spec §11.1)."""
    return legacy_actor(pure_cls, name=name)


def _reduce_actor_class(cls: _ActorMeta) -> tuple[Any, tuple[Any, ...]]:
    """Reduce a generated actor CLASS to its rebuild recipe (module-level, pickles by name)."""
    return (_rebuild_actor, (cls.__pure_class__, cls.__actor_name__))


copyreg.pickle(_ActorMeta, _reduce_actor_class)  # dispatch-table entry (spec §11.1, G1)


def legacy_actor(cls: type[PureModule], /, *, name: str | None = None) -> type[Module]:
    """Generate (cached) the legacy ``Module`` subclass deploying ``cls`` (spec §11).

    ``name`` overrides the actor's ``__name__`` (default: ``cls.__name__``) — the
    migration escape hatch when a legacy class and its pure twin would collide on
    ``__name__.lower()`` in one blueprint (spec §11.1, G8/D16). The cache is keyed
    ``(cls, name)``. The generated ``__module__`` is ``"dimos.pure.legacy"`` (honest:
    the class lives in no importable namespace — G9); pickling rides the ``copyreg``
    recipe above, not the module string.
    """
    raise NotImplementedError


def legacy_blueprint(
    cls: type[PureModule], /, *, name: str | None = None, **kwargs: Any
) -> Blueprint:
    """``legacy_actor(cls, name=name).blueprint(**kwargs)`` — the blueprint spelling (spec §11.1)."""
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
