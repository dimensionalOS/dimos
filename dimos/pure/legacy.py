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
import types
from typing import TYPE_CHECKING, Any, Final, Union, get_args, get_origin

from pydantic import create_model

from dimos.core.core import rpc  # the bridge edge (spec §1)
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.pure import rim
from dimos.pure.module import PureModule
from dimos.pure.stepspec import PureModuleDefinitionError

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
    key = (cls, name)
    cached = _ACTOR_CACHE.get(key)
    if cached is not None:
        return cached

    _validate_bridgeable(cls)

    actor_name = name or cls.__name__
    config_model = _actor_config_model(cls)
    pure_fields: tuple[str, ...] = tuple(cls.__pure_config_model__.model_fields)

    def actor_init(self: Any, **kwargs: Any) -> None:
        Module.__init__(self, **kwargs)  # legacy streams + ActorConfig + serve RPC
        pure_kwargs = {f: getattr(self.config, f) for f in pure_fields}
        self._pure = type(self).__pure_class__(**pure_kwargs)

    @rpc
    def build(self: Any) -> None:
        rim.warmup_module(self._pure)  # P13: build == warmup == resource creation

    @rpc
    def start(self: Any) -> None:
        Module.start(self)  # legacy start (main/handlers: no-ops)
        spec = type(self._pure).__pure_step__
        for field in spec.in_type.fields():
            stream = getattr(self, field)
            if getattr(stream, "_transport", None) is not None:  # wired only (§11.3)
                getattr(self._pure.i, field).source = stream  # In is Subscribable
        for field in spec.out_type.fields():
            getattr(self._pure.o, field).transport = getattr(self, field)  # Out is Publishable
        rim.start_module(self._pure)

    @rpc
    def stop(self: Any) -> None:
        rim.stop_module(self._pure)  # drain → dispose → join (idempotent)
        Module.stop(self)  # legacy close (idempotent latch)

    namespace: dict[str, Any] = {
        "__module__": "dimos.pure.legacy",  # honest: lives in no namespace (G9)
        "__qualname__": f"legacy_actor({cls.__qualname__})",
        "__doc__": f"Legacy deployment face for {cls.__qualname__} (synthesized, spec §11).",
        "__annotations__": _actor_annotations(cls, config_model),
        "__pure_class__": cls,
        "__actor_name__": name,
        "deployment": "python",
        "dedicated_worker": True,  # one module per process (P9, amendment)
        "__init__": actor_init,
        "build": build,
        "start": start,
        "stop": stop,
    }
    actor = _ActorMeta(actor_name, (Module,), namespace)
    _ACTOR_CACHE[key] = actor
    return actor


def legacy_blueprint(
    cls: type[PureModule], /, *, name: str | None = None, **kwargs: Any
) -> Blueprint:
    """``legacy_actor(cls, name=name).blueprint(**kwargs)`` — the blueprint spelling (spec §11.1)."""
    return legacy_actor(cls, name=name).blueprint(**kwargs)


def _validate_bridgeable(cls: type[PureModule]) -> None:
    """Reject In∩Out overlap and surface/config name collisions (spec §11.2, D13)."""
    spec = cls.__pure_step__
    in_names = set(spec.in_type.fields())
    out_names = set(spec.out_type.fields())

    overlap = sorted(in_names & out_names)
    if overlap:
        raise PureModuleDefinitionError(
            f"{cls.__qualname__}: field name(s) {', '.join(overlap)} appear in both In and Out — "
            f"the legacy surface is one class annotation / one (name, type) autoconnect key per "
            f"name and cannot express an In∩Out overlap. Rename one side."
        )

    surface = sorted((in_names | out_names) & _MODULE_SURFACE)
    if surface:
        raise PureModuleDefinitionError(
            f"{cls.__qualname__}: field name(s) {', '.join(surface)} shadow the legacy Module "
            f"surface — a synthesized stream annotation would overwrite a Module attribute/method. "
            f"Rename the field(s)."
        )

    config_clash = sorted(set(cls.__pure_config_model__.model_fields) & _RESERVED_CONFIG)
    if config_clash:
        raise PureModuleDefinitionError(
            f"{cls.__qualname__}: config field name(s) {', '.join(config_clash)} collide with "
            f"ModuleConfig fields absorbed by the actor. Rename the field(s)."
        )


def _stream_annotation(stream: Any, payload: Any) -> Any:
    """``stream[payload]`` built from runtime values (``In``/``Out`` subscripted dynamically)."""
    return stream[payload]


def _actor_annotations(cls: type[PureModule], config_model: type[ModuleConfig]) -> dict[str, Any]:
    """Synthesize ``In[T]``/``Out[T]``/``health``/``config`` annotations (spec §11.1, P5)."""
    spec = cls.__pure_step__
    annotations: dict[str, Any] = {}
    for field, fs in spec.in_type.fields().items():
        annotations[field] = _stream_annotation(In, _strip_optional(fs.annotation))
    for field, fs in spec.out_type.fields().items():
        annotations[field] = _stream_annotation(
            Out, _strip_optional(fs.annotation)
        )  # Optional stripped
    annotations[_HEALTH_STREAM] = _stream_annotation(Out, _HEALTH_PAYLOAD)  # §12: T9 swaps the type
    annotations["config"] = config_model
    return annotations


def _actor_config_model(cls: type[PureModule]) -> type[ModuleConfig]:
    """ModuleConfig subclass carrying the pure fields with pure defaults (P6)."""
    fields: dict[str, Any] = {
        fname: (finfo.annotation, finfo)
        for fname, finfo in cls.__pure_config_model__.model_fields.items()
    }
    model = create_model(
        f"{cls.__name__}ActorConfig",
        __base__=ModuleConfig,
        **fields,
    )
    return model  # type: ignore[no-any-return]


def _strip_optional(annotation: Any) -> Any:
    """Drop a single ``| None`` for autoconnect payload typing (spec §11.1)."""
    if get_origin(annotation) in (Union, types.UnionType):
        non_none = [a for a in get_args(annotation) if a is not type(None)]
        if len(non_none) == 1:
            return non_none[0]
    return annotation
