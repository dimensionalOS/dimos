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
§1): a leaf, never imported by ``dimos/pure/pm.py``; it sunsets with the
legacy system. Nothing in ``dimos.core`` or the engine imports it back.
"""

from __future__ import annotations

import copyreg
import types
from typing import TYPE_CHECKING, Any, Final, Union, get_args, get_origin, get_type_hints

from pydantic import create_model

from dimos.core.core import rpc  # the bridge edge (spec §1)
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.core.transport_factory import make_transport, tf_backend
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.pure import rim
from dimos.pure.health import Health
from dimos.pure.module import PureModule
from dimos.pure.rows import TfOutSpec, TfSpec
from dimos.pure.stepspec import PureModuleDefinitionError
from dimos.utils.generic import classproperty

if TYPE_CHECKING:
    from dimos.core.coordination.blueprints import Blueprint


_HEALTH_PAYLOAD: type = Health
"""Health stream payload type (T9): the ONE global flat health row.

Autoconnect keys the shared health topic on ``("health", Health)`` (spec §12,
D12) — a single named class keeps the transport registry legible. T9 swapped
the TYPE here; the mechanism (annotation synthesis, merge, adapter) is
unchanged from T8's ``HealthPlaceholder``."""

_HEALTH_STREAM: Final[str] = "health"
"""The conventionally-named extra topic (spec §12, D12): merged per-namespace."""

TF_IN_STREAM: Final[str] = "tf_in"
"""Actor-side name of the tf input stream (T14).

tf arrives as an ORDINARY subscribed ``In[TFMessage]`` — no ``PubSubTF`` service, no
``receive_msg`` rail, no second buffer: the rim feeds ``m.i.tf`` from it and
``TfBuffer.ingest`` unpacks the batch. The stream cannot be called ``tf``: that name is
taken on the legacy ``Module`` surface (``Module.tf``, guarded by ``_MODULE_SURFACE``),
so the actor pins ``tf_in`` to the TF topic instead (see ``_tf_transport``)."""

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

_RESERVED_CONFIG: Final[frozenset[str]] = frozenset(ModuleConfig.model_fields) - {
    "frame_id",
    "frame_id_prefix",
}
"""ModuleConfig field names a pure config field may not collide with (spec §11.2).

``frame_id``/``frame_id_prefix`` are exempt: they are author-facing module config a
pure module may legitimately declare, and the synthesized ActorConfig simply overrides
the same ``ModuleConfig`` slot (``create_model`` subclass field override). The remaining
names are framework-injected plumbing (``g``, transports, timeouts, ``instance_name``)
the coordinator/adapter owns."""

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
        for field, fs in spec.in_type.fields().items():
            if isinstance(fs, TfSpec):
                continue  # side channel: fed from tf_in below, never its own stream
            stream = getattr(self, field)
            if getattr(stream, "_transport", None) is not None:  # wired only (§11.3)
                getattr(self._pure.i, field).source = stream  # In is Subscribable
        for field in spec.out_type.fields():
            getattr(self._pure.o, field).transport = getattr(self, field)  # Out is Publishable
        tf_stream = getattr(self, TF_IN_STREAM, None)  # T14: only when In declares tf()
        if tf_stream is not None and getattr(tf_stream, "_transport", None) is not None:
            self._pure.i.tf.source = tf_stream  # feeds the rim's TfBuffer (spec §9.3)
        # T9: the synthesized health Out stream is the wire egress for the pacer's rows.
        rim.start_module(self._pure, health_publish=getattr(self, _HEALTH_STREAM).publish)

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
    if _declares_tf_in(cls):
        namespace["blueprint"] = _tf_pinning_blueprint()
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
    out_fields = spec.out_type.fields()
    out_names = set(out_fields)
    # tf_out() ports assert transforms onto the TF topic (the same rail Module.tf owns);
    # they are exempt from the surface guard so a port named `tf` is legal here.
    tf_out_names = {name for name, fs in out_fields.items() if isinstance(fs, TfOutSpec)}

    overlap = sorted(in_names & out_names)
    if overlap:
        raise PureModuleDefinitionError(
            f"{cls.__qualname__}: field name(s) {', '.join(overlap)} appear in both In and Out — "
            f"the legacy surface is one class annotation / one (name, type) autoconnect key per "
            f"name and cannot express an In∩Out overlap. Rename one side."
        )

    surface = sorted(((in_names | out_names) - tf_out_names) & _MODULE_SURFACE)
    if surface:
        raise PureModuleDefinitionError(
            f"{cls.__qualname__}: field name(s) {', '.join(surface)} shadow the legacy Module "
            f"surface — a synthesized stream annotation would overwrite a Module attribute/method. "
            f"Rename the field(s)."
        )

    if TF_IN_STREAM in (in_names | out_names) and _declares_tf_in(cls):
        raise PureModuleDefinitionError(
            f"{cls.__qualname__}: field name {TF_IN_STREAM} collides with the synthesized tf "
            f"input stream this module needs (its In declares tf() field(s)). Rename the field."
        )

    config_clash = sorted(set(cls.__pure_config_model__.model_fields) & _RESERVED_CONFIG)
    if config_clash:
        raise PureModuleDefinitionError(
            f"{cls.__qualname__}: config field name(s) {', '.join(config_clash)} collide with "
            f"ModuleConfig fields absorbed by the actor. Rename the field(s)."
        )


def _tf_pinning_blueprint() -> Any:
    """``Module.blueprint`` + a ``tf_in`` transport pin on the TF topic (T14).

    Autoconnect would otherwise give ``tf_in`` its name-derived ``/tf_in`` topic, which
    nothing publishes. Pinning here (not at the call site) covers both spellings —
    ``legacy_blueprint(cls)`` and ``legacy_actor(cls).blueprint(...)``.
    """

    @classproperty
    def blueprint(actor_cls: Any) -> Any:
        from dimos.core.coordination.blueprints import Blueprint

        def create(**kwargs: Any) -> Blueprint:
            return Blueprint.create(actor_cls, **kwargs).transports(
                {(TF_IN_STREAM, TFMessage): _tf_transport()}
            )

        return create

    return blueprint


def _declares_tf_in(cls: type[PureModule]) -> bool:
    """Does the pure ``In`` bundle declare any ``pm.tf()`` field? (then ``m.i.tf`` needs a feed)."""
    return any(isinstance(fs, TfSpec) for fs in cls.__pure_step__.in_type.fields().values())


def _tf_topic() -> str:
    """Logical tf channel of the active backend's TF config (LCM ``/tf``, Zenoh ``dimos/tf``).

    Read off ``tf_backend()``'s config rather than hardcoded so the LCM/Zenoh switch keeps
    working; the ``dimos/`` namespace is stripped back to the logical name because
    ``make_transport`` re-applies the right prefix per backend.
    """
    topic = get_type_hints(tf_backend())["config"]().topic
    raw = str(getattr(topic, "topic", topic))
    return raw[len("dimos/") :] if raw.startswith("dimos/") else raw


def _tf_transport() -> Any:
    """The pinned ``tf_in`` transport: ``TFMessage`` on the active backend's TF topic."""
    return make_transport(_tf_topic(), TFMessage)


def _stream_annotation(stream: Any, payload: Any) -> Any:
    """``stream[payload]`` built from runtime values (``In``/``Out`` subscripted dynamically)."""
    return stream[payload]


def _actor_annotations(cls: type[PureModule], config_model: type[ModuleConfig]) -> dict[str, Any]:
    """Synthesize ``In[T]``/``Out[T]``/``health``/``config`` annotations (spec §11.1, P5)."""
    spec = cls.__pure_step__
    annotations: dict[str, Any] = {}
    for field, fs in spec.in_type.fields().items():
        if isinstance(fs, TfSpec):
            continue  # tf() is a side channel resolved off the tf_in feed, not its own topic
        annotations[field] = _stream_annotation(In, _strip_optional(fs.annotation))
    for field, fs in spec.out_type.fields().items():
        if isinstance(fs, TfOutSpec):
            continue  # tf_out asserts onto the TF rail (Module.tf owns it), not a stream
        annotations[field] = _stream_annotation(
            Out, _strip_optional(fs.annotation)
        )  # Optional stripped
    if _declares_tf_in(cls):
        annotations[TF_IN_STREAM] = _stream_annotation(In, TFMessage)  # T14: tf is a plain input
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
