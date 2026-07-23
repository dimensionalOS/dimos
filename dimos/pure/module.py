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

"""PureModule base: typed flat-config constructor over a synthesized frozen model.

Spec: dimos/pure/tasks/t2-config.md. Deliberately absent here: ``step``,
``fold``, ``In``, ``Out``, ``State`` (T3 discovers them on subclasses),
``over``/``i``/``o`` (T4/T6/T8), ``checkpoint``/``restore`` (T10).
"""

from __future__ import annotations

from collections.abc import Iterable, Iterator, Mapping
from typing import TYPE_CHECKING, Any, ClassVar, TypeVar, overload

from typing_extensions import dataclass_transform

from dimos.pure.config import (
    FrozenModuleError,
    PureModuleConfig,
    _collect_config_fields,
    _synthesize_config_model,
)
from dimos.pure.rows import TfOutSpec, TfSpec, _merged_specs, format_frame
from dimos.pure.stepspec import PureModuleDefinitionError, StepSpec, classify
from dimos.pure.typing import AsyncStateless, EngineSurface, Fold, Mealy, Stateless

if TYPE_CHECKING:
    from dimos.core.coordination.blueprints import Blueprint

# Solver twins for the __call__ overloads (T4 doctrine: plain typevars unify
# against the concrete step; variant typevars are illegal in overload position).
_TIn = TypeVar("_TIn")
_TOut = TypeVar("_TOut")
_TState = TypeVar("_TState")

# T3 SEAM (spec §10.1) — ACTIVE: classify(cls) runs as the LAST statement of
# __init_subclass__; __pure_step__'s presence certifies that ALL definition-time
# machinery (config included) passed. stepspec never imports module.py.


@dataclass_transform(kw_only_default=True, frozen_default=True)
class PureModule(EngineSurface):
    """Base for pure modules: flat annotated fields become typed, frozen config."""

    __pure_config_model__: ClassVar[type[PureModuleConfig]]
    __pure_step__: ClassVar[StepSpec]  # stamped by the T3 seam; dunder, never a field
    __pure_tf_templates__: ClassVar[dict[tuple[str, str], TfSpec | TfOutSpec]]  # (side, field)
    __pure_config__: PureModuleConfig  # per-instance store; set via object.__setattr__
    __pure_tf_frames__: dict[tuple[str, str], tuple[str, str]]  # T11 build resolution

    def __init_subclass__(cls, **kwargs: object) -> None:
        """Collect config fields, synthesize the frozen model, then classify the step.

        Implementation order (spec §4.1):
          1. super().__init_subclass__(**kwargs)
          2. fields = _collect_config_fields(cls)                    # config.py
          3. bases = per-spec §4.3 from PureModule-derived cls.__bases__
          4. cls.__pure_config_model__ = _synthesize_config_model(cls, fields, bases)
          5. cls.__pure_step__ = classify(cls)   # T3 seam — LAST statement (§10.1)
        """
        super().__init_subclass__(**kwargs)  # 1 cooperative chain (traverses EngineSurface)
        fields = _collect_config_fields(cls)  # 2 §3
        bases: tuple[type[PureModuleConfig], ...] = tuple(  # 3 §4.3 mirrored inheritance
            b.__pure_config_model__
            for b in cls.__bases__
            if b is not PureModule and issubclass(b, PureModule)
        ) or (PureModuleConfig,)
        cls.__pure_config_model__ = _synthesize_config_model(cls, fields, bases)  # 4
        spec = classify(cls)  # 5 T3 SEAM (§4.1 step 5, §10.1)
        cls.__pure_tf_templates__ = _collect_tf_templates(spec)  # T11 declared edges
        cls.__pure_step__ = spec  # LAST — presence certifies every gate above passed

    def __init__(self, **kwargs: object) -> None:
        """Validate kwargs through the synthesized model; flatten values onto self."""
        model = getattr(type(self), "__pure_config_model__", None)
        if model is None:  # only PureModule itself lacks a synthesized model
            raise TypeError(
                "PureModule cannot be instantiated directly — subclass it "
                "(config fields + In/Out + step)"
            )
        cfg = model(**kwargs)  # pydantic validates; raw ValidationError on failure (§5.2)
        object.__setattr__(self, "__pure_config__", cfg)
        for name in model.model_fields:  # class-level access (2.12 deprecates instance)
            object.__setattr__(self, name, getattr(cfg, name))
        templates = type(self).__pure_tf_templates__
        if templates:  # T11: frame templates resolve at build, against this config
            object.__setattr__(self, "__pure_tf_frames__", _resolve_tf_frames(self, templates))

    @property
    def config(self) -> PureModuleConfig:
        """The frozen synthesized config model; ``model_dump()`` is canonical."""
        return self.__pure_config__

    def __setattr__(self, name: str, value: object) -> None:
        """Always raises FrozenModuleError — rebuild the module, never mutate."""
        model = getattr(type(self), "__pure_config_model__", None)
        if model is not None and name in model.model_fields:
            raise FrozenModuleError(
                f"{type(self).__name__}.{name} is frozen config — rebuild: "
                f"{type(self).__name__}(**{{**m.config.model_dump(), {name!r}: ...}})"
            )
        raise FrozenModuleError(
            f"{type(self).__name__} is immutable — per-run state belongs in State, "
            f"heavyweight objects in @resource"
        )

    def __delattr__(self, name: str) -> None:
        """Always raises FrozenModuleError."""
        raise FrozenModuleError(
            f"{type(self).__name__}.{name} cannot be deleted — modules are immutable"
        )

    def __eq__(self, other: object) -> bool:
        """Identity = class + config: same exact class and equal config."""
        if other is self:
            return True
        if not isinstance(other, PureModule):
            return NotImplemented
        return type(other) is type(self) and other.config == self.config

    def __hash__(self) -> int:
        """hash((type(self), config)) — frozen models hash by value."""
        return hash((type(self), self.config))

    def __repr__(self) -> str:
        """``ClassName(field=value, ...)`` in canonical field order."""
        inner = ", ".join(f"{k}={v!r}" for k, v in self.config.model_dump().items())
        return f"{type(self).__name__}({inner})"

    def __reduce__(self) -> tuple[Any, ...]:
        """Pickle/copy as rebuild-from-config: (_rebuild_module, (cls, dump))."""
        return (_rebuild_module, (type(self), self.config.model_dump()))

    def warmup(self) -> None:
        """Validate wiring and create sync-run resources (t8-rim.md §7.2)."""
        from dimos.pure import rim  # T8 RIM SEAM (S4) — lazy

        rim.warmup_module(self)

    def start(self) -> None:
        """Go live: one session thread over the wired ports (t8-rim.md §6)."""
        from dimos.pure import rim  # T8 RIM SEAM (S4) — lazy

        rim.start_module(self)

    def stop(self) -> None:
        """Drain, dispose in reverse, join — idempotent (t8-rim.md §7.3)."""
        from dimos.pure import rim  # T8 RIM SEAM (S4) — lazy

        rim.stop_module(self)

    # ── the one operator (t13-graph.md §3.3): keyword application, positional
    # transformer. Overload order mirrors over(): AsyncStateless first, so a
    # coroutine-returning step cannot unify _TOut with the sync overload.
    @overload
    def __call__(self: AsyncStateless[_TIn, _TOut], **ports: Any) -> _TOut: ...
    @overload
    def __call__(self: Mealy[_TState, _TIn, _TOut], **ports: Any) -> _TOut: ...
    @overload
    def __call__(self: Stateless[_TIn, _TOut], **ports: Any) -> _TOut: ...
    @overload
    def __call__(self: Fold[_TIn, _TOut], **ports: Any) -> _TOut: ...
    @overload
    def __call__(self, rows: Iterable[Any], /) -> Iterator[Any]: ...
    def __call__(self, *args: Any, **ports: Any) -> Any:
        """Keyword ports: symbolic application (T13). Positional iterator: transform (S5)."""
        if args:
            if len(args) > 1 or ports:
                raise TypeError(
                    f"{type(self).__name__}(...) mixes a positional row iterator with "
                    f"port kwargs — transform takes rows only, application takes ports "
                    f"only"
                )
            from dimos.pure import rim  # T8 RIM SEAM (S5) — lazy

            return rim.transformer(self)(args[0])
        from dimos.pure.graph import apply_symbolic  # T13 GRAPH SEAM — lazy

        return apply_symbolic(self, ports)

    @classmethod
    def blueprint(cls, *, name: str | None = None, **kwargs: Any) -> Blueprint:
        """Deployment-face Blueprint via the T8 bridge (t13-graph.md §7.1)."""
        from dimos.pure.legacy import legacy_blueprint  # lazy: the sanctioned core edge

        return legacy_blueprint(cls, name=name, **kwargs)


def _rebuild_module(cls: type[PureModule], dump: dict[str, Any]) -> PureModule:
    """Pickle helper: reconstruct a module as cls(**dump)."""
    return cls(**dump)


# ── T11: tf frame templates (spec: tasks/t11-tf.md §3) ───────────────────────


def _collect_tf_templates(spec: StepSpec) -> dict[tuple[str, str], TfSpec | TfOutSpec]:
    """Declared tf edges keyed (side, field) from both bundles' raw spec tables."""
    templates: dict[tuple[str, str], TfSpec | TfOutSpec] = {}
    for bundle in (spec.in_type, spec.out_type):
        for name, fs in _merged_specs(bundle).items():
            if isinstance(fs, (TfSpec, TfOutSpec)):
                templates[(fs.side, name)] = fs
    return templates


def _resolve_tf_frames(
    module: PureModule, templates: Mapping[tuple[str, str], TfSpec | TfOutSpec]
) -> dict[tuple[str, str], tuple[str, str]]:
    """Resolve declared frame templates against instance config (t11 spec §3.2)."""
    cls = type(module)
    cls_path = f"{cls.__module__}.{cls.__qualname__}"
    values = {name: getattr(module, name) for name in cls.__pure_config_model__.model_fields}
    frames: dict[tuple[str, str], tuple[str, str]] = {}
    writers: dict[frozenset[str], str] = {}
    for (side, name), fs in templates.items():
        parent = _resolved_frame(cls_path, name, fs.parent, values)
        child = _resolved_frame(cls_path, name, fs.child, values)
        if parent == child:
            raise PureModuleDefinitionError(
                f"{cls_path}: tf field {name!r} resolved to identical parent and child "
                f"frames ({parent!r}) — a tf edge relates two distinct frames. "
                f"[tf-self-edge]"
            )
        if side == "out":
            key = frozenset((parent, child))
            prior = writers.get(key)
            if prior is not None:
                raise PureModuleDefinitionError(
                    f"{cls_path}: tf_out fields {prior!r} and {name!r} both assert the "
                    f"edge between {parent!r} and {child!r} — one writer per edge "
                    f"(reversed orientation included); merge them or rename a frame. "
                    f"[tf-duplicate-edge]"
                )
            writers[key] = name
        frames[(side, name)] = (parent, child)
    return frames


def _resolved_frame(cls_path: str, field: str, template: str, values: Mapping[str, Any]) -> str:
    """Resolve one frame template, wrapping errors with build coordinates."""
    try:
        frame = format_frame(template, values)
    except KeyError as exc:
        names = ", ".join(values) or "<none>"
        raise PureModuleDefinitionError(
            f"{cls_path}: tf field {field!r} template {template!r} names {exc.args[0]!r}, "
            f"which is not a config field — frame templates resolve against the module's "
            f"own config fields ({names}). [tf-template-unknown]"
        ) from None
    except ValueError as exc:
        raise PureModuleDefinitionError(
            f"{cls_path}: tf field {field!r} template {template!r}: {exc}. [tf-template-value]"
        ) from None
    if not frame:
        raise PureModuleDefinitionError(
            f"{cls_path}: tf field {field!r} frame template {template!r} resolved to an "
            f"empty frame name — after empty segments drop, at least one segment must "
            f"remain. [tf-frame-empty]"
        )
    return frame
