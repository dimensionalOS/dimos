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

from typing import Any, ClassVar

from typing_extensions import dataclass_transform

from dimos.pure.config import (
    FrozenModuleError,
    PureModuleConfig,
    _collect_config_fields,
    _synthesize_config_model,
)
from dimos.pure.stepspec import StepSpec, classify
from dimos.pure.typing import EngineSurface

# T3 SEAM (spec §10.1) — ACTIVE: classify(cls) runs as the LAST statement of
# __init_subclass__; __pure_step__'s presence certifies that ALL definition-time
# machinery (config included) passed. stepspec never imports module.py.

__all__ = ["PureModule"]


@dataclass_transform(kw_only_default=True, frozen_default=True)
class PureModule(EngineSurface):
    """Base for pure modules: flat annotated fields become typed, frozen config."""

    __pure_config_model__: ClassVar[type[PureModuleConfig]]
    __pure_step__: ClassVar[StepSpec]  # stamped by the T3 seam; dunder, never a field
    __pure_config__: PureModuleConfig  # per-instance store; set via object.__setattr__

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
        cls.__pure_step__ = classify(cls)  # 5 T3 SEAM (§4.1 step 5, §10.1) — LAST statement

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

    def __call__(self, rows: Any) -> Any:
        """Single-input transform() equivalence over an iterator (t8-rim.md §10, S5)."""
        from dimos.pure import rim  # T8 RIM SEAM (S5) — lazy

        return rim.transformer(self)(rows)


def _rebuild_module(cls: type[PureModule], dump: dict[str, Any]) -> PureModule:
    """Pickle helper: reconstruct a module as cls(**dump)."""
    return cls(**dump)
