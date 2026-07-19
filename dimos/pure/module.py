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

from dimos.pure.config import PureModuleConfig

# T3 SEAM (spec §10): once dimos.pure.stepspec lands, module.py gains
#   from dimos.pure.stepspec import classify
# used as the LAST statement of __init_subclass__.
#
# T4 SEAM (spec §10.2): the binding final form of the class statement is
#   from dimos.pure.typing import EngineSurface
#   class PureModule(EngineSurface):
# EngineSurface hosts over() and the i/o port accessors (deliberately
# UNANNOTATED class attributes, invisible to config-field collection);
# T2 must not redeclare over, i, or o. The implementer wires the base.

__all__ = ["PureModule"]


@dataclass_transform(kw_only_default=True, frozen_default=True)
class PureModule:  # final form: PureModule(EngineSurface) — T4 seam above
    """Base for pure modules: flat annotated fields become typed, frozen config."""

    __pure_config_model__: ClassVar[type[PureModuleConfig]]

    def __init_subclass__(cls, **kwargs: object) -> None:
        """Collect config fields, synthesize the frozen model, then classify the step.

        Implementation order (spec §4.1):
          1. super().__init_subclass__(**kwargs)
          2. fields = _collect_config_fields(cls)                    # config.py
          3. bases = per-spec §4.3 from PureModule-derived cls.__bases__
          4. cls.__pure_config_model__ = _synthesize_config_model(cls, fields, bases)
          5. T3 SEAM — LAST statement, verbatim once stepspec lands:
             cls.__pure_step__ = classify(cls)
        """
        raise NotImplementedError

    def __init__(self, **kwargs: object) -> None:
        """Validate kwargs through the synthesized model; flatten values onto self."""
        raise NotImplementedError

    @property
    def config(self) -> PureModuleConfig:
        """The frozen synthesized config model; ``model_dump()`` is canonical."""
        raise NotImplementedError

    def __setattr__(self, name: str, value: object) -> None:
        """Always raises FrozenModuleError — rebuild the module, never mutate."""
        raise NotImplementedError

    def __delattr__(self, name: str) -> None:
        """Always raises FrozenModuleError."""
        raise NotImplementedError

    def __eq__(self, other: object) -> bool:
        """Identity = class + config: same exact class and equal config."""
        raise NotImplementedError

    def __hash__(self) -> int:
        """hash((type(self), config)) — frozen models hash by value."""
        raise NotImplementedError

    def __repr__(self) -> str:
        """``ClassName(field=value, ...)`` in canonical field order."""
        raise NotImplementedError

    def __reduce__(self) -> tuple[Any, ...]:
        """Pickle/copy as rebuild-from-config: (_rebuild_module, (cls, dump))."""
        raise NotImplementedError

    def warmup(self) -> None:
        """Service-interop lifecycle hook; no-op until T8 fills behavior."""
        raise NotImplementedError

    def start(self) -> None:
        """Service-interop lifecycle hook; no-op until T8 fills behavior."""
        raise NotImplementedError

    def stop(self) -> None:
        """Service-interop lifecycle hook; no-op until T8 fills behavior."""
        raise NotImplementedError


def _rebuild_module(cls: type[PureModule], dump: dict[str, Any]) -> PureModule:
    """Pickle helper: reconstruct a module as cls(**dump)."""
    raise NotImplementedError
