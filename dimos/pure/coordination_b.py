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

"""Coordinator adapter (T8b skeleton): ``live(PureCls)`` — the legacy bridge.

Spec: ``dimos/pure/tasks/t8-rim-b.md`` §12 (comparison-run suffix; final
path ``dimos/pure/coordination.py``). This file is the ONE sanctioned
bridge: it imports ``dimos.core`` freely (nothing imports back — no cycle)
and synthesizes, per PureModule class, a legacy ``Module`` subclass that is
deployable, autoconnectable and blueprint-instantiable — parity by
construction (spec §2, D1). Sunsets with the legacy system.
"""

from __future__ import annotations

from typing import Any

from dimos.core.module import Module
from dimos.pure.module import PureModule
from dimos.pure.stepspec import PureModuleDefinitionError

__all__ = ["LiveSynthesisError", "live"]


class LiveSynthesisError(PureModuleDefinitionError):
    """``live()`` could not synthesize a legacy shell (spec §11 adapter rows)."""


class _LiveMeta(type):
    """Metaclass of synthesized live classes; registered with copyreg (spec §12.1).

    Carries ``__pure_class__``/``__live_name__`` so the class pickles by
    recipe across the forkserver pipe (parity row P1).
    """

    __pure_class__: type[PureModule]
    __live_name__: str | None


def _reduce_live_class(cls: _LiveMeta) -> tuple[Any, ...]:
    """copyreg reducer: rebuild the synthesized class as ``live(pure_cls, name=…)``."""
    raise NotImplementedError


def live(cls: type[PureModule], *, name: str | None = None) -> type[Module]:
    """Synthesize (memoized) the legacy-Module shell for a PureModule class.

    Spec §12: annotations from ``fields()`` (P9), ``<Name>LiveConfig`` from
    the pure config model over ``ModuleConfig`` (P2-P4, P14),
    ``dedicated_worker = True`` (P18), lifecycle mapped to the rim
    (build→warmup, start→bind+start, stop→rim.stop+super().stop).
    """
    raise NotImplementedError
