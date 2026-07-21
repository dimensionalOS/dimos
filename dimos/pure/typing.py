#!/usr/bin/env python3
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

"""Static typing surface for pure modules (spec: dimos/pure/tasks/t4-typing.md).

Step-shape protocols, the typed engine facade (``over()``), and the ``m.i`` /
``m.o`` port-handle descriptors. Zero-runtime-dep data layer: imports nothing
from the engine at module scope. Deliberately named ``typing`` inside the
package — import it absolutely (``from dimos.pure.typing import Stateless``,
or ``import dimos.pure.typing as pure_typing``) and never bind the bare name
``typing`` to this module.
"""

from __future__ import annotations

from collections.abc import Awaitable, Callable, Iterable, Iterator
from typing import (
    TYPE_CHECKING,
    Any,
    Generic,
    Protocol,
    TypeAlias,
    TypeVar,
    overload,
    runtime_checkable,
)

if TYPE_CHECKING:
    from dimos.pure.debugrec import Debug
    from dimos.pure.tfbuffer import TfSource

__all__ = [
    "AsyncStateless",
    "EngineSurface",
    "Fold",
    "InPort",
    "InPorts",
    "Mealy",
    "OutPort",
    "OutPorts",
    "Stamped",
    "Stateless",
]

# ── type variables ───────────────────────────────────────────────────────────
# Declaration pair: variance belongs to the protocols' Generic params.
TIn = TypeVar("TIn", contravariant=True)
TOut = TypeVar("TOut", covariant=True)
TState = TypeVar("TState")  # both param and return position → invariant

# Solver twins: plain (invariant) typevars for overload signatures, where mypy
# unifies them against a concrete step. Variant typevars are illegal there.
_TIn = TypeVar("_TIn")
_TOut = TypeVar("_TOut")
_TState = TypeVar("_TState")

_TBundle = TypeVar("_TBundle")  # ports-view parameter (a bundle class)
_TMsg = TypeVar("_TMsg")  # port-handle parameter (a field's value type)


@runtime_checkable
class Stamped(Protocol):
    """One element of a timestamped stream: anything carrying a float ``ts``.

    THE protocol's single home (wave-B reconciliation): ``align`` and the T6
    drivers consume it from here. Read-only property spelling per §5.3
    doctrine — satisfied by frozen row fields, plain-attribute msgs, and
    property implementations alike; a plain ``ts: float`` member would demand
    settability and reject rows.
    """

    @property
    def ts(self) -> float: ...


# A stream argument is anything the aligner can iter() per run: memory2
# Stream objects (yield stamped Observations), lists of rows/msgs (tests),
# generators (one run). T4 §13.1 resolved here (T6 D12).
Streamable: TypeAlias = Iterable[Stamped]


# ── step-shape protocols ─────────────────────────────────────────────────────
# TIn/TOut are permanently unbound: a protocol describes step shape only;
# bundle-ness is enforced at import time by T3, not by the type system.
# Data params are positional-only (`/`) so implementations keep parameter-name
# freedom — the engine invokes steps positionally (T3 coordination, spec §2).
class Stateless(Protocol[TIn, TOut]):
    """Sync step: one In row to one Out row; ``None`` skips the tick."""

    def step(self, i: TIn, /) -> TOut | None: ...


class AsyncStateless(Protocol[TIn, TOut]):
    """Awaitable-returning step (``async def step``); None skips the tick."""

    def step(self, i: TIn, /) -> Awaitable[TOut | None]: ...


class Mealy(Protocol[TState, TIn, TOut]):
    """Stateful step: (State, In) to (State, Out | None)."""

    def step(self, state: TState, i: TIn, /) -> tuple[TState, TOut | None]: ...


class Fold(Protocol[TIn, TOut]):
    """Escape hatch: the generator owns the loop and stamps its own rows."""

    def fold(self, rows: Iterator[TIn], /) -> Iterator[TOut]: ...


# ── port handles (static surface; runtime instances come from the rim, T8) ──
class InPort(Generic[_TMsg]):
    """Handle for one input field's port: bind a transport or a local source."""

    transport: Any  # settable; T8 tightens to the transport protocol
    source: Any  # settable; mutually exclusive with transport (T8)

    @property
    def frames(self) -> tuple[str, str]:
        """Sampled (parent, child) frame edge — tf() ports only (T11)."""
        raise NotImplementedError


class OutPort(Generic[_TMsg]):
    """Handle for one output field's port: bind, subscribe, introspect."""

    transport: Any  # settable; T8 tightens to the transport protocol

    @property
    def frames(self) -> tuple[str, str]:
        """Asserted (parent, child) frame edge — tf_out ports only (T11)."""
        raise NotImplementedError

    def subscribe(self, fn: Callable[[_TMsg], None]) -> Any:
        """Deliver each published row to ``fn``; returns the rim's handle."""
        raise NotImplementedError


class InPorts(Generic[_TBundle]):
    """Per-module input-ports view: one ``InPort`` per In-bundle field."""

    def __getattr__(self, name: str) -> InPort[Any]:
        """Field-named port; unknown names raise at runtime (rim validates)."""
        raise NotImplementedError


class OutPorts(Generic[_TBundle]):
    """Per-module output-ports view: one ``OutPort`` per Out-bundle field."""

    def __getattr__(self, name: str) -> OutPort[Any]:
        """Field-named port; unknown names raise at runtime (rim validates)."""
        raise NotImplementedError


# ── port accessors: the m.i / m.o descriptors ───────────────────────────────
# Same trick as over(), one level down: __get__ is overloaded on a
# protocol-typed instance, so m.i / m.o are typed per-module with zero
# annotations on the module. Order mirrors over(): AsyncStateless first, so a
# coroutine-returning step cannot unify TOut with the sync overload.
class _InAccessor:
    """``m.i``: typed view of the module's input ports."""

    @overload
    def __get__(self, obj: None, owner: type) -> _InAccessor: ...
    @overload
    def __get__(
        self, obj: AsyncStateless[_TIn, _TOut], owner: type | None = None
    ) -> InPorts[_TIn]: ...
    @overload
    def __get__(
        self, obj: Mealy[_TState, _TIn, _TOut], owner: type | None = None
    ) -> InPorts[_TIn]: ...
    @overload
    def __get__(self, obj: Stateless[_TIn, _TOut], owner: type | None = None) -> InPorts[_TIn]: ...
    @overload
    def __get__(self, obj: Fold[_TIn, _TOut], owner: type | None = None) -> InPorts[_TIn]: ...
    def __get__(self, obj: Any, owner: type | None = None) -> Any:
        if obj is None:
            return self  # class access: the accessor itself (first overload)
        from dimos.pure.rim import ports_of  # T8 RIM SEAM (S3) — lazy: sanctioned edge #2 (t4 §5.5)

        return ports_of(obj).i


class _OutAccessor:
    """``m.o``: typed view of the module's output ports."""

    @overload
    def __get__(self, obj: None, owner: type) -> _OutAccessor: ...
    @overload
    def __get__(
        self, obj: AsyncStateless[_TIn, _TOut], owner: type | None = None
    ) -> OutPorts[_TOut]: ...
    @overload
    def __get__(
        self, obj: Mealy[_TState, _TIn, _TOut], owner: type | None = None
    ) -> OutPorts[_TOut]: ...
    @overload
    def __get__(
        self, obj: Stateless[_TIn, _TOut], owner: type | None = None
    ) -> OutPorts[_TOut]: ...
    @overload
    def __get__(self, obj: Fold[_TIn, _TOut], owner: type | None = None) -> OutPorts[_TOut]: ...
    def __get__(self, obj: Any, owner: type | None = None) -> Any:
        if obj is None:
            return self  # class access: the accessor itself (first overload)
        from dimos.pure.rim import ports_of  # T8 RIM SEAM (S3) — lazy: sanctioned edge #2 (t4 §5.5)

        return ports_of(obj).o


# ── the typed engine facade ─────────────────────────────────────────────────
class EngineSurface:
    """Typed engine facade PureModule inherits; declares no step of its own.

    Invariant: this class (and PureModule) must never declare ``step`` or
    ``fold`` — a base-level step would make every subclass match the protocols
    with the base's types and destroy per-module inference.
    """

    i = _InAccessor()
    o = _OutAccessor()

    @overload
    def over(
        self: AsyncStateless[_TIn, _TOut],
        *,
        tf: TfSource | None = None,
        debug: bool | str | Debug | None = None,
        **streams: Streamable,
    ) -> Iterator[_TOut]: ...
    @overload
    def over(
        self: Mealy[_TState, _TIn, _TOut],
        *,
        tf: TfSource | None = None,
        debug: bool | str | Debug | None = None,
        **streams: Streamable,
    ) -> Iterator[_TOut]: ...
    @overload
    def over(
        self: Stateless[_TIn, _TOut],
        *,
        tf: TfSource | None = None,
        debug: bool | str | Debug | None = None,
        **streams: Streamable,
    ) -> Iterator[_TOut]: ...
    @overload
    def over(
        self: Fold[_TIn, _TOut],
        *,
        tf: TfSource | None = None,
        debug: bool | str | Debug | None = None,
        **streams: Streamable,
    ) -> Iterator[_TOut]: ...
    def over(
        self: Any, *, tf: Any = None, debug: Any = None, **streams: Streamable
    ) -> Iterator[Any]:
        """Align streams, drive the step, yield typed Out rows (T5 + T6)."""
        from dimos.pure.debugrec import session_for  # lazy: sanctioned recorder edge
        from dimos.pure.drivers import run_over  # lazy: sanctioned edge #2
        from dimos.pure.stepspec import step_spec  # lazy: sanctioned edge #1

        return run_over(
            self, step_spec(type(self)), streams, tf=tf, debug=session_for(self, debug=debug)
        )
