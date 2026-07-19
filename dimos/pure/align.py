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

"""Alignment engine: stamped streams to tick-stamped In rows (T5).

Spec: ``dimos/pure/tasks/t5-align.md``. Pure, pull-based, thread-free,
wall-clock-free: per-port frontiers merge in ``(ts, declaration index)``
order; ``latest``/``interpolate`` fields resolve at each tick's ts; holds are
the pull order, drops are final (spec §4, §5). Data layer only — imports are
stdlib + ``dimos.pure.rows``; drivers (T6) call ``align(spec.in_type,
streams)`` behind ``over()`` and own stream-object coercion.
"""

from __future__ import annotations

from collections.abc import Callable, Iterable, Mapping
import dataclasses
import enum
from typing import Any, Final, Generic, Protocol, TypeAlias, TypeVar

from typing_extensions import Self

from dimos.pure.rows import FieldSpec, In, InterpolateSpec, LatestSpec, TickSpec

__all__ = [
    "AlignRule",
    "AlignStats",
    "Aligner",
    "AlignmentError",
    "Interpolatable",
    "PortStats",
    "Stamped",
    "align",
    "interpolator_for",
    "register_interpolator",
]

_TV = TypeVar("_TV")


class Stamped(Protocol):
    """One payload in a stream: anything carrying a readable float ts."""

    @property
    def ts(self) -> float: ...


InT = TypeVar("InT", bound=In)

InterpolatorFn: TypeAlias = Callable[[Any, Any, float], Any]
"""``fn(a, b, alpha) -> value`` with alpha strictly inside (0, 1) (spec §6)."""

_SUPPORTED_KINDS: Final = (TickSpec, LatestSpec, InterpolateSpec)
"""The In-side sampler kinds this resolver handles (spec §4.1; T11 extends)."""


class Interpolatable(Protocol):
    """Self-describing interpolation: ``a.interpolate(b, alpha)`` -> value at alpha."""

    def interpolate(self, other: Self, alpha: float) -> Self: ...


_INTERPOLATORS: Final[dict[type[Any], InterpolatorFn]] = {}
"""Registry table (spec §6.1); seeded with float lerp at implementation."""


def register_interpolator(tp: type[_TV], fn: Callable[[_TV, _TV, float], _TV]) -> None:
    """Register ``fn(a, b, alpha)`` as the interpolator for ``tp`` (last wins)."""
    raise NotImplementedError


def interpolator_for(tp: type[Any]) -> InterpolatorFn | None:
    """Resolve ``tp``'s interpolator via the spec §6.2 MRO walk, or None."""
    raise NotImplementedError


class AlignRule(enum.Enum):
    """Every alignment rule, by its message slug (spec §9)."""

    NO_TICK = "align-no-tick"
    NOT_IN_ROW = "align-not-in-row"
    UNKNOWN_PORT = "align-unknown-port"
    MISSING_TICK_STREAM = "align-missing-tick-stream"
    MISSING_REQUIRED_STREAM = "align-missing-required-stream"
    SHARED_ITERATOR = "align-shared-iterator"
    UNSUPPORTED_KIND = "align-unsupported-kind"
    NOT_INTERPOLATABLE = "align-not-interpolatable"
    INTERP_UNTYPED = "align-interp-untyped"
    BAD_ITEM = "align-bad-item"
    BAD_TS = "align-bad-ts"
    UNSTAMPED_ITEM = "align-unstamped"


class AlignmentError(TypeError):
    """A stream wiring or stream data contract violation (spec §9)."""

    rule: AlignRule | None

    def __init__(self, message: str, rule: AlignRule | None = None) -> None:
        """Message is release copy per spec §9; ``rule`` is machine-readable."""
        super().__init__(message)
        self.rule = rule


@dataclasses.dataclass
class PortStats:
    """Monotonic per-port ingestion counters (mutated only by the pull loop)."""

    accepted: int = 0
    dropped_nonmonotonic: int = 0


@dataclasses.dataclass
class AlignStats:
    """Monotonic per-aligner counters; safe to read from another thread (spec §8)."""

    ticks_fired: int = 0
    rows_emitted: int = 0
    ticks_dropped: int = 0
    drops_by_field: dict[str, int] = dataclasses.field(default_factory=dict)
    ports: dict[str, PortStats] = dataclasses.field(default_factory=dict)


@dataclasses.dataclass
class _PortState:
    """One port's O(1) merge state: frontier + newest committed item (spec §5.1)."""

    name: str
    index: int
    spec: FieldSpec
    it: Any = None  # Iterator[Stamped] | None; None = exhausted or no stream
    head: Stamped | None = None
    head_ts: float = float("-inf")
    newest: Stamped | None = None
    newest_ts: float = float("-inf")
    last_ts: float = float("-inf")
    interp: InterpolatorFn | None = None
    stats: PortStats = dataclasses.field(default_factory=PortStats)


class Aligner(Generic[InT]):
    """One-shot iterator of aligned In rows; owns counters, owns no resources."""

    def __init__(self, in_type: type[InT], streams: Mapping[str, Iterable[Stamped]]) -> None:
        """Validate wiring eagerly (spec §7); pulls nothing until iteration."""
        raise NotImplementedError

    def __iter__(self) -> Aligner[InT]:
        """Return self (one-shot iterator)."""
        raise NotImplementedError

    def __next__(self) -> InT:
        """Run the pull loop (spec §5.2) to the next emitted row."""
        raise NotImplementedError

    @property
    def stats(self) -> AlignStats:
        """Live counters (spec §8.1); reads are safe at any time."""
        raise NotImplementedError

    @property
    def held_tick_ts(self) -> float | None:
        """ts of the tick currently held mid-merge, else None (spec §8.2)."""
        raise NotImplementedError


def align(in_type: type[InT], streams: Mapping[str, Iterable[Stamped]]) -> Aligner[InT]:
    """Resolve stamped streams into tick-stamped In rows (validates now, pulls lazily)."""
    raise NotImplementedError
