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

"""Step drivers + ``run_over``: drive classified steps over aligned rows (T6).

Spec: ``dimos/pure/tasks/t6-drivers.md``. Engine layer: imports the data
layer (``rows``/``stepspec``/``typing``) and, lazily at run time, the T5
aligner — NEVER ``module.py``/``config.py`` (drivers drive anything
step-shaped; the class layer's only engine edge is ``over()``'s lazy import,
spec §1). ``run_over`` composes: align streams -> drive rows -> yield
stamped Out rows, with teardown per spec §8.

Skeleton status: signatures, error types, and ``RunHooks`` are final; every
driver body raises ``NotImplementedError`` until the implementation lands.
"""

from __future__ import annotations

from collections.abc import Awaitable, Callable, Iterator, Mapping
import dataclasses
import enum
from typing import TYPE_CHECKING, Any, Final, Protocol, TypeVar, runtime_checkable

from dimos.pure.stepspec import StepSpec
from dimos.pure.typing import AsyncStateless, Fold, Mealy, Stateless, Streamable

if TYPE_CHECKING:
    from _typeshed import DataclassInstance

__all__ = [
    "DEFAULT_MAX_INFLIGHT",
    "PureModuleRunError",
    "RunHooks",
    "RunRule",
    "StepError",
    "drive_async",
    "drive_fold",
    "drive_mealy",
    "drive_stateless",
    "run_over",
]

DEFAULT_MAX_INFLIGHT: Final[int] = 1
"""Async in-flight window for modules declaring no ``max_inflight`` field (spec §6.1)."""


# NOTE(T6-impl): relocate to dimos.pure.typing per spec §11.2 (its data-layer
# home), import it back here, and swap ``Streamable = Iterable[Stamped]``.
@runtime_checkable
class Stamped(Protocol):
    """One element of a timestamped stream: anything carrying a float ``ts``."""

    @property
    def ts(self) -> float: ...


class RunRule(enum.Enum):
    """Every runtime driver rule, by its message slug (spec §10)."""

    UNKNOWN_STREAM = "run-unknown-stream"
    INSIDE_LOOP = "run-inside-loop"
    BAD_MAX_INFLIGHT = "run-bad-max-inflight"
    STEP_ERROR = "step-error"
    STEP_NONE_NO_SKIP = "step-none-no-skip"
    FOLD_YIELDED_NONE = "fold-yielded-none"
    FOLD_UNSTAMPED = "fold-unstamped"
    FOLD_FUTURE_TS = "fold-future-ts"
    FOLD_NONMONOTONIC = "fold-nonmonotonic"


class PureModuleRunError(RuntimeError):
    """A pure-module run violated a runtime contract (spec §10.1)."""

    rule: RunRule | None

    def __init__(self, message: str, rule: RunRule | None = None) -> None:
        """Message is release copy per spec §10.3; ``rule`` is machine-readable."""
        super().__init__(message)
        self.rule = rule


class StepError(PureModuleRunError):
    """User step/fold code raised; always chained ``from`` the original (spec §10.2)."""


def _noop() -> None:
    """Default ``RunHooks.teardown``: nothing attached (T7 fills the seam)."""
    return None


async def _anoop() -> None:
    """Default ``RunHooks.ateardown``: nothing attached (T7 fills the seam)."""
    return None


@dataclasses.dataclass
class RunHooks:
    """Per-run tick accounting plus the T7 disposal seam (spec §9, §8.4).

    Counters are plain monotonic ints mutated only by the run's own thread;
    readers (T9) take lock-free, possibly momentarily-stale snapshots.
    """

    ticks: int = 0
    emits: int = 0
    skips: int = 0
    drops: int = 0
    errors: int = 0
    last_error: BaseException | None = None
    teardown: Callable[[], None] = _noop
    ateardown: Callable[[], Awaitable[None]] = _anoop


_TInRow = TypeVar("_TInRow", bound=Stamped)
_TOutRow = TypeVar("_TOutRow", bound="DataclassInstance")  # replace()-stamped (spec §2)
_TFoldOut = TypeVar("_TFoldOut", bound=Stamped)  # fold ts is read, never replaced
_TState = TypeVar("_TState")


def run_over(
    module: Any,  # Any by design: drivers never import the class layer (spec §1)
    spec: StepSpec,
    streams: Mapping[str, Streamable],
    *,
    hooks: RunHooks | None = None,
) -> Iterator[Any]:
    """Validate eagerly, align via T5, dispatch on ``spec.kind`` (spec §4)."""
    raise NotImplementedError


def drive_stateless(
    module: Stateless[_TInRow, _TOutRow],
    rows: Iterator[_TInRow],
    *,
    skips: bool,
    hooks: RunHooks,
) -> Iterator[_TOutRow]:
    """Drive a sync stateless step over rows, stamping emissions with tick ts (spec §5.1)."""
    raise NotImplementedError


def drive_mealy(
    module: Mealy[_TState, _TInRow, _TOutRow],
    rows: Iterator[_TInRow],
    initial: _TState,
    *,
    skips: bool,
    hooks: RunHooks,
) -> Iterator[_TOutRow]:
    """Thread State from ``initial`` through a Mealy step, stamping emissions (spec §5.2)."""
    raise NotImplementedError


def drive_async(
    module: AsyncStateless[_TInRow, _TOutRow],
    rows: Iterator[_TInRow],
    *,
    max_inflight: int,
    skips: bool,
    hooks: RunHooks,
) -> Iterator[_TOutRow]:
    """Windowed async driver behind a sync facade owning a private loop (spec §6)."""
    raise NotImplementedError


def drive_fold(
    module: Fold[_TInRow, _TFoldOut],
    rows: Iterator[_TInRow],
    *,
    hooks: RunHooks,
) -> Iterator[_TFoldOut]:
    """Hand fold one lazy pass over rows; validate self-stamped Out ts (spec §7)."""
    raise NotImplementedError
