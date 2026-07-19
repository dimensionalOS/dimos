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

"""Live rim: runtime ports, ingress queues, the live loop, lifecycle (T8a).

Spec: ``dimos/pure/tasks/t8-rim.md``. Rim core — imports the ENGINE only
(``typing``/``rows``/``stepspec``/``align``/``drivers``/``resources``) and is
transport-blind: bindings are structural ``Publishable``/``Subscribable``
objects (spec §4.4). One session thread per module runs aligner, driver, and
egress; delivery threads only enqueue (spec §6.2). Never imports
``dimos.core`` — the legacy bridge (``dimos/pure/legacy.py``) adapts in the
other direction.
"""

from __future__ import annotations

from collections.abc import Callable, Iterable, Iterator, Mapping
import dataclasses
import enum
import threading
from typing import Any, Final, Protocol, runtime_checkable

from dimos.pure.align import Aligner, AlignStats, align  # noqa: F401  (spec §6.1)
from dimos.pure.drivers import (  # noqa: F401  (spec §6.1 composition)
    PureModuleRunError,
    RunHooks,
    drive_async,
    drive_fold,
    drive_mealy,
    drive_stateless,
)
from dimos.pure.resources import attach_resources  # noqa: F401  (spec §6.1.2)
from dimos.pure.rows import FieldSpec
from dimos.pure.stepspec import StepSpec
from dimos.pure.typing import InPort, InPorts, OutPort, OutPorts

__all__ = [
    "DEFAULT_PORT_DEPTH",
    "STOP_JOIN_TIMEOUT",
    "PortIngress",
    "Publishable",
    "RimError",
    "RimInPort",
    "RimInPorts",
    "RimOutPort",
    "RimOutPorts",
    "RimRule",
    "RimStats",
    "Subscribable",
    "ports_of",
    "start_module",
    "stats",
    "stop_module",
    "transformer",
    "warmup_module",
]

DEFAULT_PORT_DEPTH: Final[int] = 256
"""Ingress ring depth per wired In port; drop-oldest beyond it (spec §5, D5)."""

STOP_JOIN_TIMEOUT: Final[float] = 5.0
"""stop()'s session-thread join bound — the rim's ONLY wall clock (spec §7.3 #6)."""

_RIM_ATTR: Final = "__pure_rim__"
"""Instance slot for the per-module ``_RimState`` (``object.__setattr__``, spec §4.1)."""


# ── structural transport protocols (spec §4.4) ───────────────────────────────


@runtime_checkable
class Publishable(Protocol):
    """Anything an Out port can publish to: dimos transports, legacy Out streams."""

    def publish(self, msg: Any) -> None: ...


@runtime_checkable
class Subscribable(Protocol):
    """Anything an In port can consume by callback; returns an unsubscribe handle."""

    def subscribe(self, callback: Callable[[Any], Any]) -> Any: ...


# ── errors (spec §8) ─────────────────────────────────────────────────────────


class RimRule(enum.Enum):
    """Every rim rule, by its message slug (spec §8)."""

    PORT_CONFLICT = "rim-port-conflict"
    LIVE_REBIND = "rim-live-rebind"
    NOT_SUBSCRIBABLE = "rim-not-subscribable"
    NOT_PUBLISHABLE = "rim-not-publishable"
    ALREADY_LIVE = "rim-already-live"
    UNSTAMPED_OUT = "rim-unstamped-out"
    UNKNOWN_PORT = "rim-unknown-port"
    STOP_STUCK = "rim-stop-stuck"
    TRANSFORM_SHAPE = "rim-transform-shape"


class RimError(PureModuleRunError):
    """The rim violated a live-wiring or lifecycle contract (spec §8)."""

    rim_rule: RimRule | None

    def __init__(self, message: str, rim_rule: RimRule | None = None) -> None:
        """Message is release copy per spec §8; ``rim_rule`` is machine-readable."""
        super().__init__(message, None)
        self.rim_rule = rim_rule


class UnknownPortError(AttributeError):
    """View attribute miss; AttributeError subclass so getattr defaults still work."""

    rim_rule: RimRule = RimRule.UNKNOWN_PORT


# ── stats (spec §9) ──────────────────────────────────────────────────────────


@dataclasses.dataclass
class PortIngress:
    """Per-port ingress counters, mutated only by delivery callbacks (spec §9)."""

    received: int = 0
    dropped_overflow: int = 0


@dataclasses.dataclass
class RimStats:
    """Snapshot of a module's live-session counters (spec §9); wall-clock-free."""

    state: str
    hooks: RunHooks
    align: AlignStats | None
    held_tick_ts: float | None
    ingress: dict[str, PortIngress]
    published: dict[str, int]
    egress_errors: int
    error: BaseException | None


# ── runtime ports (spec §4.1) ────────────────────────────────────────────────


class RimInPort(InPort[Any]):
    """Runtime In-port handle: ``.transport =`` / ``.source =``, mutually exclusive."""

    def __init__(self, module: Any, name: str, spec: FieldSpec) -> None:
        """Bind-less handle for one In field; validation happens on assignment."""
        raise NotImplementedError

    def __setattr__(self, name: str, value: Any) -> None:
        """Intercept ``transport``/``source`` (spec §4.3: conflict, live-rebind, shape)."""
        raise NotImplementedError


class RimOutPort(OutPort[Any]):
    """Runtime Out-port handle: ``.transport =`` (Publishable) + ``.subscribe``."""

    def __init__(self, module: Any, name: str, spec: FieldSpec) -> None:
        """Bind-less handle for one Out field; validation happens on assignment."""
        raise NotImplementedError

    def __setattr__(self, name: str, value: Any) -> None:
        """Intercept ``transport`` (spec §4.3: not-publishable, live-rebind)."""
        raise NotImplementedError

    def subscribe(self, fn: Callable[[Any], None]) -> Callable[[], None]:
        """Register a local subscriber (session thread); returns unsubscribe (spec §4.3)."""
        raise NotImplementedError

    @property
    def frames(self) -> tuple[str, str]:
        """tf_out ports only — raises until T11 defines it (t4-typing.md §5.5)."""
        raise NotImplementedError


class RimInPorts(InPorts[Any]):
    """``m.i`` runtime view: cached, name-validated ``RimInPort`` per In field."""

    def __init__(self, module: Any, spec: StepSpec) -> None:
        """View keyed off the module's StepSpec (spec §4.1)."""
        raise NotImplementedError

    def __getattr__(self, name: str) -> RimInPort:
        """Field-named port; unknown names raise ``[rim-unknown-port]`` (spec §4.1)."""
        raise NotImplementedError


class RimOutPorts(OutPorts[Any]):
    """``m.o`` runtime view: cached, name-validated ``RimOutPort`` per Out field."""

    def __init__(self, module: Any, spec: StepSpec) -> None:
        """View keyed off the module's StepSpec (spec §4.1)."""
        raise NotImplementedError

    def __getattr__(self, name: str) -> RimOutPort:
        """Field-named port; unknown names raise ``[rim-unknown-port]`` (spec §4.1)."""
        raise NotImplementedError


# ── ingress machinery (spec §5) ──────────────────────────────────────────────


class _PortQueue:
    """Bounded ring + shared Condition: delivery threads append, session pulls."""

    def __init__(self, depth: int, cond: threading.Condition, ingress: PortIngress) -> None:
        """O(1) enqueue with drop-oldest accounting (spec §5)."""
        raise NotImplementedError

    def enqueue(self, item: Any) -> None:
        """Append + notify; full ring evicts oldest and counts it (spec §5)."""
        raise NotImplementedError


class _LiveFeed(Iterator[Any]):
    """Per-port aligner feed: pop, else park on the Condition; drain at stop (spec §5)."""

    def __next__(self) -> Any:
        """Blocking, wall-clock-free pull; StopIteration once stopped AND drained."""
        raise NotImplementedError


# ── the session (spec §6, §7) ────────────────────────────────────────────────


class _LiveSession:
    """One live run: hooks, resources, queues, the session thread, stats, latches."""

    def __init__(self, module: Any, spec: StepSpec) -> None:
        """Inert until ``warmup``/``start`` (spec §7.1 states NEW→WARM→RUNNING→STOPPED)."""
        raise NotImplementedError

    def warmup(self) -> None:
        """Validate wiring; create sync-run resources eagerly (spec §7.2, D6)."""
        raise NotImplementedError

    def start(self, port_depth: Mapping[str, int] | None = None) -> None:
        """Bind feeds, compose aligner+driver, spawn the session thread (spec §6.1)."""
        raise NotImplementedError

    def stop(self) -> None:
        """Seal ingestion, drain, dispose reverse, join — idempotent (spec §7.3 table)."""
        raise NotImplementedError

    def stats(self) -> RimStats:
        """Lock-free counter snapshot (spec §9)."""
        raise NotImplementedError


class _RimState:
    """Per-module-instance rim slot: the port views + the current session."""

    def __init__(self, module: Any) -> None:
        """Build cached ``i``/``o`` views from the module's StepSpec (spec §4.1)."""
        raise NotImplementedError

    @property
    def i(self) -> RimInPorts:
        """The module's input-ports view (stable identity)."""
        raise NotImplementedError

    @property
    def o(self) -> RimOutPorts:
        """The module's output-ports view (stable identity)."""
        raise NotImplementedError


# ── module-level entry points (consumed by seams S3/S4 and the bridge) ───────


def ports_of(module: Any) -> _RimState:
    """Create-or-return the module's rim state (typing accessors call this, S3)."""
    raise NotImplementedError


def warmup_module(module: Any) -> None:
    """``m.warmup()`` body (seam S4): validate + eager sync-resource creation (§7.2)."""
    raise NotImplementedError


def start_module(module: Any, *, port_depth: Mapping[str, int] | None = None) -> None:
    """``m.start()`` body (seam S4): auto-warm (D4) then go live (spec §6.1)."""
    raise NotImplementedError


def stop_module(module: Any) -> None:
    """``m.stop()`` body (seam S4): drain → dispose → join, idempotent (spec §7.3)."""
    raise NotImplementedError


def stats(module: Any) -> RimStats:
    """Live-session counter snapshot for ``module`` (spec §9, D10)."""
    raise NotImplementedError


def transformer(module: Any) -> Callable[[Iterator[Any]], Iterator[Any]]:
    """Single-input transform() equivalence over Observation iterators (spec §10, D11)."""
    raise NotImplementedError


def _feeds_for(module: Any, spec: StepSpec) -> dict[str, Iterable[Any]]:
    """Collect wired ports into the aligner's stream mapping (spec §6.1 step 1)."""
    raise NotImplementedError
