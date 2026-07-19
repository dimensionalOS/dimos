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

"""Live rim (T8a skeleton): runtime ports, the live loop, lifecycle, stats.

Spec: ``dimos/pure/tasks/t8-rim-b.md`` (comparison-run suffix; final path
``dimos/pure/rim.py``). Engine layer: imports the data layer and the
T5/T6/T7 engine ONLY — never ``dimos.core`` (transports are structural,
spec §4.4). Bodies land with T8a; every signature here is normative.
"""

from __future__ import annotations

from collections.abc import Callable
import dataclasses
import enum
from typing import Any, Final, Protocol

from dimos.pure.align import AlignStats
from dimos.pure.drivers import PureModuleRunError, RunHooks
from dimos.pure.rows import FieldSpec
from dimos.pure.typing import InPort, InPorts, OutPort, OutPorts

__all__ = [
    "DEFAULT_CAPACITY",
    "STOP_JOIN_TIMEOUT_S",
    "LiveSession",
    "RimError",
    "RimInPort",
    "RimInPorts",
    "RimOutPort",
    "RimOutPorts",
    "RimPortStats",
    "RimPublishStats",
    "RimRule",
    "RimStats",
    "SessionState",
    "SourceLike",
    "TransportLike",
    "build_in_ports",
    "build_out_ports",
    "session",
    "start",
    "stop",
    "warmup",
]

DEFAULT_CAPACITY: Final[int] = 1
"""Default per-port ring capacity: KeepLast controller semantics (spec §4.2)."""

STOP_JOIN_TIMEOUT_S: Final[float] = 5.0
"""Engine-thread join bound at stop(); lifecycle boundary, not data path (spec §7.3)."""

PORTS_IN_ATTR: Final = "__pure_ports_in__"
"""Instance slot for the cached ``RimInPorts`` view (spec §3)."""

PORTS_OUT_ATTR: Final = "__pure_ports_out__"
"""Instance slot for the cached ``RimOutPorts`` view (spec §3)."""

RIM_ATTR: Final = "__pure_rim__"
"""Instance slot for the current/last ``LiveSession`` (spec §3)."""


class RimRule(enum.Enum):
    """Every runtime rim rule, by its message slug (spec §11)."""

    BOTH_BINDINGS = "rim-both-bindings"
    BAD_TRANSPORT = "rim-bad-transport"
    BAD_SOURCE = "rim-bad-source"
    BAD_CAPACITY = "rim-bad-capacity"
    REBIND_RUNNING = "rim-rebind-running"
    ALREADY_RUNNING = "rim-already-running"
    PUBLISH_ERROR = "rim-publish-error"  # log-only accounting slug, never raised


class RimError(PureModuleRunError):
    """The rim violated a runtime binding/lifecycle contract (spec §11)."""

    rim_rule: RimRule | None

    def __init__(self, message: str, rim_rule: RimRule | None = None) -> None:
        """Message is release copy per spec §11; ``rim_rule`` is machine-readable."""
        super().__init__(message, None)
        self.rim_rule = rim_rule


class TransportLike(Protocol):
    """Structural transport: publish + subscribe-with-unsubscribe (spec §4.4)."""

    def publish(self, msg: Any, /) -> None: ...
    def subscribe(self, callback: Callable[[Any], None], /) -> Callable[[], None]: ...


class SourceLike(Protocol):
    """Structural local source: subscribe returning an unsub callable or Disposable."""

    def subscribe(self, callback: Callable[[Any], None], /) -> Any: ...


class SessionState(enum.Enum):
    """Live-session lifecycle states (spec §7)."""

    IDLE = "idle"
    WARMED = "warmed"
    RUNNING = "running"
    STOPPED = "stopped"


# ── stats (spec §10) ─────────────────────────────────────────────────────────


@dataclasses.dataclass
class RimPortStats:
    """Per-In-port ring counters; mutated by the delivering thread only."""

    delivered: int = 0
    rim_dropped: int = 0


@dataclasses.dataclass
class RimPublishStats:
    """Per-Out-port fan-out counters; mutated by the engine thread only."""

    published: int = 0
    publish_errors: int = 0


@dataclasses.dataclass
class RimStats:
    """Composed live counters: rings + T5 align + T6 run (spec §10)."""

    ports: dict[str, RimPortStats]
    publish: dict[str, RimPublishStats]
    align: AlignStats
    run: RunHooks


# ── runtime port handles (spec §4) ───────────────────────────────────────────


class RimInPort(InPort[Any]):
    """Runtime input-port handle: transport/source binding + ring capacity."""

    capacity: int | None

    def __init__(self, module: Any, bundle: str, name: str, spec: FieldSpec) -> None:
        """Unbound handle for one In field; validation happens at assignment."""
        raise NotImplementedError

    def __setattr__(self, name: str, value: Any) -> None:
        """Duck-validate transport/source/capacity; enforce exclusivity + rebind rules (§4.2)."""
        raise NotImplementedError


class RimOutPort(OutPort[Any]):
    """Runtime output-port handle: transport binding + local subscribers."""

    def __init__(self, module: Any, bundle: str, name: str, spec: FieldSpec) -> None:
        """Unbound handle for one Out field."""
        raise NotImplementedError

    def __setattr__(self, name: str, value: Any) -> None:
        """Duck-validate transport assignment; enforce rebind rules (§4.3)."""
        raise NotImplementedError

    def subscribe(self, fn: Callable[[Any], None]) -> Callable[[], None]:
        """Register a local subscriber; returns an unsubscribe callable (§4.3)."""
        raise NotImplementedError


class RimInPorts(InPorts[Any]):
    """Per-module input-ports view; one cached ``RimInPort`` per In field (§4.1)."""

    def __init__(self, module: Any) -> None:
        """Build the view from ``step_spec(type(module)).in_type.fields()``."""
        raise NotImplementedError

    def __getattr__(self, name: str) -> RimInPort:
        """Field-named handle; unknown names raise AttributeError naming the ports."""
        raise NotImplementedError


class RimOutPorts(OutPorts[Any]):
    """Per-module output-ports view; one cached ``RimOutPort`` per Out field (§4.1)."""

    def __init__(self, module: Any) -> None:
        """Build the view from ``step_spec(type(module)).out_type.fields()``."""
        raise NotImplementedError

    def __getattr__(self, name: str) -> RimOutPort:
        """Field-named handle; unknown names raise AttributeError naming the ports."""
        raise NotImplementedError


# ── the live session (spec §5, §7) ───────────────────────────────────────────


class LiveSession:
    """One live run: rings, blocking iterators, engine thread, composed driver.

    Built by ``start()``; never restarted — a stopped session is replaced by
    a fresh one (fresh RunHooks, fresh run resources — T7 per-run doctrine).
    """

    module: Any
    state: SessionState
    error: BaseException | None

    def __init__(self, module: Any) -> None:
        """Inert container; ring/driver construction happens in ``start()`` (§7.2)."""
        raise NotImplementedError

    @property
    def stats(self) -> RimStats:
        """Live counters; lock-free snapshots, safe from any thread (§10)."""
        raise NotImplementedError


# ── module-level lifecycle surface (what the §13 seams delegate to) ──────────


def build_in_ports(module: Any) -> RimInPorts:
    """Return the module's cached input-ports view, building it on first access."""
    raise NotImplementedError


def build_out_ports(module: Any) -> RimOutPorts:
    """Return the module's cached output-ports view, building it on first access."""
    raise NotImplementedError


def warmup(module: Any) -> None:
    """Create sync-run resources now; async modules validate only (spec §7.1)."""
    raise NotImplementedError


def start(module: Any) -> None:
    """Validate wiring, subscribe bindings, spawn the engine thread (spec §7.2)."""
    raise NotImplementedError


def stop(module: Any) -> None:
    """Idempotent reverse-order drain: unsubscribe, close, join, dispose (spec §7.3)."""
    raise NotImplementedError


def session(module: Any) -> LiveSession | None:
    """The module's current/last live session, or None when never started."""
    raise NotImplementedError
