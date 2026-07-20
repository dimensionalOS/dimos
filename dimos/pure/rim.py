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
egress; delivery threads only enqueue (spec §6.2). The legacy core is never
imported here — the bridge (``legacy.py``) adapts in the other direction.
"""

from __future__ import annotations

from collections import deque
from collections.abc import Callable, Iterable, Iterator, Mapping
import dataclasses
import enum
import math
import threading
from typing import Any, Final, Protocol, runtime_checkable

from dimos.pure.align import NOTHING_PENDING, Aligner, AlignStats, align
from dimos.pure.drivers import (
    DEFAULT_MAX_INFLIGHT,
    PureModuleRunError,
    RunHooks,
    _checked_max_inflight,  # single source of the [run-bad-max-inflight] copy (spec §6.1.3)
    drive_async,
    drive_fold,
    drive_mealy,
    drive_stateless,
    run_over,
)
from dimos.pure.resources import attach_resources
from dimos.pure.rows import FieldSpec, TfOutSpec, TfSpec, TickSpec
from dimos.pure.stepspec import StepKind, StepSpec
from dimos.pure.typing import InPort, InPorts, OutPort, OutPorts, Stamped
from dimos.utils.logging_config import setup_logger

__all__ = [
    "DEFAULT_CAPACITY",
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
    "attach_tf_buffer",
    "ports_of",
    "start_module",
    "stats",
    "stop_module",
    "transformer",
    "warmup_module",
]

_TF_STREAM_SPEC: Final = TfSpec(name="tf")
"""Synthetic marker spec for the m.i.tf stream port (not a bundle field; spec §9.1)."""

DEFAULT_CAPACITY: Final[int] = 1
"""Default per-port ingress ring capacity — KeepLast, coalesce-to-latest (spec §5, AD1).

AD1's resolution: default 1, the house latest-wins semantic — a slow consumer
sees the freshest frame, never a backlog of stale ones. Per-port override
``m.i.x.capacity = n`` (shallow drop-oldest buffer) or ``= None`` (unbounded,
lossless — recorder-style modules that must not drop a synchronous burst)."""

STOP_JOIN_TIMEOUT: Final[float] = 5.0
"""stop()'s session-thread join bound — the rim's ONLY wall clock (spec §7.3 #6)."""

_RIM_ATTR: Final = "__pure_rim__"
"""Instance slot for the per-module ``_RimState`` (``object.__setattr__``, spec §4.1)."""

# setup_logger() (not a bare getLogger) so the engine's warnings/errors are
# reachable in forkserver workers — a bare logger has no handler on the worker's
# root and its lines (session death, egress/publish/teardown errors) are silently
# dropped. See dimos/utils/logging_config: it attaches a stdout + jsonl handler.
_LOG: Final = setup_logger()

_RIM_CREATE_LOCK: Final = threading.Lock()
"""Guards first-touch creation of the per-instance rim slot (stable view identity)."""


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
    BAD_CAPACITY = "rim-bad-capacity"
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


# ── message templates (release copy: spec §8) ────────────────────────────────


def _cls(module: Any) -> str:
    """``{module}.{qualname}`` of the module's class — the ``{cls}`` of spec §8."""
    t = type(module)
    return f"{t.__module__}.{t.__qualname__}"


def _e_port_conflict(module: Any, name: str) -> RimError:
    return RimError(
        f"{_cls(module)}.i.{name}: transport and source are mutually exclusive — a port "
        f"is wired to the world once. Unset the other binding first (assign None). "
        f"[rim-port-conflict]",
        RimRule.PORT_CONFLICT,
    )


def _e_live_rebind(module: Any, side: str, name: str) -> RimError:
    return RimError(
        f"{_cls(module)}.{side}.{name} cannot be rebound while the module is live — "
        f"stop() first, rebind, start() again. [rim-live-rebind]",
        RimRule.LIVE_REBIND,
    )


def _e_not_subscribable(module: Any, name: str, value: Any) -> RimError:
    return RimError(
        f"{_cls(module)}.i.{name}: {type(value).__name__} is neither subscribable "
        f"(.subscribe(cb)) nor iterable — bind a transport, a stream, or an iterable of "
        f"stamped msgs. [rim-not-subscribable]",
        RimRule.NOT_SUBSCRIBABLE,
    )


def _e_not_publishable(module: Any, name: str, value: Any) -> RimError:
    return RimError(
        f"{_cls(module)}.o.{name}: {type(value).__name__} has no publish(msg) — bind a "
        f"transport (e.g. pLCMTransport(topic)). [rim-not-publishable]",
        RimRule.NOT_PUBLISHABLE,
    )


def _e_bad_capacity(module: Any, name: str, value: Any) -> RimError:
    return RimError(
        f"{_cls(module)}.i.{name}.capacity must be an int >= 1 (drop-oldest ring) or None "
        f"(unbounded/lossless), got {value!r}. [rim-bad-capacity]",
        RimRule.BAD_CAPACITY,
    )


def _e_already_live(module: Any) -> RimError:
    return RimError(
        f"{_cls(module)} is already live — one session per instance at a time; stop() "
        f"first, or build a second instance (identity is class + config). "
        f"[rim-already-live]",
        RimRule.ALREADY_LIVE,
    )


def _e_unstamped_out(module: Any, name: str, value: Any) -> RimError:
    return RimError(
        f"{_cls(module)}.o.{name} published a {type(value).__name__} with no readable "
        f"finite ts onto a transport — the wire's timestamp authority is the payload's "
        f"own ts. Construct the msg with ts=i.ts. [rim-unstamped-out]",
        RimRule.UNSTAMPED_OUT,
    )


def _e_unknown_port(
    bundle: type[Any], name: str, fields: Mapping[str, FieldSpec]
) -> UnknownPortError:
    names = ", ".join(fields)
    return UnknownPortError(
        f"{bundle.__module__}.{bundle.__qualname__} has no port {name!r} — declared "
        f"ports: {names}. [rim-unknown-port]"
    )


def _stop_stuck_msg(module: Any, timeout: float, held: float | None) -> str:
    return (
        f"{_cls(module)} session thread did not finish within {timeout}s at stop — last "
        f"held tick: {held}. A step may be blocked; the thread is abandoned (daemon). "
        f"[rim-stop-stuck]"
    )


def _e_transform_shape(module: Any, fields: Mapping[str, FieldSpec]) -> RimError:
    names = ", ".join(fields)
    return RimError(
        f"{_cls(module)}.In declares {len(fields)} ports ({names}) — transform() "
        f"equivalence is single-input (the tick port) only; use over() with named "
        f"streams. [rim-transform-shape]",
        RimRule.TRANSFORM_SHAPE,
    )


# ── stats (spec §9) ──────────────────────────────────────────────────────────


@dataclasses.dataclass
class PortIngress:
    """Per-port ingress counters, mutated only under the ring Condition (spec §9)."""

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


# ── binding validation helpers ───────────────────────────────────────────────


def _check_not_live(module: Any, side: str, name: str) -> None:
    """Any binding assignment while the session is RUNNING raises (spec §4.3)."""
    state = getattr(module, _RIM_ATTR, None)
    session = None if state is None else state.session
    if session is not None and session.state == "running":
        raise _e_live_rebind(module, side, name)


def _is_subscribable(value: Any) -> bool:
    return callable(getattr(value, "subscribe", None))


def _unsubscriber(handle: Any) -> Callable[[], None]:
    """Sniff the unsubscribe shape: callable → .dispose → .unsubscribe → no-op (spec §4.4)."""
    if callable(handle):
        return handle  # type: ignore[no-any-return]
    disposer = getattr(handle, "dispose", None)
    if callable(disposer):
        return disposer  # type: ignore[no-any-return]
    unsub = getattr(handle, "unsubscribe", None)
    if callable(unsub):
        return unsub  # type: ignore[no-any-return]
    return lambda: None


def _unwrap_obs(item: Any) -> Any:
    """Source-path Observation unwrap: ts + data duck → the payload (spec §10)."""
    if hasattr(item, "ts") and hasattr(item, "data"):
        return item.data
    return item


def _tf_frames(module: Any, side: str, name: str, spec: FieldSpec) -> tuple[str, str]:
    """Build-resolved frame edge of a tf port; non-tf ports keep raising (T4 §5.5)."""
    if not isinstance(spec, (TfSpec, TfOutSpec)) or ("in" if side == "i" else "out", name) not in (
        getattr(module, "__pure_tf_frames__", {})
    ):
        raise NotImplementedError(f"{_cls(module)}.{side}.{name}.frames — not a tf()/tf_out() port")
    frames: dict[tuple[str, str], tuple[str, str]] = module.__pure_tf_frames__
    return frames[("in" if side == "i" else "out", name)]


def _declares_tf(fields: Mapping[str, FieldSpec]) -> bool:
    """True if the In bundle declares any tf() field (the m.i.tf gate, spec §9.1)."""
    return any(isinstance(s, TfSpec) for s in fields.values())


def _declares_tf_out(fields: Mapping[str, FieldSpec]) -> bool:
    """True if the Out bundle declares any tf_out() field (the tap gate, spec §9.3)."""
    return any(isinstance(s, TfOutSpec) for s in fields.values())


# ── runtime ports (spec §4.1) ────────────────────────────────────────────────


class RimInPort(InPort[Any]):
    """Runtime In-port handle: ``.transport =`` / ``.source =`` (exclusive) + ``.capacity``."""

    capacity: int | None
    """Ingress ring capacity: ``int >= 1`` (drop-oldest) or ``None`` (unbounded); default
    ``DEFAULT_CAPACITY`` (spec §5, AD1). Invalid values raise ``[rim-bad-capacity]``."""

    _module: Any
    _name: str
    _spec: FieldSpec

    def __init__(self, module: Any, name: str, spec: FieldSpec) -> None:
        """Bind-less handle for one In field; validation happens on assignment."""
        d = self.__dict__
        d["_module"] = module
        d["_name"] = name
        d["_spec"] = spec
        d["transport"] = None
        d["source"] = None
        d["capacity"] = DEFAULT_CAPACITY

    def __setattr__(self, name: str, value: Any) -> None:
        """Intercept ``transport``/``source``/``capacity`` (spec §4.3: conflict, live-rebind, shape)."""
        if name == "transport":
            _check_not_live(self._module, "i", self._name)
            if value is not None:
                if self.__dict__["source"] is not None:
                    raise _e_port_conflict(self._module, self._name)
                if not _is_subscribable(value):
                    raise _e_not_subscribable(self._module, self._name, value)
            self.__dict__["transport"] = value
        elif name == "source":
            _check_not_live(self._module, "i", self._name)
            if value is not None:
                if self.__dict__["transport"] is not None:
                    raise _e_port_conflict(self._module, self._name)
                if not _is_subscribable(value) and not isinstance(value, Iterable):
                    raise _e_not_subscribable(self._module, self._name, value)
            self.__dict__["source"] = value
        elif name == "capacity":
            _check_not_live(self._module, "i", self._name)
            if value is not None and not (
                isinstance(value, int) and not isinstance(value, bool) and value >= 1
            ):
                raise _e_bad_capacity(self._module, self._name, value)
            self.__dict__["capacity"] = value
        else:
            object.__setattr__(self, name, value)

    @property
    def frames(self) -> tuple[str, str]:
        """Resolved (parent, child) edge — tf() ports only (t11-tf.md §3.3)."""
        return _tf_frames(self._module, "i", self._name, self._spec)


class RimOutPort(OutPort[Any]):
    """Runtime Out-port handle: ``.transport =`` (Publishable) + ``.subscribe``."""

    _module: Any
    _name: str
    _spec: FieldSpec
    _subs: list[Callable[[Any], None]]

    def __init__(self, module: Any, name: str, spec: FieldSpec) -> None:
        """Bind-less handle for one Out field; validation happens on assignment."""
        d = self.__dict__
        d["_module"] = module
        d["_name"] = name
        d["_spec"] = spec
        d["transport"] = None
        d["_subs"] = []

    def __setattr__(self, name: str, value: Any) -> None:
        """Intercept ``transport`` (spec §4.3: not-publishable, live-rebind)."""
        if name == "transport":
            _check_not_live(self._module, "o", self._name)
            if value is not None and not callable(getattr(value, "publish", None)):
                raise _e_not_publishable(self._module, self._name, value)
            self.__dict__["transport"] = value
        else:
            object.__setattr__(self, name, value)

    def subscribe(self, fn: Callable[[Any], None]) -> Callable[[], None]:
        """Register a local subscriber (runs on the session thread); returns unsubscribe."""
        self._subs.append(fn)

        def unsubscribe() -> None:
            try:
                self._subs.remove(fn)
            except ValueError:
                pass  # already removed — unsubscribe is idempotent

        return unsubscribe

    @property
    def frames(self) -> tuple[str, str]:
        """Asserted (parent, child) edge — tf_out ports only (t11-tf.md §3.3)."""
        return _tf_frames(self._module, "o", self._name, self._spec)


class RimInPorts(InPorts[Any]):
    """``m.i`` runtime view: cached, name-validated ``RimInPort`` per In field."""

    _module: Any
    _bundle: type[Any]
    _fields: dict[str, FieldSpec]
    _ports: dict[str, RimInPort]

    def __init__(self, module: Any, spec: StepSpec) -> None:
        """View keyed off the module's StepSpec (spec §4.1)."""
        self._module = module
        self._bundle = spec.in_type
        self._fields = spec.in_type.fields()
        self._ports = {}

    def __getattr__(self, name: str) -> RimInPort:
        """Field-named port; unknown names raise ``[rim-unknown-port]`` (spec §4.1)."""
        if name.startswith("_"):  # dunder/underscore probes: default lookup semantics
            raise AttributeError(name)
        fields = self.__dict__["_fields"]
        if name == "tf" and name not in fields and _declares_tf(fields):
            # Synthetic tf-stream port: present iff the In bundle declares tf() fields
            # (name collision impossible — D4). Full binding surface (spec §9.1).
            port = self._ports.get("tf")
            if port is None:
                port = RimInPort(self._module, "tf", _TF_STREAM_SPEC)
                self._ports["tf"] = port
            return port
        if name not in fields:
            raise _e_unknown_port(self._bundle, name, fields)
        port = self._ports.get(name)
        if port is None:
            port = RimInPort(self._module, name, fields[name])
            self._ports[name] = port
        return port


class RimOutPorts(OutPorts[Any]):
    """``m.o`` runtime view: cached, name-validated ``RimOutPort`` per Out field."""

    _module: Any
    _bundle: type[Any]
    _fields: dict[str, FieldSpec]
    _ports: dict[str, RimOutPort]

    def __init__(self, module: Any, spec: StepSpec) -> None:
        """View keyed off the module's StepSpec (spec §4.1)."""
        self._module = module
        self._bundle = spec.out_type
        self._fields = spec.out_type.fields()
        self._ports = {}

    def __getattr__(self, name: str) -> RimOutPort:
        """Field-named port; unknown names raise ``[rim-unknown-port]`` (spec §4.1)."""
        if name.startswith("_"):
            raise AttributeError(name)
        fields = self.__dict__["_fields"]
        if name not in fields:
            raise _e_unknown_port(self._bundle, name, fields)
        port = self._ports.get(name)
        if port is None:
            port = RimOutPort(self._module, name, fields[name])
            self._ports[name] = port
        return port


# ── ingress machinery (spec §5) ──────────────────────────────────────────────


class _PortQueue:
    """Bounded ring + shared Condition: delivery threads append, session pulls."""

    def __init__(
        self, capacity: int | None, cond: threading.Condition, ingress: PortIngress
    ) -> None:
        """O(1) enqueue; ``capacity=None`` is unbounded/lossless, else drop-oldest (spec §5, AD1)."""
        self.capacity = capacity
        self.cond = cond
        self.ingress = ingress
        self.items: deque[Any] = deque()
        self.stopped = False

    def enqueue(self, item: Any) -> None:
        """Append + notify; a bounded full ring evicts oldest and counts it (spec §5)."""
        with self.cond:
            if self.stopped:
                return  # sealed — nothing new enters after stop step 2
            if self.capacity is not None and len(self.items) >= self.capacity:
                self.items.popleft()
                self.ingress.dropped_overflow += 1
            self.items.append(item)
            self.ingress.received += 1
            self.cond.notify_all()

    def enqueue_wait(self, item: Any) -> bool:
        """Pump enqueue: block while a bounded ring is full (pull = flow control, D9)."""
        with self.cond:
            while (
                self.capacity is not None and len(self.items) >= self.capacity and not self.stopped
            ):
                self.cond.wait()
            if self.stopped:
                return False
            self.items.append(item)
            self.ingress.received += 1
            self.cond.notify_all()
            return True


class _LiveFeed(Iterator[Any]):
    """Per-port aligner feed: pop, else park on the Condition; drain at stop (spec §5)."""

    def __init__(self, queue: _PortQueue) -> None:
        self._queue = queue

    def __next__(self) -> Any:
        """Blocking, wall-clock-free pull; StopIteration once stopped AND drained."""
        q = self._queue
        with q.cond:
            while True:
                if q.items:
                    item = q.items.popleft()
                    q.cond.notify_all()  # free a ring slot: wake a blocked pump
                    return item
                if q.stopped:
                    raise StopIteration
                q.cond.wait()

    def pull_nowait(self) -> Any:
        """Non-blocking pull: an item, ``NOTHING_PENDING``, or StopIteration when drained.

        The aligner uses this for SECONDARY ports on the live path: a live secondary
        that has nothing queued must not hold the tick (an optional latest() port
        defaults; a wired-but-silent topic would otherwise park the session forever).
        Only the tick port blocks — blocking on the driver IS the module's pacing.
        """
        q = self._queue
        with q.cond:
            if q.items:
                item = q.items.popleft()
                q.cond.notify_all()  # free a ring slot: wake a blocked pump
                return item
            if q.stopped:
                raise StopIteration
            return NOTHING_PENDING


# ── egress helpers (spec §6.3) ───────────────────────────────────────────────


_SCALAR_PAYLOADS: Final = (int, float, complex, str, bytes)
"""Primitive field values exempt from the stamp check — they cannot carry ``ts``
by construction (see the Implementation notes (T8a) appendix, relitigation #2)."""


def _check_stamped_out(module: Any, name: str, value: Any) -> None:
    """Producer-side stamped-currency check for transport-bound ports (AD4, D15)."""
    if isinstance(value, _SCALAR_PAYLOADS):
        return  # scalar currency: structurally unstampable, passes through bare
    try:
        raw = value.ts
    except Exception:
        raise _e_unstamped_out(module, name, value) from None
    if not isinstance(raw, (int, float)) or isinstance(raw, bool) or not math.isfinite(float(raw)):
        raise _e_unstamped_out(module, name, value)


# ── the session (spec §6, §7) ────────────────────────────────────────────────


class _LiveSession:
    """One live run: hooks, resources, queues, the session thread, stats, latches."""

    def __init__(self, module: Any, spec: StepSpec) -> None:
        """Inert until ``warmup``/``start`` (spec §7.1 states NEW→WARM→RUNNING→STOPPED)."""
        self._module = module
        self._spec = spec
        self._state = "new"
        self._cond = threading.Condition()  # the ONE data-path lock (spec §5)
        self._hooks: RunHooks | None = None
        self._aligner: Aligner[Any] | None = None
        self._driver: Iterator[Any] | None = None
        self._tf_ctx: Any = None  # TfContext | None — the tf side channel (T11, spec §9)
        self._thread: threading.Thread | None = None
        self._queues: dict[str, _PortQueue] = {}
        self._ingress: dict[str, PortIngress] = {}
        self._out_ports: dict[str, RimOutPort] = {}
        self._published: dict[str, int] = {}
        self._egress_errors = 0
        self._error: BaseException | None = None
        self._unsubs: list[Callable[[], None]] = []
        self._pumps: list[threading.Thread] = []
        self._closeable_sources: list[Any] = []
        self._stop_lock = threading.Lock()
        self._stop_owner: int | None = None
        self._stop_done = threading.Event()

    @property
    def state(self) -> str:
        """``"new" | "warm" | "running" | "stopped"`` (spec §7.1)."""
        return self._state

    # ── wiring introspection ─────────────────────────────────────────────────

    def _wired_in_ports(self) -> dict[str, RimInPort]:
        """Every In port with a transport or source binding, in declaration order."""
        return _wired_ports(self._module, self._spec)

    # ── lifecycle (spec §7) ──────────────────────────────────────────────────

    def warmup(self) -> None:
        """Validate wiring; create sync-run resources eagerly (spec §7.2, D6)."""
        if self._state != "new":
            return  # idempotent in WARM (spec §7.2)
        # Aligner wiring dry-check: built and discarded without pulling (spec §7.2);
        # T5's eager validation names the module's bundle before any thread exists.
        # An unwired trigger defers wiring validation to start (§6.1.1) — the
        # legacy bridge wires streams only between build() and start() (§11.3).
        fields = self._spec.in_type.fields()
        feeds = _feeds_for(self._module, self._spec)
        if any(isinstance(fields[name], TickSpec) for name in feeds):
            align(self._spec.in_type, feeds)
        hooks = RunHooks()
        attach_resources(self._module, hooks, async_run=self._spec.is_async)
        if not self._spec.is_async:
            # Sync-shaped modules create resources NOW — the legacy build() twin
            # (P13). Async factories need the run loop; they create at start (D6).
            try:
                hooks.warmup()
            except BaseException:
                # G3: dispose the already-created prefix in reverse, re-raise.
                self._safe_teardown(hooks)
                raise
            hooks.warmup = _swapped_noop  # the driver's generator-start must not create twice
        self._hooks = hooks
        self._state = "warm"

    def start(self) -> None:
        """Bind feeds (per-port capacity), compose aligner+driver, spawn the thread (spec §6.1)."""
        hooks = self._hooks
        assert hooks is not None  # start_module warms first (D4)
        try:
            wired = self._wired_in_ports()
            feeds: dict[str, _LiveFeed] = {}
            for name, port in wired.items():
                ingress = PortIngress()
                queue = _PortQueue(port.__dict__["capacity"], self._cond, ingress)
                self._queues[name] = queue
                self._ingress[name] = ingress
                feeds[name] = _LiveFeed(queue)
            # T11: build the tf side channel (buffer + m.i.tf feed + claims) before
            #      the aligner; claims raise here, on the caller thread (spec §9.3).
            self._tf_ctx = self._build_tf_context()
            # 1. Eager aligner: T5's wiring errors name the module at start, on
            #    the caller thread, before any subscription exists (spec §6.1.1).
            self._aligner = align(self._spec.in_type, feeds, tf=self._tf_ctx)
            # 3. Driver dispatch mirroring run_over (spec §6.1.3; D3 — the rim
            #    composes the public pieces, it does not call run_over live).
            self._driver = self._dispatch(self._aligner, hooks)
            if self._tf_ctx is not None and _declares_tf_out(self._spec.out_type.fields()):
                from dimos.pure.tfbuffer import tf_out_tap  # lazy (spec §9.3)

                self._driver = tf_out_tap(self._driver, self._module, self._tf_ctx)
            out_view = ports_of(self._module).o
            self._out_ports = {
                name: getattr(out_view, name) for name in self._spec.out_type.fields()
            }
            self._published = {name: 0 for name in self._out_ports}
            # Subscribe bindings last among the fallible steps (G3 unwind shrinks).
            for name, port in wired.items():
                self._bind_ingress(name, port, self._queues[name])
        except BaseException:
            self._unwind_start(hooks)
            raise
        self._state = "running"
        thread = threading.Thread(
            target=self._run,
            name=f"pure-rim:{type(self._module).__name__}",
            daemon=True,
        )
        self._thread = thread
        thread.start()
        for pump in self._pumps:
            pump.start()

    def _build_tf_context(self) -> Any:
        """Buffer (attached or fresh) + wired m.i.tf feed + claims → TfContext (spec §9.3)."""
        module, spec = self._module, self._spec
        in_tf = _declares_tf(spec.in_type.fields())
        out_tf = _declares_tf_out(spec.out_type.fields())
        if not in_tf and not out_tf:
            return None
        from dimos.pure.tfbuffer import TfBuffer, TfContext  # lazy (spec §9.3)

        state: _RimState = getattr(module, _RIM_ATTR)
        buffer = state.tf_buffer if state.tf_buffer is not None else TfBuffer()
        feed: _LiveFeed | None = None
        if in_tf:
            tf_port = ports_of(module).i._ports.get("tf")  # created only if m.i.tf touched
            if tf_port is not None and (
                tf_port.__dict__["transport"] is not None or tf_port.__dict__["source"] is not None
            ):
                ingress = PortIngress()
                queue = _PortQueue(tf_port.__dict__["capacity"], self._cond, ingress)
                self._queues["tf"] = queue
                self._ingress["tf"] = ingress
                feed = _LiveFeed(queue)
                self._bind_ingress("tf", tf_port, queue)  # pump/subscribe, started with the rest
        return TfContext.for_live(module, spec, buffer, feed)

    def _dispatch(self, rows: Iterator[Any], hooks: RunHooks) -> Iterator[Any]:
        """The run_over dispatch, over live feeds (spec §6.1.3)."""
        module, spec = self._module, self._spec
        if spec.kind is StepKind.MEALY:
            state_type = spec.state_type
            assert state_type is not None  # T3: a MEALY spec always carries State
            return drive_mealy(module, rows, state_type(), skips=spec.skips, hooks=hooks)
        if spec.kind is StepKind.ASYNC_STATELESS:
            raw = getattr(module, "max_inflight", DEFAULT_MAX_INFLIGHT)
            _checked_max_inflight(module, raw)  # D6 validation eagerly, on the caller thread
            # Live window is 1: drive_async pulls rows from inside its loop, so a
            # blocking live feed would park the loop with in-flight tasks neither
            # progressing nor emitting whenever the rings run dry (head-of-line
            # stall; StopIteration is terminal, so no yield-later signal exists).
            # over() keeps the declared window — appendix relitigation #3.
            return drive_async(module, rows, max_inflight=1, skips=spec.skips, hooks=hooks)
        if spec.kind is StepKind.FOLD:
            return drive_fold(module, rows, hooks=hooks)
        return drive_stateless(module, rows, skips=spec.skips, hooks=hooks)

    def _bind_ingress(self, name: str, port: RimInPort, queue: _PortQueue) -> None:
        """Attach one wired port: transport/source subscription or a pump thread (spec §5)."""
        transport = port.__dict__["transport"]
        source = port.__dict__["source"]
        if transport is not None:
            handle = transport.subscribe(queue.enqueue)
            self._unsubs.append(_unsubscriber(handle))
        elif _is_subscribable(source):

            def deliver(item: Any, _queue: _PortQueue = queue) -> None:
                _queue.enqueue(_unwrap_obs(item))

            handle = source.subscribe(deliver)
            self._unsubs.append(_unsubscriber(handle))
        else:  # Iterable source: a pump thread exerts flow control (D9)
            if callable(getattr(source, "close", None)):
                self._closeable_sources.append(source)
            pump = threading.Thread(
                target=self._pump,
                args=(source, queue),
                name=f"pure-rim-pump:{type(self._module).__name__}:{name}",
                daemon=True,
            )
            self._pumps.append(pump)

    def _pump(self, source: Iterable[Any], queue: _PortQueue) -> None:
        """Iterate a pull source, enqueueing with backpressure; ends at exhaustion/stop."""
        try:
            for item in source:
                if not queue.enqueue_wait(_unwrap_obs(item)):
                    return
        except Exception:
            _LOG.warning("%s ingress pump failed", _cls(self._module), exc_info=True)

    def _unwind_start(self, hooks: RunHooks) -> None:
        """G3: reverse-unwind a failed start on the caller thread; original error re-raises."""
        if self._tf_ctx is not None:
            try:
                self._tf_ctx.release()  # drop any claims taken before the failure (spec §9.3)
            except Exception:
                _LOG.warning(
                    "%s tf release raised during unwind", _cls(self._module), exc_info=True
                )
        self._seal_ingestion()
        self._safe_teardown(hooks)
        self._state = "stopped"

    def _safe_teardown(self, hooks: RunHooks) -> None:
        """Run hooks.teardown, logging (not raising) secondary failures."""
        try:
            hooks.teardown()
        except Exception:
            _LOG.warning("%s teardown raised during unwind", _cls(self._module), exc_info=True)

    # ── the session thread (spec §6.1.4, §6.1.5) ─────────────────────────────

    def _run(self) -> None:
        """The live loop: driver rows → egress; teardown via the driver's own finalizer."""
        driver = self._driver
        assert driver is not None
        error: BaseException | None = None
        try:
            for row in driver:
                self._emit(row)
        except BaseException as exc:
            error = exc
            _LOG.error("%s live session died: %r", _cls(self._module), exc, exc_info=exc)
        finally:
            close = getattr(driver, "close", None)
            try:
                if close is not None:
                    close()  # emit-error path: GeneratorExit runs the T6/T7 teardown
            except BaseException as exc:
                if error is None:
                    error = exc
                    _LOG.warning(
                        "%s driver close raised: %r", _cls(self._module), exc, exc_info=exc
                    )
            if self._tf_ctx is not None:
                try:
                    self._tf_ctx.release()  # idempotent with the tap's own finally (spec §9.3)
                except Exception:
                    _LOG.warning("%s tf release raised", _cls(self._module), exc_info=True)
            self._error = error
            self._seal_ingestion()  # a dead session consumes nothing (spec §6.1.5)
            with self._cond:
                self._state = "stopped"

    def _emit(self, row: Any) -> None:
        """Per-field fan-out in Out declaration order (spec §6.3): publish, then subscribers."""
        for name, port in self._out_ports.items():
            value = getattr(row, name)
            if value is None:
                continue  # sparse doctrine: None publishes nothing
            transport = port.__dict__["transport"]
            if transport is not None:
                _check_stamped_out(self._module, name, value)
                transport.publish(value)  # publish errors are session errors (AD3)
                self._published[name] += 1
            for fn in list(port._subs):
                try:
                    fn(value)
                except Exception:
                    self._egress_errors += 1
                    _LOG.warning(
                        "%s.o.%s subscriber raised; egress continues",
                        _cls(self._module),
                        name,
                        exc_info=True,
                    )

    # ── stop (spec §7.3 — the teardown ordering table) ───────────────────────

    def stop(self) -> None:
        """Seal ingestion, drain, dispose reverse, join — idempotent (spec §7.3 table)."""
        me = threading.get_ident()
        with self._stop_lock:
            winner = self._stop_owner is None
            if winner:
                self._stop_owner = me
            owner = self._stop_owner
        if not winner:
            if owner == me:
                return  # re-entrant on the stopping thread: no self-deadlock (step 1)
            self._stop_done.wait()  # concurrent caller JOINS the latch (G4)
            return
        try:
            self._seal_ingestion()  # 2: nothing new enters queues
            with self._cond:  # 3: wake parked feeds and pumps
                for queue in self._queues.values():
                    queue.stopped = True
                self._cond.notify_all()
            thread = self._thread
            if thread is not None:
                if thread.is_alive():
                    # 4-5 run on the session thread (drain → driver finalization);
                    # 6: the bounded join — the rim's only wall clock.
                    thread.join(STOP_JOIN_TIMEOUT)
                    if thread.is_alive():
                        held = self._aligner.held_tick_ts if self._aligner else None
                        _LOG.error(_stop_stuck_msg(self._module, STOP_JOIN_TIMEOUT, held))
            else:
                # 8: stop before start — warmup resources dispose on the caller
                # (the ctx latch keeps it exactly-once).
                hooks = self._hooks
                if hooks is not None:
                    self._safe_teardown(hooks)
            for pump in self._pumps:  # hygiene: pumps are bounded by the stop wake
                pump.join(STOP_JOIN_TIMEOUT)
            with self._cond:
                self._state = "stopped"
        finally:
            self._stop_done.set()

    def _seal_ingestion(self) -> None:
        """Unsubscribe every binding; close closeable sources. Idempotent, thread-safe."""
        with self._cond:
            unsubs = self._unsubs[:]
            self._unsubs.clear()
            sources = self._closeable_sources[:]
            self._closeable_sources.clear()
        for unsub in unsubs:
            try:
                unsub()
            except Exception:
                _LOG.warning("%s unsubscribe raised", _cls(self._module), exc_info=True)
        for source in sources:
            try:
                source.close()
            except Exception:
                _LOG.warning("%s source close raised", _cls(self._module), exc_info=True)

    # ── stats (spec §9) ──────────────────────────────────────────────────────

    def stats(self) -> RimStats:
        """Lock-free counter snapshot (spec §9)."""
        aligner = self._aligner
        hooks = self._hooks
        return RimStats(
            state=self._state,
            hooks=hooks if hooks is not None else RunHooks(),
            align=aligner.stats if aligner is not None else None,
            held_tick_ts=aligner.held_tick_ts if aligner is not None else None,
            ingress=dict(self._ingress),
            published=dict(self._published),
            egress_errors=self._egress_errors,
            error=self._error,
        )


def _swapped_noop() -> None:
    """Replaces hooks.warmup after the eager warmup() creation (spec §7.2)."""
    return None


# ── per-module rim state (spec §4.1) ─────────────────────────────────────────


class _RimState:
    """Per-module-instance rim slot: the port views + the current session."""

    def __init__(self, module: Any) -> None:
        """Build cached ``i``/``o`` views from the module's StepSpec (spec §4.1)."""
        spec: StepSpec = type(module).__pure_step__  # T3-stamped; never re-derived
        self.spec = spec
        self.session: _LiveSession | None = None
        self.tf_buffer: Any = None  # attached shared TfBuffer, or None → fresh per session (T11)
        self._i = RimInPorts(module, spec)
        self._o = RimOutPorts(module, spec)

    @property
    def i(self) -> RimInPorts:
        """The module's input-ports view (stable identity)."""
        return self._i

    @property
    def o(self) -> RimOutPorts:
        """The module's output-ports view (stable identity)."""
        return self._o


# ── module-level entry points (consumed by seams S3/S4 and the bridge) ───────


def ports_of(module: Any) -> _RimState:
    """Create-or-return the module's rim state (typing accessors call this, S3)."""
    state = getattr(module, _RIM_ATTR, None)
    if state is not None:
        return state  # type: ignore[no-any-return]
    if getattr(type(module), "__pure_step__", None) is None:
        # Objects without a T3 step spec have no ports; NotImplementedError keeps
        # the T4 accessor-stub contract for spec-less EngineSurface instances.
        raise NotImplementedError(
            f"{type(module).__name__} has no step spec — runtime ports require a "
            f"classified pure module (step/fold)"
        )
    with _RIM_CREATE_LOCK:
        state = getattr(module, _RIM_ATTR, None)
        if state is None:
            state = _RimState(module)
            object.__setattr__(module, _RIM_ATTR, state)
        return state  # type: ignore[no-any-return]


def attach_tf_buffer(module: Any, buffer: Any) -> None:
    """Store a (possibly shared) TfBuffer for the module's next session (spec §9.2)."""
    state = ports_of(module)
    session = state.session
    if session is not None and session.state == "running":
        raise _e_live_rebind(module, "i", "tf")  # live-rebind guard, like any binding
    state.tf_buffer = buffer


def warmup_module(module: Any) -> None:
    """``m.warmup()`` body (seam S4): validate + eager sync-resource creation (§7.2)."""
    state = ports_of(module)
    session = state.session
    if session is None or session.state == "stopped":
        session = _LiveSession(module, state.spec)
        state.session = session
    session.warmup()


def start_module(module: Any) -> None:
    """``m.start()`` body (seam S4): auto-warm (D4) then go live (spec §6.1).

    Per-port ingress ring capacity is read from each ``m.i.<port>.capacity``
    (default ``DEFAULT_CAPACITY``, ``None`` = unbounded) — no ``port_depth``
    argument (AD1, resolved by Ivan 2026-07-20; the knob rides the port)."""
    state = ports_of(module)
    session = state.session
    if session is not None and session.state == "running":
        raise _e_already_live(module)
    if session is None or session.state == "stopped":
        session = _LiveSession(module, state.spec)
        state.session = session
    if session.state == "new":
        session.warmup()  # D4: start() auto-warms
    session.start()


def stop_module(module: Any) -> None:
    """``m.stop()`` body (seam S4): drain → dispose → join, idempotent (spec §7.3)."""
    state = ports_of(module)
    session = state.session
    if session is not None:
        session.stop()


def stats(module: Any) -> RimStats:
    """Live-session counter snapshot for ``module`` (spec §9, D10)."""
    session = ports_of(module).session
    if session is None:
        return RimStats(
            state="new",
            hooks=RunHooks(),
            align=None,
            held_tick_ts=None,
            ingress={},
            published={},
            egress_errors=0,
            error=None,
        )
    return session.stats()


def transformer(module: Any) -> Callable[[Iterator[Any]], Iterator[Any]]:
    """Single-input transform() equivalence over Observation iterators (spec §10, D11)."""
    spec: StepSpec = ports_of(module).spec
    fields = spec.in_type.fields()
    if len(fields) != 1:
        raise _e_transform_shape(module, fields)
    (tick_name,) = fields

    def transform(rows: Iterator[Any]) -> Iterator[Any]:
        last_obs: list[Any] = [None]

        def payloads() -> Iterator[Stamped]:
            for obs in rows:
                last_obs[0] = obs
                yield obs.data

        for row in run_over(module, spec, {tick_name: payloads()}):
            envelope = last_obs[0]
            assert envelope is not None  # a row implies a consumed input (D11)
            yield envelope.derive(data=row, ts=row.ts)

    return transform


def _wired_ports(module: Any, spec: StepSpec) -> dict[str, RimInPort]:
    """Every In port of ``module`` with a binding, in declaration order (spec §6.1 step 1)."""
    view = ports_of(module).i
    wired: dict[str, RimInPort] = {}
    for name in spec.in_type.fields():
        port = getattr(view, name)
        if port.__dict__["transport"] is not None or port.__dict__["source"] is not None:
            wired[name] = port
    return wired


def _feeds_for(module: Any, spec: StepSpec) -> dict[str, Iterable[Any]]:
    """Collect wired ports into the aligner's stream mapping (spec §6.1 step 1).

    Empty placeholder feeds per wired port — exactly the wiring shape T5's
    eager validation consumes (warmup's dry-check); ``start`` swaps in live
    ``_LiveFeed`` iterators over the same names."""
    return {name: () for name in _wired_ports(module, spec)}
