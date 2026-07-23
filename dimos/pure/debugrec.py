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

"""Debug recorder: per-module flight recorder + replay over one mem2 store (T15).

Spec: ``dimos/pure/tasks/t15-debug.md`` (incl. its Implementation contract).
Capture: the tick path only appends plain-data events to a bounded drop-oldest
ring; a drain caller (T9 pacer live, inline under ``over()``) moves ring →
``DIMOS_RUN_LOG_DIR/debug.db`` — mem2 is NEVER touched on the tick path.
Read: ``load()`` → ``DebugRun`` → per-module ``ModuleDebug`` with ``ticks()``/
``drops()``/``replay()``/``fixture()``. The file is ``debugrec`` (not
``debug``) — the name ``debug`` is the API's toggle spelling (``over(debug=)``,
``DIMOS_PURE_DEBUG``) and the pm surface re-exports names, never submodules;
the tfbuffer-not-tf precedent (t11-tf.md §18.1) applies. Module-scope imports
stay stdlib + the pure data layer; mem2 loads lazily inside writer/reader.
"""

from __future__ import annotations

import collections
from collections.abc import Callable, Iterator, Mapping
import dataclasses
import enum
import importlib
import os
from pathlib import Path
import re
import threading
from typing import Any, Final, NamedTuple, TypeAlias, TypeVar, overload

from dimos.pure.rows import Progress
from dimos.pure.typing import AsyncStateless, Fold, Mealy, Stateless
from dimos.utils.logging_config import setup_logger

_LOG: Final = setup_logger()  # reachable in forkserver workers (bare getLogger is swallowed)

_T = TypeVar("_T")
# Solver twins for the replay overloads (T4 doctrine: plain typevars unify
# against the concrete step; variant typevars are illegal in overload position).
_TIn = TypeVar("_TIn")
_TOut = TypeVar("_TOut")
_TState = TypeVar("_TState")

DEBUG_SCHEMA_VERSION: Final[int] = 1
"""Version stamped into every ``ModuleConfigRecord`` (T10 kinship — migrations gate on it)."""

DEBUG_ENV_VAR: Final[str] = "DIMOS_PURE_DEBUG"
"""The capture toggle env var; grammar in ``parse_debug`` (spec §Toggle & API)."""

DEFAULT_DB_NAME: Final[str] = "debug.db"
"""One mem2 store per run: ``<run_dir>/debug.db`` (spec §Toggle & API, storage)."""

DEFAULT_RING_CAPACITY: Final[int] = 4096
"""Per-session ring bound in events; full ⇒ drop-oldest + ``dropped`` counter (Q5)."""

OVER_DRAIN_EVERY: Final[int] = 256
"""Inline drain cadence under ``over()`` (ticks between drains; doctrine #2)."""


# ── errors ────────────────────────────────────────────────────────────────────


class DebugRecRule(enum.Enum):
    """Every recorder rule, by its message slug."""

    GRAMMAR = "debug-grammar"
    NO_DB = "debug-no-db"
    UNKNOWN_MODULE = "debug-unknown-module"
    UNKNOWN_SEQ = "debug-unknown-seq"
    NO_STATE = "debug-no-state"
    NO_CONFIG = "debug-no-config"
    ZERO_ROWS = "debug-zero-rows"  # warning slug, not an error


class DebugError(RuntimeError):
    """A recorder toggle, storage, or reader contract violation."""

    rule: DebugRecRule | None

    def __init__(self, message: str, rule: DebugRecRule | None = None) -> None:
        """Message is release copy; ``rule`` is machine-readable."""
        super().__init__(message)
        self.rule = rule


# ── capture config (spec §Toggle & API) ──────────────────────────────────────


@dataclasses.dataclass(frozen=True)
class Debug:
    """Per-module capture toggles: the ``over(debug=...)`` / env-rule value."""

    decisions: bool = True
    rows: bool = False
    state: bool | None = None  # None → follows rows (spec: state ON with rows)
    raw: bool = False
    thin: int = 1  # keep every Nth payload on the rows/state/raw layers
    db: str | os.PathLike[str] | None = None  # debug.db override (over() with no run dir)

    def __post_init__(self) -> None:
        """Reject a non-positive thin at construction."""
        if self.thin < 1:
            raise ValueError(f"Debug(thin={self.thin}): thin must be an int >= 1 [debug-grammar]")

    @property
    def state_on(self) -> bool:
        """Effective state-layer toggle (``None`` defers to ``rows``)."""
        return self.rows if self.state is None else self.state


DebugConfig: TypeAlias = "tuple[tuple[str, Debug], ...]"
"""Ordered (module-path-prefix, Debug) rules; ``""`` matches every module."""


_OPT_TOKENS: Final = frozenset({"decisions", "rows", "state", "raw"})


def parse_debug(text: str) -> DebugConfig:
    """Parse the ``DIMOS_PURE_DEBUG`` grammar into ordered per-path rules.

    Grammar (spec §Toggle & API)::

        text  = "1" | "0" | rule (";" rule)*
        rule  = [path] [":" opt ("," opt)*]
        opt   = "decisions" | "rows" | "state" | "raw" | "thin=" INT

    ``"1"`` → ``(("", Debug()),)`` (decisions everywhere); ``"0"``/empty → ``()``.
    Paths are T13 member paths; ``/`` is accepted and normalized to ``.``.
    A bad token raises ``DebugError`` naming it ``[debug-grammar]``.
    """
    stripped = text.strip()
    if stripped in ("", "0"):
        return ()
    if stripped == "1":
        return (("", Debug()),)
    rules: list[tuple[str, Debug]] = []
    for raw_rule in stripped.split(";"):
        rule = raw_rule.strip()
        if not rule:
            continue
        path_part, sep, opt_part = rule.partition(":")
        path = path_part.strip().replace("/", ".")
        rows = state = raw = False
        thin = 1
        if sep:
            for raw_opt in opt_part.split(","):
                opt = raw_opt.strip()
                if not opt:
                    continue
                if opt.startswith("thin="):
                    try:
                        thin = int(opt[len("thin=") :])
                    except ValueError:
                        raise DebugError(
                            f"bad thin token {opt!r} in {DEBUG_ENV_VAR} [debug-grammar]",
                            DebugRecRule.GRAMMAR,
                        ) from None
                    if thin < 1:
                        raise DebugError(
                            f"thin must be an int >= 1, got {thin} [debug-grammar]",
                            DebugRecRule.GRAMMAR,
                        )
                elif opt == "rows":
                    rows = True
                elif opt == "state":
                    state = True
                elif opt == "raw":
                    raw = True
                elif opt == "decisions":
                    pass  # decisions is always on; explicit spelling is a no-op
                else:
                    raise DebugError(
                        f"unknown {DEBUG_ENV_VAR} option {opt!r} — expected one of "
                        f"{sorted(_OPT_TOKENS)} or thin=N [debug-grammar]",
                        DebugRecRule.GRAMMAR,
                    )
        rules.append((path, Debug(rows=rows, state=True if state else None, raw=raw, thin=thin)))
    return tuple(rules)


def debug_from_env(environ: Mapping[str, str] | None = None) -> DebugConfig | None:
    """Read ``DIMOS_PURE_DEBUG`` (``os.environ`` default); unset/off → None."""
    env = os.environ if environ is None else environ
    raw = env.get(DEBUG_ENV_VAR)
    if raw is None:
        return None
    cfg = parse_debug(raw)
    return cfg if cfg else None


def coerce_debug(arg: bool | str | Debug | DebugConfig | None) -> DebugConfig | None:
    """Normalize an ``over(debug=...)`` value; ``None`` falls back to the env, ``False`` kills both."""
    if arg is None:
        return debug_from_env()
    if arg is False:
        return None
    if arg is True:
        return (("", Debug()),)
    if isinstance(arg, Debug):
        return (("", arg),)
    if isinstance(arg, str):
        cfg = parse_debug(arg)
        return cfg if cfg else None
    # already a DebugConfig (tuple of (prefix, Debug) rules)
    return tuple(arg)


def resolve_debug(config: DebugConfig, path: str) -> Debug | None:
    """The rule for one module: longest matching path prefix wins (most specific, spec)."""
    best: Debug | None = None
    best_len = -1
    for prefix, dbg in config:
        if prefix == "" or path == prefix or path.startswith(prefix + "."):
            if len(prefix) > best_len:
                best = dbg
                best_len = len(prefix)
    return best


_CAMEL_RE1: Final = re.compile(r"(.)([A-Z][a-z]+)")
_CAMEL_RE2: Final = re.compile(r"([a-z0-9])([A-Z])")


def default_path(module: Any) -> str:
    """Module path for a lone run: ``snake_case`` of the class name (T13 member naming)."""
    name = type(module).__name__
    s = _CAMEL_RE1.sub(r"\1_\2", name)
    return _CAMEL_RE2.sub(r"\1_\2", s).lower()


# ── records (spec §Capture layers) ───────────────────────────────────────────


class TickDecision(NamedTuple):
    """One tick attempt at the aligner/step chokepoint (~100 B, plain data)."""

    seq: int  # tick attempt counter, 0-based, per module session
    tick_ts: float  # data time of the trigger message — NEVER wall clock
    fired: bool  # False = dropped before step
    drop_reason: str | None  # 'missing-required:pose' | 'tf-unresolvable:pose'
    #                          | 'nonmonotonic:<port>' | 'future-ts:<port>' | ...
    ports: dict[str, float]  # per secondary/tf field: resolved sample ts (nan = default)
    step_ms: float | None  # monotonic duration of step (None if not fired)
    emitted: bool  # step returned an Out row (vs None skip)
    hold_pulls: int  # tf items pulled while holding this tick


DecisionHook: TypeAlias = "Callable[[float, bool, str | None, dict[str, float], int], None]"
"""Aligner-side callback ``(tick_ts, fired, drop_reason, ports, hold_pulls)``.

Builtins-only on purpose: ``align.py`` stores it without importing this module;
``None`` short-circuits to zero per-tick cost (spec §Watch out for).
"""


class ModuleConfigRecord(NamedTuple):
    """One per module session: enough to rebuild the module for replay (T10 shape)."""

    schema_version: int  # DEBUG_SCHEMA_VERSION
    qualname: str  # import path: f"{cls.__module__}.{cls.__qualname__}"
    config: dict[str, Any]  # module.config.model_dump() — canonical (T2)
    step_kind: str  # StepKind.value — replay dispatch without the class


class RowRecord(NamedTuple):
    """One captured post-alignment In row or engine-stamped Out row."""

    seq: int  # the TickDecision seq this row belongs to
    tick_ts: float
    row: Any  # the bundle instance, stored whole (pickle codec)


class StateRecord(NamedTuple):
    """Mealy State snapshot at a tick boundary (hot-reload/T10 kinship)."""

    seq: int  # snapshot is AFTER this tick's step
    tick_ts: float
    state: Any  # plain-data pm.State


class StreamMeta(NamedTuple):
    """``debug_meta`` manifest row: logical name ↔ physical sqlite stream name.

    mem2 stream names must match ``[A-Za-z_][A-Za-z0-9_]*``; logical names
    (``<module_path>/decisions``) carry ``/`` and ``.``, so the writer stores a
    sanitized physical name and this manifest is the reader's only mapping —
    physical names are never re-derived (Implementation contract §storage).
    """

    logical: str  # "<module_path>/<layer>[/<port>]"
    physical: str  # sanitized, collision-suffixed sqlite identifier
    path: str  # module path
    layer: str  # "decisions" | "in" | "out" | "state" | "raw" | "config"


# ── the ring (doctrine #1: no mem2 on the tick path) ─────────────────────────


class DebugEventKind(enum.Enum):
    """What a ring event carries (one storage stream per kind per module)."""

    DECISION = "decision"
    IN_ROW = "in"
    OUT_ROW = "out"
    STATE = "state"
    RAW = "raw"
    CONFIG = "config"


class DebugEvent(NamedTuple):
    """One ring slot: plain data, built on the tick path, drained off it."""

    kind: DebugEventKind
    path: str  # module path (the stream namespace)
    seq: int  # tick seq (-1 for CONFIG)
    ts: float  # data time: tick_ts, or the raw msg's own ts
    port: str | None  # RAW only: the In port name
    payload: Any  # TickDecision | row | State | msg | ModuleConfigRecord


# Physical stream layer suffixes (spec §C2; reader prefix-splits list_streams()).
_LAYER_NAME: Final[dict[DebugEventKind, str]] = {
    DebugEventKind.DECISION: "decisions",
    DebugEventKind.IN_ROW: "in",
    DebugEventKind.OUT_ROW: "out",
    DebugEventKind.STATE: "state",
    DebugEventKind.CONFIG: "config",
}
_LAYER_SUFFIXES: Final = frozenset(_LAYER_NAME.values()) | {"health"}
"""Reader-discoverable stream suffixes: the ring layers + T9's ``health`` rows
(appended via ``DebugWriter.append_row``, not a ring ``DebugEventKind``)."""
_RECORD_TYPE: Final[dict[DebugEventKind, type[Any]]] = {
    DebugEventKind.DECISION: TickDecision,
    DebugEventKind.IN_ROW: RowRecord,
    DebugEventKind.OUT_ROW: RowRecord,
    DebugEventKind.STATE: StateRecord,
    DebugEventKind.CONFIG: ModuleConfigRecord,
}


def _stream_name(path: str, kind: DebugEventKind, port: str | None) -> str:
    """Logical == physical stream name: ``<path>.<layer>`` (raw: ``<path>.raw.<port>``)."""
    if kind is DebugEventKind.RAW:
        return f"{path}.raw.{port}"
    return f"{path}.{_LAYER_NAME[kind]}"


def _parse_stream(name: str) -> tuple[str, str, str | None] | None:
    """Physical stream name → (module_path, layer, port); None if not a debug stream."""
    segs = name.split(".")
    if len(segs) >= 3 and segs[-2] == "raw":
        return ".".join(segs[:-2]), "raw", segs[-1]
    if len(segs) >= 2 and segs[-1] in _LAYER_SUFFIXES:
        return ".".join(segs[:-1]), segs[-1], None
    return None


class DebugRing:
    """Bounded per-session event ring: tick path pushes, drain caller pops.

    ``push`` is O(1) under a short mutex shared only with ``drain``'s batched
    pop — no I/O, no mem2, no waiting; full ⇒ evict oldest + count. The
    recorder must never block or slow a tick (doctrine #1).
    """

    def __init__(self, capacity: int = DEFAULT_RING_CAPACITY) -> None:
        """Empty ring; ``capacity`` is the event bound (>= 1)."""
        self._capacity = capacity
        self._lock = threading.Lock()
        self._items: collections.deque[DebugEvent] = collections.deque()
        self._dropped = 0

    def push(self, event: DebugEvent) -> None:
        """Tick path: append; a full ring evicts oldest and increments ``dropped``."""
        with self._lock:
            if len(self._items) >= self._capacity:
                self._items.popleft()
                self._dropped += 1
            self._items.append(event)

    def drain(self, limit: int | None = None) -> list[DebugEvent]:
        """Drain caller only: pop up to ``limit`` (None = all) oldest-first."""
        out: list[DebugEvent] = []
        with self._lock:
            n = len(self._items) if limit is None else min(limit, len(self._items))
            for _ in range(n):
                out.append(self._items.popleft())
        return out

    def __len__(self) -> int:
        """Events currently queued."""
        with self._lock:
            return len(self._items)

    @property
    def dropped(self) -> int:
        """Events evicted by overflow since construction (monotonic)."""
        with self._lock:
            return self._dropped


# ── capture session + writer ─────────────────────────────────────────────────


class DebugSession:
    """One module run's capture handle: hook targets in, ring out.

    Threading contract: every ``on_*``/``tap_*`` method runs on the run's own
    thread (over(): the caller; live: the session thread), in tick order, and
    only ever pushes to the ring. ``drain``/``close`` run on the drain caller
    (T9 pacer thread live; the run thread inline under ``over()``); they alone
    touch mem2, via the writer.
    """

    path: str
    debug: Debug
    ring: DebugRing

    def __init__(
        self,
        path: str,
        debug: Debug,
        writer: DebugWriter,
        *,
        ring_capacity: int = DEFAULT_RING_CAPACITY,
        live: bool = False,
    ) -> None:
        """Bind path + toggles + writer; ``session_for`` is the normal factory."""
        self.path = path
        self.debug = debug
        self.ring = DebugRing(ring_capacity)
        self._writer = writer
        self._live = live
        self._seq = 0
        self._pending: tuple[int, float, dict[str, float], int] | None = None
        self._last_seq = -1
        self._in_seen = 0
        self._out_seen = 0
        self._state_seen = 0
        self._raw_seen: dict[str, int] = {}
        self._attempts_since_drain = 0  # over() inline-drain pacer, per tick attempt (doctrine #2)
        self._disabled = False
        self._closed = False

    def _fault(self, exc: BaseException) -> None:
        """A recorder fault degrades to a one-line warning + disabled capture — never raises."""
        if not self._disabled:
            self._disabled = True
            _LOG.warning("debug recorder disabled for %s: %r", self.path, exc)

    # ── tick-path surface (push-only; never mem2) ────────────────────────────

    def on_decision(
        self,
        tick_ts: float,
        fired: bool,
        drop_reason: str | None,
        ports: dict[str, float],
        hold_pulls: int,
    ) -> None:
        """``DecisionHook`` target: assign seq; a drop pushes now, a fire pends for ``on_step``.

        Every call is one tick ATTEMPT — so the over() inline drain is paced here,
        per attempt (doctrine #2: "drain inline every N ticks"), NOT per emitted
        out row: a slow-emitting module (many ticks, few emits) must still drain.
        """
        if self._disabled:
            return
        try:
            seq = self._seq
            self._seq += 1
            if fired:
                self._pending = (seq, tick_ts, ports, hold_pulls)
            else:
                self.ring.push(
                    DebugEvent(
                        DebugEventKind.DECISION,
                        self.path,
                        seq,
                        tick_ts,
                        None,
                        TickDecision(
                            seq, tick_ts, False, drop_reason, ports, None, False, hold_pulls
                        ),
                    )
                )
            if not self._live:  # over(): drain inline every N ATTEMPTS (no thread; doctrine #2)
                self._attempts_since_drain += 1
                if self._attempts_since_drain >= OVER_DRAIN_EVERY:
                    self._attempts_since_drain = 0
                    self.drain()
        except BaseException as exc:  # never into the tick path
            self._fault(exc)

    def on_step(self, tick_ts: float, step_ms: float, emitted: bool) -> None:
        """``RunHooks.on_step`` target: complete + push the pending fired decision."""
        if self._disabled:
            return
        try:
            pending = self._pending
            self._pending = None
            if pending is None:
                return
            seq, p_ts, ports, hold_pulls = pending
            self._last_seq = seq
            self.ring.push(
                DebugEvent(
                    DebugEventKind.DECISION,
                    self.path,
                    seq,
                    p_ts,
                    None,
                    TickDecision(seq, p_ts, True, None, ports, step_ms, emitted, hold_pulls),
                )
            )
        except BaseException as exc:
            self._fault(exc)

    def on_state(self, tick_ts: float, state: Any) -> None:
        """``RunHooks.on_state`` target: push a ``StateRecord`` (thinned) at an emit boundary."""
        if self._disabled or not self.debug.state_on:
            return
        try:
            keep = self._state_seen % self.debug.thin == 0
            self._state_seen += 1
            if not keep:
                return
            seq = self._last_seq if self._pending is None else self._pending[0]
            self.ring.push(
                DebugEvent(
                    DebugEventKind.STATE,
                    self.path,
                    seq,
                    tick_ts,
                    None,
                    StateRecord(seq, tick_ts, state),
                )
            )
        except BaseException as exc:
            self._fault(exc)

    def on_raw(self, port: str, msg: Any) -> None:
        """Raw layer: push one pre-alignment port message (thinned; rarely on)."""
        if self._disabled or not self.debug.raw:
            return
        try:
            seen = self._raw_seen.get(port, 0)
            self._raw_seen[port] = seen + 1
            if seen % self.debug.thin != 0:
                return
            ts = float(getattr(msg, "ts", 0.0))
            self.ring.push(DebugEvent(DebugEventKind.RAW, self.path, -1, ts, port, msg))
        except BaseException as exc:
            self._fault(exc)

    def tap_in_rows(self, rows: Iterator[Any]) -> Iterator[Any]:
        """Wrap the aligner's row iterator: push each post-alignment In row (thinned)."""
        for row in rows:
            if isinstance(row, Progress):  # frontier marker, not a row — pass, don't record
                yield row
                continue
            if not self._disabled and self.debug.rows:
                try:
                    keep = self._in_seen % self.debug.thin == 0
                    self._in_seen += 1
                    if keep:
                        seq = self._pending[0] if self._pending is not None else self._last_seq
                        ts = float(getattr(row, "ts", 0.0))
                        self.ring.push(
                            DebugEvent(
                                DebugEventKind.IN_ROW,
                                self.path,
                                seq,
                                ts,
                                None,
                                RowRecord(seq, ts, row),
                            )
                        )
                except BaseException as exc:
                    self._fault(exc)
            yield row

    def tap_out_rows(self, rows: Iterator[_T]) -> Iterator[_T]:
        """Wrap the driver: push each stamped Out row (thinned); close at teardown.

        Inline draining is paced by tick ATTEMPTS in ``on_decision`` (doctrine #2),
        not by out rows here — a slow-emitting module drains just the same."""
        try:
            for row in rows:
                if isinstance(row, Progress):  # frontier marker, not a row — pass, don't record
                    yield row
                    continue
                if not self._disabled and self.debug.rows:
                    try:
                        keep = self._out_seen % self.debug.thin == 0
                        self._out_seen += 1
                        if keep:
                            seq = self._last_seq
                            ts = float(getattr(row, "ts", 0.0))
                            self.ring.push(
                                DebugEvent(
                                    DebugEventKind.OUT_ROW,
                                    self.path,
                                    seq,
                                    ts,
                                    None,
                                    RowRecord(seq, ts, row),
                                )
                            )
                    except BaseException as exc:
                        self._fault(exc)
                yield row
        finally:
            if not self._live:
                self.close()

    def record_config(self, module: Any) -> None:
        """Push the one ``ModuleConfigRecord`` for this session (at session start)."""
        if self._disabled:
            return
        try:
            cls = type(module)
            qualname = f"{cls.__module__}.{cls.__qualname__}"
            config = module.config.model_dump()
            spec = getattr(cls, "__pure_step__", None)
            step_kind = spec.kind.value if spec is not None else ""
            self.ring.push(
                DebugEvent(
                    DebugEventKind.CONFIG,
                    self.path,
                    -1,
                    0.0,
                    None,
                    ModuleConfigRecord(DEBUG_SCHEMA_VERSION, qualname, config, step_kind),
                )
            )
        except BaseException as exc:
            self._fault(exc)

    # ── drain-side surface ───────────────────────────────────────────────────

    def drain(self) -> int:
        """Move queued events ring → store via the writer; returns events written."""
        events = self.ring.drain()
        if events:
            self._writer.write(events)
        return len(events)

    def drain_into(self, observe: Callable[[TickDecision], None]) -> int:
        """Drain the ring; fold each ``TickDecision`` through ``observe``; persist the batch.

        The T9 pacer's ring drain: the counter fold (the guarantee's live feed)
        and the writer persistence ride ONE drain, so live and post-hoc see the
        same decision sequence. Returns the event count.
        """
        events = self.ring.drain()
        for ev in events:
            if ev.kind is DebugEventKind.DECISION and isinstance(ev.payload, TickDecision):
                observe(ev.payload)
        if events:
            self._writer.write(events)
        return len(events)

    @property
    def dropped(self) -> int:
        """Ring overflow counter (events evicted since session start; monotonic)."""
        return self.ring.dropped

    @property
    def writer(self) -> DebugWriter:
        """The session's writer (the H7 health-record closure appends through it)."""
        return self._writer

    def close(self) -> None:
        """Final drain + overflow-counter flush + unregister; idempotent."""
        if self._closed:
            return
        self._closed = True
        try:
            self.drain()
        except BaseException as exc:  # teardown must not raise into the run
            _LOG.warning("debug final drain raised for %s: %r", self.path, exc)
        dropped = self.ring.dropped
        if dropped:
            _LOG.warning(
                "debug ring dropped %d event(s) for %s (recorder fell behind)",
                dropped,
                self.path,
            )
        self._writer.unregister(self)


class DebugWriter:
    """Process-local owner of one debug.db mem2 store; all mem2 I/O happens here.

    One writer per (process, db path) — ``open`` caches. Cross-process safety
    is sqlite WAL + the stdlib 5 s busy timeout; transactions are short and
    issued only from drain callers, and a connection handle is never shared
    across processes (spec §Watch out for).
    """

    db: str
    """Absolute path of the backing debug.db."""

    _INSTANCES: dict[str, DebugWriter] = {}
    _INSTANCES_LOCK = threading.Lock()

    def __init__(self, db: str | os.PathLike[str]) -> None:
        """Open (lazily) the mem2 SqliteStore at ``db``; prefer ``DebugWriter.open``."""
        self.db = str(Path(db).resolve())
        self._store: Any = None
        self._lock = threading.Lock()
        self._sessions: list[DebugSession] = []

    @classmethod
    def open(cls, db: str | os.PathLike[str]) -> DebugWriter:
        """The shared writer for ``db`` in this process (created on first call)."""
        key = str(Path(db).resolve())
        with cls._INSTANCES_LOCK:
            writer = cls._INSTANCES.get(key)
            if writer is None:
                writer = cls(key)
                cls._INSTANCES[key] = writer
            return writer

    def _get_store(self) -> Any:
        """Lazily build the mem2 SqliteStore (per process; never shared cross-process)."""
        if self._store is None:
            from dimos.memory2.store.sqlite import SqliteStore  # lazy: leaf mem2 (spec §C1)

            parent = os.path.dirname(self.db)
            if parent:
                os.makedirs(parent, exist_ok=True)
            self._store = SqliteStore(path=self.db)
        return self._store

    def register(self, session: DebugSession) -> None:
        """Adopt a session's ring into this writer's drain set."""
        with self._lock:
            self._sessions.append(session)

    def unregister(self, session: DebugSession) -> None:
        """Drop a closed session from the drain set (after its final drain)."""
        with self._lock:
            if session in self._sessions:
                self._sessions.remove(session)

    def write(self, events: list[DebugEvent]) -> None:
        """Append a drained batch to the store (drain callers only; short txns)."""
        with self._lock:
            store = self._get_store()
            for ev in events:
                name = _stream_name(ev.path, ev.kind, ev.port)
                ptype = type(ev.payload) if ev.kind is DebugEventKind.RAW else _RECORD_TYPE[ev.kind]
                store.stream(name, ptype).append(ev.payload, ts=ev.ts)

    def drain(self) -> int:
        """Drain every registered ring into the store (teardown / multi-session)."""
        with self._lock:
            sessions = list(self._sessions)
        return sum(session.drain() for session in sessions)

    def append_row(self, stream: str, payload: Any, ts: float) -> None:
        """Append one non-ring row (drain callers only; short txn, same lock as write).

        T9's health persistence seam: ``store.stream(stream, type(payload))
        .append(payload, ts=ts)`` — ``ts`` always explicit (mem2's default is
        wall clock, banned from records; H7 passes ``max(frontier_ts, 0.0)``).
        """
        with self._lock:
            store = self._get_store()
            store.stream(stream, type(payload)).append(payload, ts=ts)

    def close(self) -> None:
        """Final drain, stop the store; idempotent."""
        self.drain()
        with self._lock:
            store = self._store
            self._store = None
        if store is not None:
            try:
                store.stop()
            except BaseException as exc:
                _LOG.warning("debug store stop raised: %r", exc)
        with self._INSTANCES_LOCK:
            if self._INSTANCES.get(self.db) is self:
                del self._INSTANCES[self.db]


def _resolve_db(debug: Debug, run_dir: str | os.PathLike[str] | None) -> str:
    """db resolution chain: ``Debug.db`` → ``run_dir`` → ``$DIMOS_RUN_LOG_DIR`` → mint (T16)."""
    if debug.db is not None:
        return str(Path(debug.db).resolve())
    if run_dir is not None:
        return str(Path(run_dir).resolve() / DEFAULT_DB_NAME)
    env = os.environ.get("DIMOS_RUN_LOG_DIR")
    if env:
        return str(Path(env).resolve() / DEFAULT_DB_NAME)
    # No run dir anywhere: minting one (label "debug") keeps this capture out of
    # a shared logs/debug.db — the no-silent-cross-run-mixing invariant (T16).
    from dimos.utils.rundir import mint_run_dir

    return str(mint_run_dir("debug") / DEFAULT_DB_NAME)


def session_for(
    module: Any,
    *,
    path: str | None = None,
    debug: bool | str | Debug | DebugConfig | None = None,
    run_dir: str | os.PathLike[str] | None = None,
    live: bool = False,
) -> DebugSession | None:
    """The seam entry: resolve toggles + db, build a registered session, or None.

    Precedence (most specific wins): ``debug`` arg → ``DIMOS_PURE_DEBUG`` →
    live default (decisions-only when a run dir exists; Q4). db resolution:
    ``Debug.db`` → ``run_dir`` → ``$DIMOS_RUN_LOG_DIR`` → ``./debug.db``.
    ``path`` defaults to ``default_path(module)``. Returns None when capture
    is off — callers then wire NO hooks (zero tick-path cost).
    """
    if debug is False:
        return None
    cfg = coerce_debug(debug)
    if cfg is None and live and (os.environ.get("DIMOS_RUN_LOG_DIR") or run_dir is not None):
        cfg = (("", Debug()),)  # Q4: live decisions-only ON when a run-log dir exists
    if cfg is None:
        return None
    resolved_path = path if path is not None else default_path(module)
    rule = resolve_debug(cfg, resolved_path)
    if rule is None:
        return None
    try:
        writer = DebugWriter.open(_resolve_db(rule, run_dir))
        session = DebugSession(resolved_path, rule, writer, live=live)
        writer.register(session)
        return session
    except BaseException as exc:  # a broken recorder must never fail the run
        _LOG.warning("debug session_for(%s) failed, capture off: %r", resolved_path, exc)
        return None


def silent_run_warning(
    target: str, ticks: int, rows: int, drops_by_field: Mapping[str, int]
) -> str | None:
    """The unconditional teardown check: warning copy when ticks > 0 and rows == 0.

    Called by ``run_over`` teardown and the rim's session finally — debug
    capture enabled or not (spec §End-of-run report). Returns None otherwise.
    """
    if ticks <= 0 or rows > 0:
        return None
    if drops_by_field:
        field, count = max(drops_by_field.items(), key=lambda kv: kv[1])
        cause = f"top dropped field {field!r} ({count}/{ticks} ticks)"
    else:
        cause = "no rows emitted"
    return (
        f"{target}: {ticks} tick(s) but 0 rows emitted — {cause}. Run with "
        f"{DEBUG_ENV_VAR}=1 for the per-tick drop reasons. [debug-zero-rows]"
    )


# ── reader (spec §Read + replay) ─────────────────────────────────────────────


def load(source: str | os.PathLike[str]) -> DebugRun:
    """Open a recorded run: a run dir (containing ``debug.db``) or a db file path."""
    p = Path(source)
    db = p / DEFAULT_DB_NAME if p.is_dir() else p
    if not db.exists():
        raise DebugError(f"no debug.db at {db} [debug-no-db]", DebugRecRule.NO_DB)
    return DebugRun(db)


def latest(root: str | os.PathLike[str] | None = None) -> DebugRun:
    """The newest run's debug.db: the ``latest`` symlink first, else highest-NNN scan (T16).

    ``root`` defaults to ``LOG_DIR`` (the counter root). Resolution order:
    ``root/latest`` symlink → highest ``NNN_*`` run dir with a debug.db →
    any debug.db under ``root`` by mtime (legacy flat layout).
    """
    from dimos.constants import LOG_DIR

    base = Path(root) if root is not None else LOG_DIR

    # 1. The atomically-maintained latest symlink (T16 mint repoints it).
    link = base / "latest"
    if link.is_symlink() or link.exists():
        db = link / DEFAULT_DB_NAME if link.is_dir() else link
        if db.exists():
            return DebugRun(db)

    # 2. Highest NNN_* run dir carrying a debug.db.
    numbered = []
    if base.is_dir():
        for child in base.iterdir():
            m = re.match(r"^(\d{3,})_", child.name)
            if m and (child / DEFAULT_DB_NAME).exists():
                numbered.append((int(m.group(1)), child / DEFAULT_DB_NAME))
    if numbered:
        return DebugRun(max(numbered, key=lambda nc: nc[0])[1])

    # 3. Legacy flat layout: any debug.db under base, newest by mtime.
    candidates = [base / DEFAULT_DB_NAME, *base.glob(f"**/{DEFAULT_DB_NAME}")]
    existing = [c for c in candidates if c.exists()]
    if not existing:
        raise DebugError(f"no debug.db under {base} [debug-no-db]", DebugRecRule.NO_DB)
    newest = max(existing, key=lambda c: c.stat().st_mtime)
    return DebugRun(newest)


class DebugRun:
    """Read view over one debug.db: every module session of one run."""

    db: str
    """Absolute path of the backing debug.db."""

    def __init__(self, db: str | os.PathLike[str]) -> None:
        """Open the store read-only-in-spirit; ``load``/``latest`` are the factories."""
        from dimos.memory2.store.sqlite import SqliteStore  # lazy: leaf mem2 (spec §C1)

        self.db = str(Path(db).resolve())
        self._store: Any = SqliteStore(path=self.db, must_exist=True)

    def _streams(self) -> list[str]:
        return [n for n in self._store.list_streams() if _parse_stream(n) is not None]

    def modules(self) -> list[str]:
        """Recorded module paths, sorted (from the stream-name prefixes)."""
        paths = set()
        for name in self._streams():
            parsed = _parse_stream(name)
            if parsed is not None:
                paths.add(parsed[0])
        return sorted(paths)

    def module(self, path: str) -> ModuleDebug:
        """One module's view; unknown path raises ``[debug-unknown-module]`` listing paths."""
        known = self.modules()
        if path not in known:
            raise DebugError(
                f"no recorded module {path!r} — recorded: {known} [debug-unknown-module]",
                DebugRecRule.UNKNOWN_MODULE,
            )
        return ModuleDebug(self, path)

    def summary(self) -> str:
        """The end-of-run report: per module, rows vs ticks, drops by field AND reason."""
        lines = [f"debug run: {self.db}"]
        for path in self.modules():
            lines.append(self.module(path).summary())
        if not self.modules():
            lines.append("  (no recorded modules)")
        return "\n".join(lines)

    def close(self) -> None:
        """Release the store; idempotent."""
        store = getattr(self, "_store", None)
        if store is not None:
            self._store = None
            store.stop()

    def __enter__(self) -> DebugRun:
        """Context-managed read session."""
        return self

    def __exit__(self, *exc: object) -> None:
        """Close on scope exit."""
        self.close()


class ModuleDebug:
    """One module session's recorded layers + replay/fixture."""

    run: DebugRun
    path: str

    def __init__(self, run: DebugRun, path: str) -> None:
        """View handle; ``DebugRun.module`` is the factory."""
        self.run = run
        self.path = path

    def _read(self, kind: DebugEventKind, port: str | None = None) -> list[Any]:
        name = _stream_name(self.path, kind, port)
        store = self.run._store
        if name not in store.list_streams():
            return []
        stream: Any = store.stream(name)
        return [obs.data for obs in stream]

    def summary(self) -> str:
        """Ticks fired/dropped/emitted, drops by reason, step_ms p50/p95/max, ring drops."""
        from dimos.pure.health import (
            HealthCounters,
        )  # lazy: keeps debugrec module-scope stdlib-only

        ticks = list(self.ticks())
        counters = HealthCounters()
        for t in ticks:
            counters.observe(t)
        stats = counters.snapshot()
        emitted = [t for t in ticks if t.fired and t.emitted]
        lines = [
            f"  {self.path}: {len(ticks)} attempts, {stats.ticks_fired} fired, "
            f"{stats.rows_emitted} emitted, {stats.ticks_dropped} dropped"
        ]
        for reason, count in sorted(stats.drops_by_reason.items(), key=lambda kv: -kv[1]):
            lines.append(f"    drop {reason}: {count}/{len(ticks)}")
        if (
            stats.step_ms_p50 is not None
            and stats.step_ms_p95 is not None
            and stats.step_ms_max is not None
        ):
            lines.append(
                f"    step_ms p50={stats.step_ms_p50:.3f} p95={stats.step_ms_p95:.3f} "
                f"max={stats.step_ms_max:.3f}"
            )
        for cad in self._contract_cadence(emitted):
            lines.append(cad)
        return "\n".join(lines)

    def _contract_cadence(self, emitted: list[TickDecision]) -> list[str]:
        """Per pm.contract(min_hz) Out field: actual DATA-time cadence vs min_hz (T9 addendum).

        Cadence is measured from emit ts (out-row ts when the rows layer is on,
        else the tick_ts of emitted ticks) — replay-speed-invariant, valid under
        over()/--pure where wall-clock liveness (T9) does not apply.
        """
        try:
            cls = _import_qualname(self.config().qualname)
            fields = cls.__pure_step__.out_type.fields()
        except BaseException:
            return []
        from dimos.pure.rows import ContractSpec

        contracts = {n: fs for n, fs in fields.items() if isinstance(fs, ContractSpec)}
        if not contracts:
            return []
        out_ts = [r.tick_ts for r in self.out_rows()] or [t.tick_ts for t in emitted]
        if len(out_ts) < 2:
            return [
                f"    contract {n}: min_hz={fs.min_hz} (too few emits to measure)"
                for n, fs in contracts.items()
            ]
        span = out_ts[-1] - out_ts[0]
        actual = (len(out_ts) - 1) / span if span > 0 else float("inf")
        return [
            f"    contract {n}: actual_hz={actual:.2f} min_hz={fs.min_hz} "
            f"ok={actual >= fs.min_hz} (data time)"
            for n, fs in contracts.items()
        ]

    def ticks(self) -> Iterator[TickDecision]:
        """Every recorded tick attempt, in seq order."""
        rows = [r for r in self._read(DebugEventKind.DECISION) if isinstance(r, TickDecision)]
        yield from sorted(rows, key=lambda t: t.seq)

    def drops(self, reason: str | None = None) -> Iterator[TickDecision]:
        """Dropped ticks; ``reason`` prefix-matches (``"tf-unresolvable"`` hits ``...:pose``)."""
        for t in self.ticks():
            if t.fired:
                continue
            if reason is None or (t.drop_reason is not None and t.drop_reason.startswith(reason)):
                yield t

    def in_rows(self) -> Iterator[RowRecord]:
        """Captured post-alignment In rows (empty unless the rows layer was on)."""
        rows = [r for r in self._read(DebugEventKind.IN_ROW) if isinstance(r, RowRecord)]
        yield from sorted(rows, key=lambda r: r.seq)

    def out_rows(self) -> Iterator[RowRecord]:
        """Captured engine-stamped Out rows (empty unless the rows layer was on)."""
        rows = [r for r in self._read(DebugEventKind.OUT_ROW) if isinstance(r, RowRecord)]
        yield from sorted(rows, key=lambda r: r.seq)

    def states(self) -> Iterator[StateRecord]:
        """Captured State snapshots (empty unless the state layer was on)."""
        rows = [r for r in self._read(DebugEventKind.STATE) if isinstance(r, StateRecord)]
        yield from sorted(rows, key=lambda r: r.seq)

    def raw(self, port: str) -> Iterator[Any]:
        """Captured pre-alignment messages for one port (raw layer)."""
        yield from self._read(DebugEventKind.RAW, port)

    def health(self) -> Iterator[Any]:
        """Recorded T9 ``HealthRecord`` rows (``<path>.health``); empty when none captured."""
        from dimos.pure.health import HealthRecord, health_stream

        name = health_stream(self.path)
        store = self.run._store
        if name not in store.list_streams():
            return
        for obs in store.stream(name, HealthRecord):
            yield obs.data

    def config(self) -> ModuleConfigRecord:
        """The session's config record; missing → ``[debug-no-config]``."""
        recs = [r for r in self._read(DebugEventKind.CONFIG) if isinstance(r, ModuleConfigRecord)]
        if not recs:
            raise DebugError(
                f"no config record for {self.path!r} — replay needs it [debug-no-config]",
                DebugRecRule.NO_CONFIG,
            )
        return recs[0]

    # Replay re-drives step over the RECORDED In rows — no aligner, no graph,
    # no robot. Overload order mirrors over(): AsyncStateless before Stateless.
    @overload
    def replay(self, module: None = None, *, from_seq: int | None = None) -> Iterator[Any]: ...
    @overload
    def replay(
        self, module: AsyncStateless[_TIn, _TOut], *, from_seq: int | None = None
    ) -> Iterator[_TOut]: ...
    @overload
    def replay(
        self, module: Mealy[_TState, _TIn, _TOut], *, from_seq: int | None = None
    ) -> Iterator[_TOut]: ...
    @overload
    def replay(
        self, module: Stateless[_TIn, _TOut], *, from_seq: int | None = None
    ) -> Iterator[_TOut]: ...
    @overload
    def replay(
        self, module: Fold[_TIn, _TOut], *, from_seq: int | None = None
    ) -> Iterator[_TOut]: ...
    def replay(self, module: Any = None, *, from_seq: int | None = None) -> Iterator[Any]:
        """Drive ``module`` (default: rebuilt from ``config()``) over the recorded In rows.

        ``from_seq`` starts mid-run: Mealy modules are seeded with the newest
        recorded State at a seq <= ``from_seq`` (state layer required, else
        ``[debug-no-state]``). Out rows are stamped with the recorded tick ts.
        Determinism covers step logic given identical In rows + State;
        resource-internal nondeterminism (e.g. CUDA reductions) is documented,
        not promised away (spec §Watch out for).
        """
        from dimos.pure.drivers import (
            RunHooks,
            drive_async,
            drive_fold,
            drive_mealy,
            drive_stateless,
        )
        from dimos.pure.stepspec import StepKind

        if module is None:
            rec = self.config()
            module = _import_qualname(rec.qualname)(**rec.config)
        spec = type(module).__pure_step__

        seed_seq = -1
        state: Any = None
        if spec.kind is StepKind.MEALY:
            assert spec.state_type is not None
            state = spec.state_type()
            if from_seq is not None:
                snaps = [s for s in self.states() if s.seq <= from_seq]
                if not snaps:
                    raise DebugError(
                        f"replay(from_seq={from_seq}) needs a State snapshot at seq <= "
                        f"{from_seq}; capture the state layer [debug-no-state]",
                        DebugRecRule.NO_STATE,
                    )
                snap = max(snaps, key=lambda s: s.seq)
                state = snap.state
                seed_seq = snap.seq

        low = (
            seed_seq
            if from_seq is not None and spec.kind is StepKind.MEALY
            else ((from_seq - 1) if from_seq is not None else -1)
        )
        in_rows = [r.row for r in self.in_rows() if r.seq > low]
        hooks = RunHooks()
        if spec.kind is StepKind.MEALY:
            yield from drive_mealy(module, iter(in_rows), state, skips=spec.skips, hooks=hooks)
        elif spec.kind is StepKind.ASYNC_STATELESS:
            yield from drive_async(
                module, iter(in_rows), max_inflight=1, skips=spec.skips, hooks=hooks
            )
        elif spec.kind is StepKind.FOLD:
            yield from drive_fold(module, iter(in_rows), hooks=hooks)
        else:
            yield from drive_stateless(module, iter(in_rows), skips=spec.skips, hooks=hooks)

    def fixture(self, seq: int, path: str | os.PathLike[str] | None = None) -> str:
        """Export tick ``seq`` as a runnable pytest file (+ pickled row sidecar); returns the path.

        The test rebuilds the module from the config record, feeds the recorded
        In row (and State for Mealy), and asserts the recorded Out row —
        self-contained, no dependency on the source debug.db.
        """
        import pickle

        from dimos.pure.stepspec import StepKind

        rec = self.config()
        in_row = next((r.row for r in self.in_rows() if r.seq == seq), None)
        out_row = next((r.row for r in self.out_rows() if r.seq == seq), None)
        if in_row is None or out_row is None:
            raise DebugError(
                f"tick {seq} has no captured in/out row — capture the rows layer "
                f"[debug-unknown-seq]",
                DebugRecRule.UNKNOWN_SEQ,
            )
        mealy = rec.step_kind == StepKind.MEALY.value
        state = None
        if mealy:
            snaps = [s for s in self.states() if s.seq < seq]
            state = max(snaps, key=lambda s: s.seq).state if snaps else None

        out_path = Path(path) if path is not None else Path(f"test_regression_{seq}.py")
        sidecar = out_path.with_suffix(".pickle")
        with open(sidecar, "wb") as fh:
            pickle.dump(
                {
                    "qualname": rec.qualname,
                    "config": rec.config,
                    "in_row": in_row,
                    "state": state,
                    "out_row": out_row,
                    "mealy": mealy,
                },
                fh,
            )
        out_path.write_text(_FIXTURE_TEMPLATE.format(sidecar=sidecar.name, seq=seq))
        return str(out_path)


_FIXTURE_TEMPLATE: Final = '''\
# Auto-generated by dimos.pure.debugrec ModuleDebug.fixture — tick {seq}.
"""Committed regression: replay one recorded tick, assert the recorded Out row."""

import dataclasses
import importlib
import pickle
from pathlib import Path


def _rebuild(qualname):
    module_path, _, cls_name = qualname.rpartition(".")
    return getattr(importlib.import_module(module_path), cls_name)


def test_regression_{seq}():
    data = pickle.loads((Path(__file__).parent / "{sidecar}").read_bytes())
    cls = _rebuild(data["qualname"])
    module = cls(**data["config"])
    in_row = data["in_row"]
    if data["mealy"]:
        state = data["state"]
        if state is None:
            state = cls.__pure_step__.state_type()
        _new_state, got = module.step(state, in_row)
    else:
        got = module.step(in_row)
    assert dataclasses.replace(got, ts=in_row.ts) == data["out_row"]
'''


def _import_qualname(qualname: str) -> Any:
    """Import a ``module.Cls`` dotted path to the class object."""
    module_path, _, cls_name = qualname.rpartition(".")
    return getattr(importlib.import_module(module_path), cls_name)
