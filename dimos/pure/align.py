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
import math
import types
from typing import (
    TYPE_CHECKING,
    Any,
    Final,
    Generic,
    Protocol,
    TypeAlias,
    TypeVar,
    Union,
    get_args,
    get_origin,
)

from typing_extensions import Self

from dimos.pure.rows import FieldSpec, In, InterpolateSpec, LatestSpec, TfSpec, TickSpec
from dimos.pure.typing import Stamped
from dimos.utils.logging_config import setup_logger as _setup_logger

_DBG = _setup_logger()

# TEMPORARY (Go2 firmware bug) — see the drop gate in Aligner.__next__. A sample
# whose ts jumps more than this far past the port frontier is treated as a corrupt
# future stamp and dropped without advancing the frontier.
_FUTURE_TS_OUTLIER_S = 3600.0


class _NothingPending:
    """Sentinel type for ``NOTHING_PENDING`` (a class so it reprs legibly)."""

    def __repr__(self) -> str:
        return "NOTHING_PENDING"


NOTHING_PENDING: Final = _NothingPending()
"""A non-blocking feed has no item queued right now (see ``pull_nowait``).

Returned by live feeds (``rim._LiveFeed.pull_nowait``) — never by a plain
iterator, so offline ``over()`` alignment is untouched and stays deterministic."""

if TYPE_CHECKING:
    from dimos.pure.tfbuffer import TfContext

__all__ = [
    "NOTHING_PENDING",
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

_NEG_INF: Final = float("-inf")


InT = TypeVar("InT", bound=In)

InterpolatorFn: TypeAlias = Callable[[Any, Any, float], Any]
"""``fn(a, b, alpha) -> value`` with alpha strictly inside (0, 1) (spec §6)."""

_SUPPORTED_KINDS: Final = (TickSpec, LatestSpec, InterpolateSpec, TfSpec)
"""The In-side sampler kinds this resolver handles (spec §4.1; tf() is a side channel)."""


class Interpolatable(Protocol):
    """Self-describing interpolation: ``a.interpolate(b, alpha)`` -> value at alpha."""

    def interpolate(self, other: Self, alpha: float) -> Self: ...


# ── interpolator registry (spec §6) ──────────────────────────────────────────


def _float_lerp(a: float, b: float, alpha: float) -> float:
    """The seeded float interpolator (spec §6.1)."""
    return a + (b - a) * alpha


_INTERPOLATORS: Final[dict[type[Any], InterpolatorFn]] = {float: _float_lerp}
"""Registry table (spec §6.1); float lerp pre-seeded, ``numpy.float64`` via MRO."""


def register_interpolator(tp: type[_TV], fn: Callable[[_TV, _TV, float], _TV]) -> None:
    """Register ``fn(a, b, alpha)`` as the interpolator for ``tp`` (last wins)."""
    _INTERPOLATORS[tp] = fn


def interpolator_for(tp: type[Any]) -> InterpolatorFn | None:
    """Resolve ``tp``'s interpolator via the spec §6.2 MRO walk, or None."""
    for cls in tp.__mro__:
        registered = _INTERPOLATORS.get(cls)
        if registered is not None:
            return registered
        method = cls.__dict__.get("interpolate")
        if callable(method):
            return lambda a, b, alpha: a.interpolate(b, alpha)
    return None


def _strip_optional(ann: Any) -> Any:
    """Drop a single ``| None`` (any spelling); leave everything else as-is (spec §6.3)."""
    if get_origin(ann) is Union or get_origin(ann) is types.UnionType:
        non_none = tuple(a for a in get_args(ann) if a is not type(None))
        if len(non_none) == 1:
            return non_none[0]
    return ann


# ── errors (spec §9) ──────────────────────────────────────────────────────────


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


def _e_not_in_row(obj: object) -> AlignmentError:
    return AlignmentError(
        f"align() takes a pm.In row bundle, got {obj!r} — pass the module's In "
        "class (its step signature names it; T6 passes spec.in_type). [align-not-in-row]",
        AlignRule.NOT_IN_ROW,
    )


def _e_no_tick(bundle: str) -> AlignmentError:
    return AlignmentError(
        f"{bundle}: declares no tick() field — alignment needs exactly one trigger "
        "to drive rows. Mark exactly one In field as tick(); latest()/interpolate() "
        "fields resolve at its ts. [align-no-tick]",
        AlignRule.NO_TICK,
    )


def _e_unsupported_kind(bundle: str, name: str, kind: str) -> AlignmentError:
    return AlignmentError(
        f"{bundle}: field {name!r} declares {kind}(), which this resolver does not "
        "support (tick/latest/interpolate). New sampler kinds must extend the "
        "alignment engine (tf() arrives with T11). [align-unsupported-kind]",
        AlignRule.UNSUPPORTED_KIND,
    )


def _e_unknown_port(bundle: str, key: str, names: str, ts_note: str) -> AlignmentError:
    return AlignmentError(
        f"{bundle}: unknown stream {key!r} — declared ports: {names}.{ts_note} "
        "[align-unknown-port]",
        AlignRule.UNKNOWN_PORT,
    )


def _e_missing_tick_stream(bundle: str, name: str) -> AlignmentError:
    return AlignmentError(
        f"{bundle}: no stream for tick port {name!r} — the trigger drives every row; "
        f"without it nothing ticks. Pass {name}=<stream>. [align-missing-tick-stream]",
        AlignRule.MISSING_TICK_STREAM,
    )


def _e_missing_required_stream(bundle: str, kind: str, name: str) -> AlignmentError:
    return AlignmentError(
        f"{bundle}: no stream for required {kind}() port {name!r} — every tick would "
        f"drop unresolved. Pass {name}=<stream>, or give the field default= to make it "
        "optional. [align-missing-required-stream]",
        AlignRule.MISSING_REQUIRED_STREAM,
    )


def _e_shared_iterator(bundle: str, a: str, b: str) -> AlignmentError:
    return AlignmentError(
        f"{bundle}: ports {a!r} and {b!r} share one iterator object — each port "
        "consumes its stream independently; a shared iterator would interleave-steal "
        "items. Pass independent iterables. [align-shared-iterator]",
        AlignRule.SHARED_ITERATOR,
    )


def _e_interp_untyped(bundle: str, name: str, ann: Any) -> AlignmentError:
    return AlignmentError(
        f"{bundle}: field {name!r} is interpolate() but its annotation {ann} is not a "
        "single class — interpolation dispatches on the declared payload type "
        "(| None is fine with default=None). [align-interp-untyped]",
        AlignRule.INTERP_UNTYPED,
    )


def _e_not_interpolatable(bundle: str, name: str, cls: type[Any]) -> AlignmentError:
    return AlignmentError(
        f"{bundle}: field {name!r} is interpolate() but {cls} has no interpolator — "
        "implement interpolate(self, other, alpha) on the type or "
        f"register_interpolator({cls.__name__}, fn); use latest() if nearest-sample "
        "semantics suffice. [align-not-interpolatable]",
        AlignRule.NOT_INTERPOLATABLE,
    )


def _e_bad_item(bundle: str, port: str, item: object) -> AlignmentError:
    return AlignmentError(
        f"{bundle}.{port}: stream yielded {type(item).__name__} with no readable float "
        "ts — align consumes stamped payloads (msgs and rows carry ts). [align-bad-item]",
        AlignRule.BAD_ITEM,
    )


def _e_bad_ts(bundle: str, port: str, value: float) -> AlignmentError:
    return AlignmentError(
        f"{bundle}.{port}: item ts is {value!r} — ts must be a finite float. [align-bad-ts]",
        AlignRule.BAD_TS,
    )


def _e_unstamped(bundle: str, port: str) -> AlignmentError:
    return AlignmentError(
        f"{bundle}.{port}: item ts is UNSTAMPED (-inf) — an Out row reached alignment "
        "without being stamped by its driver. Rows on the wire are stamped by the "
        "engine; hand-built feeds must set ts. [align-unstamped]",
        AlignRule.UNSTAMPED_ITEM,
    )


# ── stats (spec §8) ───────────────────────────────────────────────────────────


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


# ── the resolver (spec §5) ────────────────────────────────────────────────────


class Aligner(Generic[InT]):
    """One-shot iterator of aligned In rows; owns counters, owns no resources."""

    def __init__(
        self,
        in_type: type[InT],
        streams: Mapping[str, Iterable[Stamped]],
        *,
        tf: TfContext | None = None,
    ) -> None:
        """Validate wiring eagerly (spec §7); pulls nothing until iteration."""
        # §7 #1 — in_type must be a pm.In subclass; fields() is called once and a
        # BundleDefinitionError (T1 E-UNRESOLVED) propagates untouched.
        obj: object = in_type
        if not isinstance(obj, type) or not issubclass(obj, In):
            raise _e_not_in_row(obj)
        self._in_type: type[InT] = in_type
        self._bundle = f"{in_type.__module__}.{in_type.__qualname__}"
        fields = in_type.fields()

        # §7 #2 — exactly one tick (at-least-one here; T1 rejects >1 at definition).
        tick_name: str | None = None
        for name, spec in fields.items():
            if isinstance(spec, TickSpec):
                tick_name = name
        if tick_name is None:
            raise _e_no_tick(self._bundle)

        # §7 #3 — every In-side kind is tick/latest/interpolate (T11 extension seam).
        for name, spec in fields.items():
            if not isinstance(spec, _SUPPORTED_KINDS):
                raise _e_unsupported_kind(self._bundle, name, spec.kind)

        # §7 #4 — every stream key names a declared port.
        for key in streams:
            if key not in fields:
                if key == "ts":
                    note = " (ts is row infrastructure, stamped by the engine — not a port.)"
                elif key == "tf":
                    note = " (tf is the reserved tf-stream key — pass it as the tf= argument, not a stream.)"
                else:
                    note = ""
                raise _e_unknown_port(self._bundle, key, ", ".join(fields), note)

        # §7 #5 — tick + required secondaries need streams; optionals may be omitted.
        if tick_name not in streams:
            raise _e_missing_tick_stream(self._bundle, tick_name)
        for name, spec in fields.items():
            if isinstance(spec, TfSpec):
                continue  # tf fields are a demand-pulled side channel, not a merge port
            if name != tick_name and spec.required and name not in streams:
                raise _e_missing_required_stream(self._bundle, spec.kind, name)

        # §7 #6 — iter() once per stream value; a shared iterator object is illegal
        # (two ports draining one iterator would interleave-steal items).
        iters: dict[str, Any] = {}
        seen: dict[int, str] = {}
        for name in fields:
            if name in streams:
                stream_it = iter(streams[name])
                if id(stream_it) in seen:
                    raise _e_shared_iterator(self._bundle, seen[id(stream_it)], name)
                seen[id(stream_it)] = name
                iters[name] = stream_it

        # §7 #7 + build port states. Interpolators resolve for *wired* interpolate
        # ports only: a streamless optional interpolate port always defaults and
        # never interpolates, so its type need not be interpolatable (see the
        # Implementation notes appendix in tasks/t5-align.md).
        self._stats = AlignStats()
        self._ports: list[_PortState] = []
        self._secondaries: list[_PortState] = []
        self._tf_fields: list[tuple[str, TfSpec]] = []  # demand-pulled side channel (spec §6.1)
        tick_port: _PortState | None = None
        for index, (name, spec) in enumerate(fields.items()):
            if isinstance(spec, TfSpec):
                self._tf_fields.append((name, spec))  # no _PortState — resolved at fire time
                continue
            it = iters.get(name)
            interp: InterpolatorFn | None = None
            if isinstance(spec, InterpolateSpec) and it is not None:
                interp = self._resolve_interp(name, spec)
            port = _PortState(name=name, index=index, spec=spec, it=it, interp=interp)
            self._stats.ports[name] = port.stats
            self._ports.append(port)
            if isinstance(spec, TickSpec):
                tick_port = port
            else:
                self._secondaries.append(port)
        assert tick_port is not None  # a tick field was found in §7 #2
        self._tick: _PortState = tick_port
        self._held_tick_ts: float | None = None
        self._ctx: TfContext | None = tf
        if tf is not None:
            self._stats.ports["tf"] = tf.port_stats  # T9 sees the stream-level counters

    def _resolve_interp(self, name: str, spec: FieldSpec) -> InterpolatorFn:
        """Resolve an interpolate port's interpolator from its declared type (spec §6.3)."""
        stripped = _strip_optional(spec.annotation)
        if stripped is Any or not isinstance(stripped, type):
            raise _e_interp_untyped(self._bundle, name, spec.annotation)
        interp = interpolator_for(stripped)
        if interp is None:
            raise _e_not_interpolatable(self._bundle, name, stripped)
        return interp

    def _checked_ts(self, port: _PortState, item: Stamped) -> float:
        """Read and validate ``item.ts`` (spec §4.2, §9 data errors)."""
        raw: object
        try:
            raw = item.ts
        except Exception:
            raise _e_bad_item(self._bundle, port.name, item) from None
        if not isinstance(raw, (int, float)):
            raise _e_bad_item(self._bundle, port.name, item) from None
        ts = float(raw)
        if ts == _NEG_INF:
            raise _e_unstamped(self._bundle, port.name) from None
        if not math.isfinite(ts):
            raise _e_bad_ts(self._bundle, port.name, ts) from None
        return ts

    def _fire(self, tick_item: Stamped, tick_ts: float) -> InT | None:
        """Resolve one tick into a row, or None if a required field is unresolvable (spec §5.4)."""
        stats = self._stats
        stats.ticks_fired += 1
        resolved: dict[str, Any] = {}
        brackets: list[tuple[str, Stamped, Stamped, float, InterpolatorFn]] = []
        missing_required: list[str] = []
        # pass 1 — decide resolvability; interpolators are NOT called here (D12).
        for p in self._secondaries:
            spec = p.spec
            head = p.head
            newest = p.newest
            if isinstance(spec, InterpolateSpec):
                if head is not None and p.head_ts == tick_ts:
                    resolved[p.name] = head  # exact hit via pending head
                elif newest is not None and p.newest_ts == tick_ts:
                    resolved[p.name] = newest  # exact hit via committed newest
                elif newest is not None and head is not None:
                    interp = p.interp
                    assert interp is not None  # wired interpolate ports resolve one (§6.3)
                    alpha = (tick_ts - p.newest_ts) / (p.head_ts - p.newest_ts)
                    brackets.append((p.name, newest, head, alpha, interp))
                elif not spec.required:
                    resolved[p.name] = spec.default
                else:
                    missing_required.append(p.name)
            else:  # latest
                candidate = head if (head is not None and p.head_ts == tick_ts) else newest
                if candidate is not None:
                    resolved[p.name] = candidate
                elif not spec.required:
                    resolved[p.name] = spec.default
                else:
                    missing_required.append(p.name)
        if missing_required:
            stats.ticks_dropped += 1
            for field_name in missing_required:
                stats.drops_by_field[field_name] = stats.drops_by_field.get(field_name, 0) + 1
            return None
        # tf block — the demand-pulled side channel resolves at fire time (spec §6.3/§6.4).
        # Resolution IS the decision: required chains hold (pull) until resolvable or the
        # stream exhausts; optional chains take their default when unresolvable.
        if self._tf_fields:
            ctx = self._ctx
            unresolved = {
                name
                for name, spec in self._tf_fields
                if spec.required and (ctx is None or ctx.resolve_field(name, tick_ts) is None)
            }
            while unresolved and ctx is not None and ctx.pull():  # THE HOLD — pulling is the wait
                unresolved = {n for n in unresolved if ctx.resolve_field(n, tick_ts) is None}
            if unresolved:
                stats.ticks_dropped += 1
                for field_name in unresolved:
                    stats.drops_by_field[field_name] = stats.drops_by_field.get(field_name, 0) + 1
                return None
            for name, spec in self._tf_fields:
                value = None if ctx is None else ctx.resolve_field(name, tick_ts)
                resolved[name] = value if value is not None else spec.default
        # pass 2 — materialize brackets (user interpolation runs only for emitted rows).
        for bname, left, right, alpha, interp in brackets:
            resolved[bname] = interp(left, right, alpha)
        resolved[self._tick.name] = tick_item
        stats.rows_emitted += 1
        return self._in_type(ts=tick_ts, **resolved)

    def __iter__(self) -> Aligner[InT]:
        """Return self (one-shot iterator)."""
        return self

    def __next__(self) -> InT:
        """Run the pull loop (spec §5.2) to the next emitted row."""
        tick = self._tick
        while True:
            # (a) refill: every live-iterator port with an empty head pulls one
            #     kept item, in declaration order (the deterministic pull order).
            for p in self._ports:
                while p.it is not None and p.head is None:
                    try:
                        # A live OPTIONAL secondary with nothing queued must not hold the
                        # tick: it has a default, so "nothing yet" is resolvable, and a
                        # subscribed-but-silent topic would otherwise park the session
                        # forever. The tick still blocks (that IS the pacing) and REQUIRED
                        # secondaries still hold (spec §5 — they cannot be defaulted).
                        # Plain iterators have no pull_nowait: offline over() is untouched.
                        pull_nowait = (
                            getattr(p.it, "pull_nowait", None)
                            if p is not tick and not p.spec.required
                            else None
                        )
                        item = next(p.it) if pull_nowait is None else pull_nowait()
                        if item is NOTHING_PENDING:
                            break
                    except StopIteration:
                        p.it = None
                        break
                    # Control input (a click, a command): its ts is the producer's own
                    # wall clock and bears no relation to the data clock, so it is
                    # stamped with the current tick frontier and skips the measurement
                    # gates below. Untouched payload — only the merge key is ours.
                    if getattr(p.spec, "control", False):
                        p.head = item
                        p.head_ts = tick.last_ts  # -inf before the first tick: resolves at once
                        p.last_ts = p.head_ts
                        p.stats.accepted += 1
                        continue
                    ts = self._checked_ts(p, item)
                    # TEMPORARY — Go2 firmware bug: the Go2 lidar occasionally stamps a
                    # scan with a corrupt timestamp ~75 days in the future. Accepting it
                    # latches this port's frontier (last_ts) to the garbage value, so the
                    # monotonic gate below then drops every subsequent (correct) sample —
                    # the stream starves permanently. Drop any item jumping >1h past the
                    # frontier WITHOUT latching last_ts. Remove once recordings/firmware
                    # carry sane stamps.
                    if math.isfinite(p.last_ts) and ts > p.last_ts + _FUTURE_TS_OUTLIER_S:
                        _DBG.warning(
                            "dropping sample with implausible future ts (Go2 firmware bug)",
                            port=p.name,
                            ts=ts,
                            last_ts=p.last_ts,
                            ahead_s=ts - p.last_ts,
                        )
                        continue
                    if ts <= p.last_ts:
                        p.stats.dropped_nonmonotonic += 1
                        continue
                    p.head = item
                    p.head_ts = ts
                    p.last_ts = ts
                    p.stats.accepted += 1
                    if p is tick:
                        self._held_tick_ts = ts

            # (b) terminate: an exhausted trigger ends alignment (secondaries are
            #     left with their pending heads — lazy, never drained).
            if tick.head is None and tick.it is None:
                raise StopIteration

            # (c) select the least event by (head_ts, declaration index).
            ready = [q for q in self._ports if q.head is not None]
            selected = min(ready, key=lambda q: (q.head_ts, q.index))

            # (d) data event: commit into newest and loop.
            if selected is not tick:
                selected.newest = selected.head
                selected.newest_ts = selected.head_ts
                selected.head = None
                continue

            # (e) tick event fires: every other frontier is provably past T (§5.3),
            #     so the resolution is final either way.
            assert tick.head is not None  # min selected it from ports-with-head
            if self._ctx is not None:
                self._ctx.advance_past(tick.head_ts)  # ingest the tf frontier past T (§6.2)
            row = self._fire(tick.head, tick.head_ts)
            tick.head = None
            self._held_tick_ts = None
            if row is None:
                continue
            return row

    @property
    def stats(self) -> AlignStats:
        """Live counters (spec §8.1); reads are safe at any time."""
        return self._stats

    @property
    def held_tick_ts(self) -> float | None:
        """ts of the tick currently held mid-merge, else None (spec §8.2)."""
        return self._held_tick_ts


def align(
    in_type: type[InT],
    streams: Mapping[str, Iterable[Stamped]],
    *,
    tf: TfContext | None = None,
) -> Aligner[InT]:
    """Resolve stamped streams into tick-stamped In rows (validates now, pulls lazily)."""
    return Aligner(in_type, streams, tf=tf)
