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

"""TF engine: the shared edge-history buffer + the tf sampler run context (T11).

Spec: ``dimos/pure/tasks/t11-tf.md``. Leaf engine module — imports msg types
(like ``interpolators.py``), so it stays OFF the ``pm`` surface and out of
``align.py``'s module scope; the aligner, ``run_over`` and the rim import it
lazily, only when a bundle declares tf fields. The file is named ``tfbuffer``
(not ``tf``) because a ``dimos.pure.tf`` submodule would clobber the ``pm.tf``
specifier binding on the package (spec §1.2). Adapted from
``dimos/protocol/tf/tf.py`` math (spec §13); the service API, the Condition,
and ``_wait_get`` are gone — hold-the-tick replaces blocking gets.
"""

from __future__ import annotations

import bisect
from collections import defaultdict, deque
from collections.abc import Iterable, Iterator, Mapping
import dataclasses
import enum
from functools import reduce
import math
import threading
from typing import Any, Final, TypeAlias, TypeVar

from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.pure.align import _FUTURE_TS_OUTLIER_S, AlignmentError, PortStats
from dimos.pure.interpolators import interp_transform
from dimos.pure.rows import Progress, TfOutSpec, TfSpec
from dimos.pure.stepspec import StepSpec
from dimos.utils.logging_config import setup_logger

__all__ = [
    "DEFAULT_TF_HORIZON",
    "STATIC_OWNER",
    "EdgeKey",
    "TfBuffer",
    "TfContext",
    "TfEdgeStats",
    "TfError",
    "TfRule",
    "TfSource",
    "TfStats",
    "attach",
    "tf_out_tap",
]

DEFAULT_TF_HORIZON: Final[float] = 10.0
"""Per-edge retention window in data-time seconds (tf.py ``buffer_size`` parity)."""

_LOG: Final = setup_logger()  # reachable in forkserver workers (bare getLogger is swallowed)

STATIC_OWNER: Final[str] = "<static>"
"""Writer-registry owner token for statics (spec §4.4)."""

EdgeKey: TypeAlias = "tuple[str, str]"
"""A directed frame edge: (parent, child)."""

TfSource: TypeAlias = "TfBuffer | Iterable[Any] | tuple[TfBuffer, Iterable[Any]]"
"""What ``over(tf=...)`` accepts: a stream of transforms, a (pre-seeded,
possibly shared) buffer serving statics only, or a (buffer, stream) pair."""

_R = TypeVar("_R")


# ── errors (spec §10) ────────────────────────────────────────────────────────


class TfRule(enum.Enum):
    """Every tf runtime rule, by its message slug (spec §10)."""

    UNEXPECTED_STREAM = "tf-unexpected-stream"
    MISSING_STREAM = "tf-missing-stream"
    BAD_SOURCE = "tf-bad-source"
    MULTI_WRITER = "tf-multi-writer"
    FRAME_CYCLE = "tf-frame-cycle"
    BAD_ITEM = "tf-bad-item"
    OUT_BAD_PAYLOAD = "tf-out-bad-payload"
    OUT_UNSTAMPED = "tf-out-unstamped"
    OUT_FRAME_MISMATCH = "tf-out-frame-mismatch"


class TfError(AlignmentError):
    """A tf wiring, ingestion, or emission contract violation (spec §10)."""

    tf_rule: TfRule | None

    def __init__(self, message: str, tf_rule: TfRule | None = None) -> None:
        """Message is release copy per spec §10; ``tf_rule`` is machine-readable."""
        super().__init__(message, None)
        self.tf_rule = tf_rule


# ── message templates (release copy: spec §10) ───────────────────────────────


def _cls(module: Any) -> str:
    """``{module}.{qualname}`` of the module's class — the ``{cls}`` of spec §10."""
    t = type(module)
    return f"{t.__module__}.{t.__qualname__}"


def _e_unexpected_stream(module: Any) -> TfError:
    return TfError(
        f"{_cls(module)}.over() got tf=<...> but its In bundle declares no tf() fields — "
        f"the tf stream feeds tf() samplers only. Remove tf=, or declare a tf() field. "
        f"[tf-unexpected-stream]",
        TfRule.UNEXPECTED_STREAM,
    )


def _e_missing_stream(module: Any, names: Iterable[str]) -> TfError:
    joined = ", ".join(repr(n) for n in names)
    return TfError(
        f"{_cls(module)}: In declares required tf() field(s) {joined} but the run got no "
        f"tf source — pass tf=<stream or TfBuffer> (over) / bind m.i.tf or attach a "
        f"buffer (live), or give the fields default= to make them optional. "
        f"[tf-missing-stream]",
        TfRule.MISSING_STREAM,
    )


def _e_bad_source(module: Any, value: Any) -> TfError:
    return TfError(
        f"{_cls(module)}: tf= must be a TfBuffer, an iterable of stamped transforms, or "
        f"a (TfBuffer, iterable) pair, got {type(value).__name__}. [tf-bad-source]",
        TfRule.BAD_SOURCE,
    )


def _e_multi_writer(edge: EdgeKey, owner: str, claimant: str) -> TfError:
    parent, child = edge
    return TfError(
        f"tf edge {parent!r} -> {child!r} is already asserted by {owner!r}; {claimant!r} "
        f"cannot claim it — one writer per edge (reversed orientation included). "
        f"[tf-multi-writer]",
        TfRule.MULTI_WRITER,
    )


def _e_frame_cycle(edge: EdgeKey, claimant: str) -> TfError:
    parent, child = edge
    return TfError(
        f"tf edge {parent!r} -> {child!r} claimed by {claimant!r} would close a cycle in "
        f"the asserted tf graph — asserted edges (tf_out declarations plus statics) must "
        f"form a forest: exactly one asserted chain may connect any two frames. "
        f"[tf-frame-cycle]",
        TfRule.FRAME_CYCLE,
    )


def _e_bad_item(item: Any) -> TfError:
    return TfError(
        f"tf stream yielded {type(item).__name__} with no readable frame pair and finite "
        f"ts — the tf stream carries Transform msgs (or TFMessage batches of them). "
        f"[tf-bad-item]",
        TfRule.BAD_ITEM,
    )


def _e_out_bad_payload(module: Any, field: str, value: Any) -> TfError:
    return TfError(
        f"{_cls(module)}.{field}: tf_out payload is {type(value).__name__}, not a "
        f"Transform — a tf_out field carries its asserted edge's Transform (or None to "
        f"stay silent). [tf-out-bad-payload]",
        TfRule.OUT_BAD_PAYLOAD,
    )


def _e_out_unstamped(module: Any, field: str, value: Any) -> TfError:
    return TfError(
        f"{_cls(module)}.{field}: tf_out Transform has ts {getattr(value, 'ts', None)!r} "
        f"— assertions are timeline samples; construct with ts=i.ts (data time, never "
        f"wall clock). [tf-out-unstamped]",
        TfRule.OUT_UNSTAMPED,
    )


def _e_out_frame_mismatch(module: Any, field: str, got: EdgeKey, declared: EdgeKey) -> TfError:
    return TfError(
        f"{_cls(module)}.{field}: tf_out Transform carries {got[0]!r} -> {got[1]!r} but "
        f"the declared edge is {declared[0]!r} -> {declared[1]!r} — the payload's frames "
        f"must match the declaration. [tf-out-frame-mismatch]",
        TfRule.OUT_FRAME_MISMATCH,
    )


# ── stats (spec §4.6, the T9 seam) ───────────────────────────────────────────


@dataclasses.dataclass
class TfEdgeStats:
    """Monotonic per-edge ingestion counters (sorted-insert model, spec §4.2)."""

    accepted: int = 0
    dropped_duplicate: int = 0  # exact-ts duplicate on the edge; first-in wins
    dropped_expired: int = 0  # older than the edge's window at ingestion
    evicted: int = 0  # aged out by a newer sample's window advance


@dataclasses.dataclass
class TfStats:
    """Snapshot of a buffer's counters; reads are safe at any time."""

    edges: dict[EdgeKey, TfEdgeStats] = dataclasses.field(default_factory=dict)
    statics: int = 0


# ── per-edge dynamic history (spec §4.1, §5.1) ───────────────────────────────


class _EdgeHistory:
    """One dynamic edge's ts-sorted samples (strictly increasing ts by construction)."""

    def __init__(self) -> None:
        self._ts: list[float] = []
        self._samples: list[Transform] = []

    def value_at(self, at: float) -> Transform | None:
        """Sample at ``at``: exact hit (the object), bracketed interp, or None (spec §5.1)."""
        ts = self._ts
        if not ts:
            return None
        j = bisect.bisect_left(ts, at)
        if j < len(ts) and ts[j] == at:
            return self._samples[j]  # exact hit — the sample itself (interp NOT called)
        if j == 0 or j == len(ts):
            return None  # one-sided: no extrapolation, ever
        left, right = self._samples[j - 1], self._samples[j]
        alpha = (at - left.ts) / (right.ts - left.ts)
        return interp_transform(left, right, alpha)  # ts lerped to `at`


# ── the buffer (spec §4, §5) ─────────────────────────────────────────────────


class TfBuffer:
    """Engine-owned per-edge transform history: ingest, claim, resolve.

    Run/engine-context object, passed in — never a process global. Thread-safe
    behind one plain mutex (no Condition, no waiting: holds live in the pull).
    """

    def __init__(
        self,
        *,
        horizon: float = DEFAULT_TF_HORIZON,
        statics: Iterable[Transform] = (),
    ) -> None:
        """Empty buffer with a per-edge data-time window; statics seed via set_static."""
        if not horizon > 0:
            raise ValueError(f"TfBuffer(): horizon must be > 0, got {horizon}")
        self._horizon = horizon
        self._lock = threading.Lock()
        self._edges: dict[EdgeKey, _EdgeHistory] = {}
        self._statics: dict[EdgeKey, Transform] = {}
        self._edge_stats: dict[EdgeKey, TfEdgeStats] = {}
        self._statics_count = 0
        self._claims: dict[frozenset[str], tuple[EdgeKey, str]] = {}
        self._order: list[EdgeKey] = []  # first-creation order (BFS tie-break, spec §5.2)
        for transform in statics:
            self.set_static(transform)

    @property
    def horizon(self) -> float:
        """The per-edge retention window (data-time seconds)."""
        return self._horizon

    # ── ingestion (spec §4.2) ────────────────────────────────────────────────

    def ingest(self, item: Transform | Any) -> None:
        """Ingest one dynamic sample (Transform, or a TFMessage-duck batch); spec §4.2."""
        with self._lock:
            transforms = getattr(item, "transforms", None)
            if transforms is not None and not isinstance(item, Transform):
                for transform in transforms:
                    self._ingest_one(transform)
            else:
                self._ingest_one(item)

    def _ingest_one(self, transform: Any) -> None:
        parent = getattr(transform, "frame_id", None)
        child = getattr(transform, "child_frame_id", None)
        raw_ts = getattr(transform, "ts", None)
        if not (
            isinstance(parent, str)
            and isinstance(child, str)
            and isinstance(raw_ts, (int, float))
            and not isinstance(raw_ts, bool)
            and math.isfinite(float(raw_ts))
        ):
            raise _e_bad_item(transform)
        ts = float(raw_ts)
        edge = (parent, child)
        stats = self._edge_stats.setdefault(edge, TfEdgeStats())
        hist = self._edges.get(edge)
        if hist is None:
            hist = _EdgeHistory()
            self._edges[edge] = hist
            if edge not in self._order:
                self._order.append(edge)
        tss = hist._ts
        j = bisect.bisect_left(tss, ts)
        if j < len(tss) and tss[j] == ts:
            stats.dropped_duplicate += 1  # first-in wins (spec §4.2, §7.4)
            return
        if tss and ts < tss[-1] - self._horizon:
            stats.dropped_expired += 1  # outside the live window
            return
        tss.insert(j, ts)  # sorted insert — out-of-order tolerant (spec §4.2, D6)
        hist._samples.insert(j, transform)
        stats.accepted += 1
        cutoff = tss[-1] - self._horizon
        evicted = 0
        while len(tss) > 1 and tss[0] < cutoff:  # newest always survives
            tss.pop(0)
            hist._samples.pop(0)
            evicted += 1
        if evicted:
            stats.evicted += evicted

    # ── statics (spec §4.3) ──────────────────────────────────────────────────

    def set_static(self, transform: Transform) -> None:
        """Seed a static edge: one value, valid at every ts, never evicted (spec §4.3)."""
        with self._lock:
            edge = (transform.frame_id, transform.child_frame_id)
            self._claim_locked(edge, STATIC_OWNER)  # loud on collision / cycle
            if edge not in self._statics:
                self._statics_count += 1
                if edge not in self._order:
                    self._order.append(edge)
            self._statics[edge] = transform

    # ── claims (spec §4.4) ───────────────────────────────────────────────────

    def claim(self, edge: EdgeKey, owner: str) -> None:
        """Register a writer for an edge; single-writer + forest checks (spec §4.4)."""
        with self._lock:
            self._claim_locked(edge, owner)

    def _claim_locked(self, edge: EdgeKey, owner: str) -> None:
        key = frozenset(edge)
        existing = self._claims.get(key)
        if existing is not None:
            if existing[1] == owner:
                return  # idempotent (session restart)
            raise _e_multi_writer(edge, existing[1], owner)
        if self._connected(edge[0], edge[1]):
            raise _e_frame_cycle(edge, owner)
        self._claims[key] = (edge, owner)

    def _connected(self, a: str, b: str) -> bool:
        """True if a and b already connect over claimed + static edges (spec §4.4)."""
        adj: dict[str, set[str]] = defaultdict(set)
        for key in self._claims:
            nodes = tuple(key)
            if len(nodes) == 2:
                u, v = nodes
                adj[u].add(v)
                adj[v].add(u)
        seen = {a}
        queue = deque([a])
        while queue:
            cur = queue.popleft()
            if cur == b:
                return True
            for nb in adj[cur]:
                if nb not in seen:
                    seen.add(nb)
                    queue.append(nb)
        return b in seen

    def release(self, owner: str) -> None:
        """Release every claim held by ``owner`` (idempotent; run teardown calls it)."""
        with self._lock:
            self._claims = {k: v for k, v in self._claims.items() if v[1] != owner}

    # ── resolution (spec §5) ─────────────────────────────────────────────────

    def resolve(self, parent: str, child: str, at: float) -> Transform | None:
        """Chain-composed parent<-child transform at ``at``, or None (spec §5)."""
        with self._lock:
            if parent == child:
                return self._identity(parent, at)  # spec §5.4 fast path
            direct = self._edge_value(parent, child, at)
            if direct is not None:
                return direct
            chain = self._search(parent, child, at)
            if chain is None:
                return None
            composed = reduce(lambda a, b: a + b, chain)  # spec §5.3, T_A_B + T_B_C
            return self._restamp(composed, parent, child, at)

    def _edge_value(self, parent: str, child: str, at: float) -> Transform | None:
        """A single edge at ``at``: direct, or reversed-inverted (spec §5.1)."""
        fwd = self._forward_value(parent, child, at)
        if fwd is not None:
            return fwd
        rev = self._forward_value(child, parent, at)
        if rev is not None:
            inv = rev.inverse()
            inv.ts = at  # inverse() re-stamps to self.ts; force (ctor 0.0 → wall clock)
            return inv
        return None

    def _forward_value(self, parent: str, child: str, at: float) -> Transform | None:
        static = self._statics.get((parent, child))
        if static is not None:
            return self._restamp(static, parent, child, at)  # any ts, frames preserved
        hist = self._edges.get((parent, child))
        if hist is not None:
            return hist.value_at(at)
        return None

    def _search(self, parent: str, child: str, at: float) -> list[Transform] | None:
        """BFS shortest chain over hops resolvable at ``at`` (spec §5.2)."""
        adj: dict[str, list[str]] = defaultdict(list)
        for p, c in self._order:
            if c not in adj[p]:
                adj[p].append(c)
            if p not in adj[c]:
                adj[c].append(p)
        queue: deque[tuple[str, list[Transform]]] = deque([(parent, [])])
        visited = {parent}
        while queue:
            cur, path = queue.popleft()
            if cur == child:
                return path
            for nb in adj[cur]:
                if nb in visited:
                    continue
                hop = self._edge_value(cur, nb, at)  # resolvability prunes before pathing
                if hop is None:
                    continue
                visited.add(nb)
                queue.append((nb, [*path, hop]))
        return None

    def _identity(self, frame: str, at: float) -> Transform:
        out = Transform(frame_id=frame, child_frame_id=frame, ts=at)
        out.ts = at
        return out

    def _restamp(self, transform: Transform, parent: str, child: str, at: float) -> Transform:
        out = Transform(
            translation=transform.translation,
            rotation=transform.rotation,
            frame_id=parent,
            child_frame_id=child,
            ts=at,
        )
        out.ts = at  # ctor swaps ts=0.0 for wall clock; force the data time
        return out

    # ── introspection (spec §4.6) ────────────────────────────────────────────

    def frames(self) -> set[str]:
        """Every frame appearing in any ingested/static edge."""
        with self._lock:
            out: set[str] = set()
            for p, c in self._order:
                out.add(p)
                out.add(c)
            return out

    def edges(self) -> set[EdgeKey]:
        """Every edge with samples (dynamic or static), directed as ingested."""
        with self._lock:
            return set(self._edges) | set(self._statics)

    @property
    def stats(self) -> TfStats:
        """Live counters (spec §4.6)."""
        with self._lock:
            edges = {k: dataclasses.replace(v) for k, v in self._edge_stats.items()}
            return TfStats(edges=edges, statics=self._statics_count)


# ── the run context (spec §6, §7) ────────────────────────────────────────────


@dataclasses.dataclass
class TfContext:
    """Per-run tf state handed to the aligner and the tf_out tap.

    Owns nothing but its claims: the buffer is run/engine-context property; the
    stream iterator's lifecycle belongs to whoever built it (T6/T8 rules).
    """

    buffer: TfBuffer
    frames_in: Mapping[str, EdgeKey]  # tf() field -> resolved edge
    frames_out: Mapping[str, EdgeKey]  # tf_out field -> resolved edge
    required: frozenset[str]  # tf() fields without default=
    owner: str  # claim token: the module's class path
    stream: Iterator[Any] | None  # the tf stream, or None (statics/buffer only)
    port_stats: PortStats = dataclasses.field(default_factory=PortStats)  # aligner "tf" entry
    _frontier: float = float("-inf")  # running-max ts of every ingested item (spec §6.2)

    @classmethod
    def for_over(cls, module: Any, spec: StepSpec, tf_arg: Any) -> TfContext | None:
        """Build the over()/run_over context from the popped ``tf=`` value (spec §8)."""
        frames_in, frames_out, required, owner = _collect_frames(module, spec)
        if not frames_in and not frames_out:
            if tf_arg is not None:
                raise _e_unexpected_stream(module)
            return None  # tf-free module pays nothing
        buffer, stream = _parse_source(module, tf_arg)
        if required and tf_arg is None:  # over(): no source at all (spec §8.2)
            raise _e_missing_stream(module, sorted(required))
        ctx = cls(
            buffer=buffer,
            frames_in=frames_in,
            frames_out=frames_out,
            required=frozenset(required),
            owner=owner,
            stream=stream,
        )
        ctx._claim_out(module)
        return ctx

    @classmethod
    def for_live(
        cls, module: Any, spec: StepSpec, buffer: TfBuffer, feed: Iterator[Any] | None
    ) -> TfContext | None:
        """Build the live-session context over the m.i.tf feed (spec §9)."""
        frames_in, frames_out, required, owner = _collect_frames(module, spec)
        if not frames_in and not frames_out:
            return None
        ctx = cls(
            buffer=buffer,
            frames_in=frames_in,
            frames_out=frames_out,
            required=frozenset(required),
            owner=owner,
            stream=feed,  # unwired-legal live: statics/attached buffer may resolve (spec §9.1)
        )
        ctx._claim_out(module)
        return ctx

    def _claim_out(self, module: Any) -> None:
        """Claim every declared tf_out edge; release partials on conflict (spec §7.2)."""
        try:
            for edge in self.frames_out.values():
                self.buffer.claim(edge, self.owner)
        except Exception:
            self.buffer.release(self.owner)
            raise

    def _pull_one(self) -> bool:
        """Pull + ingest one tf item, advancing the frontier; False once exhausted."""
        stream = self.stream
        if stream is None:
            return False
        try:
            item = next(stream)
        except StopIteration:
            self.stream = None
            return False
        if isinstance(item, Progress):
            # Interior-edge frontier marker: the producing member ticked without
            # emitting a tf sample. Advance the frontier so holds stay bounded.
            self._frontier = max(self._frontier, item.ts)
            return True
        # TEMPORARY — Go2 firmware bug, the tf twin of the aligner ports' guard: a
        # corrupt stamp ~75 days in the future would latch the frontier (a running
        # max — advance_past and the tick hold stop pulling forever) AND evict the
        # whole edge history via the horizon window, expiring every later good
        # sample. Drop the item WITHOUT latching. Remove with the aligner's guard.
        ts = _item_max_ts(item)
        if math.isfinite(self._frontier) and ts > self._frontier + _FUTURE_TS_OUTLIER_S:
            _LOG.warning(
                "dropping tf item with implausible future ts (Go2 firmware bug)",
                owner=self.owner,
                ts=ts,
                frontier=self._frontier,
                ahead_s=ts - self._frontier,
            )
            self.port_stats.dropped_nonmonotonic += 1
            return True
        self.buffer.ingest(item)  # data errors raise here (spec §6.2, §10 D-family)
        self.port_stats.accepted += 1
        self._frontier = max(self._frontier, ts)
        return True

    def advance_past(self, ts: float) -> None:
        """Pull + ingest the tf stream until an item with ts > ``ts`` lands (spec §6.2)."""
        while self._frontier <= ts and self._pull_one():
            pass

    def pull(self) -> bool:
        """Pull + ingest one tf item; False once the stream is exhausted (spec §6.3)."""
        return self._pull_one()

    @property
    def frontier(self) -> float:
        """Running-max ts of every ingested tf item (-inf before the first)."""
        return self._frontier

    def resolve_field(self, name: str, at: float) -> Transform | None:
        """Resolve one tf() field's declared chain at ``at`` (spec §6.4)."""
        parent, child = self.frames_in[name]
        return self.buffer.resolve(parent, child, at)

    def release(self) -> None:
        """Release this run's writer claims (teardown; idempotent)."""
        self.buffer.release(self.owner)


def _collect_frames(
    module: Any, spec: StepSpec
) -> tuple[dict[str, EdgeKey], dict[str, EdgeKey], set[str], str]:
    """Resolved tf/tf_out edges + required set + claim owner for a module (spec §6, §8)."""
    tf_frames: Mapping[tuple[str, str], EdgeKey] = getattr(module, "__pure_tf_frames__", {})
    in_specs = {n: s for n, s in spec.in_type.fields().items() if isinstance(s, TfSpec)}
    out_specs = {n: s for n, s in spec.out_type.fields().items() if isinstance(s, TfOutSpec)}
    frames_in = {n: tf_frames[("in", n)] for n in in_specs}
    frames_out = {n: tf_frames[("out", n)] for n in out_specs}
    required = {n for n, s in in_specs.items() if s.required}
    owner = f"{type(module).__module__}.{type(module).__qualname__}"
    return frames_in, frames_out, required, owner


def _parse_source(module: Any, tf_arg: Any) -> tuple[TfBuffer, Iterator[Any] | None]:
    """Normalize ``tf=`` into (buffer, stream) — spec §8.2 table."""
    if tf_arg is None:
        return TfBuffer(), None
    if isinstance(tf_arg, TfBuffer):
        return tf_arg, None
    if isinstance(tf_arg, tuple) and len(tf_arg) == 2 and isinstance(tf_arg[0], TfBuffer):
        return tf_arg[0], iter(_coerce(tf_arg[1]))
    if isinstance(tf_arg, Iterable) and not isinstance(tf_arg, (str, bytes)):
        return TfBuffer(), iter(_coerce(tf_arg))
    raise _e_bad_source(module, tf_arg)


def _coerce(raw: Any) -> Iterable[Any]:
    """Unwrap memory2 Stream Observations like any stream value (T6 boundary, spec §8.2)."""
    from dimos.pure.drivers import _coerce_stream

    return _coerce_stream(raw)


def _item_max_ts(item: Any) -> float:
    """Representative frontier ts of an ingested item (batch → max, spec §6.2)."""
    transforms = getattr(item, "transforms", None)
    if transforms is not None and not isinstance(item, Transform):
        values = [
            float(t.ts) for t in transforms if isinstance(getattr(t, "ts", None), (int, float))
        ]
        return max(values) if values else float("-inf")
    return float(item.ts)


def tf_out_tap(rows: Iterator[_R], module: Any, ctx: TfContext) -> Iterator[_R]:
    """Route tf_out emissions: validate frames + ts, ingest into the buffer (spec §7)."""
    frames_out = ctx.frames_out
    try:
        for row in rows:
            if isinstance(row, Progress):  # frontier marker: no fields to route
                yield row
                continue
            for name, declared in frames_out.items():
                value = getattr(row, name)
                if value is None:
                    continue  # sparse doctrine: nothing routed
                if not isinstance(value, Transform):
                    raise _e_out_bad_payload(module, name, value)
                ts = getattr(value, "ts", None)
                if (
                    not isinstance(ts, (int, float))
                    or isinstance(ts, bool)
                    or not math.isfinite(float(ts))
                ):
                    raise _e_out_unstamped(module, name, value)
                got = (value.frame_id, value.child_frame_id)
                if got != declared:
                    raise _e_out_frame_mismatch(module, name, got, declared)
                ctx.buffer.ingest(value)
            yield row
    finally:
        ctx.release()  # claims outlive the last row, die before over() returns (spec §8.3)


def attach(module: Any, buffer: TfBuffer) -> None:
    """Attach a (possibly shared) buffer to a module's next live session (spec §9.2)."""
    from dimos.pure import rim

    rim.attach_tf_buffer(module, buffer)
