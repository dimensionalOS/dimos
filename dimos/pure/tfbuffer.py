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

from collections.abc import Iterable, Iterator, Mapping
import dataclasses
import enum
import threading
from typing import Any, Final, TypeAlias, TypeVar

from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.pure.align import AlignmentError, PortStats
from dimos.pure.stepspec import StepSpec

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
        raise NotImplementedError("T11 impl pending")

    @property
    def horizon(self) -> float:
        """The per-edge retention window (data-time seconds)."""
        raise NotImplementedError("T11 impl pending")

    def ingest(self, item: Transform | Any) -> None:
        """Ingest one dynamic sample (Transform, or a TFMessage-duck batch); spec §4.2."""
        raise NotImplementedError("T11 impl pending")

    def set_static(self, transform: Transform) -> None:
        """Seed a static edge: one value, valid at every ts, never evicted (spec §4.3)."""
        raise NotImplementedError("T11 impl pending")

    def claim(self, edge: EdgeKey, owner: str) -> None:
        """Register a writer for an edge; single-writer + forest checks (spec §4.4)."""
        raise NotImplementedError("T11 impl pending")

    def release(self, owner: str) -> None:
        """Release every claim held by ``owner`` (idempotent; run teardown calls it)."""
        raise NotImplementedError("T11 impl pending")

    def resolve(self, parent: str, child: str, at: float) -> Transform | None:
        """Chain-composed parent<-child transform at ``at``, or None (spec §5)."""
        raise NotImplementedError("T11 impl pending")

    def frames(self) -> set[str]:
        """Every frame appearing in any ingested/static edge."""
        raise NotImplementedError("T11 impl pending")

    def edges(self) -> set[EdgeKey]:
        """Every edge with samples (dynamic or static), directed as ingested."""
        raise NotImplementedError("T11 impl pending")

    @property
    def stats(self) -> TfStats:
        """Live counters (spec §4.6)."""
        raise NotImplementedError("T11 impl pending")


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

    @classmethod
    def for_over(cls, module: Any, spec: StepSpec, tf_arg: Any) -> TfContext | None:
        """Build the over()/run_over context from the popped ``tf=`` value (spec §8)."""
        raise NotImplementedError("T11 impl pending")

    @classmethod
    def for_live(
        cls, module: Any, spec: StepSpec, buffer: TfBuffer, feed: Iterator[Any] | None
    ) -> TfContext | None:
        """Build the live-session context over the m.i.tf feed (spec §9)."""
        raise NotImplementedError("T11 impl pending")

    def advance_past(self, ts: float) -> None:
        """Pull + ingest the tf stream until an item with ts > ``ts`` lands (spec §6.2)."""
        raise NotImplementedError("T11 impl pending")

    def pull(self) -> bool:
        """Pull + ingest one tf item; False once the stream is exhausted (spec §6.3)."""
        raise NotImplementedError("T11 impl pending")

    def resolve_field(self, name: str, at: float) -> Transform | None:
        """Resolve one tf() field's declared chain at ``at`` (spec §6.4)."""
        raise NotImplementedError("T11 impl pending")

    def release(self) -> None:
        """Release this run's writer claims (teardown; idempotent)."""
        raise NotImplementedError("T11 impl pending")


def tf_out_tap(rows: Iterator[_R], module: Any, ctx: TfContext) -> Iterator[_R]:
    """Route tf_out emissions: validate frames + ts, ingest into the buffer (spec §7)."""
    raise NotImplementedError("T11 impl pending")


def attach(module: Any, buffer: TfBuffer) -> None:
    """Attach a (possibly shared) buffer to a module's next live session (spec §9.2)."""
    raise NotImplementedError("T11 impl pending")
