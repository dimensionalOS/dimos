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

"""Fault events emitted by the memory engine.

Every observable mutation of the :class:`~dimos.agents.memory.page_table.PageTable`
— an eviction, a fidelity drop, a pin rebalance, a rehydrate request — is
surfaced as a :class:`FaultEvent` so operators can tune the token budget
without spelunking into logs.

:class:`FaultObserver` routes events to two sinks in parallel:

1. A structured-logging JSONL stream (``structlog``) for post-hoc analysis.
2. An optional ``Out[FaultEvent]`` stream on the owning module for live UI.

Design goals:

* Never raise — an observability path must not kill the agent.
* Cheap to call from tight loops; event construction is the bulk of the cost.
* Stable on-the-wire shape: dataclass fields only, no references to live
  ``Page`` objects (we copy the ``page_id`` string instead).

Implementation notes:

* :class:`FaultEvent` ``ts`` and ``details`` use ``default_factory``
  values so tests can construct events positionally without boilerplate.
* :class:`FaultObserver` exposes observability helpers beyond the core
  :meth:`emit`:

  - ``counts``: per-``FaultKind`` counter (useful for regression tests
    and metrics scrape).
  - ``last_event``: reference to the most recent emitted event.
  - Convenience emitters: :meth:`evict`, :meth:`degrade`,
    :meth:`refetch`, :meth:`physical_insufficiency`,
    :meth:`pin_rebalance`. Each constructs a :class:`FaultEvent` and
    calls :meth:`emit`; useful so call sites don't have to import the
    ``FaultKind`` enum for common paths.
* :meth:`FaultEvent.to_dict` helper for JSON-friendly serialization
  (used by structlog and tests).
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
import time
from typing import TYPE_CHECKING, Any

from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from dimos.core.stream import Out

__all__ = [
    "FaultEvent",
    "FaultKind",
    "FaultObserver",
]

_logger = setup_logger()


class FaultKind(Enum):
    """What happened to a page that we want operators to know about."""

    PAGE_EVICTED = "page_evicted"
    """A page was fully dropped from the assembled prompt (non-pinned only)."""

    PAGE_DEGRADED = "page_degraded"
    """A page rendered at a lower fidelity than its maximum (e.g. FULL -> COMPRESSED)."""

    REFETCH_FAULT = "refetch_fault"
    """The LLM called ``get_artefact`` to pull an EVIDENCE page back to FULL."""

    PHYSICAL_INSUFFICIENCY = "physical_insufficiency"
    """The token budget cannot fit even the minimum-fidelity assembly — an
    exception was either raised or narrowly avoided. Operators should bump
    the budget or shrink pinning."""

    PIN_REBALANCE = "pin_rebalance"
    """The set of ``pinned_at_full`` pages changed (usually because a new
    EVIDENCE page was ingested and the oldest previously-pinned image
    dropped to non-pinned)."""


@dataclass(frozen=True)
class FaultEvent:
    """One observable memory-layer event.

    Attributes:
        kind: See :class:`FaultKind`.
        page_id: The affected page. ``None`` for events that are not
            page-scoped (e.g. ``PHYSICAL_INSUFFICIENCY`` reflecting a whole
            turn rather than one page).
        turn_seq: The ingest turn number at the moment of the event. Useful
            for reconstructing ordering in a trace.
        ts: ``time.time()`` at emission.
        details: Free-form payload (e.g. ``{"from": "FULL", "to": "COMPRESSED"}``).
            Must be JSON-serialisable; the observer does not enforce this,
            but the structlog renderer will complain loudly if it is not.
    """

    kind: FaultKind
    page_id: str | None
    turn_seq: int
    ts: float = field(default_factory=time.time)
    details: dict[str, Any] = field(default_factory=dict)

    def to_dict(self) -> dict[str, Any]:
        """Serialize to a plain dict; stable schema for logs and UIs."""
        return {
            "kind": self.kind.value,
            "page_id": self.page_id,
            "turn_seq": self.turn_seq,
            "ts": self.ts,
            "details": dict(self.details),
        }


class FaultObserver:
    """Sink for :class:`FaultEvent`.

    The observer is deliberately tolerant: any exception raised by a
    downstream sink is caught and logged but never re-raised, because the
    caller is almost always on the agent worker thread and we do not want
    an observability failure to kill a turn.

    Args:
        stream_out: Optional ``Out[FaultEvent]`` to publish events on. When
            present, events are published to the stream *after* being
            logged to structlog.
    """

    def __init__(self, stream_out: "Out[FaultEvent] | None" = None) -> None:
        self._stream = stream_out
        self._last_event: FaultEvent | None = None
        # Kept for tests — number of events seen of each kind. Not exposed
        # as a metric yet; operators consume the ``Out`` stream instead.
        self.counts: dict[FaultKind, int] = {k: 0 for k in FaultKind}

    @property
    def last_event(self) -> FaultEvent | None:
        """Most recently emitted event, or ``None`` if the observer is fresh."""
        return self._last_event

    def emit(self, event: FaultEvent) -> None:
        """Publish *event* to structlog and (if configured) the ``Out`` stream.

        This is the single entry point — callers never touch either sink
        directly. Guaranteed not to raise.
        """
        self._last_event = event
        self.counts[event.kind] = self.counts.get(event.kind, 0) + 1

        try:
            _logger.info(
                "memory_fault",
                kind=event.kind.value,
                page_id=event.page_id,
                turn_seq=event.turn_seq,
                ts=event.ts,
                **event.details,
            )
        except Exception:  # pragma: no cover — logging must never break the agent
            pass

        if self._stream is not None:
            try:
                self._stream.publish(event)
            except Exception:  # pragma: no cover
                _logger.exception("FaultObserver: failed to publish event to Out stream")

    # --- convenience emitters used by the rest of the package -----------

    def evict(self, page_id: str, turn_seq: int, reason: str) -> None:
        self.emit(FaultEvent(
            kind=FaultKind.PAGE_EVICTED,
            page_id=page_id,
            turn_seq=turn_seq,
            details={"reason": reason},
        ))

    def degrade(
        self,
        page_id: str,
        turn_seq: int,
        *,
        from_level: str,
        to_level: str,
    ) -> None:
        self.emit(FaultEvent(
            kind=FaultKind.PAGE_DEGRADED,
            page_id=page_id,
            turn_seq=turn_seq,
            details={"from": from_level, "to": to_level},
        ))

    def refetch(self, page_id: str, turn_seq: int, artefact_uuid: str) -> None:
        self.emit(FaultEvent(
            kind=FaultKind.REFETCH_FAULT,
            page_id=page_id,
            turn_seq=turn_seq,
            details={"artefact_uuid": artefact_uuid},
        ))

    def physical_insufficiency(
        self,
        turn_seq: int,
        *,
        needed: int,
        available: int,
        exception: str | None = None,
    ) -> None:
        details: dict[str, Any] = {"needed": needed, "available": available}
        if exception is not None:
            details["exception"] = exception
        self.emit(FaultEvent(
            kind=FaultKind.PHYSICAL_INSUFFICIENCY,
            page_id=None,
            turn_seq=turn_seq,
            details=details,
        ))

    def pin_rebalance(
        self,
        turn_seq: int,
        *,
        pinned_page_ids: list[str],
        unpinned_page_ids: list[str],
    ) -> None:
        self.emit(FaultEvent(
            kind=FaultKind.PIN_REBALANCE,
            page_id=None,
            turn_seq=turn_seq,
            details={"pinned": pinned_page_ids, "unpinned": unpinned_page_ids},
        ))
