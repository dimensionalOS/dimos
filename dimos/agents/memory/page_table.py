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

"""In-memory page store for the agent's conversation state.

:class:`PageTable` is the physical-memory analogue: it owns every
:class:`Page` the agent has seen, with O(1) lookups by id and by
artefact UUID. The selector reads from here, the ingestor writes to here,
and the engine coordinates both.

Threading: callers hold the module's own RLock during turn processing, but
the table also keeps an internal :class:`threading.RLock` so that background
readers (e.g. a UI probe subscribed to the ``agent`` stream) can safely
iterate without racing the ingest writer.

Pinning policy:

* ``pinned`` pages are never evicted (the selector must keep them in the
  assembled prompt at a minimum of ``min_fidelity``).
* ``pinned_at_full`` further requires ``FULL`` rendering.
* :meth:`PageTable.rebalance_evidence_pins` is called by the engine after
  every EVIDENCE page is added: it ensures *exactly* the last
  ``pin_recent_evidence`` EVIDENCE pages carry ``pinned_at_full`` and
  returns the diff so the observer can emit a ``PIN_REBALANCE`` event.

Implementation notes:

* Auto-rebalance is caller-driven, not implicit on ``add``. The engine
  invokes ``rebalance_evidence_pins`` itself under its own lock so it
  can emit the ``PIN_REBALANCE`` fault event with the right
  ``turn_seq``. Keeping ``add`` storage-only avoids coupling this
  module to ``FaultObserver``.
* ``rebalance_evidence_pins`` returns a :class:`PinRebalance` dataclass
  so the caller receives both the newly-pinned and newly-unpinned ids
  for a single self-contained fault event without a separate
  before/after diff.
* ``_auto_pinned_ids`` tracks pages that *this rebalancer* installed,
  so manual promotions via :meth:`mark_pinned_full` (the one-way
  REFETCH_FAULT upgrade path) are never implicitly un-pinned.
* Public API: :meth:`mark_pinned_full`, :meth:`evidence_pages`,
  :meth:`clear`, :meth:`__len__`, the :attr:`pin_recent_evidence`
  property, and the exported :class:`PinRebalance` dataclass. All
  consumed by the engine or :mod:`dimos.agents.memory.artefact_tool`.
"""

from __future__ import annotations

from dataclasses import dataclass
import threading

from dimos.agents.memory.pages import FidelityLevel, Page, PageType

__all__ = [
    "PageTable",
    "PinRebalance",
]


@dataclass(frozen=True)
class PinRebalance:
    """Diff returned by :meth:`PageTable.rebalance_evidence_pins`.

    Attributes:
        pinned: Page ids that were newly marked ``pinned_at_full``.
        unpinned: Page ids that lost ``pinned_at_full`` (they drop back to
            plain ``pinned=False`` unless they had some other pin).
    """

    pinned: list[str]
    unpinned: list[str]

    @property
    def empty(self) -> bool:
        return not self.pinned and not self.unpinned


class PageTable:
    """Thread-safe store for :class:`Page` objects.

    Operations that matter for correctness hold :attr:`_lock`; iteration
    helpers (:meth:`ordered`, :meth:`__len__`) return defensive copies so
    callers never observe the store mid-mutation.

    Args:
        pin_recent_evidence: How many of the *most recent* EVIDENCE pages
            should carry ``pinned_at_full``. Default matches the locked
            plan decision (``3``).
    """

    def __init__(self, pin_recent_evidence: int = 3) -> None:
        if pin_recent_evidence < 0:
            raise ValueError(f"pin_recent_evidence must be non-negative, got {pin_recent_evidence}")
        self._pages: list[Page] = []
        self._by_id: dict[str, Page] = {}
        self._by_artefact: dict[str, Page] = {}
        self._lock = threading.RLock()
        self._pin_recent_evidence = pin_recent_evidence
        # Ids whose ``pinned_at_full`` was flipped *by the auto rebalancer*
        # rather than by an explicit REFETCH_FAULT promotion. Only pages in
        # this set may be un-pinned by a subsequent rebalance — pages that
        # the LLM has rehydrated stay pinned for the rest of the session
        # (per the plan's one-way-upgrade rule).
        self._auto_pinned_ids: set[str] = set()

    # basic store operations
    def __len__(self) -> int:
        with self._lock:
            return len(self._pages)

    @property
    def pin_recent_evidence(self) -> int:
        return self._pin_recent_evidence

    def add(self, page: Page) -> None:
        """Append *page* to the store (turn_seq order is the caller's job).

        Raises ``ValueError`` if a page with the same id or artefact UUID
        already exists; that would desynchronise the lookups.
        """
        with self._lock:
            if page.id in self._by_id:
                raise ValueError(f"PageTable already contains a page with id {page.id!r}")
            if page.artefact_uuid and page.artefact_uuid in self._by_artefact:
                raise ValueError(
                    f"PageTable already contains an artefact UUID {page.artefact_uuid!r}"
                )
            self._pages.append(page)
            self._by_id[page.id] = page
            if page.artefact_uuid:
                self._by_artefact[page.artefact_uuid] = page

    def get(self, page_id: str) -> Page | None:
        with self._lock:
            return self._by_id.get(page_id)

    def get_by_artefact(self, artefact_uuid: str) -> Page | None:
        with self._lock:
            return self._by_artefact.get(artefact_uuid)

    def ordered(self) -> list[Page]:
        """Return a new list of pages in ingest order (cheap — shallow copy)."""
        with self._lock:
            return list(self._pages)

    def clear(self) -> None:
        with self._lock:
            self._pages.clear()
            self._by_id.clear()
            self._by_artefact.clear()
            self._auto_pinned_ids.clear()

    # pinning
    def evidence_pages(self) -> list[Page]:
        """Return EVIDENCE pages in ingest order."""
        with self._lock:
            return [p for p in self._pages if p.type is PageType.EVIDENCE]

    def rebalance_evidence_pins(self) -> PinRebalance:
        """Ensure exactly the last ``pin_recent_evidence`` EVIDENCE pages
        carry ``pinned_at_full``.

        The rebalance is *idempotent*: calling it repeatedly without any
        intervening mutation returns an empty :class:`PinRebalance`. It
        preserves the ``pinned`` flag for pages the LLM has explicitly
        rehydrated via ``get_artefact`` — those pages are handled as a
        separate, strictly additive set (we never implicitly un-pin them,
        per the plan's 'one-way upgrade' decision for REFETCH_FAULT).

        Returns:
            A :class:`PinRebalance` describing which page ids transitioned.
        """
        with self._lock:
            evidence = [p for p in self._pages if p.type is PageType.EVIDENCE]
            n = self._pin_recent_evidence
            # The "target" set is the last ``n`` evidence pages by ingest
            # order; earlier pages must drop their ``pinned_at_full``.
            target_ids = {p.id for p in evidence[-n:]} if n > 0 else set()

            pinned_ids: list[str] = []
            unpinned_ids: list[str] = []

            for page in evidence:
                should_pin = page.id in target_ids
                if should_pin and not page.pinned_at_full:
                    # Auto-pin this page. Track it so a subsequent
                    # rebalance knows it's safe to un-pin.
                    page.pinned_at_full = True
                    page.pinned = True
                    page.min_fidelity = FidelityLevel.FULL
                    self._auto_pinned_ids.add(page.id)
                    pinned_ids.append(page.id)
                elif not should_pin and page.id in self._auto_pinned_ids:
                    # Only un-pin pages that this rebalancer installed.
                    # Pages elevated via :meth:`mark_pinned_full` (e.g. by
                    # a REFETCH_FAULT) are left alone — the plan treats
                    # rehydrate as a one-way upgrade.
                    page.pinned_at_full = False
                    page.pinned = False
                    page.min_fidelity = FidelityLevel.POINTER
                    self._auto_pinned_ids.discard(page.id)
                    unpinned_ids.append(page.id)

            return PinRebalance(pinned=pinned_ids, unpinned=unpinned_ids)

    # rehydration
    def mark_pinned_full(self, page_id: str) -> bool:
        """Promote a page to ``pinned_at_full`` (one-way upgrade).

        This is the hook used by :mod:`dimos.agents.memory.artefact_tool`
        when the LLM asks for a full artefact — we never want to revert
        that decision automatically because the LLM would just ask again
        next turn.

        Returns:
            ``True`` if the page existed and was flipped (or was already
            pinned). ``False`` if no such page.
        """
        with self._lock:
            page = self._by_id.get(page_id)
            if page is None:
                return False
            page.pinned_at_full = True
            page.pinned = True
            page.min_fidelity = FidelityLevel.FULL
            # This is a manual promotion — make sure the auto-rebalance
            # tracker does not consider itself responsible for this pin.
            self._auto_pinned_ids.discard(page.id)
            return True
