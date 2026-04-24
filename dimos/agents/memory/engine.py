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

"""Façade that wires the memory layer into the agent.

:class:`MemoryEngine` is the single object the agent module talks to: it
owns the :class:`~dimos.agents.memory.page_table.PageTable`, the token
counter, the fault observer, and the monotonic turn counter. It exposes
two lifecycle methods — :meth:`ingest` and :meth:`assemble` — plus
:meth:`get_artefact_tool` for wiring the ``get_artefact`` LangChain tool
into the agent's tool list.

Thread safety: :class:`MemoryEngine` is called from the agent's single
worker thread. The underlying :class:`PageTable` is still locked because
other threads (UI probes, structlog consumers) may iterate it.
"""

from __future__ import annotations

import threading
import time
from typing import TYPE_CHECKING

from dimos.agents.memory.budget import (
    DEFAULT_OUTPUT_RESERVE,
    DEFAULT_SYSTEM_OVERHEAD,
    ModelBudget,
    resolve_budget,
)
from dimos.agents.memory.faults import FaultObserver
from dimos.agents.memory.ingestion import ingest_message
from dimos.agents.memory.page_table import PageTable
from dimos.agents.memory.pages import FidelityLevel, Page, PageType
from dimos.agents.memory.selector import AssembledPrompt, assemble_prompt
from dimos.agents.memory.tokens import HeuristicCounter, TiktokenCounter, TokenCounter

if TYPE_CHECKING:
    from langchain_core.messages.base import BaseMessage
    from langchain_core.tools import StructuredTool

    from dimos.agents.memory.faults import FaultEvent
    from dimos.core.stream import Out

__all__ = [
    "MemoryEngine",
]


def _make_default_counter(model: str) -> TokenCounter:
    """Best-effort :class:`TokenCounter` construction.

    If tiktoken is installed we prefer it because it matches provider
    billing; otherwise we fall back to the character-heuristic. Any
    unexpected failure downgrades silently — the heuristic is correct
    enough that losing tiktoken is not a correctness issue.
    """
    try:
        return TiktokenCounter(model)
    except Exception:  # pragma: no cover — pure defensive path
        return HeuristicCounter()


class MemoryEngine:
    """Owner of the agent's memory state.

    Args:
        model_name: LangChain / ollama / anthropic model string. Used to
            pick a default context window and to parameterise the
            :class:`TiktokenCounter` backend.
        token_budget: If set, used directly as the context window
            (overriding the model lookup). ``None`` uses the table default.
        pin_recent_evidence: How many of the most recent EVIDENCE pages
            stay pinned at FULL.
        output_reserve_tokens: Tokens held back for the LLM's response.
        system_overhead: Formatting / tool-schema slack.
        faults_out: Optional ``Out[FaultEvent]`` stream for live observability.
        token_counter: Optional override for the token counter (tests use
            this to force a deterministic backend).
    """

    def __init__(
        self,
        *,
        model_name: str,
        token_budget: int | None = None,
        pin_recent_evidence: int = 3,
        output_reserve_tokens: int = DEFAULT_OUTPUT_RESERVE,
        system_overhead: int = DEFAULT_SYSTEM_OVERHEAD,
        faults_out: Out[FaultEvent] | None = None,
        token_counter: TokenCounter | None = None,
    ) -> None:
        self.model_name = model_name
        self._budget: ModelBudget = resolve_budget(
            model_name,
            override=token_budget,
            output_reserve=output_reserve_tokens,
            system_overhead=system_overhead,
        )
        self._table = PageTable(pin_recent_evidence=pin_recent_evidence)
        self._counter: TokenCounter = token_counter or _make_default_counter(model_name)
        self._observer = FaultObserver(stream_out=faults_out)
        self._turn_seq = 0
        self._lock = threading.RLock()

    # introspection
    @property
    def budget(self) -> ModelBudget:
        """The immutable budget used for assembly (exposed for tests)."""
        return self._budget

    @property
    def page_table(self) -> PageTable:
        return self._table

    @property
    def observer(self) -> FaultObserver:
        return self._observer

    @property
    def turn_seq(self) -> int:
        """Current turn counter; increments on every :meth:`ingest`."""
        return self._turn_seq

    def pages(self) -> list[Page]:
        """Snapshot of all pages in ingest order."""
        return self._table.ordered()

    # lifecycle
    def ingest(
        self,
        msg: BaseMessage,
        *,
        page_type_hint: PageType | None = None,
    ) -> list[Page]:
        """Turn *msg* into one or more :class:`Page` objects and add
        them to the page table.

        Increments the turn counter, runs ingestion, stores each
        produced page in content-part order, and rebalances evidence
        pins exactly once per call if any of the produced pages is
        EVIDENCE. Emits a ``PIN_REBALANCE`` fault when the pinned-at-full
        set changes so operators can see evidence pressure at a glance.

        Returns:
            The full list of pages produced by :func:`ingest_message`
            for this call, in ingest order. A multimodal ``HumanMessage``
            may split into a CONVERSATION page followed by one EVIDENCE
            page per image; simple messages produce a single-element
            list.
        """
        with self._lock:
            self._turn_seq += 1
            pages = ingest_message(
                msg,
                turn_seq=self._turn_seq,
                ts=time.time(),
                token_counter=self._counter,
                page_type_hint=page_type_hint,
            )
            for page in pages:
                self._table.add(page)
            if any(p.type is PageType.EVIDENCE for p in pages):
                diff = self._table.rebalance_evidence_pins()
                if not diff.empty:
                    self._observer.pin_rebalance(
                        self._turn_seq,
                        pinned_page_ids=diff.pinned,
                        unpinned_page_ids=diff.unpinned,
                    )
            return pages

    def assemble(self) -> AssembledPrompt:
        """Build the per-turn assembled prompt and emit observability faults.

        The returned :class:`AssembledPrompt` can be fed directly into
        LangGraph's ``state_graph.stream({"messages": ...})``.
        """
        with self._lock:
            pages = self._table.ordered()
            result = assemble_prompt(pages, budget=self._budget)

            # Fault emission — after assembly so we can attach levels.
            for pid in result.evicted_page_ids:
                self._observer.evict(pid, self._turn_seq, reason="over_budget")
            for pid in result.degraded_page_ids:
                page = self._table.get(pid)
                if page is None:  # pragma: no cover — defensive
                    continue
                chosen = result.chosen_levels.get(pid, FidelityLevel.POINTER)
                self._observer.degrade(
                    pid,
                    self._turn_seq,
                    from_level=page.max_available().name,
                    to_level=chosen.name,
                )

            if result.physical_insufficient:
                # Report the *effective* cap the selector actually
                # enforced (``input_budget - per_message_overhead *
                # n_surviving``), not the gross ``input_budget``.
                # Otherwise operators see ``needed=X, available=input_budget``
                # and wonder why the selector gave up when X was well
                # under the gross ceiling. ``n_surviving`` is the count
                # of messages that actually go on the wire
                # (``result.messages``), which matches the ``n`` the
                # selector fed into
                # :meth:`ModelBudget.effective_budget_for_messages` at the
                # point it flipped ``physical_insufficient=True``.
                # ``result.total_tokens`` is still content-only per round
                # 9, so the comparison is apples-to-apples.
                n_surviving = len(result.messages)
                self._observer.physical_insufficiency(
                    self._turn_seq,
                    needed=result.total_tokens,
                    available=self._budget.effective_budget_for_messages(n_surviving),
                )

            return result

    def emit_physical_insufficiency(self, exception: Exception | None = None) -> None:
        """Public hook for callers (the agent worker thread) to report a
        failed turn — typically because the LLM returned
        ``context_length_exceeded`` *despite* our best-effort assembly.

        Reports ``available=input_budget`` (gross) rather than the
        effective cap because this method is invoked by the worker after
        an LLM-side ``context_length_exceeded``, where the surviving-
        message count is not known at the call site. The asymmetry with
        :meth:`assemble`'s emission is deliberate: that path knows
        exactly how many messages it queued up and can subtract their
        per-message overhead; this path only knows the budget ceiling.
        """
        self._observer.physical_insufficiency(
            self._turn_seq,
            needed=-1,
            available=self._budget.input_budget,
            exception=repr(exception) if exception is not None else None,
        )

    def clear(self) -> None:
        """Reset the page table. Does not reset the turn counter — that
        keeps monotonicity across clears (tests rely on this)."""
        with self._lock:
            self._table.clear()

    # artefact rehydration
    def request_full(self, artefact_uuid: str) -> bool:
        """Flag an EVIDENCE page for FULL-fidelity rendering on the next
        :meth:`assemble`. Used by the ``get_artefact`` tool.

        Returns:
            ``True`` if the UUID was found and the page was flagged.
            ``False`` if no such artefact exists in the table.
        """
        with self._lock:
            page = self._table.get_by_artefact(artefact_uuid)
            if page is None:
                return False
            self._table.mark_pinned_full(page.id)
            self._observer.refetch(page.id, self._turn_seq, artefact_uuid)
            return True

    def get_artefact_tool(self) -> StructuredTool:
        """Build the LangChain tool that lets the LLM rehydrate evidence.

        Deferred import: this method is the only path that needs
        LangChain's tool runtime, so we pay the import cost only when a
        consumer actually asks for the tool.
        """
        from dimos.agents.memory.artefact_tool import build_get_artefact_tool

        return build_get_artefact_tool(self)
