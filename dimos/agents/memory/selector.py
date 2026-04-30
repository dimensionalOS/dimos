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

"""Two-phase greedy selector — MMU for the agent's conversation.

Given a list of :class:`Page` objects (from a :class:`PageTable`) and a
:class:`ModelBudget`, :func:`assemble_prompt` returns a list of LangChain
:class:`BaseMessage` objects whose total token cost is guaranteed not to
exceed the budget's ``input_budget``.

The algorithm runs in two phases:

**Phase 1 — pin satisfaction and eviction.**
Every page starts at its ``min_fidelity``. Pinned pages are non-droppable.
If the sum already exceeds the budget, we evict the least-valuable
non-pinned page and record the eviction on :class:`AssembledPrompt`.
If we cannot get under budget even after dropping every non-pinned page,
we set ``physical_insufficient=True`` so the caller can emit a
``PHYSICAL_INSUFFICIENCY`` fault.

**Phase 2 — greedy upgrade.**
The remaining pages are upgraded, cheapest cost per unit value first,
until the next upgrade would exceed the budget. Recency dominates: more
recent pages tie-break ahead of older ones, so the assembled prompt
always contains recent turns at their maximum available fidelity when it
fits.

Tool-call pairing: ``(AIMessage with tool_calls, matching ToolMessage)``
pairs are treated as an atomic unit — they are always selected or
degraded together. LangChain rejects orphan tool messages at the API
boundary, so breaking a pair would crash the state graph.

System prompts (BOOTSTRAP pages) always render at FULL. The plan locks
this; the rationale is that ``create_agent`` bundles the tool schema
into the system prompt and truncating it corrupts tool dispatch.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import TYPE_CHECKING

from dimos.agents.memory.pages import FidelityLevel, Page, PageType

if TYPE_CHECKING:
    from langchain_core.messages.base import BaseMessage

    from dimos.agents.memory.budget import ModelBudget

__all__ = [
    "AssembledPrompt",
    "assemble_prompt",
]


@dataclass
class AssembledPrompt:
    """Result of one :func:`assemble_prompt` call.

    Attributes:
        messages: LangChain messages in ingest order, ready for ``state_graph.stream``.
        total_tokens: Sum of representation ``token_estimate`` values for
            the chosen fidelity of each non-evicted page. This is a
            **content-only** sum — the per-message ChatML prelude
            (~4 tokens per message) is *not* included here. Per-message
            overhead is reserved on the :class:`ModelBudget` side via
            ``effective_budget_for_messages``; keeping it off
            :attr:`total_tokens` preserves the counter contract that
            ``count_text``/``count_message`` return content tokens only.
        chosen_levels: Which fidelity each included page was rendered at,
            keyed by page id. Useful for tests and for debugging.
        evicted_page_ids: Non-pinned pages dropped from this turn.
        degraded_page_ids: Pages rendered below their max-available fidelity.
        physical_insufficient: ``True`` when even the minimum-fidelity
            assembly (all non-pinned dropped) doesn't fit the budget. The
            caller emits a ``PHYSICAL_INSUFFICIENCY`` fault.
    """

    messages: list[BaseMessage]
    total_tokens: int
    chosen_levels: dict[str, FidelityLevel]
    evicted_page_ids: list[str] = field(default_factory=list)
    degraded_page_ids: list[str] = field(default_factory=list)
    physical_insufficient: bool = False


# Algorithm
def assemble_prompt(
    pages: list[Page],
    *,
    budget: ModelBudget,
) -> AssembledPrompt:
    """Build a token-capped prompt from the given pages.

    The pages must already be in ingest order (newest last). This order
    is preserved in :attr:`AssembledPrompt.messages`.

    Args:
        pages: All pages from the :class:`PageTable`, in ingest order.
        budget: The token ceiling; we target
            ``budget.effective_budget_for_messages(n_surviving)``, which
            is ``input_budget - per_message_overhead * n_surviving``. The
            cap therefore floats as Phase-1 evictions reduce
            ``n_surviving`` — each evicted page frees both its content
            tokens and its share of the per-message ChatML prelude.

    Returns:
        A populated :class:`AssembledPrompt`.
    """
    if not pages:
        return AssembledPrompt(messages=[], total_tokens=0, chosen_levels={})

    # Build atomic selection groups: tool-call pairs are one group; every
    # other page stands alone.
    groups = _build_selection_groups(pages)

    # Phase 1: start everyone at their ``min_fidelity``; evict non-pinned
    # groups if the floor over-budgets.
    chosen: dict[str, FidelityLevel] = {}
    for p in pages:
        chosen[p.id] = _starting_level(p)

    total = sum(chosen_token_cost(p, chosen[p.id]) for p in pages)
    evicted_ids: set[str] = set()

    # Per-message overhead (~4 tokens × n messages for the ChatML
    # prelude) is owned by :class:`ModelBudget`, not by representation
    # ``token_estimate`` values (which are content-only). The effective
    # cap therefore depends on how many messages survive; every Phase-1
    # eviction frees both the evicted content tokens AND one per-message
    # overhead slot, so we must recompute ``cap`` after each eviction
    # rather than pin it to the initial
    # ``input_budget - per_message_overhead * len(pages)``.
    surviving_count = len(pages)
    cap = budget.effective_budget_for_messages(surviving_count)

    # Evict least-valuable non-pinned group first. Utility order:
    # 1. Recency (older = lower value).
    # 2. Page type (CONVERSATION is expendable, EVIDENCE less so).
    if total > cap:
        eviction_order = _group_eviction_order(groups)
        for group in eviction_order:
            if total <= cap:
                break
            if any(p.pinned for p in group):
                continue  # pinned groups cannot be evicted
            saved = sum(chosen_token_cost(p, chosen[p.id]) for p in group)
            if saved == 0:
                continue
            total -= saved
            for p in group:
                evicted_ids.add(p.id)
            surviving_count -= len(group)
            cap = budget.effective_budget_for_messages(surviving_count)

    physical_insufficient = total > cap

    # Phase 2: greedy upgrade of surviving groups. We operate on
    # representation-level upgrades, not whole groups, so a tool-call pair
    # that cannot afford FULL can still upgrade to STRUCTURED together.
    if not physical_insufficient:
        _upgrade_until_full(groups, chosen, evicted_ids, cap_remaining=cap - total)

    # Final bookkeeping + message assembly.
    degraded_ids: list[str] = []
    messages: list[BaseMessage] = []
    total_tokens = 0

    for p in pages:
        if p.id in evicted_ids:
            continue
        level = chosen[p.id]
        rep = p.rep_at(level)
        messages.append(p.as_message(level))
        total_tokens += rep.token_estimate
        if level < p.max_available():
            degraded_ids.append(p.id)

    return AssembledPrompt(
        messages=messages,
        total_tokens=total_tokens,
        chosen_levels={pid: lvl for pid, lvl in chosen.items() if pid not in evicted_ids},
        evicted_page_ids=sorted(evicted_ids),
        degraded_page_ids=degraded_ids,
        physical_insufficient=physical_insufficient,
    )


# Internals
def chosen_token_cost(page: Page, level: FidelityLevel) -> int:
    """Return the cached token cost of *page* rendered at *level*."""
    return page.rep_at(level).token_estimate


def _starting_level(page: Page) -> FidelityLevel:
    """Floor fidelity for Phase 1.

    BOOTSTRAP pages start at FULL (they are always pinned at FULL). Other
    pages start at ``min_fidelity`` — which is POINTER for ordinary
    conversation and FULL for pinned-at-full evidence.
    """
    if page.type is PageType.BOOTSTRAP:
        return FidelityLevel.FULL
    return max(page.min_fidelity, FidelityLevel.POINTER)


def _build_selection_groups(pages: list[Page]) -> list[list[Page]]:
    """Partition *pages* into atomic selection groups.

    Every group is a list of pages that must be chosen and rendered at the
    same fidelity bucket (i.e. all FULL or all <= STRUCTURED). The only
    kind of multi-page group in practice today is ``(AIMessage with
    tool_calls, ToolMessage(s) matching those calls)`` — LangChain's graph
    rejects orphan tool messages.

    Groups preserve the input order: the first page in each group is the
    earliest by ingest order.
    """
    groups: list[list[Page]] = []
    # Map tool_call_id -> index of the group whose AI message owns it.
    call_id_to_group: dict[str, int] = {}

    for page in pages:
        if page.role == "ai" and page.ai_tool_calls:
            groups.append([page])
            gidx = len(groups) - 1
            for call in page.ai_tool_calls:
                call_id = call.get("id")
                if isinstance(call_id, str):
                    call_id_to_group[call_id] = gidx
            continue

        if page.role == "tool" and page.tool_call_id in call_id_to_group:
            gidx = call_id_to_group[page.tool_call_id]
            groups[gidx].append(page)
            continue

        groups.append([page])

    return groups


def _group_eviction_order(groups: list[list[Page]]) -> list[list[Page]]:
    """Return groups ordered from most-expendable to least-expendable.

    Only non-pinned groups will actually be evicted; we still include all
    groups in the ordering for a stable iteration order.

    Heuristic:

    * Oldest first (low ``turn_seq``).
    * Within a turn, CONVERSATION outranks PLAN outranks PREFERENCE outranks
      CONSTRAINT (i.e. CONVERSATION is the most expendable).

    EVIDENCE groups that aren't pinned also follow the age rule but are
    slightly stickier (they get a small constant added). BOOTSTRAP is
    always pinned so never participates.
    """

    def _expendability(group: list[Page]) -> tuple[int, int, str]:
        head = group[0]
        # Lower tuple = evict earlier.
        type_sticky = {
            PageType.BOOTSTRAP: 1000,  # never evicted (safety)
            PageType.CONSTRAINT: 500,
            PageType.PREFERENCE: 400,
            PageType.PLAN: 300,
            PageType.EVIDENCE: 100,
            PageType.CONVERSATION: 0,
        }.get(head.type, 0)
        return (head.turn_seq, type_sticky, head.id)

    return sorted(groups, key=_expendability)


def _upgrade_until_full(
    groups: list[list[Page]],
    chosen: dict[str, FidelityLevel],
    evicted_ids: set[str],
    *,
    cap_remaining: int,
) -> None:
    """Phase 2: greedily upgrade remaining groups within the budget.

    We iterate until no upgrade fits. Each iteration picks the single
    upgrade with the best (lowest) delta cost that still satisfies two
    tie-breakers: recency first, then group identity.

    This is O(N * U) where N is the number of upgrade opportunities and
    U is the number of upgrades applied. Good enough for the scale we
    care about (hundreds of pages).
    """
    if cap_remaining <= 0:
        return

    while cap_remaining > 0:
        best: tuple[int, int, int, list[Page], FidelityLevel] | None = None
        # (delta_cost, -turn_seq, group_idx, group, target_level)
        #
        # Lower delta is better; higher turn_seq (negated so "smaller
        # negative") wins ties. group_idx keeps ordering stable.

        for gidx, group in enumerate(groups):
            if any(p.id in evicted_ids for p in group):
                continue
            # Current common upgrade level for this group: we upgrade the
            # whole group together. We pick the *minimum* current level in
            # the group; that is the one that needs upgrading the most.
            current = min(chosen[p.id] for p in group)
            if current >= FidelityLevel.FULL:
                continue
            target = FidelityLevel(current + 1)

            delta = 0
            group_has_target = False
            for p in group:
                if chosen[p.id] >= target:
                    continue
                # If this page doesn't have the higher rung cached, we use
                # its fallback (which ``rep_at`` resolves) and skip
                # upgrading its stored level beyond what actually exists.
                if target > p.max_available():
                    continue
                old = chosen_token_cost(p, chosen[p.id])
                new = chosen_token_cost(p, target)
                delta += new - old
                group_has_target = True

            if not group_has_target:
                continue
            if delta > cap_remaining:
                continue

            head = group[0]
            candidate = (delta, -head.turn_seq, gidx, group, target)
            if best is None or candidate < best:
                best = candidate

        if best is None:
            break

        delta, _recency, _gidx, group, target = best
        for p in group:
            if target <= p.max_available() and chosen[p.id] < target:
                chosen[p.id] = target
        cap_remaining -= delta
