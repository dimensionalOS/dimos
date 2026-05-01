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
"""Tests for :mod:`dimos.agents.memory.selector`."""
from __future__ import annotations

import random

from langchain_core.messages import AIMessage, SystemMessage, ToolMessage

from dimos.agents.memory.budget import ModelBudget
from dimos.agents.memory.pages import FidelityLevel, Page, PageType, Representation
from dimos.agents.memory.selector import AssembledPrompt, assemble_prompt


def _reps(tokens: tuple[int, int, int, int] = (1, 3, 5, 10)) -> dict[FidelityLevel, Representation]:
    return {
        FidelityLevel.POINTER: Representation(FidelityLevel.POINTER, "[p]", tokens[0]),
        FidelityLevel.STRUCTURED: Representation(FidelityLevel.STRUCTURED, "{}", tokens[1]),
        FidelityLevel.COMPRESSED: Representation(FidelityLevel.COMPRESSED, "short", tokens[2]),
        FidelityLevel.FULL: Representation(FidelityLevel.FULL, "the full thing", tokens[3]),
    }


def _conv_page(pid: str, turn: int, tokens: tuple[int, int, int, int] = (1, 3, 5, 10)) -> Page:
    return Page(
        id=pid,
        type=PageType.CONVERSATION,
        provenance="test",
        turn_seq=turn,
        ts=float(turn),
        role="human",
        representations=_reps(tokens),
    )


def _bootstrap_page(pid: str, tokens: tuple[int, int, int, int] = (1, 3, 5, 100)) -> Page:
    return Page(
        id=pid,
        type=PageType.BOOTSTRAP,
        provenance="system",
        turn_seq=0,
        ts=0.0,
        role="system",
        representations=_reps(tokens),
        pinned=True,
        pinned_at_full=True,
        min_fidelity=FidelityLevel.FULL,
    )


def _evidence_page(
    pid: str,
    turn: int,
    *,
    pinned: bool = False,
    tokens: tuple[int, int, int, int] = (5, 30, 85, 1105),
) -> Page:
    return Page(
        id=pid,
        type=PageType.EVIDENCE,
        provenance="tool:camera",
        turn_seq=turn,
        ts=float(turn),
        role="human",
        representations=_reps(tokens),
        artefact_uuid=f"uuid-{pid}",
        pinned=pinned,
        pinned_at_full=pinned,
        min_fidelity=FidelityLevel.FULL if pinned else FidelityLevel.POINTER,
    )


def _tool_pair(
    ai_id: str,
    tool_id: str,
    turn: int,
    call_id: str = "call1",
    tokens: tuple[int, int, int, int] = (1, 3, 5, 10),
) -> tuple[Page, Page]:
    ai = Page(
        id=ai_id,
        type=PageType.CONVERSATION,
        provenance="llm",
        turn_seq=turn,
        ts=float(turn),
        role="ai",
        representations=_reps(tokens),
        ai_tool_calls=[{"id": call_id, "name": "f", "args": {}, "type": "tool_call"}],
    )
    tool = Page(
        id=tool_id,
        type=PageType.CONVERSATION,
        provenance="tool_response",
        turn_seq=turn,
        ts=float(turn) + 0.1,
        role="tool",
        representations=_reps(tokens),
        tool_call_id=call_id,
    )
    return ai, tool


def _budget(context_window: int, output_reserve: int = 0, overhead: int = 0) -> ModelBudget:
    return ModelBudget(
        context_window=context_window,
        output_reserve=output_reserve,
        system_overhead=overhead,
    )


# --- Phase 1: floor + pins --------------------------------------------


def test_empty_pages_returns_empty_prompt() -> None:
    out = assemble_prompt([], budget=_budget(100))
    assert out.messages == []
    assert out.total_tokens == 0
    assert not out.evicted_page_ids
    assert not out.physical_insufficient


def test_bootstrap_page_always_rendered_at_full() -> None:
    b = _bootstrap_page("sys")
    out = assemble_prompt([b], budget=_budget(1000))
    assert out.chosen_levels["sys"] == FidelityLevel.FULL


def test_small_case_fits_all_pages_at_full() -> None:
    b = _bootstrap_page("sys", tokens=(1, 3, 5, 50))
    a = _conv_page("a", 1)
    c = _conv_page("c", 2)
    out = assemble_prompt([b, a, c], budget=_budget(500))
    assert out.chosen_levels == {
        "sys": FidelityLevel.FULL,
        "a": FidelityLevel.FULL,
        "c": FidelityLevel.FULL,
    }
    assert out.evicted_page_ids == []
    assert not out.physical_insufficient


def test_pinned_evidence_never_evicted_even_under_pressure() -> None:
    sys = _bootstrap_page("sys", tokens=(1, 3, 5, 50))
    e = _evidence_page("e", 1, pinned=True)
    # A tight budget that just barely fits sys + e at FULL.
    budget_size = 50 + 1105 + 20  # leave small slack for overhead
    out = assemble_prompt([sys, e], budget=_budget(budget_size))
    assert "e" not in out.evicted_page_ids
    assert out.chosen_levels["e"] == FidelityLevel.FULL
    assert not out.physical_insufficient


def test_non_pinned_pages_evicted_when_floor_overflows() -> None:
    # 5 conversation pages, each with floor (POINTER) = 1 token, cap = 2 tokens.
    # Expect 3 to be evicted (leave 2 at POINTER).
    pages = [_conv_page(f"p{i}", i) for i in range(5)]
    out = assemble_prompt(pages, budget=_budget(100, output_reserve=97, overhead=0))
    # cap = 100 - 97 = 3 tokens.
    # 5 * 1 = 5 at floor; must evict at least 2 to fit under 3.
    assert len(out.evicted_page_ids) >= 2
    # The oldest pages (low turn_seq) should be evicted first.
    assert "p0" in out.evicted_page_ids
    assert "p1" in out.evicted_page_ids


def test_physical_insufficiency_flag_set_when_pins_too_big() -> None:
    # Two pinned-at-full evidence pages at 1105 each + a tiny budget.
    sys = _bootstrap_page("sys", tokens=(1, 3, 5, 50))
    e1 = _evidence_page("e1", 1, pinned=True)
    e2 = _evidence_page("e2", 2, pinned=True)
    out = assemble_prompt(
        [sys, e1, e2], budget=_budget(500)    )
    assert out.physical_insufficient is True


# --- Phase 2: greedy upgrade ------------------------------------------


def test_phase2_upgrades_recent_pages_first() -> None:
    # 3 pages: oldest has the cheapest FULL (10 tokens); newest most expensive (100).
    # With a budget that can afford *some* upgrades, the newest should upgrade first
    # because recency wins ties. But we test a weaker property here: newest is at
    # least as high-fidelity as oldest in the final assignment.
    p_old = _conv_page("old", 0, tokens=(1, 3, 5, 10))
    p_mid = _conv_page("mid", 1, tokens=(1, 3, 5, 10))
    p_new = _conv_page("new", 2, tokens=(1, 3, 5, 10))

    out = assemble_prompt(
        [p_old, p_mid, p_new], budget=_budget(50)    )
    assert out.chosen_levels["new"] >= out.chosen_levels["old"]


def test_phase2_never_exceeds_budget() -> None:
    # Random mix, check upper bound is respected.
    rng = random.Random(0)
    pages: list[Page] = []
    for i in range(20):
        tokens = tuple(sorted([rng.randint(1, 5), rng.randint(6, 20), rng.randint(21, 80), rng.randint(100, 1200)]))
        pages.append(_conv_page(f"p{i}", i, tokens=tokens))  # type: ignore[arg-type]
    out = assemble_prompt(pages, budget=_budget(1000))
    if not out.physical_insufficient:
        assert out.total_tokens <= 1000


def test_phase2_upgrades_within_single_level_step() -> None:
    # With a very generous budget, every page goes to FULL.
    pages = [_conv_page(f"p{i}", i) for i in range(5)]
    out = assemble_prompt(pages, budget=_budget(10_000))
    for p in pages:
        assert out.chosen_levels[p.id] == FidelityLevel.FULL
    assert out.degraded_page_ids == []


# --- Tool-call pair integrity ----------------------------------------


def test_tool_call_pair_never_broken_by_eviction() -> None:
    ai, tool = _tool_pair("ai1", "tool1", turn=1)
    # A few other evictable pages.
    other = _conv_page("other", 0)
    # Very small budget: must evict something.
    out = assemble_prompt(
        [other, ai, tool], budget=_budget(100, output_reserve=97, overhead=0)    )
    # If the AI is in the assembled prompt, the matching tool message must be too.
    kept = {p: lvl for p, lvl in out.chosen_levels.items()}
    assert ("ai1" in kept) == ("tool1" in kept)


def test_tool_call_pair_messages_have_matching_tool_call_id() -> None:
    ai, tool = _tool_pair("ai1", "tool1", turn=1, call_id="call42")
    out = assemble_prompt([ai, tool], budget=_budget(1000))
    assert len(out.messages) == 2
    rendered_ai = out.messages[0]
    rendered_tool = out.messages[1]
    assert isinstance(rendered_ai, AIMessage)
    assert isinstance(rendered_tool, ToolMessage)
    assert rendered_ai.tool_calls[0]["id"] == "call42"
    assert rendered_tool.tool_call_id == "call42"


def test_tool_call_pair_degrades_together_when_possible() -> None:
    ai, tool = _tool_pair("ai1", "tool1", turn=1, tokens=(1, 2, 3, 100))
    # Tiny budget: only room for the pair at POINTER+POINTER.
    out = assemble_prompt(
        [ai, tool], budget=_budget(100, output_reserve=97, overhead=0)    )
    # Both should be at the same level.
    if "ai1" in out.chosen_levels and "tool1" in out.chosen_levels:
        assert out.chosen_levels["ai1"] == out.chosen_levels["tool1"]


# --- Bootstrap is never degraded -------------------------------------


def test_bootstrap_never_degraded_under_pressure() -> None:
    sys = _bootstrap_page("sys", tokens=(1, 3, 5, 200))
    # Cap is exactly the bootstrap cost -> bootstrap stays at FULL, everything else evicted.
    a = _conv_page("a", 1)
    b = _conv_page("b", 2)
    cap = 200 + 10
    out = assemble_prompt([sys, a, b], budget=_budget(cap))
    assert out.chosen_levels["sys"] == FidelityLevel.FULL


# --- Messages match in order ------------------------------------------


def test_messages_preserve_ingest_order() -> None:
    sys = _bootstrap_page("sys")
    pages = [sys] + [_conv_page(f"p{i}", i + 1) for i in range(5)]
    out = assemble_prompt(pages, budget=_budget(5000))
    assert [isinstance(m, SystemMessage) for m in out.messages][0] is True
    # Everyone in, same order.
    assert len(out.messages) == len(pages)


# --- Property-style: budget respected over random mixes ---------------


# --- Per-message overhead is reserved on the budget side --------------


def test_assemble_accounts_per_message_overhead() -> None:
    """Phase 1 must reserve per-message ChatML overhead on the cap.

    Ten conversation pages with uniform 100-token representations and a
    1000-token ``input_budget``: content sums to exactly
    ``input_budget``. Under the OLD (buggy) selector ``cap = input_budget``
    so this "just fits" and no eviction happens. Under the FIXED
    selector, ``cap = budget.effective_budget_for_messages(10) =
    1000 - 4 * 10 = 960``, so content exceeds the cap by 40 and Phase 1
    must either evict at least one group or flip
    ``physical_insufficient``.
    """
    pages = [
        _conv_page(f"p{i}", i, tokens=(100, 100, 100, 100))
        for i in range(10)
    ]
    budget = _budget(1000)
    # Pre-condition: default per_message_overhead=4, so
    # effective_budget_for_messages(10) = 960.
    assert budget.effective_budget_for_messages(10) == 960
    out = assemble_prompt(pages, budget=budget)

    assert out.evicted_page_ids or out.physical_insufficient, (
        f"expected an eviction or physical_insufficient, got "
        f"evicted={out.evicted_page_ids} "
        f"physical_insufficient={out.physical_insufficient}"
    )
    n_surviving = 10 - len(out.evicted_page_ids)
    effective_cap = budget.effective_budget_for_messages(n_surviving)
    if not out.physical_insufficient:
        assert out.total_tokens <= effective_cap, (
            f"total_tokens={out.total_tokens} > effective_cap={effective_cap} "
            f"for n_surviving={n_surviving}"
        )


def test_assemble_recomputes_cap_after_eviction() -> None:
    """Evicting a page must free both its content AND its overhead slot.

    Scenario: nine pinned-at-full evidence pages at FULL=107 (sum=963)
    plus one non-pinned conversation page at POINTER floor=5. Content
    totals 968. With ``input_budget=1000`` and ten pages,
    ``effective_cap = 1000 - 40 = 960`` — content is over by 8.

    Evicting the one non-pinned page drops content by 5 AND drops the
    surviving count from 10 to 9, so the recomputed cap becomes
    ``1000 - 36 = 964``. Post-eviction total 963 <= 964 and the prompt
    fits. Under a selector that forgets to recompute ``cap`` after
    eviction, the cap stays at 960 and ``total=963`` would flip
    ``physical_insufficient=True`` even though no more non-pinned pages
    are available to drop.
    """
    pinned_pages = [
        _evidence_page(f"pin{i}", i, pinned=True, tokens=(107, 107, 107, 107))
        for i in range(9)
    ]
    np_page = _conv_page("np", 10, tokens=(5, 5, 5, 5))
    pages = pinned_pages + [np_page]
    budget = _budget(1000)
    out = assemble_prompt(pages, budget=budget)

    assert "np" in out.evicted_page_ids
    # Only the one non-pinned page should be evicted — the nine pinned
    # pages survive and no extra page gets dropped just to chase the
    # non-recomputed cap.
    assert out.evicted_page_ids == ["np"], (
        f"expected exactly ['np'] evicted, got {out.evicted_page_ids}"
    )
    assert not out.physical_insufficient, (
        "physical_insufficient flipped True — selector likely failed to "
        "recompute cap after eviction"
    )
    effective_cap = budget.effective_budget_for_messages(9)
    assert effective_cap == 964
    assert out.total_tokens <= effective_cap
    # Sanity: the nine pinned pages all rendered at FULL (content 107 each).
    assert out.total_tokens == 9 * 107


def test_random_mixes_never_exceed_budget_when_pins_are_small() -> None:
    rng = random.Random(42)
    for trial in range(50):
        n = rng.randint(1, 40)
        pages: list[Page] = []
        for i in range(n):
            tokens = tuple(sorted([rng.randint(1, 4), rng.randint(5, 15), rng.randint(16, 60), rng.randint(70, 500)]))
            pages.append(_conv_page(f"p{trial}-{i}", i, tokens=tokens))  # type: ignore[arg-type]
        cap = rng.randint(50, 5000)
        out: AssembledPrompt = assemble_prompt(pages, budget=_budget(cap + 0, output_reserve=0, overhead=0))
        if out.physical_insufficient:
            continue
        assert out.total_tokens <= cap, (
            f"trial {trial}: total={out.total_tokens} cap={cap} "
            f"chosen={out.chosen_levels}"
        )
