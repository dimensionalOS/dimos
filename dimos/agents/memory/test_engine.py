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
"""Tests for :mod:`dimos.agents.memory.engine`."""
from __future__ import annotations

import base64

import cv2
import numpy as np
from langchain_core.messages import HumanMessage, SystemMessage

from dimos.agents.memory.engine import MemoryEngine
from dimos.agents.memory.faults import FaultEvent, FaultKind
from dimos.agents.memory.pages import FidelityLevel, PageType
from dimos.agents.memory.tokens import HeuristicCounter


class _FakeOut:
    def __init__(self) -> None:
        self.published: list[FaultEvent] = []

    def publish(self, ev: FaultEvent) -> None:
        self.published.append(ev)


def _image_msg(size: int = 256) -> HumanMessage:
    img = np.full((size, size, 3), 64, dtype=np.uint8)
    ok, buf = cv2.imencode(".jpg", img, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
    assert ok
    b64 = base64.b64encode(buf.tobytes()).decode("ascii")
    return HumanMessage(
        content=[
            {"type": "text", "text": "camera snapshot"},
            {"type": "image_url", "image_url": {"url": f"data:image/jpeg;base64,{b64}"}},
        ]
    )


def _engine(token_budget: int = 8000, pin_recent_evidence: int = 3) -> MemoryEngine:
    return MemoryEngine(
        model_name="gpt-4o",
        token_budget=token_budget,
        pin_recent_evidence=pin_recent_evidence,
        output_reserve_tokens=0,
        system_overhead=0,
        token_counter=HeuristicCounter(),
    )


def test_ingest_increments_turn_seq_and_stores_page() -> None:
    eng = _engine()
    assert eng.turn_seq == 0
    pages = eng.ingest(HumanMessage(content="hi"))
    assert eng.turn_seq == 1
    # ``ingest`` returns ``list[Page]``. A text-only HumanMessage yields
    # exactly one page.
    assert len(pages) == 1
    page = pages[0]
    assert page.id in {p.id for p in eng.pages()}


def test_assemble_keeps_recent_evidence_at_full() -> None:
    eng = _engine(token_budget=4000, pin_recent_evidence=3)
    eng.ingest(SystemMessage(content="You are a robot."))
    for _ in range(5):
        eng.ingest(_image_msg())
    out = eng.assemble()

    # The last 3 evidence pages should render at FULL; earlier ones may be
    # degraded.
    evidence_pages = [p for p in eng.pages() if p.type is PageType.EVIDENCE]
    recent_ids = {p.id for p in evidence_pages[-3:]}
    for pid in recent_ids:
        assert out.chosen_levels[pid] == FidelityLevel.FULL


def test_assemble_never_exceeds_input_budget_unless_insufficient() -> None:
    eng = _engine(token_budget=4000, pin_recent_evidence=1)
    eng.ingest(SystemMessage(content="system prompt"))
    for i in range(10):
        eng.ingest(HumanMessage(content=f"turn {i}: " + "x" * 500))
    out = eng.assemble()
    if not out.physical_insufficient:
        assert out.total_tokens <= eng.budget.input_budget


def test_pin_rebalance_fault_emitted_when_evidence_set_changes() -> None:
    out_stream = _FakeOut()
    eng = MemoryEngine(
        model_name="gpt-4o",
        token_budget=8000,
        pin_recent_evidence=2,
        output_reserve_tokens=0,
        system_overhead=0,
        token_counter=HeuristicCounter(),
        faults_out=out_stream,  # type: ignore[arg-type]
    )
    eng.ingest(_image_msg())
    eng.ingest(_image_msg())
    eng.ingest(_image_msg())
    kinds = [ev.kind for ev in out_stream.published]
    assert FaultKind.PIN_REBALANCE in kinds


def test_request_full_rehydrates_evidence_and_emits_refetch_fault() -> None:
    out_stream = _FakeOut()
    eng = MemoryEngine(
        model_name="gpt-4o",
        # Tight enough that the non-pinned image cannot afford a FULL
        # upgrade, but loose enough that it survives Phase 1 eviction at
        # its POINTER floor.
        token_budget=1300,
        pin_recent_evidence=1,
        output_reserve_tokens=0,
        system_overhead=0,
        token_counter=HeuristicCounter(),
        faults_out=out_stream,  # type: ignore[arg-type]
    )
    eng.ingest(SystemMessage(content="sys"))
    eng.ingest(_image_msg())  # e1
    eng.ingest(_image_msg())  # e2

    # At this point e1 is no longer pinned (only 1 evidence is pinned).
    e1 = [p for p in eng.pages() if p.type is PageType.EVIDENCE][0]
    uuid = e1.artefact_uuid
    assert uuid is not None

    out1 = eng.assemble()
    # e1 almost certainly below FULL because the budget is tight.
    assert out1.chosen_levels[e1.id] < FidelityLevel.FULL

    ok = eng.request_full(uuid)
    assert ok is True
    assert FaultKind.REFETCH_FAULT in [ev.kind for ev in out_stream.published]

    # Unknown UUID → False, no extra fault beyond what's already there.
    assert eng.request_full("not-a-uuid") is False


def test_physical_insufficiency_emitted_when_pins_exceed_budget() -> None:
    out_stream = _FakeOut()
    eng = MemoryEngine(
        model_name="gpt-4o",
        token_budget=200,  # tiny — images cost ~1105 each at FULL
        pin_recent_evidence=3,
        output_reserve_tokens=0,
        system_overhead=0,
        token_counter=HeuristicCounter(),
        faults_out=out_stream,  # type: ignore[arg-type]
    )
    eng.ingest(_image_msg())
    eng.ingest(_image_msg())
    eng.ingest(_image_msg())
    out = eng.assemble()
    assert out.physical_insufficient is True
    assert FaultKind.PHYSICAL_INSUFFICIENCY in [ev.kind for ev in out_stream.published]


def test_physical_insufficient_fault_reports_effective_budget() -> None:
    """The PHYSICAL_INSUFFICIENCY fault must report the cap the selector
    actually enforced (``effective_budget_for_messages(n_surviving)``),
    not the gross ``input_budget``. Otherwise operators see
    ``needed=X, available=input_budget`` and wonder why the selector
    gave up when X looks well under the gross ceiling.
    """
    out_stream = _FakeOut()
    eng = MemoryEngine(
        model_name="gpt-4o",
        token_budget=200,  # tiny — images cost ~1105 each at FULL
        pin_recent_evidence=3,
        output_reserve_tokens=0,
        system_overhead=0,
        token_counter=HeuristicCounter(),
        faults_out=out_stream,  # type: ignore[arg-type]
    )
    # One BOOTSTRAP system page (always pinned at FULL) plus three image
    # EVIDENCE pages (all pinned by pin_recent_evidence=3). With
    # token_budget=200, the pinned pages cannot fit even at FULL, so
    # physical_insufficient flips True and at least one pinned page
    # survives on the wire (BOOTSTRAP cannot be evicted).
    eng.ingest(SystemMessage(content="you are a robot"))
    eng.ingest(_image_msg())
    eng.ingest(_image_msg())
    eng.ingest(_image_msg())
    result = eng.assemble()

    assert result.physical_insufficient is True
    n_surviving = len(result.messages)
    assert n_surviving >= 1, (
        "precondition: at least one message must survive so the "
        "strictly-less-than assertion below is meaningful"
    )

    ev = eng.observer.last_event
    assert ev is not None
    assert ev.kind is FaultKind.PHYSICAL_INSUFFICIENCY
    expected_available = eng.budget.effective_budget_for_messages(n_surviving)
    assert ev.details["available"] == expected_available, (
        f"expected available={expected_available} for n_surviving="
        f"{n_surviving}, got {ev.details['available']}"
    )
    # Effective cap must be strictly less than the gross
    # ``input_budget`` whenever ``n_surviving`` is positive and
    # ``per_message_overhead`` is positive (both true here).
    assert ev.details["available"] < eng.budget.input_budget, (
        f"available={ev.details['available']} must be strictly less than "
        f"input_budget={eng.budget.input_budget} — effective-cap reporting "
        f"regressed to the gross budget"
    )
    # Sanity: ``needed`` is still content-only.
    assert ev.details["needed"] == result.total_tokens


def test_emit_physical_insufficiency_still_reports_input_budget() -> None:
    """``emit_physical_insufficiency`` intentionally reports
    ``available=input_budget`` (gross), not the effective cap. The
    asymmetry with ``assemble()``'s emission is deliberate — this hook
    is invoked by the worker after an LLM-side context_length_exceeded
    where the surviving-message count is not known at the call site.
    This regression lock flags any future attempt to unify the two
    paths without reading the docstring rationale.
    """
    out_stream = _FakeOut()
    eng = MemoryEngine(
        model_name="gpt-4o",
        token_budget=8000,
        pin_recent_evidence=3,
        output_reserve_tokens=0,
        system_overhead=0,
        token_counter=HeuristicCounter(),
        faults_out=out_stream,  # type: ignore[arg-type]
    )
    eng.emit_physical_insufficiency(Exception("context_length_exceeded"))

    ev = eng.observer.last_event
    assert ev is not None
    assert ev.kind is FaultKind.PHYSICAL_INSUFFICIENCY
    # Gross, not effective — intentional asymmetry documented on
    # ``MemoryEngine.emit_physical_insufficiency``.
    assert ev.details["available"] == eng.budget.input_budget
    assert ev.details["needed"] == -1


def test_emit_physical_insufficiency_hook_from_worker_thread() -> None:
    out_stream = _FakeOut()
    eng = MemoryEngine(
        model_name="gpt-4o",
        token_budget=8000,
        pin_recent_evidence=3,
        output_reserve_tokens=0,
        system_overhead=0,
        token_counter=HeuristicCounter(),
        faults_out=out_stream,  # type: ignore[arg-type]
    )
    eng.emit_physical_insufficiency(Exception("context_length_exceeded"))
    kinds = [ev.kind for ev in out_stream.published]
    assert FaultKind.PHYSICAL_INSUFFICIENCY in kinds
    ev = next(e for e in out_stream.published if e.kind == FaultKind.PHYSICAL_INSUFFICIENCY)
    assert "context_length_exceeded" in ev.details.get("exception", "")


def test_clear_resets_pages_but_keeps_turn_seq() -> None:
    eng = _engine()
    eng.ingest(HumanMessage(content="x"))
    eng.ingest(HumanMessage(content="y"))
    assert eng.turn_seq == 2
    eng.clear()
    assert eng.pages() == []
    assert eng.turn_seq == 2


def test_budget_resolved_from_model_name() -> None:
    eng = MemoryEngine(model_name="gpt-4o", token_counter=HeuristicCounter())
    assert eng.budget.context_window == 128_000


def test_long_session_stays_within_budget() -> None:
    """End-to-end soak test: 50 text turns + 20 images must stay under budget."""
    eng = _engine(token_budget=8000, pin_recent_evidence=3)
    eng.ingest(SystemMessage(content="robot"))
    for i in range(50):
        eng.ingest(HumanMessage(content=f"user turn {i}: " + "word " * 20))
        if i % 2 == 0:
            eng.ingest(_image_msg())
    out = eng.assemble()
    if not out.physical_insufficient:
        assert out.total_tokens <= eng.budget.input_budget

    # Last 3 images still at FULL.
    evidence_pages = [p for p in eng.pages() if p.type is PageType.EVIDENCE]
    for p in evidence_pages[-3:]:
        assert out.chosen_levels[p.id] == FidelityLevel.FULL
