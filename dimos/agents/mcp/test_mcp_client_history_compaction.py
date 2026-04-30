# Copyright 2025-2026 Dimensional Inc.
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
"""Integration tests for :class:`McpClient` ↔ :class:`MemoryEngine`.

Two tests live here:

* ``test_history_compaction_keeps_recent_images_at_full_under_pressure``
  — drives a real :class:`MemoryEngine` from a bypass-constructed
  :class:`McpClient` under a tight token budget and locks in the key
  compaction invariants (effective-budget ceiling, recent-evidence
  pinning, degrade-fault emission, no physical insufficiency).

* ``test_thread_loop_recovers_from_process_message_exception`` —
  regression guard for the motivating bug: *"once the worker thread
  dies, subsequent messages accumulate in the queue but never get
  processed; ``agent_idle`` stays at ``False`` forever."* Verifies the
  ``try/except`` wrapper in ``_thread_loop`` keeps the worker alive
  across turn failures AND forwards the exception to the fault stream.
"""

from __future__ import annotations

import base64
from dataclasses import dataclass, field
from queue import Queue
from threading import Event, RLock, Thread
import time
from unittest.mock import MagicMock

import cv2
from langchain_core.messages import HumanMessage
import numpy as np
import pytest

from dimos.agents.mcp.mcp_client import McpClient
from dimos.agents.memory.engine import MemoryEngine
from dimos.agents.memory.faults import FaultEvent, FaultKind
from dimos.agents.memory.pages import FidelityLevel, PageType

# fixtures / helpers
#
# Kept file-local rather than shared via conftest.py: other test files
# in this package use their own small helpers (e.g. ``_image_msg``
# inside ``test_artefact_tool.py``) rather than any common fixture
# module.


class _FakeOut:
    """Minimal ``Out[FaultEvent]`` stand-in that records every publish.

    Mirrors the pattern from ``test_artefact_tool.py`` so the engine's
    real :class:`FaultObserver` has somewhere to deliver events without
    pulling in the full LCM transport stack.
    """

    def __init__(self) -> None:
        self.published: list[FaultEvent] = []

    def publish(self, ev: FaultEvent) -> None:
        self.published.append(ev)


@dataclass
class _FakeAgentIdleStream:
    """Stand-in for ``client.agent_idle`` (an ``Out[bool]``)."""

    idle_history: list[bool] = field(default_factory=list)

    def publish(self, value: bool) -> None:
        self.idle_history.append(value)


def _image_msg(size: int = 256) -> HumanMessage:
    """Build a real JPEG-encoded image :class:`HumanMessage`.

    Matches the shape used by ``test_artefact_tool.py``; reproduced
    inline here because importing from another test file is brittle
    (pytest test modules aren't a public API).
    """
    img = np.full((size, size, 3), 128, dtype=np.uint8)
    ok, buf = cv2.imencode(".jpg", img, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
    assert ok
    b64 = base64.b64encode(buf.tobytes()).decode("ascii")
    return HumanMessage(
        content=[
            {"type": "image_url", "image_url": {"url": f"data:image/jpeg;base64,{b64}"}},
        ]
    )


def _make_config_mock(
    *,
    model: str = "gpt-4o",
    token_budget: int = 8000,
    pin_recent_evidence: int = 3,
    output_reserve_tokens: int = 256,
) -> MagicMock:
    """Build a ``config`` MagicMock with the engine knobs populated.

    :class:`McpClient`'s ``__init__`` reads ``self.config.model`` etc. to
    construct the engine. We bypass ``__init__`` in these tests, so the
    config need only satisfy the assertions we actually exercise.
    """
    cfg = MagicMock()
    cfg.model = model
    cfg.token_budget = token_budget
    cfg.pin_recent_evidence = pin_recent_evidence
    cfg.output_reserve_tokens = output_reserve_tokens
    return cfg


def _build_bypass_client(
    *,
    faults_out: _FakeOut,
    agent_idle: _FakeAgentIdleStream | None = None,
    state_graph: object | None = None,
) -> McpClient:
    """Construct an :class:`McpClient` via ``__new__`` with a real
    :class:`MemoryEngine` and MagicMock stubs for every other attribute
    the test does not explicitly drive.

    This sidesteps pydantic ``ModuleConfig`` validation and
    :meth:`Module.__init__`'s transport wiring, both of which would
    require a full blueprint / coordinator harness we don't have in
    unit-test scope.
    """
    client = McpClient.__new__(McpClient)
    client.config = _make_config_mock()
    client._engine = MemoryEngine(
        model_name=client.config.model,
        token_budget=client.config.token_budget,
        pin_recent_evidence=client.config.pin_recent_evidence,
        output_reserve_tokens=client.config.output_reserve_tokens,
        faults_out=faults_out,  # type: ignore[arg-type]
    )
    client._lock = RLock()
    client._message_queue = Queue()
    client._tool_registry = {}
    client._stop_event = Event()
    client._state_graph = state_graph if state_graph is not None else MagicMock()
    client._http_client = MagicMock()
    client._seq_ids = MagicMock()
    if agent_idle is not None:
        client.agent_idle = agent_idle  # type: ignore[assignment]
    return client


# Test 1 — compaction invariants
def test_history_compaction_keeps_recent_images_at_full_under_pressure() -> None:
    """Under a tight token budget, compaction keeps the most recent
    EVIDENCE pages at FULL fidelity, degrades older
    CONVERSATION pages, fits under the effective budget, and stays
    physically sufficient.

    This locks in:

    * Ingestion correctly produces EVIDENCE pages for images and
      CONVERSATION pages for text.
    * The selector targets
      ``budget.effective_budget_for_messages(n_surviving)`` (NOT the
      raw ``input_budget``).
    * The pin policy — ``pin_recent_evidence=3`` keeps the last three
      EVIDENCE pages ``pinned_at_full=True`` so the selector cannot
      degrade them.
    """
    fake_faults_out = _FakeOut()
    client = _build_bypass_client(faults_out=fake_faults_out)

    # Ingest images first. The pin rebalance in MemoryEngine.ingest
    # keeps the LAST 3 EVIDENCE pages pinned regardless of ingest order,
    # so seeding all 10 up front and then burying them under text is
    # the cleanest way to stress the selector: if the recent-evidence
    # invariant holds, all 3 kept-pinned images survive; if the engine
    # were to re-pin by wall-clock-recency instead of evidence-recency
    # (a plausible refactor bug), the text-heavy tail would displace
    # the images and the test would catch it.
    for _ in range(10):
        client._engine.ingest(_image_msg())

    # ~100 tokens each at the heuristic's 4-chars-per-token rate; 100
    # turns at that weight is enough to comfortably exceed an 8000-token
    # budget once per-message overhead is reserved, which forces Phase 1
    # evictions on the CONVERSATION tail.
    for i in range(100):
        client._engine.ingest(HumanMessage(content=f"filler turn #{i} " * 20))

    result = client._engine.assemble()

    # Assertion 1: effective-budget ceiling
    effective_cap = client._engine.budget.effective_budget_for_messages(len(result.messages))
    assert result.total_tokens <= effective_cap, (
        f"selector exceeded effective cap: total_tokens={result.total_tokens}, "
        f"effective_cap={effective_cap}, n_messages={len(result.messages)}"
    )

    # Assertion 2: last 3 EVIDENCE pages are rendered at FULL
    evidence_pages = [p for p in client._engine.pages() if p.type is PageType.EVIDENCE]
    # ``turn_seq`` descending → newest first. Ties are broken by ingest
    # order, but ``ingest_message`` assigns a fresh turn_seq to each
    # ingest call so there are no ties here.
    evidence_by_recency = sorted(evidence_pages, key=lambda p: p.turn_seq, reverse=True)
    top_three = evidence_by_recency[:3]
    assert len(top_three) == 3, f"expected at least 3 EVIDENCE pages, got {len(top_three)}"
    for page in top_three:
        assert page.id in result.chosen_levels, (
            f"recent EVIDENCE page {page.id} (turn_seq={page.turn_seq}) "
            f"was evicted entirely; pin policy broken"
        )
        assert result.chosen_levels[page.id] == FidelityLevel.FULL, (
            f"recent EVIDENCE page {page.id} (turn_seq={page.turn_seq}) "
            f"rendered at {result.chosen_levels[page.id]!r}, expected FULL"
        )

    # Assertion 3: at least one PAGE_DEGRADED fault was emitted
    degrade_events = [ev for ev in fake_faults_out.published if ev.kind is FaultKind.PAGE_DEGRADED]
    assert degrade_events, (
        "no PAGE_DEGRADED faults emitted; either the budget was not "
        "actually under pressure (test fixture drifted) or the engine "
        "is silently degrading without telling the fault stream"
    )

    # Assertion 4: no physical insufficiency
    # 3 pinned images × ~1105 tokens + at least some text easily fits
    # in 8000 tokens after per-message overhead. If this flips True,
    # either the pin policy over-counted pages (the auto-pin set is
    # larger than it should be) or the effective-budget math regressed.
    assert result.physical_insufficient is False, (
        f"unexpectedly flipped physical_insufficient=True on a budget "
        f"that should comfortably fit 3 pinned images plus degraded "
        f"text; published faults: {[ev.kind for ev in fake_faults_out.published]}"
    )


# Test 2 — regression guard for the motivating bug
def test_thread_loop_recovers_from_process_message_exception() -> None:
    """Regression guard for the original motivating bug.

    Original bug: a turn-level exception in ``_process_message`` killed
    the ``McpClient`` worker thread. ``agent_idle`` stayed at ``False``
    forever and every subsequent enqueued message sat in the queue
    unprocessed.

    The fix wraps the call in a ``try/except`` that catches the
    exception, routes it through
    ``self._engine.emit_physical_insufficiency(exception=exc)``, logs
    it, and continues draining. This test drives three messages through
    the worker, makes the second one raise, and proves:

    * All three messages are processed (the thread survived).
    * The fault stream received a ``PHYSICAL_INSUFFICIENCY`` event
      carrying the exception in ``details["exception"]``.
    * ``agent_idle`` is restored to ``True`` after the final message.
    * The thread exits cleanly via ``_stop_event``.
    """
    fake_faults_out = _FakeOut()
    fake_idle = _FakeAgentIdleStream()
    client = _build_bypass_client(
        faults_out=fake_faults_out,
        agent_idle=fake_idle,
        state_graph=MagicMock(),  # passes the ``if not self._state_graph`` guard
    )

    # Monkeypatch ``_process_message`` so the second call raises. Mirrors
    # the real method's externally-observable contract (publishes
    # idle=False on entry; idle=True on clean exit when the queue is
    # empty) without touching any LangGraph state machine.
    call_count = {"n": 0}

    def fake_process(state_graph: object, message: object) -> None:
        call_count["n"] += 1
        client.agent_idle.publish(False)
        if call_count["n"] == 2:
            raise RuntimeError("simulated turn-level failure")
        if client._message_queue.empty():
            client.agent_idle.publish(True)

    client._process_message = fake_process  # type: ignore[assignment]

    client._thread = Thread(target=client._thread_loop, daemon=True)
    try:
        client._thread.start()

        for i in (1, 2, 3):
            client._message_queue.put(HumanMessage(content=f"turn {i}"))

        # Poll the queue until it drains, with a hard 5-second cap. The
        # 5-second ceiling is the exact condition that pathologically
        # triggered the original bug: if the thread dies mid-queue, the
        # queue NEVER drains and the test must fail here (not silently
        # time out on the whole test suite).
        deadline = time.monotonic() + 5.0
        while not client._message_queue.empty():
            if time.monotonic() >= deadline:
                pytest.fail(
                    "thread hung after 5s; queue never drained — the "
                    "try/except wrapper in _thread_loop regressed and "
                    "the original motivating bug is back"
                )
            time.sleep(0.05)

        # Give the worker a beat to finish its last iteration: after the
        # final ``get()`` returns a message, the loop body runs
        # ``_process_message`` (which publishes idle=True when the queue
        # is empty) before blocking on the next ``get(timeout=0.5)``.
        # 0.1s is an order of magnitude tighter than the get() timeout
        # while being generous to the scheduler.
        time.sleep(0.1)
    finally:
        # Unconditional teardown — even if an assertion below fails we
        # must stop the daemon thread so subsequent pytest test
        # sessions don't accumulate a pile of zombie workers.
        client._stop_event.set()
        client._thread.join(timeout=2.0)

    # Assertion 1: all three messages processed
    assert call_count["n"] == 3, (
        f"expected 3 processed turns, got {call_count['n']}. "
        "If this is 2, the thread died on the simulated exception "
        "and the queue stalled — exactly the original bug."
    )

    # Assertion 2: PHYSICAL_INSUFFICIENCY fault was emitted
    assert any(ev.kind is FaultKind.PHYSICAL_INSUFFICIENCY for ev in fake_faults_out.published), (
        "no PHYSICAL_INSUFFICIENCY fault observed; "
        "_thread_loop's except-branch did not call emit_physical_insufficiency"
    )

    # Assertion 3: idle restored to True after the final turn
    # Explicit ``is True`` — the stream takes bool, not truthy values;
    # a regression that publishes ``1`` or ``"True"`` would be bad and
    # we want the test to catch it.
    assert fake_idle.idle_history, "agent_idle never received any publish"
    assert fake_idle.idle_history[-1] is True, (
        f"agent_idle did not return to True; history tail: {fake_idle.idle_history[-5:]!r}"
    )

    # Assertion 4: thread exited cleanly
    assert client._thread.is_alive() is False, (
        "thread still alive after stop_event set + join(timeout=2.0); "
        "worker loop is hung on something other than the message queue"
    )

    # Assertion 5: exception forwarded through the fault stream
    # ``MemoryEngine.emit_physical_insufficiency`` stringifies the
    # exception via ``repr()`` and stores it in ``details["exception"]``.
    # Filter the PHYSICAL_INSUFFICIENCY events down to those carrying
    # an exception payload — this test configuration can only emit them
    # via the ``_thread_loop`` except branch, but being explicit keeps
    # the assertion robust if a future change adds another
    # physical-insufficiency emission path.
    phys_events_with_exc = [
        ev
        for ev in fake_faults_out.published
        if ev.kind is FaultKind.PHYSICAL_INSUFFICIENCY and "exception" in ev.details
    ]
    assert phys_events_with_exc, (
        "no PHYSICAL_INSUFFICIENCY fault carried an 'exception' key; "
        "emit_physical_insufficiency(exception=exc) contract regressed"
    )
    exc_repr = phys_events_with_exc[0].details["exception"]
    assert "simulated turn-level failure" in exc_repr, (
        f"exception text did not propagate into fault details; got: "
        f"{exc_repr!r}. ``emit_physical_insufficiency`` must forward "
        f"the exception into ``details['exception']``."
    )
