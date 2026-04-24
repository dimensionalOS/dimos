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
"""Tests for :mod:`dimos.agents.memory.faults`."""
from __future__ import annotations

import pytest

from dimos.agents.memory.faults import FaultEvent, FaultKind, FaultObserver


class _FakeOut:
    """Minimal ``Out``-like stream for tests; records what was published."""

    def __init__(self) -> None:
        self.published: list[FaultEvent] = []

    def publish(self, ev: FaultEvent) -> None:
        self.published.append(ev)


# --- FaultEvent dataclass ------------------------------------------------


def test_fault_event_required_fields() -> None:
    ev = FaultEvent(kind=FaultKind.PAGE_EVICTED, page_id="p1", turn_seq=3)
    assert ev.kind == FaultKind.PAGE_EVICTED
    assert ev.page_id == "p1"
    assert ev.turn_seq == 3
    assert ev.ts > 0
    assert ev.details == {}


def test_fault_event_is_frozen() -> None:
    ev = FaultEvent(kind=FaultKind.PAGE_EVICTED, page_id="p1", turn_seq=0)
    with pytest.raises(Exception):
        ev.page_id = "p2"  # type: ignore[misc]


def test_fault_event_to_dict_stable_schema() -> None:
    ev = FaultEvent(
        kind=FaultKind.PAGE_DEGRADED,
        page_id="p7",
        turn_seq=12,
        ts=42.0,
        details={"from": "FULL", "to": "COMPRESSED"},
    )
    d = ev.to_dict()
    assert d == {
        "kind": "page_degraded",
        "page_id": "p7",
        "turn_seq": 12,
        "ts": 42.0,
        "details": {"from": "FULL", "to": "COMPRESSED"},
    }


def test_fault_event_details_copied_on_to_dict() -> None:
    d_orig = {"x": 1}
    ev = FaultEvent(kind=FaultKind.PAGE_EVICTED, page_id="p", turn_seq=0, details=d_orig)
    out = ev.to_dict()
    out["details"]["x"] = 999  # must not leak back into the dataclass
    assert ev.details["x"] == 1


# --- FaultObserver -------------------------------------------------------


def test_observer_records_last_event_and_counts() -> None:
    obs = FaultObserver()
    assert obs.last_event is None
    ev = FaultEvent(kind=FaultKind.PAGE_EVICTED, page_id="p1", turn_seq=0)
    obs.emit(ev)
    assert obs.last_event is ev
    assert obs.counts[FaultKind.PAGE_EVICTED] == 1
    assert obs.counts[FaultKind.PAGE_DEGRADED] == 0


def test_observer_publishes_to_stream() -> None:
    out = _FakeOut()
    obs = FaultObserver(stream_out=out)  # type: ignore[arg-type]
    ev = FaultEvent(kind=FaultKind.PAGE_EVICTED, page_id="p1", turn_seq=0)
    obs.emit(ev)
    assert out.published == [ev]


def test_observer_swallows_stream_errors() -> None:
    class ExplodingOut:
        def publish(self, _ev: FaultEvent) -> None:
            raise RuntimeError("boom")

    obs = FaultObserver(stream_out=ExplodingOut())  # type: ignore[arg-type]
    ev = FaultEvent(kind=FaultKind.PAGE_EVICTED, page_id="p1", turn_seq=0)
    obs.emit(ev)  # must not raise
    assert obs.last_event is ev


# --- convenience emitters ----------------------------------------------


def test_observer_evict_helper() -> None:
    out = _FakeOut()
    obs = FaultObserver(stream_out=out)  # type: ignore[arg-type]
    obs.evict("p3", 5, reason="over_budget")
    assert len(out.published) == 1
    ev = out.published[0]
    assert ev.kind == FaultKind.PAGE_EVICTED
    assert ev.page_id == "p3"
    assert ev.turn_seq == 5
    assert ev.details == {"reason": "over_budget"}


def test_observer_degrade_helper_records_level_transition() -> None:
    out = _FakeOut()
    obs = FaultObserver(stream_out=out)  # type: ignore[arg-type]
    obs.degrade("p1", 1, from_level="FULL", to_level="COMPRESSED")
    ev = out.published[0]
    assert ev.kind == FaultKind.PAGE_DEGRADED
    assert ev.details == {"from": "FULL", "to": "COMPRESSED"}


def test_observer_refetch_helper_records_uuid() -> None:
    out = _FakeOut()
    obs = FaultObserver(stream_out=out)  # type: ignore[arg-type]
    obs.refetch("p2", 4, artefact_uuid="abc-123")
    ev = out.published[0]
    assert ev.kind == FaultKind.REFETCH_FAULT
    assert ev.details == {"artefact_uuid": "abc-123"}


def test_observer_physical_insufficiency_helper() -> None:
    out = _FakeOut()
    obs = FaultObserver(stream_out=out)  # type: ignore[arg-type]
    obs.physical_insufficiency(7, needed=9000, available=8000, exception="ctx_overflow")
    ev = out.published[0]
    assert ev.kind == FaultKind.PHYSICAL_INSUFFICIENCY
    assert ev.page_id is None
    assert ev.details == {
        "needed": 9000,
        "available": 8000,
        "exception": "ctx_overflow",
    }


def test_observer_pin_rebalance_helper() -> None:
    out = _FakeOut()
    obs = FaultObserver(stream_out=out)  # type: ignore[arg-type]
    obs.pin_rebalance(2, pinned_page_ids=["p4", "p5", "p6"], unpinned_page_ids=["p1"])
    ev = out.published[0]
    assert ev.kind == FaultKind.PIN_REBALANCE
    assert ev.details == {
        "pinned": ["p4", "p5", "p6"],
        "unpinned": ["p1"],
    }


def test_no_stream_observer_still_counts() -> None:
    obs = FaultObserver()
    obs.evict("p", 0, reason="r")
    obs.evict("p2", 0, reason="r2")
    assert obs.counts[FaultKind.PAGE_EVICTED] == 2
