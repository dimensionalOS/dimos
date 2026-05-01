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
"""Tests for :mod:`dimos.agents.memory.page_table`."""
from __future__ import annotations

import pytest

from dimos.agents.memory.page_table import PageTable, PinRebalance
from dimos.agents.memory.pages import FidelityLevel, Page, PageType, Representation


def _reps(
    *,
    tokens: tuple[int, int, int, int] = (1, 3, 5, 10),
) -> dict[FidelityLevel, Representation]:
    return {
        FidelityLevel.POINTER: Representation(FidelityLevel.POINTER, "[p]", tokens[0]),
        FidelityLevel.STRUCTURED: Representation(FidelityLevel.STRUCTURED, "{}", tokens[1]),
        FidelityLevel.COMPRESSED: Representation(FidelityLevel.COMPRESSED, "short", tokens[2]),
        FidelityLevel.FULL: Representation(FidelityLevel.FULL, "the full thing", tokens[3]),
    }


def _page(
    page_id: str,
    page_type: PageType = PageType.CONVERSATION,
    artefact_uuid: str | None = None,
    role: str = "human",
    turn_seq: int = 0,
) -> Page:
    extra: dict[str, object] = {}
    if role == "tool":
        extra["tool_call_id"] = f"call-{page_id}"
    return Page(
        id=page_id,
        type=page_type,
        provenance="test",
        turn_seq=turn_seq,
        ts=1.0 + turn_seq,
        role=role,  # type: ignore[arg-type]
        representations=_reps(),
        artefact_uuid=artefact_uuid,
        **extra,  # type: ignore[arg-type]
    )


# --- basic store ------------------------------------------------------


def test_empty_table_has_len_zero() -> None:
    t = PageTable()
    assert len(t) == 0
    assert t.ordered() == []
    assert t.get("missing") is None
    assert t.get_by_artefact("nope") is None


def test_add_and_lookup_by_id() -> None:
    t = PageTable()
    p = _page("p1")
    t.add(p)
    assert t.get("p1") is p
    assert len(t) == 1
    assert t.ordered() == [p]


def test_add_and_lookup_by_artefact() -> None:
    t = PageTable()
    p = _page("p1", page_type=PageType.EVIDENCE, artefact_uuid="aa-bb")
    t.add(p)
    assert t.get_by_artefact("aa-bb") is p


def test_add_rejects_duplicate_id() -> None:
    t = PageTable()
    t.add(_page("p1"))
    with pytest.raises(ValueError, match="already contains a page"):
        t.add(_page("p1"))


def test_add_rejects_duplicate_artefact_uuid() -> None:
    t = PageTable()
    t.add(_page("p1", page_type=PageType.EVIDENCE, artefact_uuid="u1"))
    with pytest.raises(ValueError, match="artefact UUID"):
        t.add(_page("p2", page_type=PageType.EVIDENCE, artefact_uuid="u1"))


def test_ordered_returns_copy() -> None:
    t = PageTable()
    p = _page("p1")
    t.add(p)
    snapshot = t.ordered()
    snapshot.append(_page("p2"))  # mutate snapshot
    assert len(t) == 1


def test_clear_resets_all_indexes() -> None:
    t = PageTable()
    t.add(_page("p1", page_type=PageType.EVIDENCE, artefact_uuid="u1"))
    t.add(_page("p2"))
    t.clear()
    assert len(t) == 0
    assert t.get("p1") is None
    assert t.get_by_artefact("u1") is None


def test_negative_pin_rejected() -> None:
    with pytest.raises(ValueError):
        PageTable(pin_recent_evidence=-1)


def test_evidence_pages_filter() -> None:
    t = PageTable()
    t.add(_page("c1"))
    t.add(_page("e1", page_type=PageType.EVIDENCE, artefact_uuid="u1"))
    t.add(_page("c2"))
    ev = t.evidence_pages()
    assert [p.id for p in ev] == ["e1"]


# --- pin rebalancing ---------------------------------------------------


def test_rebalance_pins_last_n_evidence() -> None:
    t = PageTable(pin_recent_evidence=3)
    for i in range(5):
        t.add(_page(f"e{i}", page_type=PageType.EVIDENCE, artefact_uuid=f"u{i}", turn_seq=i))

    diff = t.rebalance_evidence_pins()
    # Last 3 (e2, e3, e4) must be pinned_at_full.
    assert {p.id for p in t.evidence_pages() if p.pinned_at_full} == {"e2", "e3", "e4"}
    # None of the earlier ones must be pinned.
    assert not any(p.pinned_at_full for p in t.evidence_pages() if p.id in {"e0", "e1"})

    # First call surfaces 3 new pins, 0 unpins.
    assert set(diff.pinned) == {"e2", "e3", "e4"}
    assert diff.unpinned == []
    assert diff.empty is False


def test_rebalance_is_idempotent() -> None:
    t = PageTable(pin_recent_evidence=2)
    for i in range(3):
        t.add(_page(f"e{i}", page_type=PageType.EVIDENCE, artefact_uuid=f"u{i}", turn_seq=i))
    t.rebalance_evidence_pins()  # first call flips things
    diff = t.rebalance_evidence_pins()
    assert diff.empty
    assert diff.pinned == []
    assert diff.unpinned == []


def test_rebalance_unpins_displaced_older_pages() -> None:
    t = PageTable(pin_recent_evidence=2)
    # Add 3 EVIDENCE pages, with the oldest one expected to drop out of the
    # pin window once the 3rd arrives.
    for i in range(3):
        t.add(_page(f"e{i}", page_type=PageType.EVIDENCE, artefact_uuid=f"u{i}", turn_seq=i))

    diff = t.rebalance_evidence_pins()
    assert set(diff.pinned) == {"e1", "e2"}
    assert diff.unpinned == []  # e0 was never pinned in the first place

    # Now add a 4th; e1 should fall out of the window, e3 should come in.
    t.add(_page("e3", page_type=PageType.EVIDENCE, artefact_uuid="u3", turn_seq=3))
    diff2 = t.rebalance_evidence_pins()
    assert set(diff2.pinned) == {"e3"}
    assert set(diff2.unpinned) == {"e1"}
    pinned_now = {p.id for p in t.evidence_pages() if p.pinned_at_full}
    assert pinned_now == {"e2", "e3"}


def test_rebalance_zero_pins_never_pins_anything() -> None:
    t = PageTable(pin_recent_evidence=0)
    for i in range(3):
        t.add(_page(f"e{i}", page_type=PageType.EVIDENCE, artefact_uuid=f"u{i}", turn_seq=i))
    diff = t.rebalance_evidence_pins()
    assert diff.empty
    assert not any(p.pinned_at_full for p in t.evidence_pages())


def test_rebalance_preserves_manual_pins() -> None:
    # REFETCH_FAULT marks a page pinned via ``mark_pinned_full`` and we
    # must not un-pin it via a subsequent rebalance (one-way upgrade).
    t = PageTable(pin_recent_evidence=1)
    for i in range(3):
        t.add(_page(f"e{i}", page_type=PageType.EVIDENCE, artefact_uuid=f"u{i}", turn_seq=i))
    t.rebalance_evidence_pins()
    # Manually promote the oldest page.
    assert t.mark_pinned_full("e0") is True

    diff = t.rebalance_evidence_pins()
    # e0 stays pinned_at_full because we just promoted it.
    assert diff.empty or "e0" not in diff.unpinned
    e0 = t.get("e0")
    assert e0 is not None and e0.pinned_at_full is True


def test_mark_pinned_full_returns_false_for_missing() -> None:
    t = PageTable()
    assert t.mark_pinned_full("does-not-exist") is False


def test_pin_rebalance_empty_property() -> None:
    assert PinRebalance(pinned=[], unpinned=[]).empty is True
    assert PinRebalance(pinned=["x"], unpinned=[]).empty is False
    assert PinRebalance(pinned=[], unpinned=["y"]).empty is False
