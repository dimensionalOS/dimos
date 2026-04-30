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
"""Tests for :mod:`dimos.agents.memory.pages`."""
from __future__ import annotations

from langchain_core.messages import AIMessage, HumanMessage, SystemMessage, ToolMessage
import pytest

from dimos.agents.memory.pages import (
    FidelityLevel,
    Page,
    PageType,
    Representation,
)


def _reps(
    *,
    pointer_text: str = "[p1]",
    structured_text: str = "{role:ai}",
    compressed_text: str = "short",
    full_text: str = "the full thing",
    tokens: tuple[int, int, int, int] = (1, 3, 5, 10),
) -> dict[FidelityLevel, Representation]:
    """Helper to build a standard 4-level representation set."""
    return {
        FidelityLevel.POINTER: Representation(FidelityLevel.POINTER, pointer_text, tokens[0]),
        FidelityLevel.STRUCTURED: Representation(FidelityLevel.STRUCTURED, structured_text, tokens[1]),
        FidelityLevel.COMPRESSED: Representation(FidelityLevel.COMPRESSED, compressed_text, tokens[2]),
        FidelityLevel.FULL: Representation(FidelityLevel.FULL, full_text, tokens[3]),
    }


def _page(**overrides: object) -> Page:
    defaults: dict[str, object] = {
        "id": "page-1",
        "type": PageType.CONVERSATION,
        "provenance": "user_input",
        "turn_seq": 0,
        "ts": 1.0,
        "role": "human",
        "representations": _reps(),
    }
    defaults.update(overrides)
    return Page(**defaults)  # type: ignore[arg-type]


# -- FidelityLevel ordering -----------------------------------------------


def test_fidelity_level_ordered_int_enum() -> None:
    assert FidelityLevel.POINTER < FidelityLevel.STRUCTURED < FidelityLevel.COMPRESSED < FidelityLevel.FULL
    assert int(FidelityLevel.POINTER) == 0
    assert int(FidelityLevel.FULL) == 3


def test_fidelity_level_min_and_max() -> None:
    levels = [FidelityLevel.POINTER, FidelityLevel.COMPRESSED, FidelityLevel.STRUCTURED]
    assert min(levels) == FidelityLevel.POINTER
    assert max(levels) == FidelityLevel.COMPRESSED


# -- Representation invariants --------------------------------------------


def test_representation_rejects_negative_tokens() -> None:
    with pytest.raises(ValueError, match="non-negative"):
        Representation(FidelityLevel.FULL, "hello", -1)


def test_representation_rejects_empty_text() -> None:
    with pytest.raises(ValueError, match="non-empty"):
        Representation(FidelityLevel.FULL, "", 0)


def test_representation_rejects_empty_multimodal() -> None:
    with pytest.raises(ValueError, match="multimodal"):
        Representation(FidelityLevel.FULL, [], 0)


def test_representation_rejects_wrong_content_type() -> None:
    with pytest.raises(TypeError):
        Representation(FidelityLevel.FULL, 42, 1)  # type: ignore[arg-type]


def test_representation_accepts_multimodal_parts() -> None:
    rep = Representation(
        FidelityLevel.FULL,
        [{"type": "image_url", "image_url": {"url": "data:image/jpeg;base64,..."}}],
        1105,
    )
    assert isinstance(rep.content, list)


# -- Page invariants ------------------------------------------------------


def test_page_requires_pointer_structured_and_compressed_or_full() -> None:
    reps = _reps()
    del reps[FidelityLevel.POINTER]
    with pytest.raises(ValueError, match="POINTER"):
        _page(representations=reps)

    reps = _reps()
    del reps[FidelityLevel.STRUCTURED]
    with pytest.raises(ValueError, match="STRUCTURED"):
        _page(representations=reps)

    reps = _reps()
    del reps[FidelityLevel.FULL]
    del reps[FidelityLevel.COMPRESSED]
    with pytest.raises(ValueError, match="COMPRESSED/FULL"):
        _page(representations=reps)


def test_page_representation_level_must_match_key() -> None:
    reps = _reps()
    # Corrupt the POINTER entry with a STRUCTURED-level value.
    reps[FidelityLevel.POINTER] = Representation(FidelityLevel.STRUCTURED, "x", 1)
    with pytest.raises(ValueError, match="they must match"):
        _page(representations=reps)


def test_page_token_estimate_must_be_monotonic() -> None:
    with pytest.raises(ValueError, match="monotonic"):
        _page(representations=_reps(tokens=(5, 4, 3, 2)))


def test_page_negative_turn_seq_rejected() -> None:
    with pytest.raises(ValueError, match="turn_seq"):
        _page(turn_seq=-1)


def test_page_empty_id_rejected() -> None:
    with pytest.raises(ValueError, match="id"):
        _page(id="")


def test_page_rejects_pinned_at_full_without_pinned() -> None:
    """Revert of silent auto-mutation: caller must set pinned=True explicitly."""
    with pytest.raises(ValueError, match="pinned=True"):
        _page(pinned_at_full=True, pinned=False, min_fidelity=FidelityLevel.FULL)


def test_page_rejects_pinned_at_full_without_min_full() -> None:
    """Revert of silent auto-mutation: caller must set min_fidelity=FULL explicitly."""
    with pytest.raises(ValueError, match="min_fidelity=FidelityLevel.FULL"):
        _page(pinned_at_full=True, pinned=True, min_fidelity=FidelityLevel.COMPRESSED)


def test_page_accepts_pinned_at_full_with_consistent_flags() -> None:
    """Consistent flag-triple is accepted; pinned_at_full implies pinned=True and min_fidelity=FULL."""
    p = _page(pinned_at_full=True, pinned=True, min_fidelity=FidelityLevel.FULL)
    assert p.pinned is True
    assert p.pinned_at_full is True
    assert p.min_fidelity is FidelityLevel.FULL


def test_page_tool_role_requires_tool_call_id() -> None:
    with pytest.raises(ValueError, match="tool_call_id"):
        _page(role="tool")


def test_page_tool_role_with_id_accepted() -> None:
    p = _page(role="tool", tool_call_id="call_abc")
    assert p.tool_call_id == "call_abc"


def test_page_two_level_only_compressed_ok() -> None:
    # A text page may legitimately have the same content at COMPRESSED and
    # FULL (e.g. a short message); leaving out FULL entirely is allowed as
    # long as COMPRESSED is present.
    reps = _reps()
    del reps[FidelityLevel.FULL]
    p = _page(representations=reps)
    assert p.max_available() == FidelityLevel.COMPRESSED


# -- Page.rep_at fallback -------------------------------------------------


def test_rep_at_returns_exact_level_when_present() -> None:
    p = _page()
    assert p.rep_at(FidelityLevel.STRUCTURED).level == FidelityLevel.STRUCTURED


def test_rep_at_falls_back_to_highest_available_below() -> None:
    reps = _reps()
    del reps[FidelityLevel.COMPRESSED]
    p = _page(representations=reps)
    # Asking for COMPRESSED should fall back to STRUCTURED (next below).
    assert p.rep_at(FidelityLevel.COMPRESSED).level == FidelityLevel.STRUCTURED


def test_rep_at_falls_back_to_lowest_when_none_below() -> None:
    reps = _reps()
    # Only FULL is present — asking for POINTER/STRUCTURED should fall up.
    reps_full_only: dict[FidelityLevel, Representation] = {
        FidelityLevel.POINTER: reps[FidelityLevel.POINTER],
        FidelityLevel.STRUCTURED: reps[FidelityLevel.STRUCTURED],
        FidelityLevel.FULL: reps[FidelityLevel.FULL],
    }
    p = _page(representations=reps_full_only)
    assert p.rep_at(FidelityLevel.COMPRESSED).level == FidelityLevel.STRUCTURED


# -- Page.as_message preserves role + tool_call_id -----------------------


def test_as_message_system_role() -> None:
    p = _page(role="system", type=PageType.BOOTSTRAP)
    msg = p.as_message(FidelityLevel.FULL)
    assert isinstance(msg, SystemMessage)
    assert msg.content == "the full thing"


def test_as_message_human_role() -> None:
    p = _page(role="human")
    msg = p.as_message(FidelityLevel.COMPRESSED)
    assert isinstance(msg, HumanMessage)
    assert msg.content == "short"


def test_as_message_ai_role_without_tool_calls() -> None:
    p = _page(role="ai")
    msg = p.as_message(FidelityLevel.FULL)
    assert isinstance(msg, AIMessage)
    assert msg.content == "the full thing"
    assert not msg.tool_calls


def test_as_message_ai_role_with_tool_calls_roundtrip() -> None:
    tool_calls = [{"id": "c1", "name": "foo", "args": {"x": 1}, "type": "tool_call"}]
    p = _page(role="ai", ai_tool_calls=tool_calls)
    msg = p.as_message(FidelityLevel.FULL)
    assert isinstance(msg, AIMessage)
    assert len(msg.tool_calls) == 1
    assert msg.tool_calls[0]["id"] == "c1"


def test_as_message_tool_role_preserves_tool_call_id() -> None:
    p = _page(role="tool", tool_call_id="call_xyz")
    msg = p.as_message(FidelityLevel.STRUCTURED)
    assert isinstance(msg, ToolMessage)
    assert msg.tool_call_id == "call_xyz"


def test_max_available() -> None:
    reps = _reps()
    del reps[FidelityLevel.FULL]
    assert _page(representations=reps).max_available() == FidelityLevel.COMPRESSED
    assert _page().max_available() == FidelityLevel.FULL


# -- Restored step-2 validations -----------------------------------------
#
# These tests guard the invariants the ingestion / selector pipelines
# assume. A silent regression here tends to surface later as "why is the
# LLM hallucinating a tool-call pair?" or "why did an unknown enum sneak
# through?" so they're cheap insurance.


def test_representation_rejects_non_fidelity_level() -> None:
    with pytest.raises(TypeError, match="FidelityLevel"):
        Representation(level=1, content="x", token_estimate=1)  # type: ignore[arg-type]


def test_page_rejects_non_page_type_type() -> None:
    with pytest.raises(TypeError, match="PageType"):
        _page(type="conversation")  # type: ignore[arg-type]


def test_page_rejects_non_fidelity_min_fidelity() -> None:
    with pytest.raises(TypeError, match="FidelityLevel"):
        _page(min_fidelity=0)  # type: ignore[arg-type]


def test_page_rejects_unknown_role() -> None:
    with pytest.raises(ValueError, match="role"):
        _page(role="assistant")  # type: ignore[arg-type]


def test_tool_call_id_rejected_on_non_tool_role() -> None:
    # Bidirectional invariant: only role='tool' pages may carry a
    # tool_call_id. Setting it on human/ai/system is a caller bug and
    # would silently misroute LangChain pairing at egress.
    with pytest.raises(ValueError, match="tool_call_id"):
        _page(role="human", tool_call_id="call_bogus")
    with pytest.raises(ValueError, match="tool_call_id"):
        _page(role="ai", tool_call_id="call_bogus")
    with pytest.raises(ValueError, match="tool_call_id"):
        _page(role="system", type=PageType.BOOTSTRAP, tool_call_id="call_bogus")


def test_rep_at_never_raises_given_invariants() -> None:
    # POINTER is required by __post_init__, so for *any* requested level
    # there is always at least one rep <= it (POINTER == 0). This test
    # asserts the lenient contract: rep_at must never raise on a
    # well-formed Page, regardless of which rung is asked for or which
    # subset of non-POINTER rungs happens to be present.
    base = _reps()

    subsets: list[dict[FidelityLevel, Representation]] = [
        base,  # all four
        {k: v for k, v in base.items() if k != FidelityLevel.COMPRESSED},
        {k: v for k, v in base.items() if k != FidelityLevel.FULL},
        # Only POINTER + STRUCTURED + (one of COMPRESSED/FULL).
        {
            FidelityLevel.POINTER: base[FidelityLevel.POINTER],
            FidelityLevel.STRUCTURED: base[FidelityLevel.STRUCTURED],
            FidelityLevel.COMPRESSED: base[FidelityLevel.COMPRESSED],
        },
        {
            FidelityLevel.POINTER: base[FidelityLevel.POINTER],
            FidelityLevel.STRUCTURED: base[FidelityLevel.STRUCTURED],
            FidelityLevel.FULL: base[FidelityLevel.FULL],
        },
    ]

    for reps in subsets:
        p = _page(representations=reps)
        for level in FidelityLevel:
            # Must not raise, and must return a Representation whose
            # level is actually present in the page.
            rep = p.rep_at(level)
            assert rep.level in reps
