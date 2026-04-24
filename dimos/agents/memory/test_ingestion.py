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
"""Tests for :mod:`dimos.agents.memory.ingestion`."""
from __future__ import annotations

import base64
import json
import time

import cv2
import numpy as np
import pytest
from langchain_core.messages import AIMessage, HumanMessage, SystemMessage, ToolMessage

from dimos.agents.memory.ingestion import (
    THUMBNAIL_JPEG_QUALITY,
    THUMBNAIL_SIZE,
    _build_text_representations,
    ingest_message,
)
from dimos.agents.memory.pages import FidelityLevel, PageType
from dimos.agents.memory.tokens import HeuristicCounter


def _make_jpeg_data_url(size: int = 256, color: int = 128) -> str:
    img = np.full((size, size, 3), color, dtype=np.uint8)
    ok, buf = cv2.imencode(".jpg", img, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
    assert ok
    b64 = base64.b64encode(buf.tobytes()).decode("ascii")
    return f"data:image/jpeg;base64,{b64}"


# --- text messages ----------------------------------------------------
#
# All simple (non-multimodal) messages must produce exactly one Page.
# The ``len(pages) == 1`` assertion is mandatory in every single-page
# test so any regression that accidentally over-produces is caught
# immediately.


def test_ingest_human_text_creates_conversation_page_with_four_reps() -> None:
    msg = HumanMessage(content="Move forward two meters and stop.")
    pages = ingest_message(
        msg, turn_seq=0, ts=1.0, token_counter=HeuristicCounter()
    )
    assert len(pages) == 1
    page = pages[0]
    assert page.type is PageType.CONVERSATION
    assert page.role == "human"
    assert set(page.representations.keys()) == {
        FidelityLevel.POINTER,
        FidelityLevel.STRUCTURED,
        FidelityLevel.COMPRESSED,
        FidelityLevel.FULL,
    }


def test_ingest_text_token_estimates_are_monotonic() -> None:
    msg = HumanMessage(
        content="This is a multi-sentence message. It has more than one sentence. "
        "And in fact quite a bit more content to compress down. Lots of content. More."
    )
    pages = ingest_message(
        msg, turn_seq=0, ts=1.0, token_counter=HeuristicCounter()
    )
    assert len(pages) == 1
    page = pages[0]
    p, s, c, f = (
        page.representations[FidelityLevel.POINTER].token_estimate,
        page.representations[FidelityLevel.STRUCTURED].token_estimate,
        page.representations[FidelityLevel.COMPRESSED].token_estimate,
        page.representations[FidelityLevel.FULL].token_estimate,
    )
    assert p <= s <= c <= f


def test_ingest_system_message_becomes_bootstrap_and_pinned_full() -> None:
    msg = SystemMessage(content="You are a helpful robot.")
    pages = ingest_message(
        msg, turn_seq=0, ts=1.0, token_counter=HeuristicCounter()
    )
    assert len(pages) == 1
    page = pages[0]
    assert page.type is PageType.BOOTSTRAP
    assert page.role == "system"
    assert page.pinned is True
    assert page.pinned_at_full is True
    assert page.min_fidelity is FidelityLevel.FULL


def test_ingest_ai_message_preserves_tool_calls() -> None:
    msg = AIMessage(
        content="calling tool",
        tool_calls=[
            {"id": "c1", "name": "add", "args": {"x": 1, "y": 2}, "type": "tool_call"}
        ],
    )
    pages = ingest_message(
        msg, turn_seq=1, ts=2.0, token_counter=HeuristicCounter()
    )
    assert len(pages) == 1
    page = pages[0]
    assert page.role == "ai"
    assert page.ai_tool_calls is not None
    assert page.ai_tool_calls[0]["id"] == "c1"
    # Round-trip via as_message should keep the tool call.
    rendered = page.as_message(FidelityLevel.FULL)
    assert isinstance(rendered, AIMessage)
    assert rendered.tool_calls[0]["id"] == "c1"


def test_ingest_tool_message_preserves_tool_call_id() -> None:
    msg = ToolMessage(content="42", tool_call_id="call-abc")
    pages = ingest_message(
        msg, turn_seq=2, ts=3.0, token_counter=HeuristicCounter()
    )
    assert len(pages) == 1
    page = pages[0]
    assert page.role == "tool"
    assert page.tool_call_id == "call-abc"


def test_ingest_empty_text_does_not_crash() -> None:
    msg = HumanMessage(content="")
    pages = ingest_message(
        msg, turn_seq=0, ts=1.0, token_counter=HeuristicCounter()
    )
    assert len(pages) == 1
    page = pages[0]
    # Representations must still be non-empty.
    assert page.representations[FidelityLevel.FULL].content
    assert page.representations[FidelityLevel.COMPRESSED].content


def test_ingest_page_type_hint_overrides_default() -> None:
    msg = HumanMessage(content="A preference statement")
    pages = ingest_message(
        msg,
        turn_seq=0,
        ts=1.0,
        token_counter=HeuristicCounter(),
        page_type_hint=PageType.PREFERENCE,
    )
    assert len(pages) == 1
    assert pages[0].type is PageType.PREFERENCE


def test_ingest_explicit_page_id_honored() -> None:
    # Explicit id is honored on single-page results.
    msg = HumanMessage(content="x")
    pages = ingest_message(
        msg, turn_seq=0, ts=1.0, token_counter=HeuristicCounter(), page_id="fixed-id"
    )
    assert len(pages) == 1
    assert pages[0].id == "fixed-id"


def test_ingest_message_ts_defaults_to_now_when_omitted() -> None:
    """Regression: ts is optional; omission falls back to time.time()."""
    before = time.time()
    pages = ingest_message(
        HumanMessage(content="hello"),
        turn_seq=0,
        token_counter=HeuristicCounter(),
    )
    after = time.time()
    assert len(pages) == 1
    page = pages[0]
    assert before <= page.ts <= after, (
        f"page.ts={page.ts} not in [{before}, {after}]"
    )


# --- multimodal / image messages --------------------------------------


def test_ingest_image_human_message_becomes_evidence_page() -> None:
    # Multimodal ``[text, image]`` splits into [CONVERSATION, EVIDENCE].
    # The EVIDENCE page carries the artefact UUID.
    data_url = _make_jpeg_data_url(size=128)
    msg = HumanMessage(
        content=[
            {"type": "text", "text": "camera snapshot"},
            {"type": "image_url", "image_url": {"url": data_url}},
        ]
    )
    pages = ingest_message(
        msg, turn_seq=3, ts=1.0, token_counter=HeuristicCounter()
    )
    assert len(pages) == 2
    conv, evidence = pages
    assert conv.type is PageType.CONVERSATION
    assert evidence.type is PageType.EVIDENCE
    assert evidence.artefact_uuid is not None


def test_ingest_image_full_representation_is_original_image_part() -> None:
    # Case (e.2): single image, no text → exactly one EVIDENCE page.
    data_url = _make_jpeg_data_url()
    msg = HumanMessage(
        content=[{"type": "image_url", "image_url": {"url": data_url}}]
    )
    pages = ingest_message(
        msg, turn_seq=0, ts=1.0, token_counter=HeuristicCounter()
    )
    assert len(pages) == 1
    page = pages[0]
    full_content = page.representations[FidelityLevel.FULL].content
    assert isinstance(full_content, list)
    # Find the image part in the full rendering.
    img_parts = [p for p in full_content if isinstance(p, dict) and p.get("type") == "image_url"]
    assert len(img_parts) == 1
    assert img_parts[0]["image_url"]["url"] == data_url


def test_ingest_image_compressed_representation_is_96x96_thumbnail() -> None:
    data_url = _make_jpeg_data_url(size=512)
    msg = HumanMessage(
        content=[{"type": "image_url", "image_url": {"url": data_url}}]
    )
    pages = ingest_message(
        msg, turn_seq=0, ts=1.0, token_counter=HeuristicCounter()
    )
    assert len(pages) == 1
    page = pages[0]
    compressed = page.representations[FidelityLevel.COMPRESSED].content
    assert isinstance(compressed, list)
    img_part = next(p for p in compressed if isinstance(p, dict) and p.get("type") == "image_url")
    url = img_part["image_url"]["url"]
    assert url.startswith("data:image/jpeg;base64,")
    # Decode the thumbnail and assert size.
    b64 = url.split("base64,", 1)[1]
    raw = base64.b64decode(b64)
    arr = np.frombuffer(raw, dtype=np.uint8)
    thumb = cv2.imdecode(arr, cv2.IMREAD_COLOR)
    assert thumb is not None
    assert thumb.shape[:2] == (THUMBNAIL_SIZE, THUMBNAIL_SIZE)
    assert img_part["image_url"].get("detail") == "low"
    # Sanity — quality 30 should produce something substantially smaller
    # than the source.
    assert len(raw) < 12 * 1024  # 96x96 q30 is ~2-4 KB in practice


def test_image_structured_representation_is_bracketed_text() -> None:
    # STRUCTURED uses a single-line bracketed artefact format — NOT a
    # JSON object (a JSON rep would double-stringify inside the prompt).
    # The UUID inside the bracket is the ``artefact_uuid`` (resolvable
    # via ``PageTable.get_by_artefact`` / ``MemoryEngine.request_full``),
    # NOT the ``page.id`` (which carries a ``"page-"`` prefix).
    data_url = _make_jpeg_data_url(size=128)
    msg = HumanMessage(
        content=[{"type": "image_url", "image_url": {"url": data_url}}]
    )
    pages = ingest_message(
        msg,
        turn_seq=0,
        ts=1.0,
        token_counter=HeuristicCounter(),
        page_id="fixed-image-id",
    )
    # Single-image no-text case (e.2): explicit page_id is honored.
    assert len(pages) == 1
    page = pages[0]
    structured = page.representations[FidelityLevel.STRUCTURED].content

    # Plain str — never a JSON-serialised dict or a list of parts.
    assert isinstance(structured, str)
    assert not structured.startswith("{"), (
        f"STRUCTURED regressed to JSON: {structured!r}"
    )

    # Exact bracketed shape: single line, starts/ends predictably.
    assert "\n" not in structured
    assert page.artefact_uuid is not None
    assert structured.startswith(f"[image artefact uuid={page.artefact_uuid} ")
    assert structured.endswith("kB]")
    # Bug guard: ``page_id`` (with its ``"page-"`` prefix) must not
    # appear inside the bracket — the bracket UUID must be the
    # ``artefact_uuid`` that ``PageTable._by_artefact`` indexes on.
    assert "fixed-image-id" not in structured

    # Expected fields for a decoded image.
    assert "dims=128x128" in structured
    assert "size=1kB" in structured or " size=" in structured
    # src tag must be space-free so the artefact stays single-tokenable.
    src_segment = structured.split("src=", 1)[1].split(" ", 1)[0]
    assert " " not in src_segment
    # Data URIs get a short mime-prefix tag rather than the base64 payload.
    assert src_segment == "data:image/jpeg"


def test_image_structured_representation_handles_unknown_dims() -> None:
    # Non-data URL we cannot decode (no base64 body) — STRUCTURED must
    # emit ``dims=unknown`` / ``size=unknown`` rather than None or crash.
    msg = HumanMessage(
        content=[
            {
                "type": "image_url",
                "image_url": {"url": "https://example.com/cat.jpg"},
            }
        ]
    )
    pages = ingest_message(
        msg,
        turn_seq=0,
        ts=1.0,
        token_counter=HeuristicCounter(),
        page_id="unknown-dims-id",
    )
    assert len(pages) == 1
    page = pages[0]
    structured = page.representations[FidelityLevel.STRUCTURED].content
    assert isinstance(structured, str)
    # UUID inside the bracket is ``artefact_uuid``, not ``page_id``.
    assert page.artefact_uuid is not None
    assert structured.startswith(f"[image artefact uuid={page.artefact_uuid} ")
    assert structured.endswith("=unknown]")
    assert "unknown-dims-id" not in structured
    assert "dims=unknown" in structured
    assert "size=unknown" in structured
    assert "None" not in structured
    # src tag preserved verbatim for non-data URLs (no spaces in this fixture).
    src_segment = structured.split("src=", 1)[1].split(" ", 1)[0]
    assert src_segment == "https://example.com/cat.jpg"


def test_ingest_image_pointer_contains_artefact_uuid() -> None:
    data_url = _make_jpeg_data_url()
    msg = HumanMessage(
        content=[{"type": "image_url", "image_url": {"url": data_url}}]
    )
    pages = ingest_message(
        msg, turn_seq=0, ts=1.0, token_counter=HeuristicCounter()
    )
    assert len(pages) == 1
    page = pages[0]
    pointer = page.representations[FidelityLevel.POINTER].content
    assert isinstance(pointer, str)
    assert page.artefact_uuid in pointer


def test_ingest_image_structured_uses_artefact_uuid_not_page_id() -> None:
    """The UUID inside the STRUCTURED bracket MUST be
    ``page.artefact_uuid`` — the key ``PageTable._by_artefact`` indexes
    on, which ``MemoryEngine.request_full`` /
    ``PageTable.get_by_artefact`` resolve against. If the bracket ever
    renders ``page.id`` (which carries a ``"page-"`` prefix), an LLM
    that extracts the UUID from a STRUCTURED rep and hands it to
    ``get_artefact`` hits the "not found" path even though the page is
    live in the table.
    """
    data_url = _make_jpeg_data_url()
    msg = HumanMessage(
        content=[{"type": "image_url", "image_url": {"url": data_url}}]
    )
    pages = ingest_message(
        msg, turn_seq=0, ts=1.0, token_counter=HeuristicCounter()
    )
    assert len(pages) == 1
    page = pages[0]

    structured = page.rep_at(FidelityLevel.STRUCTURED).content
    assert isinstance(structured, str)

    assert page.artefact_uuid is not None
    assert page.artefact_uuid in structured, (
        f"STRUCTURED rep must contain page.artefact_uuid "
        f"({page.artefact_uuid!r}); got {structured!r}"
    )
    # ``page.id`` carries the ``"page-"`` prefix. It must not leak into
    # the LLM-visible bracket or rehydration breaks.
    assert page.id not in structured, (
        f"STRUCTURED rep must NOT contain page.id ({page.id!r}) — "
        f"UUID-agreement invariant regressed. Got {structured!r}"
    )


def test_ingest_image_token_estimates_monotonic() -> None:
    # Lock in POINTER <= STRUCTURED <= COMPRESSED <= FULL. The selector
    # relies on this invariant to reason about fidelity cost, so keep
    # the assertion strict.
    data_url = _make_jpeg_data_url()
    msg = HumanMessage(
        content=[{"type": "image_url", "image_url": {"url": data_url}}]
    )
    pages = ingest_message(
        msg, turn_seq=0, ts=1.0, token_counter=HeuristicCounter()
    )
    assert len(pages) == 1
    page = pages[0]
    p, s, c, f = (
        page.representations[FidelityLevel.POINTER].token_estimate,
        page.representations[FidelityLevel.STRUCTURED].token_estimate,
        page.representations[FidelityLevel.COMPRESSED].token_estimate,
        page.representations[FidelityLevel.FULL].token_estimate,
    )
    assert p <= s, f"POINTER ({p}) must be <= STRUCTURED ({s})"
    assert s <= c, f"STRUCTURED ({s}) must be <= COMPRESSED ({c})"
    assert c <= f, f"COMPRESSED ({c}) must be <= FULL ({f})"


def test_ingest_image_token_estimates_monotonic_unknown_dims() -> None:
    # Same monotonicity contract on the non-data-URL fallback path,
    # where STRUCTURED collapses to ``dims=unknown size=unknown``.
    msg = HumanMessage(
        content=[
            {
                "type": "image_url",
                "image_url": {"url": "https://example.com/cat.jpg"},
            }
        ]
    )
    pages = ingest_message(
        msg, turn_seq=0, ts=1.0, token_counter=HeuristicCounter()
    )
    assert len(pages) == 1
    page = pages[0]
    p, s, c, f = (
        page.representations[FidelityLevel.POINTER].token_estimate,
        page.representations[FidelityLevel.STRUCTURED].token_estimate,
        page.representations[FidelityLevel.COMPRESSED].token_estimate,
        page.representations[FidelityLevel.FULL].token_estimate,
    )
    assert p <= s <= c <= f


def test_ingest_image_extracts_existing_uuid_from_preamble() -> None:
    # Matches the McpClient convention: "... tool with UUID:=<uuid>.".
    # This is case (e.4) → [CONV, EVIDENCE]; the EVIDENCE page is the
    # one that carries the artefact uuid.
    data_url = _make_jpeg_data_url()
    preamble = (
        "This is the artefact for the 'camera' tool with "
        "UUID:=12345678-1234-5678-abcd-1234567890ab."
    )
    msg = HumanMessage(
        content=[
            {"type": "text", "text": preamble},
            {"type": "image_url", "image_url": {"url": data_url}},
        ]
    )
    pages = ingest_message(
        msg, turn_seq=0, ts=1.0, token_counter=HeuristicCounter()
    )
    assert len(pages) == 2
    conv, evidence = pages
    assert conv.type is PageType.CONVERSATION
    assert evidence.type is PageType.EVIDENCE
    assert evidence.artefact_uuid == "12345678-1234-5678-abcd-1234567890ab"


def test_ingest_unsupported_message_type_raises() -> None:
    class FakeMessage:
        content = "x"

    with pytest.raises(TypeError):
        ingest_message(
            FakeMessage(),  # type: ignore[arg-type]
            turn_seq=0,
            ts=1.0,
            token_counter=HeuristicCounter(),
        )


def test_thumbnail_defaults_match_plan() -> None:
    # Locked decision: 96x96 JPEG, quality 30.
    assert THUMBNAIL_SIZE == 96
    assert THUMBNAIL_JPEG_QUALITY == 30


# --- multi-image split ------------------------------------------------
#
# Contract: one EVIDENCE page per image part; text splits into a
# CONVERSATION page. These tests lock in the full split matrix so a
# future regression cannot silently drop images from multimodal
# HumanMessages.


def test_ingest_humanmessage_text_plus_one_image_returns_two_pages() -> None:
    data_url = _make_jpeg_data_url(size=128)
    msg = HumanMessage(
        content=[
            {"type": "text", "text": "What do you see?"},
            {"type": "image_url", "image_url": {"url": data_url}},
        ]
    )
    pages = ingest_message(
        msg, turn_seq=7, ts=42.0, token_counter=HeuristicCounter()
    )
    assert len(pages) == 2
    conv, evidence = pages
    assert conv.type is PageType.CONVERSATION
    assert evidence.type is PageType.EVIDENCE

    # CONVERSATION FULL text contains the preamble line and the evidence
    # page's uuid so the LLM can reference it via ``get_artefact``.
    conv_full = conv.representations[FidelityLevel.FULL].content
    assert isinstance(conv_full, str)
    assert "Attached artefacts: [" in conv_full
    assert evidence.artefact_uuid is not None
    assert evidence.artefact_uuid in conv_full

    # The user's original question survives as the first line (before
    # the appended Attached-artefacts line).
    user_line, sep, tail = conv_full.partition("\n\n")
    assert sep == "\n\n"
    assert user_line == "What do you see?"
    assert tail.startswith("Attached artefacts: [")

    # Shared provenance across all pages produced by a single call.
    assert conv.turn_seq == evidence.turn_seq == 7
    assert conv.ts == evidence.ts == 42.0


def test_ingest_humanmessage_text_plus_three_images_returns_four_pages() -> None:
    # Use visually-distinct images so each has a unique base64 payload —
    # that lets us verify positional order via the preserved FULL
    # rendering (which keeps the original image_part dict verbatim).
    url_a = _make_jpeg_data_url(size=128, color=32)
    url_b = _make_jpeg_data_url(size=128, color=96)
    url_c = _make_jpeg_data_url(size=128, color=192)
    msg = HumanMessage(
        content=[
            {"type": "text", "text": "Describe each scene."},
            {"type": "image_url", "image_url": {"url": url_a}},
            {"type": "image_url", "image_url": {"url": url_b}},
            {"type": "image_url", "image_url": {"url": url_c}},
        ]
    )
    pages = ingest_message(
        msg, turn_seq=11, ts=7.0, token_counter=HeuristicCounter()
    )
    assert len(pages) == 4
    assert [p.type for p in pages] == [
        PageType.CONVERSATION,
        PageType.EVIDENCE,
        PageType.EVIDENCE,
        PageType.EVIDENCE,
    ]

    # Order preservation: pages[1+i] wraps the i-th image part in content order.
    expected_urls = [url_a, url_b, url_c]
    for page, expected in zip(pages[1:], expected_urls):
        full = page.representations[FidelityLevel.FULL].content
        assert isinstance(full, list)
        assert full[0]["image_url"]["url"] == expected

    # CONVERSATION preamble lists all three uuids in the same order as
    # the EVIDENCE pages — i.e. content-part order.
    conv_full = pages[0].representations[FidelityLevel.FULL].content
    assert isinstance(conv_full, str)
    attach_line = conv_full.split("\n\n", 1)[1]
    positions = [
        attach_line.find(f"[{pages[i + 1].artefact_uuid}]")
        for i in range(3)
    ]
    assert all(pos >= 0 for pos in positions), (
        f"Evidence uuids missing from preamble: {attach_line!r}"
    )
    assert positions == sorted(positions), (
        f"Attached-artefact order not preserved: {positions} in {attach_line!r}"
    )

    # Shared turn_seq and ts across all four pages.
    for p in pages:
        assert p.turn_seq == 11
        assert p.ts == 7.0


def test_ingest_humanmessage_two_images_no_text_returns_two_evidence_pages() -> None:
    url_a = _make_jpeg_data_url(size=96, color=40)
    url_b = _make_jpeg_data_url(size=96, color=200)
    msg = HumanMessage(
        content=[
            {"type": "image_url", "image_url": {"url": url_a}},
            {"type": "image_url", "image_url": {"url": url_b}},
        ]
    )
    pages = ingest_message(
        msg, turn_seq=5, ts=9.0, token_counter=HeuristicCounter()
    )
    assert len(pages) == 2
    assert all(p.type is PageType.EVIDENCE for p in pages)
    # No CONVERSATION page — case (e.3) has nothing to say.
    assert not any(p.type is PageType.CONVERSATION for p in pages)

    # Order preserved.
    assert pages[0].representations[FidelityLevel.FULL].content[0][
        "image_url"
    ]["url"] == url_a
    assert pages[1].representations[FidelityLevel.FULL].content[0][
        "image_url"
    ]["url"] == url_b

    for p in pages:
        assert p.turn_seq == 5
        assert p.ts == 9.0


def test_ingest_multimodal_message_ignores_page_id_kwarg() -> None:
    # Multi-page result: the explicit ``page_id`` contract only applies
    # to single-page results. We must NOT silently assign it to the
    # first page (that would be a disambiguation trap for callers).
    url_a = _make_jpeg_data_url(size=96, color=40)
    url_b = _make_jpeg_data_url(size=96, color=200)
    msg = HumanMessage(
        content=[
            {"type": "text", "text": "look"},
            {"type": "image_url", "image_url": {"url": url_a}},
            {"type": "image_url", "image_url": {"url": url_b}},
        ]
    )
    pages = ingest_message(
        msg,
        turn_seq=0,
        ts=1.0,
        token_counter=HeuristicCounter(),
        page_id="fixed-id",
    )
    assert len(pages) == 3
    assert all(p.id != "fixed-id" for p in pages), (
        f"page_id='fixed-id' must be ignored in multi-page result; "
        f"got ids={[p.id for p in pages]}"
    )


def test_page_type_hint_ignored_on_multimodal_split() -> None:
    # When a multimodal HumanMessage splits, the hint cannot apply to
    # both the CONVERSATION and the EVIDENCE page coherently, so the
    # split path ignores it entirely.
    url_a = _make_jpeg_data_url(size=96)
    msg = HumanMessage(
        content=[
            {"type": "text", "text": "prefer this"},
            {"type": "image_url", "image_url": {"url": url_a}},
        ]
    )
    pages = ingest_message(
        msg,
        turn_seq=0,
        ts=1.0,
        token_counter=HeuristicCounter(),
        page_type_hint=PageType.PREFERENCE,
    )
    assert len(pages) == 2
    conv, evidence = pages
    assert conv.type is PageType.CONVERSATION, (
        "CONV page must be CONVERSATION regardless of hint in multi-page result"
    )
    assert evidence.type is PageType.EVIDENCE, (
        "EVIDENCE is structural — hint must never override it"
    )


# --- AI tool_calls token accounting -----------------------------------
#
# ``Page.as_message`` reattaches ``tool_calls`` at every fidelity
# level to preserve LangChain AIMessage/ToolMessage pairing, so the
# serialized JSON cost must be added to every representation's
# ``token_estimate``. Adding the same constant to all four rungs
# preserves the POINTER <= STRUCTURED <= COMPRESSED <= FULL invariant
# that the selector relies on.


_ALL_FIDELITY_LEVELS = (
    FidelityLevel.POINTER,
    FidelityLevel.STRUCTURED,
    FidelityLevel.COMPRESSED,
    FidelityLevel.FULL,
)


def test_ingest_ai_message_with_tool_calls_adds_payload_tokens_to_every_rep() -> None:
    counter = HeuristicCounter()
    tool_calls = [
        {
            "id": "call_x",
            "name": "get_weather",
            "args": {"city": "Paris", "days": 3},
            "type": "tool_call",
        }
    ]
    msg = AIMessage(content="thinking…", tool_calls=tool_calls)
    pages = ingest_message(msg, turn_seq=0, ts=1.0, token_counter=counter)
    assert len(pages) == 1
    page = pages[0]
    assert page.ai_tool_calls == tool_calls

    # Twin without tool_calls: same content, so _build_text_representations
    # produces identical content/tokens; the per-rep difference isolates
    # exactly the tool_calls payload cost.
    twin = AIMessage(content="thinking…")
    twin_pages = ingest_message(twin, turn_seq=0, ts=1.0, token_counter=counter)
    assert len(twin_pages) == 1
    twin_page = twin_pages[0]
    assert twin_page.ai_tool_calls is None

    # Recompute the expected payload cost exactly as the ingestor does.
    # The tool_calls list ingest_message sees is a shallow-cloned copy
    # (via _clone_tool_calls) whose dict contents are identical to the
    # input, so ``sort_keys=True`` yields the same serialization and
    # thus the same token count.
    expected_payload_json = json.dumps(
        page.ai_tool_calls, sort_keys=True, default=str
    )
    expected_payload_tokens = counter.count_text(expected_payload_json)
    assert expected_payload_tokens > 0, (
        "fixture chosen so the payload is non-trivial"
    )

    for level in _ALL_FIDELITY_LEVELS:
        with_tc = page.representations[level].token_estimate
        without_tc = twin_page.representations[level].token_estimate
        assert with_tc >= expected_payload_tokens, (
            f"{level.name}: {with_tc} < payload {expected_payload_tokens}"
        )
        assert with_tc - without_tc == expected_payload_tokens, (
            f"{level.name}: overhead delta {with_tc - without_tc} "
            f"!= expected {expected_payload_tokens}"
        )

    # Monotonicity must still hold with the uniform overhead added.
    p, s, c, f = (
        page.representations[FidelityLevel.POINTER].token_estimate,
        page.representations[FidelityLevel.STRUCTURED].token_estimate,
        page.representations[FidelityLevel.COMPRESSED].token_estimate,
        page.representations[FidelityLevel.FULL].token_estimate,
    )
    assert p <= s <= c <= f, (
        f"Monotonicity broken with tool_calls overhead: "
        f"POINTER={p} STRUCTURED={s} COMPRESSED={c} FULL={f}"
    )


def test_ingest_ai_message_without_tool_calls_has_no_tool_call_overhead() -> None:
    # Zero-regression test for the no-tool-calls path: the per-rep
    # token_estimate must match exactly what ``_build_text_representations``
    # produces with no overhead added. This catches a regression where
    # someone accidentally enters the tool_calls accounting branch for
    # an empty/missing payload (e.g. ``json.dumps(None)`` → 1 extra
    # token on every rep).
    counter = HeuristicCounter()
    content = "no tools used here, just prose."
    msg = AIMessage(content=content)
    pages = ingest_message(msg, turn_seq=0, ts=1.0, token_counter=counter)
    assert len(pages) == 1
    page = pages[0]
    assert page.ai_tool_calls is None

    # Reconstruct the expected per-rep token counts from the same helper
    # that ``_build_single_text_page`` calls. Using the produced page's
    # own id keeps POINTER token counts byte-identical to the ingested
    # representation.
    expected_reps = _build_text_representations(
        page.role, content, page.id, counter
    )
    for level in _ALL_FIDELITY_LEVELS:
        assert (
            page.representations[level].token_estimate
            == expected_reps[level].token_estimate
        ), (
            f"{level.name}: AIMessage-without-tool_calls "
            f"({page.representations[level].token_estimate}) must equal "
            f"unmodified _build_text_representations output "
            f"({expected_reps[level].token_estimate}) — no phantom overhead"
        )

    # Sanity: FULL token_estimate is at least the raw content-token count
    # (monotonicity clamps may push it slightly higher, but never lower).
    full = page.representations[FidelityLevel.FULL].token_estimate
    content_tokens = counter.count_text(content)
    assert full >= content_tokens


def test_ingest_ai_message_with_multiple_tool_calls_accounts_serialized_payload() -> None:
    counter = HeuristicCounter()
    tool_calls = [
        {
            "id": "call_1",
            "name": "read_file",
            "args": {"path": "/etc/hosts"},
            "type": "tool_call",
        },
        {
            "id": "call_2",
            "name": "list_dir",
            "args": {"path": "/tmp"},
            "type": "tool_call",
        },
    ]
    msg = AIMessage(content="reading and listing", tool_calls=tool_calls)
    pages = ingest_message(msg, turn_seq=0, ts=1.0, token_counter=counter)
    assert len(pages) == 1
    page = pages[0]

    # Round-trip contract: ``Page.ai_tool_calls`` preserves the exact
    # tool_calls payload that ``as_message`` will reattach.
    assert page.ai_tool_calls == tool_calls

    expected_payload_json = json.dumps(
        page.ai_tool_calls, sort_keys=True, default=str
    )
    expected_payload_tokens = counter.count_text(expected_payload_json)
    assert expected_payload_tokens > 0

    # Twin with the same content and no tool_calls gives us the
    # baseline per-rep token cost to subtract against.
    twin_pages = ingest_message(
        AIMessage(content="reading and listing"),
        turn_seq=0,
        ts=1.0,
        token_counter=counter,
    )
    assert len(twin_pages) == 1
    twin_page = twin_pages[0]

    # FULL: exact arithmetic — plain-content tokens + payload tokens.
    assert (
        page.representations[FidelityLevel.FULL].token_estimate
        == twin_page.representations[FidelityLevel.FULL].token_estimate
        + expected_payload_tokens
    )

    # POINTER: the full expected_payload is added, NOT a scaled-down
    # fraction. This locks in the "uniform overhead on every rung"
    # contract — the selector must see the same tool_calls cost
    # whether it picks POINTER or FULL, because as_message reattaches
    # the payload either way.
    assert (
        page.representations[FidelityLevel.POINTER].token_estimate
        == twin_page.representations[FidelityLevel.POINTER].token_estimate
        + expected_payload_tokens
    )
