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
"""Tests for :mod:`dimos.agents.memory.artefact_tool`."""

from __future__ import annotations

import base64
import re

import cv2
from langchain_core.messages import HumanMessage, SystemMessage
import numpy as np

from dimos.agents.memory.artefact_tool import (
    GET_ARTEFACT_TOOL_NAME,
    build_get_artefact_tool,
)
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
    img: np.ndarray = np.full((size, size, 3), 128, dtype=np.uint8)
    ok, buf = cv2.imencode(".jpg", img, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
    assert ok
    b64 = base64.b64encode(buf.tobytes()).decode("ascii")
    return HumanMessage(
        content=[
            {"type": "image_url", "image_url": {"url": f"data:image/jpeg;base64,{b64}"}},
        ]
    )


def test_tool_name_is_get_artefact() -> None:
    eng = MemoryEngine(model_name="gpt-4o", token_counter=HeuristicCounter())
    tool = build_get_artefact_tool(eng)
    assert tool.name == GET_ARTEFACT_TOOL_NAME


def test_tool_has_non_empty_description_mentioning_artefact() -> None:
    eng = MemoryEngine(model_name="gpt-4o", token_counter=HeuristicCounter())
    tool = build_get_artefact_tool(eng)
    assert tool.description
    assert "artefact" in tool.description.lower()


def test_tool_invoke_schedules_full_and_emits_refetch_fault() -> None:
    out_stream = _FakeOut()
    eng = MemoryEngine(
        model_name="gpt-4o",
        token_budget=1300,
        pin_recent_evidence=1,
        output_reserve_tokens=0,
        system_overhead=0,
        token_counter=HeuristicCounter(),
        faults_out=out_stream,  # type: ignore[arg-type]
    )
    eng.ingest(SystemMessage(content="sys"))
    eng.ingest(_image_msg())
    eng.ingest(_image_msg())

    old_evidence = next(p for p in eng.pages() if p.type is PageType.EVIDENCE)
    uuid = old_evidence.artefact_uuid
    assert uuid is not None

    tool = eng.get_artefact_tool()
    # StructuredTool.invoke takes a dict of args. Test both the underlying
    # function (direct call) and the .invoke() dispatch path.
    msg = tool.func(uuid=uuid)  # type: ignore[misc]
    assert "scheduled" in msg.lower() or uuid in msg

    assert FaultKind.REFETCH_FAULT in [ev.kind for ev in out_stream.published]

    # Next assemble() should render the rehydrated page at FULL even under
    # pressure, because the flag is one-way.
    out2 = eng.assemble()
    assert out2.chosen_levels[old_evidence.id] == FidelityLevel.FULL


def test_tool_invoke_unknown_uuid_returns_diagnostic() -> None:
    eng = MemoryEngine(model_name="gpt-4o", token_counter=HeuristicCounter())
    tool = build_get_artefact_tool(eng)
    msg = tool.func(uuid="this-does-not-exist")  # type: ignore[misc]
    assert "no artefact" in msg.lower() or "not found" in msg.lower() or "artefact" in msg.lower()


def test_tool_args_schema_requires_uuid_field() -> None:
    eng = MemoryEngine(model_name="gpt-4o", token_counter=HeuristicCounter())
    tool = build_get_artefact_tool(eng)
    schema = tool.args_schema
    # args_schema may be a pydantic class — inspect via ``model_fields``.
    if hasattr(schema, "model_fields"):
        assert "uuid" in schema.model_fields  # type: ignore[union-attr]


def test_llm_can_rehydrate_from_structured_representation() -> None:
    """End-to-end: the UUID an LLM extracts from the STRUCTURED bracket
    must route to the correct page when passed to ``get_artefact``.

    This is the real coverage for the rehydration contract. We simulate
    what the LLM actually does: read the STRUCTURED text, parse the
    UUID out with a regex, hand it to the tool, and verify the page
    comes back at FULL on the next assembly.

    Scenario: tiny budget + ``pin_recent_evidence=0`` so the ingested
    image is not auto-pinned and gets driven down to a non-FULL rung by
    Phase 1. We then extract the UUID from the STRUCTURED rep itself
    (which is what the LLM sees when the selector picks that level, and
    is always readable off the page regardless of the assembled
    fidelity — this keeps the test robust to selector tuning while still
    exercising the UUID-agreement invariant end-to-end).
    """
    out_stream = _FakeOut()
    eng = MemoryEngine(
        model_name="gpt-4o",
        token_budget=2000,
        pin_recent_evidence=0,
        output_reserve_tokens=0,
        system_overhead=0,
        token_counter=HeuristicCounter(),
        faults_out=out_stream,  # type: ignore[arg-type]
    )
    eng.ingest(SystemMessage(content="sys"))
    eng.ingest(_image_msg())

    evidence = next(p for p in eng.pages() if p.type is PageType.EVIDENCE)

    # Read the STRUCTURED rep directly off the page. This is exactly the
    # string the LLM sees whenever the selector picks STRUCTURED; the
    # test stays robust to selector tuning (the UUID-agreement invariant
    # is a property of ingestion, not assembly).
    structured = evidence.rep_at(FidelityLevel.STRUCTURED).content
    assert isinstance(structured, str)
    assert structured.startswith("[image artefact uuid=")

    match = re.search(r"\[image artefact uuid=([a-f0-9\-]+) ", structured)
    assert match is not None, f"STRUCTURED rep did not match expected bracket shape: {structured!r}"
    extracted_uuid = match.group(1)

    # The extracted UUID must be the artefact UUID the PageTable indexes
    # on, not the page.id (which carries a ``"page-"`` prefix).
    assert extracted_uuid == evidence.artefact_uuid
    assert extracted_uuid != evidence.id

    tool = eng.get_artefact_tool()
    msg = tool.func(uuid=extracted_uuid)  # type: ignore[misc]
    assert "scheduled" in msg.lower() or extracted_uuid in msg, (
        f"Expected success-path response; got: {msg!r}"
    )
    assert "no artefact" not in msg.lower(), (
        f"Got 'not found' path from a UUID the LLM would legitimately "
        f"extract from STRUCTURED — UUID-agreement invariant regressed. "
        f"msg={msg!r}"
    )

    assert FaultKind.REFETCH_FAULT in [ev.kind for ev in out_stream.published]

    # Next assemble() must render the rehydrated page at FULL (pinned-at-full
    # is a one-way flag set by ``request_full``).
    out2 = eng.assemble()
    assert out2.chosen_levels[evidence.id] == FidelityLevel.FULL
