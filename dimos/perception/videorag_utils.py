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

"""
VideoRAG utilities for temporal memory - adapted from videorag/evidence.py

This module ports the sophisticated prompts and logic from VideoRAG for use
with dimos's VlModel abstraction instead of OpenAI API directly.
"""

import json
from typing import Any

from dimos.msgs.sensor_msgs import Image


def next_entity_id_hint(roster: Any) -> str:
    """Generate next entity ID based on existing roster (e.g., E1, E2, E3...)."""
    if not isinstance(roster, list):
        return "E1"
    max_n = 0
    for e in roster:
        if not isinstance(e, dict):
            continue
        eid = e.get("id")
        if isinstance(eid, str) and eid.startswith("E"):
            tail = eid[1:]
            if tail.isdigit():
                max_n = max(max_n, int(tail))
    return f"E{max_n + 1}"


def clamp_text(text: str, max_chars: int) -> str:
    """Clamp text to maximum characters."""
    if len(text) <= max_chars:
        return text
    return text[:max_chars] + "..."


def build_window_prompt(
    *,
    w_start: float,
    w_end: float,
    frame_count: int,
    state: dict[str, Any],
) -> str:
    """
    Build comprehensive VLM prompt for analyzing a video window.

    This is adapted from videorag's build_window_messages() but formatted
    as a single text prompt for VlModel.query() instead of OpenAI's messages format.

    Args:
        w_start: Window start time in seconds
        w_end: Window end time in seconds
        frame_count: Number of frames in this window
        state: Current temporal memory state (entity_roster, rolling_summary, etc.)

    Returns:
        Formatted prompt string
    """
    roster = state.get("entity_roster", [])
    rolling_summary = state.get("rolling_summary", "")
    next_id = next_entity_id_hint(roster)

    # System instructions (from VideoRAG)
    system_context = """You analyze short sequences of video frames.
You must stay grounded in what is visible.
Do not identify real people or guess names/identities; describe people anonymously.
Extract general entities (people, objects, screens, text, locations) and relations between them.
Use stable entity IDs like E1, E2 based on the provided roster."""

    # Main prompt (from VideoRAG's build_window_messages)
    prompt = f"""{system_context}

Time window: [{w_start:.3f}, {w_end:.3f}) seconds
Number of frames: {frame_count}

Existing entity roster (may be empty):
{json.dumps(roster, ensure_ascii=False)}

Rolling summary so far (may be empty):
{clamp_text(str(rolling_summary), 1500)}

Task:
1) Write a dense, grounded caption describing what is visible across the frames in this time window.
2) Identify which existing roster entities appear in these frames.
3) Add any new salient entities (people/objects/screens/text/locations) with a short grounded descriptor.
4) Extract grounded relations/events between entities (e.g., looks_at, holds, uses, walks_past, speaks_to (inferred)).

New entity IDs must start at: {next_id}

Rules (important):
- You MUST stay grounded in what is visible in the provided frames.
- You MUST NOT mention any entity ID unless it appears in the provided roster OR you include it in new_entities in this same output.
- If the roster is empty, introduce any salient entities you reference (start with E1, E2, ...).
- Do not invent on-screen text: only include text you can read.
- If a relation is inferred (e.g., speaks_to without audio), include it but lower confidence and explain the visual cues.

Output JSON ONLY with this schema:
{{
  "window": {{"start_s": {w_start:.3f}, "end_s": {w_end:.3f}}},
  "caption": "dense grounded description",
  "entities_present": [{{"id": "E1", "confidence": 0.0-1.0}}],
  "new_entities": [{{"id": "E3", "type": "person|object|screen|text|location|other", "descriptor": "..."}}],
  "relations": [
    {{
      "type": "speaks_to|looks_at|holds|uses|moves|gesture|scene_change|other",
      "subject": "E1|unknown",
      "object": "E2|unknown",
      "confidence": 0.0-1.0,
      "evidence": ["describe which frames show this"],
      "notes": "short, grounded"
    }}
  ],
  "on_screen_text": ["verbatim snippets"],
  "uncertainties": ["things that are unclear"],
  "confidence": 0.0-1.0
}}
"""
    return prompt


# JSON schema for window responses (from VideoRAG)
WINDOW_RESPONSE_SCHEMA = {
    "type": "object",
    "properties": {
        "window": {
            "type": "object",
            "properties": {"start_s": {"type": "number"}, "end_s": {"type": "number"}},
            "required": ["start_s", "end_s"],
        },
        "caption": {"type": "string"},
        "entities_present": {
            "type": "array",
            "items": {
                "type": "object",
                "properties": {
                    "id": {"type": "string"},
                    "confidence": {"type": "number", "minimum": 0, "maximum": 1},
                },
                "required": ["id"],
            },
        },
        "new_entities": {
            "type": "array",
            "items": {
                "type": "object",
                "properties": {
                    "id": {"type": "string"},
                    "type": {
                        "type": "string",
                        "enum": ["person", "object", "screen", "text", "location", "other"],
                    },
                    "descriptor": {"type": "string"},
                },
                "required": ["id", "type"],
            },
        },
        "relations": {
            "type": "array",
            "items": {
                "type": "object",
                "properties": {
                    "type": {"type": "string"},
                    "subject": {"type": "string"},
                    "object": {"type": "string"},
                    "confidence": {"type": "number", "minimum": 0, "maximum": 1},
                    "evidence": {"type": "array", "items": {"type": "string"}},
                    "notes": {"type": "string"},
                },
                "required": ["type", "subject", "object"],
            },
        },
        "on_screen_text": {"type": "array", "items": {"type": "string"}},
        "uncertainties": {"type": "array", "items": {"type": "string"}},
        "confidence": {"type": "number", "minimum": 0, "maximum": 1},
    },
    "required": ["window", "caption"],
}


def build_summary_prompt(
    *,
    rolling_summary: str,
    chunk_windows: list[dict[str, Any]],
) -> str:
    """
    Build prompt for updating rolling summary.

    This is adapted from videorag's build_summary_messages() but formatted
    as a single text prompt for VlModel.query().

    Args:
        rolling_summary: Current rolling summary text
        chunk_windows: List of recent window results to incorporate

    Returns:
        Formatted prompt string
    """
    # System context (from VideoRAG)
    system_context = """You summarize timestamped video-window logs into a concise rolling summary.
Stay grounded in the provided window captions/relations.
Do not invent entities or rename entity IDs; preserve IDs like E1, E2 exactly.
You MAY incorporate new entity IDs if they appear in the provided chunk windows (e.g., in new_entities).
Be concise, but keep relevant entity continuity and key relations."""

    prompt = f"""{system_context}

Update the rolling summary using the newest chunk.

Previous rolling summary (may be empty):
{clamp_text(rolling_summary, 2500)}

New chunk windows (JSON):
{json.dumps(chunk_windows, ensure_ascii=False)}

Output a concise summary as PLAIN TEXT (no JSON, no code fences).
Length constraints (important):
- Target <= 120 words total.
- Hard cap <= 900 characters.
"""
    return prompt


def parse_window_response(
    response_text: str, w_start: float, w_end: float, frame_count: int
) -> dict[str, Any]:
    """
    Parse VLM response for a window analysis.

    Args:
        response_text: Raw text response from VLM
        w_start: Window start time
        w_end: Window end time
        frame_count: Number of frames in window

    Returns:
        Parsed dictionary with defaults filled in
    """
    # Try to extract JSON (handles code fences)
    text = response_text.strip()
    if text.startswith("```"):
        parts = text.split("```")
        if len(parts) >= 2:
            candidate = parts[1].lstrip()
            if candidate.startswith("json"):
                candidate = candidate[4:].lstrip()
            text = candidate.strip()

    try:
        parsed = json.loads(text)
    except json.JSONDecodeError as e:
        # Return error structure
        return {
            "window": {"start_s": w_start, "end_s": w_end},
            "frame_count": frame_count,
            "_error": {"kind": "parse_error", "message": str(e), "raw": text[:500]},
            "caption": "",
            "entities_present": [],
            "new_entities": [],
            "relations": [],
            "on_screen_text": [],
            "uncertainties": [],
            "confidence": 0.0,
        }

    # Fill in defaults
    parsed.setdefault("window", {"start_s": w_start, "end_s": w_end})
    parsed.setdefault("frame_count", frame_count)
    parsed.setdefault("caption", "")
    parsed.setdefault("entities_present", [])
    parsed.setdefault("new_entities", [])
    parsed.setdefault("relations", [])
    parsed.setdefault("on_screen_text", [])
    parsed.setdefault("uncertainties", [])
    parsed.setdefault("confidence", 1.0)

    return parsed


def update_state_from_window(
    state: dict[str, Any],
    parsed: dict[str, Any],
    w_end: float,
    summary_interval_s: float,
) -> bool:
    """
    Update temporal memory state from a parsed window result.

    This implements the state update logic from VideoRAG's generate_evidence().

    Args:
        state: Current state dictionary (modified in place)
        parsed: Parsed window result
        w_end: Window end time
        summary_interval_s: How often to trigger summary updates

    Returns:
        True if summary update is needed, False otherwise
    """
    # Skip if there was an error
    if isinstance(parsed.get("_error"), dict):
        return False

    new_entities = parsed.get("new_entities", [])
    present = parsed.get("entities_present", [])

    # Handle new entities
    if new_entities:
        roster = list(state.get("entity_roster", []))
        known = {e.get("id") for e in roster if isinstance(e, dict)}
        for e in new_entities:
            if isinstance(e, dict) and e.get("id") not in known:
                roster.append(e)
                known.add(e.get("id"))
        state["entity_roster"] = roster

    # Handle referenced entities (auto-add if mentioned but not in roster)
    roster = list(state.get("entity_roster", []))
    known = {e.get("id") for e in roster if isinstance(e, dict)}
    referenced: set[str] = set()
    for p in present or []:
        if isinstance(p, dict) and isinstance(p.get("id"), str):
            referenced.add(p["id"])
    for rel in parsed.get("relations") or []:
        if isinstance(rel, dict):
            for k in ("subject", "object"):
                v = rel.get(k)
                if isinstance(v, str) and v != "unknown":
                    referenced.add(v)
    for rid in sorted(referenced):
        if rid not in known:
            roster.append(
                {"id": rid, "type": "other", "descriptor": "unknown (auto-added; rerun recommended)"}
            )
            known.add(rid)
    state["entity_roster"] = roster
    state["last_present"] = present

    # Add to chunk buffer
    chunk_buffer = state.get("chunk_buffer", [])
    if not isinstance(chunk_buffer, list):
        chunk_buffer = []
    chunk_buffer.append(
        {
            "window": parsed.get("window"),
            "caption": parsed.get("caption", ""),
            "entities_present": parsed.get("entities_present", []),
            "new_entities": parsed.get("new_entities", []),
            "relations": parsed.get("relations", []),
            "on_screen_text": parsed.get("on_screen_text", []),
        }
    )
    state["chunk_buffer"] = chunk_buffer

    # Check if summary update is needed
    if summary_interval_s > 0:
        next_at = float(state.get("next_summary_at_s", summary_interval_s))
        if w_end + 1e-6 >= next_at and chunk_buffer:
            return True  # Need to update summary

    return False


def apply_summary_update(state: dict[str, Any], summary_text: str, w_end: float, summary_interval_s: float) -> None:
    """
    Apply a summary update to the state.

    Args:
        state: State dictionary (modified in place)
        summary_text: New summary text
        w_end: Current window end time
        summary_interval_s: Summary update interval
    """
    if summary_text and summary_text.strip():
        state["rolling_summary"] = summary_text.strip()
    state["chunk_buffer"] = []

    # Advance next_summary_at_s
    next_at = float(state.get("next_summary_at_s", summary_interval_s))
    while next_at <= w_end + 1e-6:
        next_at += float(summary_interval_s)
    state["next_summary_at_s"] = next_at


def get_structured_output_format() -> dict[str, Any]:
    """
    Get OpenAI-compatible structured output format for window responses.

    This uses the json_schema mode available in OpenAI API (GPT-4o mini) to enforce
    the VideoRAG response schema.

    Returns:
        Dictionary for response_format parameter:
        {"type": "json_schema", "json_schema": {...}}
    """

    return {
        "type": "json_schema",
        "json_schema": {
            "name": "video_window_analysis",
            "description": "Analysis of a video window with entities and relations",
            "schema": WINDOW_RESPONSE_SCHEMA,
            "strict": False,  # Allow additional fields
        },
    }


__all__ = [
    "WINDOW_RESPONSE_SCHEMA",
    "apply_summary_update",
    "build_summary_prompt",
    "build_window_prompt",
    "clamp_text",
    "get_structured_output_format",
    "next_entity_id_hint",
    "parse_window_response",
    "update_state_from_window",
]
