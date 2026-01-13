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
import re
from typing import Any

from dimos.models.vl.base import VlModel
from dimos.msgs.sensor_msgs import Image
from dimos.utils.llm_utils import extract_json
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


def default_state() -> dict[str, Any]:
    """Create default temporal memory state dictionary."""
    return {
        "entity_roster": [],
        "rolling_summary": "",
        "chunk_buffer": [],
        "next_summary_at_s": 0.0,
        "last_present": [],
    }


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


def format_timestamp(seconds: float) -> str:
    """Format seconds as MM:SS.mmm timestamp string."""
    m = int(seconds // 60)
    s = seconds - 60 * m
    return f"{m:02d}:{s:06.3f}"


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


def build_query_prompt(
    *,
    question: str,
    context: dict[str, Any],
) -> str:
    """
    Build prompt for querying temporal memory.

    Args:
        question: User's question about the video stream
        context: Context dict containing entity_roster, rolling_summary, etc.

    Returns:
        Formatted prompt string
    """
    currently_present = context.get("currently_present_entities", [])
    currently_present_str = (
        f"Entities recently detected in recent windows: {currently_present}"
        if currently_present
        else "No entities were detected in recent windows (list is empty)"
    )

    prompt = f"""Answer the following question about the video stream using the provided context.

**Question:** {question}

**Context:**
{json.dumps(context, indent=2, ensure_ascii=False)}

**Important Notes:**
- Entities have stable IDs like E1, E2, etc.
- The 'currently_present_entities' list contains entity IDs that were detected in recent video windows (not necessarily in the current frame you're viewing)
- {currently_present_str}
- The 'entity_roster' contains all known entities with their descriptions
- The 'rolling_summary' describes what has happened over time
- If 'currently_present_entities' is empty, it means no entities were detected in recent windows, but entities may still exist in the roster from earlier
- Answer based on the provided context (entity_roster, rolling_summary, currently_present_entities) AND what you see in the current frame
- If the context says entities were present but you don't see them in the current frame, mention both: what was recently detected AND what you currently see

Provide a concise answer.
"""
    return prompt


def extract_time_window(
    question: str,
    vlm: VlModel,
    latest_frame: Image | None = None,
) -> float | None:
    """Extract time window from question using VLM with example-based learning.

    Uses a few example keywords as patterns, then asks VLM to extrapolate
    similar time references and return seconds.

    Args:
        question: User's question
        vlm: VLM instance to use for extraction
        latest_frame: Optional frame (required for VLM call, but image is ignored)

    Returns:
        Time window in seconds, or None if no time reference found
    """
    question_lower = question.lower()

    # Quick check for common patterns (fast path)
    if "last week" in question_lower or "past week" in question_lower:
        return 7 * 24 * 3600
    if "today" in question_lower or "last hour" in question_lower:
        return 3600
    if "recently" in question_lower or "recent" in question_lower:
        return 600

    # Use VLM to extract time reference from question
    # Provide examples and let VLM extrapolate similar patterns
    # Note: latest_frame is required by VLM interface but image content is ignored
    if not latest_frame:
        return None

    extraction_prompt = f"""Extract any time reference from this question and convert it to seconds.

Question: {question}

Examples of time references and their conversions:
- "last week" or "past week" -> 604800 seconds (7 days)
- "yesterday" -> 86400 seconds (1 day)
- "today" or "last hour" -> 3600 seconds (1 hour)
- "recently" or "recent" -> 600 seconds (10 minutes)
- "few minutes ago" -> 300 seconds (5 minutes)
- "just now" -> 60 seconds (1 minute)

Extrapolate similar patterns (e.g., "2 days ago", "this morning", "last month", etc.)
and convert to seconds. If no time reference is found, return "none".

Return ONLY a number (seconds) or the word "none". Do not include any explanation."""

    try:
        response = vlm.query(latest_frame, extraction_prompt)
        response = response.strip().lower()

        if "none" in response or not response:
            return None

        # Extract number from response
        numbers = re.findall(r"\d+(?:\.\d+)?", response)
        if numbers:
            seconds = float(numbers[0])
            # Sanity check: reasonable time windows (1 second to 1 year)
            if 1 <= seconds <= 365 * 24 * 3600:
                return seconds
    except Exception as e:
        logger.debug(f"Time extraction failed: {e}")

    return None


def build_distance_estimation_prompt(
    *,
    entity_a_descriptor: str,
    entity_a_id: str,
    entity_b_descriptor: str,
    entity_b_id: str,
) -> str:
    """
    Build prompt for estimating distance between two entities.

    Args:
        entity_a_descriptor: Description of first entity
        entity_a_id: ID of first entity
        entity_b_descriptor: Description of second entity
        entity_b_id: ID of second entity

    Returns:
        Formatted prompt string for distance estimation
    """
    prompt = f"""Look at this image and estimate the distance between these two entities:

Entity A: {entity_a_descriptor} (ID: {entity_a_id})
Entity B: {entity_b_descriptor} (ID: {entity_b_id})

Provide:
1. Distance category: "near" (< 1m), "medium" (1-3m), or "far" (> 3m)
2. Approximate distance in meters (best guess)
3. Confidence: 0.0-1.0 (how certain are you?)

Respond in this format:
category: [near/medium/far]
distance_m: [number]
confidence: [0.0-1.0]
reasoning: [brief explanation]"""
    return prompt


def build_batch_distance_estimation_prompt(
    entity_pairs: list[tuple[dict[str, Any], dict[str, Any]]],
) -> str:
    """
    Build prompt for estimating distances between multiple entity pairs in one call.

    Args:
        entity_pairs: List of (entity_a, entity_b) tuples, each entity is a dict with 'id' and 'descriptor'

    Returns:
        Formatted prompt string for batched distance estimation
    """
    pairs_text = []
    for i, (entity_a, entity_b) in enumerate(entity_pairs, 1):
        pairs_text.append(
            f"Pair {i}:\n"
            f"  Entity A: {entity_a['descriptor']} (ID: {entity_a['id']})\n"
            f"  Entity B: {entity_b['descriptor']} (ID: {entity_b['id']})"
        )

    prompt = f"""Look at this image and estimate the distances between the following entity pairs:

{chr(10).join(pairs_text)}

For each pair, provide:
1. Distance category: "near" (< 1m), "medium" (1-3m), or "far" (> 3m)
2. Approximate distance in meters (best guess)
3. Confidence: 0.0-1.0 (how certain are you?)

Respond in this format (one block per pair):
Pair 1:
category: [near/medium/far]
distance_m: [number]
confidence: [0.0-1.0]

Pair 2:
category: [near/medium/far]
distance_m: [number]
confidence: [0.0-1.0]

(etc.)"""
    return prompt


def parse_batch_distance_response(
    response: str, entity_pairs: list[tuple[dict[str, Any], dict[str, Any]]]
) -> list[dict[str, Any]]:
    """
    Parse batched distance estimation response.

    Args:
        response: VLM response text
        entity_pairs: Original entity pairs used in the prompt

    Returns:
        List of dicts with keys: entity_a_id, entity_b_id, category, distance_m, confidence
    """
    results = []
    lines = response.strip().split("\n")

    current_pair_idx = None
    category = None
    distance_m = None
    confidence = 0.5

    for line in lines:
        line = line.strip()

        # Check for pair marker
        if line.startswith("Pair "):
            # Save previous pair if exists
            if current_pair_idx is not None and category:
                entity_a, entity_b = entity_pairs[current_pair_idx]
                results.append(
                    {
                        "entity_a_id": entity_a["id"],
                        "entity_b_id": entity_b["id"],
                        "category": category,
                        "distance_m": distance_m,
                        "confidence": confidence,
                    }
                )

            # Start new pair
            try:
                pair_num = int(line.split()[1].rstrip(":"))
                current_pair_idx = pair_num - 1  # Convert to 0-indexed
                category = None
                distance_m = None
                confidence = 0.5
            except (IndexError, ValueError):
                continue

        # Parse distance fields
        elif line.startswith("category:"):
            category = line.split(":", 1)[1].strip().lower()
        elif line.startswith("distance_m:"):
            try:
                distance_m = float(line.split(":", 1)[1].strip())
            except (ValueError, IndexError):
                pass
        elif line.startswith("confidence:"):
            try:
                confidence = float(line.split(":", 1)[1].strip())
            except (ValueError, IndexError):
                pass

    # Save last pair
    if current_pair_idx is not None and category and current_pair_idx < len(entity_pairs):
        entity_a, entity_b = entity_pairs[current_pair_idx]
        results.append(
            {
                "entity_a_id": entity_a["id"],
                "entity_b_id": entity_b["id"],
                "category": category,
                "distance_m": distance_m,
                "confidence": confidence,
            }
        )

    return results


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
    parsed = extract_json(response_text)
    if parsed is None:
        raise ValueError(f"Failed to parse response: {response_text}")

    # Ensure we return a dict (extract_json can return a list)
    if isinstance(parsed, list):
        # If we got a list, wrap it in a dict with a default structure
        # This shouldn't happen with proper structured output, but handle gracefully
        return {
            "window": {"start": w_start, "end": w_end},
            "caption": "",
            "entities_present": [],
            "new_entities": [],
            "relations": [],
            "on_screen_text": [],
            "_error": f"Unexpected list response: {parsed}",
        }

    # Ensure it's a dict
    if not isinstance(parsed, dict):
        raise ValueError(f"Expected dict or list, got {type(parsed)}: {parsed}")

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
    if "_error" in parsed:
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
                {
                    "id": rid,
                    "type": "other",
                    "descriptor": "unknown (auto-added; rerun recommended)",
                }
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


def apply_summary_update(
    state: dict[str, Any], summary_text: str, w_end: float, summary_interval_s: float
) -> None:
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
    "build_distance_estimation_prompt",
    "build_query_prompt",
    "build_summary_prompt",
    "build_window_prompt",
    "clamp_text",
    "default_state",
    "extract_time_window",
    "format_timestamp",
    "get_structured_output_format",
    "next_entity_id_hint",
    "parse_window_response",
    "update_state_from_window",
]
