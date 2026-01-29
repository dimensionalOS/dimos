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

"""Prompt building functions for VLM queries."""

import json
from typing import Any

from .helpers import clamp_text, next_entity_id_hint

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
- The 'graph_knowledge.entity_timestamps' contains visibility information for each entity:
  - 'first_seen_ts': timestamp (in seconds) when the entity was first detected
  - 'last_seen_ts': timestamp (in seconds) when the entity was last detected
  - 'duration_s': total time span from first to last appearance (last_seen_ts - first_seen_ts)
  - Use this information to answer questions about when entities appeared, disappeared, or how long they were visible
- If 'currently_present_entities' is empty, it means no entities were detected in recent windows, but entities may still exist in the roster from earlier
- Answer based on the provided context (entity_roster, rolling_summary, currently_present_entities, graph_knowledge) AND what you see in the current frame
- If the context says entities were present but you don't see them in the current frame, mention both: what was recently detected AND what you currently see
- For duration questions, use the 'duration_s' field from 'entity_timestamps' if available

Provide a concise answer.
"""
    return prompt


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
