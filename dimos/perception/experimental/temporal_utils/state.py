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

"""State management functions for temporal memory."""

from typing import Any


def default_state() -> dict[str, Any]:
    """Create default temporal memory state dictionary."""
    return {
        "entity_roster": [],
        "rolling_summary": "",
        "chunk_buffer": [],
        "next_summary_at_s": 0.0,
        "last_present": [],
    }


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
