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

"""Response parsing functions for VLM outputs."""

from typing import Any

from dimos.utils.llm_utils import extract_json


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
        Parsed dictionary with defaults filled in. If parsing fails, returns
        a dict with "_error" key instead of raising.
    """
    # Try to extract JSON (handles code fences)
    parsed = extract_json(response_text)
    if parsed is None:
        return {
            "window": {"start_s": w_start, "end_s": w_end},
            "caption": "",
            "entities_present": [],
            "new_entities": [],
            "relations": [],
            "on_screen_text": [],
            "_error": f"Failed to parse JSON from response: {response_text[:200]}...",
        }

    # Ensure we return a dict (extract_json can return a list)
    if isinstance(parsed, list):
        # If we got a list, wrap it in a dict with a default structure
        # This shouldn't happen with proper structured output, but handle gracefully
        return {
            "window": {"start_s": w_start, "end_s": w_end},
            "caption": "",
            "entities_present": [],
            "new_entities": [],
            "relations": [],
            "on_screen_text": [],
            "_error": f"Unexpected list response: {parsed}",
        }

    # Ensure it's a dict
    if not isinstance(parsed, dict):
        return {
            "window": {"start_s": w_start, "end_s": w_end},
            "caption": "",
            "entities_present": [],
            "new_entities": [],
            "relations": [],
            "on_screen_text": [],
            "_error": f"Expected dict or list, got {type(parsed)}: {parsed}",
        }

    return parsed
