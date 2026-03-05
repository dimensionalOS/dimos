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

"""Graph database utility functions for temporal memory."""

import re
from typing import TYPE_CHECKING, Any

from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from dimos.models.vl.base import VlModel
    from dimos.msgs.sensor_msgs import Image

    from ..entity_graph_db import EntityGraphDB

logger = setup_logger()


def extract_time_window(
    question: str,
    vlm: "VlModel",
    latest_frame: "Image | None" = None,
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


def build_graph_context(
    graph_db: "EntityGraphDB",
    entity_ids: list[str],
    time_window_s: float | None = None,
    max_relations_per_entity: int = 10,
    nearby_distance_meters: float = 5.0,
    current_video_time_s: float | None = None,
) -> dict[str, Any]:
    """Build enriched context from graph database for given entities.

    Args:
        graph_db: Entity graph database instance
        entity_ids: List of entity IDs to get context for
        time_window_s: Optional time window in seconds (e.g., 3600 for last hour)
        max_relations_per_entity: Maximum relations to include per entity (default: 10)
        nearby_distance_meters: Distance threshold for "nearby" entities (default: 5.0)
        current_video_time_s: Current video timestamp in seconds (for time window queries).
            If None, uses latest entity timestamp from DB as reference.

    Returns:
        Dictionary with graph context including relationships, distances, and semantics
    """
    if not graph_db or not entity_ids:
        return {}

    try:
        graph_context: dict[str, Any] = {
            "relationships": [],
            "spatial_info": [],
            "semantic_knowledge": [],
            "entity_timestamps": [],
        }

        # Convert time_window_s to a (start_ts, end_ts) tuple if provided
        # Use video-relative timestamps, not wall-clock time
        time_window_tuple = None
        if time_window_s is not None:
            if current_video_time_s is not None:
                ref_time = current_video_time_s
            else:
                # Fallback: get the latest timestamp from entities in DB
                all_entities = graph_db.get_all_entities()
                ref_time = max((e.get("last_seen_ts", 0) for e in all_entities), default=0)
            time_window_tuple = (max(0, ref_time - time_window_s), ref_time)

        # Get entity timestamp information for visibility duration queries
        for entity_id in entity_ids:
            entity = graph_db.get_entity(entity_id)
            if entity:
                first_seen = entity.get("first_seen_ts")
                last_seen = entity.get("last_seen_ts")
                duration_s = None
                if first_seen is not None and last_seen is not None:
                    duration_s = last_seen - first_seen

                graph_context["entity_timestamps"].append(
                    {
                        "entity_id": entity_id,
                        "first_seen_ts": first_seen,
                        "last_seen_ts": last_seen,
                        "duration_s": duration_s,
                    }
                )

        # Get recent relationships for each entity
        for entity_id in entity_ids:
            # Get relationships (Graph 1: interactions)
            relations = graph_db.get_relations_for_entity(
                entity_id=entity_id,
                relation_type=None,  # all types
                time_window=time_window_tuple,
            )
            for rel in relations[-max_relations_per_entity:]:
                graph_context["relationships"].append(
                    {
                        "subject": rel["subject_id"],
                        "relation": rel["relation_type"],
                        "object": rel["object_id"],
                        "confidence": rel["confidence"],
                        "when": rel["timestamp_s"],
                    }
                )

            # Get spatial relationships (Graph 2: distances)
            nearby = graph_db.get_nearby_entities(
                entity_id=entity_id, max_distance=nearby_distance_meters, latest_only=True
            )
            for dist in nearby:
                graph_context["spatial_info"].append(
                    {
                        "entity_a": entity_id,
                        "entity_b": dist["entity_id"],
                        "distance": dist.get("distance_meters"),
                        "category": dist.get("distance_category"),
                        "confidence": dist["confidence"],
                    }
                )

            # Get semantic knowledge (Graph 3: conceptual relations)
            semantic_rels = graph_db.get_semantic_relations(
                entity_id=entity_id,
                relation_type=None,
            )
            for sem in semantic_rels:
                graph_context["semantic_knowledge"].append(
                    {
                        "entity_a": sem["entity_a_id"],
                        "relation": sem["relation_type"],
                        "entity_b": sem["entity_b_id"],
                        "confidence": sem["confidence"],
                        "observations": sem["observation_count"],
                    }
                )

        # Get graph statistics for context
        if entity_ids:
            stats = graph_db.get_stats()
            graph_context["total_entities"] = stats.get("entities", 0)
            graph_context["total_relations"] = stats.get("relations", 0)

        return graph_context

    except Exception as e:
        logger.warning(f"failed to build graph context: {e}")
        return {}
