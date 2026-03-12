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

from __future__ import annotations

import re
from typing import TYPE_CHECKING, Any

from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from ..entity_graph_db import EntityGraphDB

logger = setup_logger()

# ── Time extraction (regex only — no VLM call) ─────────────────────

_KEYWORD_MAP: list[tuple[re.Pattern[str], float]] = [
    # Exact phrases
    (re.compile(r"\bjust now\b", re.I), 60),
    (re.compile(r"\bfew seconds? ago\b", re.I), 30),
    (re.compile(r"\bfew minutes? ago\b", re.I), 300),
    (re.compile(r"\brecently\b|\brecent\b", re.I), 600),
    (re.compile(r"\blast hour\b|\bpast hour\b", re.I), 3600),
    (re.compile(r"\btoday\b", re.I), 3600),
    (re.compile(r"\byesterday\b", re.I), 86400),
    (re.compile(r"\blast night\b", re.I), 43200),
    (re.compile(r"\bthis morning\b", re.I), 21600),
    (re.compile(r"\blast week\b|\bpast week\b", re.I), 7 * 86400),
    (re.compile(r"\blast month\b|\bpast month\b", re.I), 30 * 86400),
    (re.compile(r"\blast year\b|\bpast year\b", re.I), 365 * 86400),
]

_QUANTITY_PAT = re.compile(
    r"(?:(?:last|past|previous)\s+)?(\d+)\s+"
    r"(seconds?|minutes?|mins?|hours?|hrs?|days?|weeks?|months?|years?)\s*(?:ago)?",
    re.I,
)

_UNIT_TO_SECONDS: dict[str, float] = {
    "second": 1,
    "seconds": 1,
    "minute": 60,
    "minutes": 60,
    "min": 60,
    "mins": 60,
    "hour": 3600,
    "hours": 3600,
    "hr": 3600,
    "hrs": 3600,
    "day": 86400,
    "days": 86400,
    "week": 7 * 86400,
    "weeks": 7 * 86400,
    "month": 30 * 86400,
    "months": 30 * 86400,
    "year": 365 * 86400,
    "years": 365 * 86400,
}


def extract_time_window(question: str) -> float | None:
    """Extract a time-window (in seconds) from a question using regex heuristics.

    No VLM call is made — this replaces the old image-based approach.

    Returns:
        Seconds lookback, or None if no time reference found.
    """
    # Check quantity patterns first (e.g., "3 hours ago", "last 2 days")
    m = _QUANTITY_PAT.search(question)
    if m:
        num = int(m.group(1))
        unit = m.group(2).lower()
        factor = _UNIT_TO_SECONDS.get(unit)
        if factor is not None:
            return num * factor

    # Check keyword patterns
    for pat, seconds in _KEYWORD_MAP:
        if pat.search(question):
            return seconds

    return None


def build_graph_context(
    graph_db: EntityGraphDB,
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
            "entity_timestamps": [],
        }

        # Convert time_window_s to a (start_ts, end_ts) tuple if provided
        time_window_tuple = None
        if time_window_s is not None:
            if current_video_time_s is not None:
                ref_time = current_video_time_s
            else:
                all_entities = graph_db.get_all_entities()
                ref_time = max((e.get("last_seen_ts", 0) for e in all_entities), default=0)
            time_window_tuple = (max(0, ref_time - time_window_s), ref_time)

        # Entity timestamp info
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

        # Relationships and spatial info
        for entity_id in entity_ids:
            relations = graph_db.get_relations_for_entity(
                entity_id=entity_id,
                relation_type=None,
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

            nearby = graph_db.get_nearby_entities(
                entity_id=entity_id,
                max_distance=nearby_distance_meters,
                latest_only=True,
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

        # Graph statistics
        if entity_ids:
            stats = graph_db.get_stats()
            graph_context["total_entities"] = stats.get("entities", 0)
            graph_context["total_relations"] = stats.get("relations", 0)

        return graph_context

    except Exception as e:
        logger.warning(f"failed to build graph context: {e}")
        return {}
