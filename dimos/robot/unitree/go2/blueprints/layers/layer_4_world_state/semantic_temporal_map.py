#!/usr/bin/env python3
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

from __future__ import annotations

from typing import Any

from dimos.core.core import rpc
from dimos.core.module import Module
from dimos.perception.spatial_memory_spec import SpatialMemorySpec
from dimos.perception.temporal_memory_spec import TemporalMemorySpec
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class _Go2SemanticTemporalMap(Module):
    """Layer 4 semantic-temporal memory view for Go2.

    This module does not create a new database. It is a thin normalization layer
    over the existing spatial and temporal memory modules, so callers can ask
    one Layer 4 component for memory evidence instead of separately knowing
    every underlying memory implementation.
    """

    _spatial_memory: SpatialMemorySpec | None = None
    _temporal_memory: TemporalMemorySpec | None = None

    @rpc
    def query_semantic_temporal_map(
        self, query: str = "", spatial_limit: int = 3
    ) -> dict[str, Any]:
        """Return spatial and temporal memory evidence for a task or query."""
        query = query.strip()
        spatial_limit = max(0, min(spatial_limit, 10))
        errors: list[str] = []

        result: dict[str, Any] = {
            "query": query,
            "sources": {
                "spatial_memory": self._spatial_memory is not None,
                "temporal_memory": self._temporal_memory is not None,
            },
            "spatial": self._spatial_section(query, spatial_limit, errors),
            "temporal": self._temporal_section(query, errors),
        }
        result["fused"] = self._fused_section(result)
        if errors:
            result["errors"] = errors
        return _to_jsonable(result)

    def _spatial_section(
        self, query: str, spatial_limit: int, errors: list[str]
    ) -> dict[str, Any]:
        if self._spatial_memory is None:
            return {"available": False, "matches": []}
        if not query:
            return {"available": True, "matches": [], "reason": "query is empty"}
        if spatial_limit == 0:
            return {"available": True, "matches": [], "reason": "spatial_limit is 0"}

        try:
            matches = self._spatial_memory.query_by_text(query, limit=spatial_limit)
        except Exception as exc:
            logger.warning("Failed to query spatial memory", exc_info=True)
            errors.append(f"spatial_memory: {exc}")
            return {"available": True, "matches": []}

        return {
            "available": True,
            "matches": [_summarize_spatial_match(match) for match in matches],
        }

    def _temporal_section(self, query: str, errors: list[str]) -> dict[str, Any]:
        if self._temporal_memory is None:
            return {"available": False}

        context: dict[str, Any] = {"available": True}
        if query:
            try:
                context["answer"] = self._temporal_memory.query(query)
            except Exception as exc:
                logger.warning("Failed to query temporal memory", exc_info=True)
                errors.append(f"temporal_memory.query: {exc}")

        try:
            context["rolling_summary"] = self._temporal_memory.get_rolling_summary()
        except Exception as exc:
            logger.warning("Failed to read temporal rolling summary", exc_info=True)
            errors.append(f"temporal_memory.summary: {exc}")

        try:
            context["state"] = self._temporal_memory.get_state()
        except Exception as exc:
            logger.warning("Failed to read temporal state", exc_info=True)
            errors.append(f"temporal_memory.state: {exc}")

        try:
            context["entity_roster"] = self._temporal_memory.get_entity_roster()
        except Exception as exc:
            logger.warning("Failed to read temporal entity roster", exc_info=True)
            errors.append(f"temporal_memory.roster: {exc}")

        try:
            context["graph"] = self._temporal_memory.get_graph_db_stats()
        except Exception as exc:
            logger.warning("Failed to read temporal graph stats", exc_info=True)
            errors.append(f"temporal_memory.graph: {exc}")

        return context

    def _fused_section(self, result: dict[str, Any]) -> dict[str, Any]:
        spatial = result["spatial"]
        temporal = result["temporal"]
        spatial_matches = spatial.get("matches") or []
        temporal_answer = temporal.get("answer")
        temporal_summary = temporal.get("rolling_summary")
        entries = _build_evidence_entries(spatial_matches, temporal)
        return {
            "available": bool(spatial.get("available") or temporal.get("available")),
            "spatial_match_count": len(spatial_matches),
            "has_temporal_answer": bool(temporal_answer),
            "has_temporal_summary": bool(temporal_summary),
            "entry_count": len(entries),
            "entries": entries,
        }


def _summarize_spatial_match(match: dict[str, Any]) -> dict[str, Any]:
    summary: dict[str, Any] = {}
    for key in ("distance", "score", "id", "text"):
        if key in match:
            summary[key] = match[key]
    if "metadata" in match:
        summary["metadata"] = _to_jsonable(match["metadata"])
    if not summary:
        summary["keys"] = sorted(match.keys())
    return _to_jsonable(summary)


def _build_evidence_entries(
    spatial_matches: list[dict[str, Any]], temporal: dict[str, Any]
) -> list[dict[str, Any]]:
    entries: list[dict[str, Any]] = []

    for index, match in enumerate(spatial_matches[:5]):
        if not isinstance(match, dict):
            continue
        entries.append(_spatial_evidence_entry(match, index))

    for source, entity in _temporal_entities(temporal):
        entries.append(_temporal_evidence_entry(entity, source))

    return [_to_jsonable(entry) for entry in _dedupe_entries(entries)[:10]]


def _spatial_evidence_entry(match: dict[str, Any], index: int) -> dict[str, Any]:
    metadata = _first_metadata(match)
    entity_id = _first_text(metadata, "entity_id", "id", "label", "name") or _first_text(
        match, "id", "label", "name"
    )
    description = (
        _first_text(match, "text", "description")
        or _first_text(metadata, "description", "descriptor", "label", "name")
        or entity_id
        or f"spatial_match_{index}"
    )
    return {
        "entity": {
            "id": entity_id or f"spatial_match_{index}",
            "type": _first_text(metadata, "type", "entity_type") or "spatial_observation",
            "description": description,
        },
        "location": _location_from(match, metadata),
        "time": _time_from(match, metadata),
        "evidence_source": "spatial_memory",
        "confidence": _confidence_from(match),
        "source_index": index,
        "summary": description,
    }


def _temporal_evidence_entry(entity: dict[str, Any], source: str) -> dict[str, Any]:
    metadata = _metadata_dict(entity.get("metadata"))
    entity_id = _first_text(entity, "entity_id", "id", "name", "label")
    description = _first_text(entity, "descriptor", "description", "label", "name") or entity_id
    return {
        "entity": {
            "id": entity_id or description or "unknown_entity",
            "type": _first_text(entity, "entity_type", "type") or "unknown",
            "description": description or "unknown",
        },
        "location": _location_from(entity, metadata),
        "time": _time_from(entity, metadata),
        "evidence_source": source,
        "confidence": _confidence_from(entity),
        "summary": description or entity_id or "unknown entity",
    }


def _temporal_entities(temporal: dict[str, Any]) -> list[tuple[str, dict[str, Any]]]:
    entities: list[tuple[str, dict[str, Any]]] = []
    state = temporal.get("state")
    if isinstance(state, dict):
        for entity in state.get("entities") or state.get("entity_roster") or []:
            if isinstance(entity, dict):
                entities.append(("temporal_memory.state", entity))
        for entity in state.get("currently_present") or []:
            if isinstance(entity, dict):
                entities.append(("temporal_memory.currently_present", entity))

    for entity in temporal.get("entity_roster") or []:
        if isinstance(entity, dict):
            entities.append(("temporal_memory.entity_roster", entity))

    graph = temporal.get("graph")
    if isinstance(graph, dict):
        for entity in graph.get("entities") or []:
            if isinstance(entity, dict):
                entities.append(("temporal_memory.graph", entity))

    return entities


def _dedupe_entries(entries: list[dict[str, Any]]) -> list[dict[str, Any]]:
    deduped: list[dict[str, Any]] = []
    seen: set[tuple[str, str, str]] = set()
    for entry in entries:
        entity = entry.get("entity") if isinstance(entry.get("entity"), dict) else {}
        entity_id = str(entity.get("id") or "")
        source = str(entry.get("evidence_source") or "")
        location = entry.get("location")
        location_key = str(location) if location else ""
        key = (entity_id, source, location_key)
        if key in seen:
            continue
        seen.add(key)
        deduped.append(entry)
    return deduped


def _first_metadata(match: dict[str, Any]) -> dict[str, Any]:
    return _metadata_dict(match.get("metadata"))


def _metadata_dict(value: Any) -> dict[str, Any]:
    if isinstance(value, dict):
        return value
    if isinstance(value, list):
        for item in value:
            if isinstance(item, dict):
                return item
    return {}


def _location_from(*sources: dict[str, Any]) -> dict[str, Any] | None:
    for source in sources:
        x = _first_number(source, "world_x", "pos_x", "x")
        y = _first_number(source, "world_y", "pos_y", "y")
        z = _first_number(source, "world_z", "pos_z", "z")
        if x is None or y is None:
            continue
        location: dict[str, Any] = {
            "x": round(x, 3),
            "y": round(y, 3),
        }
        if z is not None:
            location["z"] = round(z, 3)
        frame_id = _first_text(source, "frame_id", "frame")
        if frame_id:
            location["frame_id"] = frame_id
        label = _first_text(source, "location", "place", "label", "name")
        if label:
            location["label"] = label
        return location
    return None


def _time_from(*sources: dict[str, Any]) -> dict[str, Any] | None:
    for source in sources:
        first_seen = _first_number(source, "first_seen_ts", "first_seen", "start_ts")
        last_seen = _first_number(source, "last_seen_ts", "last_seen", "end_ts")
        timestamp = _first_number(source, "timestamp_s", "timestamp", "time", "ts")
        if first_seen is None and last_seen is None and timestamp is None:
            continue
        result: dict[str, Any] = {}
        if timestamp is not None:
            result["timestamp"] = round(timestamp, 3)
        if first_seen is not None:
            result["first_seen_ts"] = round(first_seen, 3)
        if last_seen is not None:
            result["last_seen_ts"] = round(last_seen, 3)
        return result
    return None


def _confidence_from(source: dict[str, Any]) -> float | None:
    explicit = _first_number(source, "confidence", "score", "similarity")
    if explicit is not None:
        return round(max(0.0, min(1.0, explicit)), 3)
    distance = _first_number(source, "distance")
    if distance is None:
        return None
    return round(max(0.0, min(1.0, 1.0 - distance)), 3)


def _first_text(source: dict[str, Any], *keys: str) -> str | None:
    for key in keys:
        value = source.get(key)
        if isinstance(value, str) and value.strip():
            return value.strip()
    return None


def _first_number(source: dict[str, Any], *keys: str) -> float | None:
    for key in keys:
        value = source.get(key)
        if isinstance(value, bool):
            continue
        if isinstance(value, int | float):
            return float(value)
    return None


def _to_jsonable(value: Any, max_string_length: int = 500) -> Any:
    if value is None or isinstance(value, bool | int | float):
        return value
    if isinstance(value, str):
        return value[:max_string_length]
    if isinstance(value, dict):
        return {
            str(key): _to_jsonable(item, max_string_length)
            for key, item in value.items()
            if not str(key).startswith("_")
        }
    if isinstance(value, list | tuple):
        return [_to_jsonable(item, max_string_length) for item in value[:10]]
    return str(value)[:max_string_length]


__all__ = ["_Go2SemanticTemporalMap"]
