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

        return context

    def _fused_section(self, result: dict[str, Any]) -> dict[str, Any]:
        spatial = result["spatial"]
        temporal = result["temporal"]
        spatial_matches = spatial.get("matches") or []
        temporal_answer = temporal.get("answer")
        temporal_summary = temporal.get("rolling_summary")
        return {
            "available": bool(spatial.get("available") or temporal.get("available")),
            "spatial_match_count": len(spatial_matches),
            "has_temporal_answer": bool(temporal_answer),
            "has_temporal_summary": bool(temporal_summary),
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
