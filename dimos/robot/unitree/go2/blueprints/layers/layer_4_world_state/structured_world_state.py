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

import math
from typing import Any

from reactivex.disposable import Disposable

from dimos.core.core import rpc
from dimos.core.global_config import global_config
from dimos.core.module import Module
from dimos.core.stream import In
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.navigation.navigation_spec import NavigationInterfaceSpec
from dimos.perception.spatial_memory_spec import SpatialMemorySpec
from dimos.perception.temporal_memory_spec import TemporalMemorySpec
from dimos.robot.unitree.go2.blueprints.layers.layer_4_world_state.world_state_spec import (
    SemanticTemporalMapSpec,
)
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class _Go2StructuredWorldState(Module):
    """Layer 4 structured world-state facade for Go2.

    Layer 3 should not need to know which low-level module owns odom,
    navigation, spatial memory, or temporal memory. This module provides a
    compact RPC snapshot with stable keys while keeping the underlying modules
    unchanged.
    """

    _semantic_temporal_map: SemanticTemporalMapSpec | None = None
    _spatial_memory: SpatialMemorySpec | None = None
    _temporal_memory: TemporalMemorySpec | None = None
    _navigation: NavigationInterfaceSpec | None = None
    _latest_odom: PoseStamped | None = None

    odom: In[PoseStamped]

    @rpc
    def start(self) -> None:
        super().start()
        self.register_disposable(Disposable(self.odom.subscribe(self._on_odom)))

    def _on_odom(self, odom: PoseStamped) -> None:
        self._latest_odom = odom

    @rpc
    def get_world_snapshot(self, task: str = "", spatial_limit: int = 3) -> dict[str, Any]:
        """Return the current normalized Layer 4 snapshot."""
        task = task.strip()
        spatial_limit = max(0, min(spatial_limit, 10))
        errors: list[str] = []
        semantic_temporal = self._semantic_temporal_section(task, spatial_limit, errors)

        snapshot: dict[str, Any] = {
            "task": task,
            "sources": self._source_status(semantic_temporal),
            "runtime": self.get_runtime_state(),
            "robot_state": self.get_robot_state(),
            "memory_state": self._memory_state(task, spatial_limit, semantic_temporal, errors),
            "semantic_temporal_map": semantic_temporal,
        }
        if errors:
            snapshot["errors"] = errors
        return _to_jsonable(snapshot)

    @rpc
    def get_robot_state(self) -> dict[str, Any]:
        """Return robot state owned or normalized by Layer 4."""
        state: dict[str, Any] = {
            "odom": self._pose_to_dict(self._latest_odom),
            "navigation": None,
        }

        if self._navigation is None:
            return state

        try:
            navigation_state = self._navigation.get_state()
            state["navigation"] = {
                "state": getattr(navigation_state, "value", str(navigation_state)),
                "goal_reached": self._navigation.is_goal_reached(),
            }
        except Exception as exc:
            logger.warning("Failed to read navigation state", exc_info=True)
            state["navigation"] = {"error": str(exc)}

        return _to_jsonable(state)

    @rpc
    def get_runtime_state(self) -> dict[str, Any]:
        """Return runtime mode and relevant blueprint config values."""
        simulation = bool(getattr(global_config, "simulation", False))
        replay = bool(getattr(global_config, "replay", False))
        mode = "simulation" if simulation else "replay" if replay else "hardware"
        return {
            "mode": mode,
            "simulation": simulation,
            "replay": replay,
            "robot_ip": getattr(global_config, "robot_ip", None),
            "viewer": getattr(global_config, "viewer", None),
            "mcp_port": getattr(global_config, "mcp_port", None),
            "n_workers": getattr(global_config, "n_workers", None),
        }

    @rpc
    def get_memory_state(self, task: str = "", spatial_limit: int = 3) -> dict[str, Any]:
        """Return Layer 4 memory state without robot or runtime state."""
        task = task.strip()
        spatial_limit = max(0, min(spatial_limit, 10))
        errors: list[str] = []
        semantic_temporal = self._semantic_temporal_section(task, spatial_limit, errors)
        state = self._memory_state(task, spatial_limit, semantic_temporal, errors)
        if errors:
            state["errors"] = errors
        return _to_jsonable(state)

    def _source_status(self, semantic_temporal: dict[str, Any]) -> dict[str, bool]:
        semantic_sources = semantic_temporal.get("sources", {})
        return {
            "odom": self._latest_odom is not None,
            "navigation": self._navigation is not None,
            "spatial_memory": self._spatial_memory is not None
            or bool(semantic_sources.get("spatial_memory")),
            "temporal_memory": self._temporal_memory is not None
            or bool(semantic_sources.get("temporal_memory")),
            "semantic_temporal_map": self._semantic_temporal_map is not None,
            "runtime": True,
        }

    def _semantic_temporal_section(
        self, task: str, spatial_limit: int, errors: list[str]
    ) -> dict[str, Any]:
        if self._semantic_temporal_map is None:
            return {
                "available": False,
                "query": task,
                "spatial": self._spatial_fallback(task, spatial_limit, errors),
                "temporal": self._temporal_fallback(errors),
                "fused": {"available": False},
            }

        try:
            return self._semantic_temporal_map.query_semantic_temporal_map(
                query=task,
                spatial_limit=spatial_limit,
            )
        except Exception as exc:
            logger.warning("Failed to query semantic-temporal map", exc_info=True)
            errors.append(f"semantic_temporal_map: {exc}")
            return {
                "available": True,
                "query": task,
                "spatial": self._spatial_fallback(task, spatial_limit, errors),
                "temporal": self._temporal_fallback(errors),
                "fused": {"available": False},
            }

    def _memory_state(
        self,
        task: str,
        spatial_limit: int,
        semantic_temporal: dict[str, Any],
        errors: list[str],
    ) -> dict[str, Any]:
        if semantic_temporal.get("spatial") or semantic_temporal.get("temporal"):
            return {
                "query": task,
                "spatial": semantic_temporal.get("spatial", {"available": False, "matches": []}),
                "temporal": semantic_temporal.get("temporal", {"available": False}),
            }

        return {
            "query": task,
            "spatial": self._spatial_fallback(task, spatial_limit, errors),
            "temporal": self._temporal_fallback(errors),
        }

    def _spatial_fallback(
        self, task: str, spatial_limit: int, errors: list[str]
    ) -> dict[str, Any]:
        if self._spatial_memory is None:
            return {"available": False, "matches": []}
        if not task or spatial_limit == 0:
            return {"available": True, "matches": []}

        try:
            matches = self._spatial_memory.query_by_text(task, limit=spatial_limit)
        except Exception as exc:
            logger.warning("Failed to query spatial memory", exc_info=True)
            errors.append(f"spatial_memory: {exc}")
            return {"available": True, "matches": []}

        return {
            "available": True,
            "matches": [_summarize_spatial_match(match) for match in matches],
        }

    def _temporal_fallback(self, errors: list[str]) -> dict[str, Any]:
        if self._temporal_memory is None:
            return {"available": False}

        context: dict[str, Any] = {"available": True}
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

        return context

    def _pose_to_dict(self, pose: PoseStamped | None) -> dict[str, Any] | None:
        if pose is None:
            return None
        return {
            "frame_id": pose.frame_id,
            "timestamp": pose.ts,
            "position": {
                "x": round(pose.position.x, 3),
                "y": round(pose.position.y, 3),
                "z": round(pose.position.z, 3),
            },
            "yaw_degrees": round(math.degrees(pose.yaw), 1),
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


__all__ = ["_Go2StructuredWorldState"]
