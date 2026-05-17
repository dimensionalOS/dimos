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

from dimos.agents.annotation import skill
from dimos.agents.skill_result import SkillResult
from dimos.core.core import rpc
from dimos.core.global_config import global_config
from dimos.core.module import Module
from dimos.core.stream import In
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.navigation.navigation_spec import NavigationInterfaceSpec
from dimos.perception.spatial_memory_spec import SpatialMemorySpec
from dimos.perception.temporal_memory_spec import TemporalMemorySpec
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class _Go2ContextProvider(Module):
    """Layer 3 context aggregator for the Go2 agent brain."""

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

    @skill
    def get_context(self, task: str, focus: str = "", spatial_limit: int = 3) -> SkillResult:
        """Get compact context for the agent's next decision.

        This tool gathers task, world-state, robot-state, and runtime context
        without planning or executing robot actions. Use it before choosing
        navigation, perception, speech, or recovery tools.

        Args:
            task: The current user goal or agent subtask.
            focus: Optional area to emphasize, such as navigation, memory,
                safety, perception, or recovery.
            spatial_limit: Maximum number of spatial-memory matches to include.
        """
        task = task.strip()
        focus = focus.strip()
        if not task:
            return SkillResult.fail("INVALID_INPUT", "task is required")

        spatial_limit = max(0, min(spatial_limit, 10))
        errors: list[str] = []
        metadata: dict[str, Any] = {
            "task": task,
            "focus": focus,
            "sources": self._source_status(),
            "runtime": self._runtime_context(),
            "robot_state": self._robot_context(errors),
            "world_state": self._world_context(task, spatial_limit, errors),
            "skill_state": {
                "available": False,
                "reason": "No shared skill-outcome store is wired into ContextProvider yet.",
            },
            "external_context": {
                "available": False,
                "reason": "External context sources are not wired into ContextProvider yet.",
            },
        }
        if errors:
            metadata["errors"] = errors

        message = self._format_message(metadata)
        return SkillResult(success=True, message=message, metadata=_to_jsonable(metadata))

    def _source_status(self) -> dict[str, bool]:
        return {
            "task": True,
            "spatial_memory": self._spatial_memory is not None,
            "temporal_memory": self._temporal_memory is not None,
            "odom": self._latest_odom is not None,
            "navigation": self._navigation is not None,
            "runtime": True,
        }

    def _runtime_context(self) -> dict[str, Any]:
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

    def _robot_context(self, errors: list[str]) -> dict[str, Any]:
        context: dict[str, Any] = {
            "odom": self._pose_to_dict(self._latest_odom),
            "navigation": None,
        }

        if self._navigation is None:
            return context

        try:
            state = self._navigation.get_state()
            context["navigation"] = {
                "state": getattr(state, "value", str(state)),
                "goal_reached": self._navigation.is_goal_reached(),
            }
        except Exception as exc:
            logger.warning("Failed to read navigation context", exc_info=True)
            errors.append(f"navigation: {exc}")

        return context

    def _world_context(self, task: str, spatial_limit: int, errors: list[str]) -> dict[str, Any]:
        return {
            "spatial": self._spatial_context(task, spatial_limit, errors),
            "temporal": self._temporal_context(errors),
        }

    def _spatial_context(
        self, task: str, spatial_limit: int, errors: list[str]
    ) -> dict[str, Any]:
        if self._spatial_memory is None:
            return {"available": False, "matches": []}
        if spatial_limit == 0:
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

    def _temporal_context(self, errors: list[str]) -> dict[str, Any]:
        if self._temporal_memory is None:
            return {"available": False}

        context: dict[str, Any] = {"available": True}
        try:
            context["rolling_summary"] = self._temporal_memory.get_rolling_summary()
        except Exception as exc:
            logger.warning("Failed to read temporal rolling summary", exc_info=True)
            errors.append(f"temporal_memory.summary: {exc}")

        try:
            state = self._temporal_memory.get_state()
            context["state"] = _to_jsonable(state)
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

    def _format_message(self, metadata: dict[str, Any]) -> str:
        task = metadata["task"]
        focus = metadata["focus"] or "general"
        runtime = metadata["runtime"]
        robot_state = metadata["robot_state"]
        world_state = metadata["world_state"]

        lines = [
            f"Task: {task}",
            f"Focus: {focus}",
            f"Runtime: {runtime['mode']}",
        ]

        odom = robot_state.get("odom")
        if odom:
            pos = odom["position"]
            lines.append(
                "Robot pose: "
                f"x={pos['x']}, y={pos['y']}, z={pos['z']}, yaw={odom['yaw_degrees']}deg"
            )
        else:
            lines.append("Robot pose: unavailable")

        navigation = robot_state.get("navigation")
        if navigation:
            lines.append(
                "Navigation: "
                f"state={navigation['state']}, goal_reached={navigation['goal_reached']}"
            )

        spatial = world_state.get("spatial", {})
        matches = spatial.get("matches") or []
        if matches:
            lines.append(f"Spatial memory: {len(matches)} relevant match(es)")
        elif spatial.get("available"):
            lines.append("Spatial memory: available, no relevant matches")
        else:
            lines.append("Spatial memory: unavailable")

        temporal = world_state.get("temporal", {})
        summary = temporal.get("rolling_summary")
        if summary:
            lines.append(f"Temporal summary: {summary}")
        elif temporal.get("available"):
            lines.append("Temporal memory: available, no rolling summary")
        else:
            lines.append("Temporal memory: unavailable")

        if metadata.get("errors"):
            lines.append(f"Context warnings: {metadata['errors']}")

        return "\n".join(lines)


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


__all__ = ["_Go2ContextProvider"]
