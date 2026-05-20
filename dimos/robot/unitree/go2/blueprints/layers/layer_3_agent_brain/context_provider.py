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
from dimos.robot.unitree.go2.blueprints.layers.layer_3_agent_brain.causal_world_model import (
    CausalWorldModelSpec,
)
from dimos.robot.unitree.go2.blueprints.layers.layer_3_agent_brain.skill_outcome_store import (
    SkillOutcomeStoreSpec,
)
from dimos.robot.unitree.go2.blueprints.layers.layer_4_world_state.world_state_spec import (
    WorldStateSpec,
)
from dimos.robot.unitree.go2.blueprints.layers.layer_5_skill_interface.skill_interface_spec import (
    SkillInterfaceSpec,
)
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class _Go2ContextProvider(Module):
    """Layer 3 context aggregator for the Go2 agent brain."""

    _world_state: WorldStateSpec | None = None
    _spatial_memory: SpatialMemorySpec | None = None
    _temporal_memory: TemporalMemorySpec | None = None
    _navigation: NavigationInterfaceSpec | None = None
    _skill_outcomes: SkillOutcomeStoreSpec | None = None
    _causal_world_model: CausalWorldModelSpec | None = None
    _skill_interface: SkillInterfaceSpec | None = None
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
        world_snapshot = self._layer4_snapshot(task, spatial_limit, errors)
        metadata: dict[str, Any] = {
            "task": task,
            "focus": focus,
            "sources": self._source_status(world_snapshot),
            "runtime": self._runtime_context(world_snapshot),
            "robot_state": self._robot_context(errors, world_snapshot),
            "world_state": self._world_context(task, spatial_limit, errors, world_snapshot),
            "skill_state": self._skill_context(errors),
            "causal_state": self._causal_context(errors),
            "external_context": {
                "available": False,
                "reason": "External context sources are not wired into ContextProvider yet.",
            },
        }
        if errors:
            metadata["errors"] = errors

        message = self._format_message(metadata)
        return SkillResult(success=True, message=message, metadata=_to_jsonable(metadata))

    def _source_status(self, world_snapshot: dict[str, Any] | None) -> dict[str, bool]:
        layer4_sources = (world_snapshot or {}).get("sources", {})
        return {
            "task": True,
            "structured_world_state": self._world_state is not None,
            "spatial_memory": self._spatial_memory is not None
            or bool(layer4_sources.get("spatial_memory")),
            "temporal_memory": self._temporal_memory is not None
            or bool(layer4_sources.get("temporal_memory")),
            "odom": self._latest_odom is not None or bool(layer4_sources.get("odom")),
            "navigation": self._navigation is not None or bool(layer4_sources.get("navigation")),
            "skill_outcomes": self._skill_outcomes is not None,
            "causal_world_model": self._causal_world_model is not None,
            "skill_interface": self._skill_interface is not None,
            "runtime": True,
        }

    def _layer4_snapshot(
        self, task: str, spatial_limit: int, errors: list[str]
    ) -> dict[str, Any] | None:
        if self._world_state is None:
            return None
        try:
            return self._world_state.get_world_snapshot(task=task, spatial_limit=spatial_limit)
        except Exception as exc:
            logger.warning("Failed to read Layer 4 world state", exc_info=True)
            errors.append(f"world_state: {exc}")
            return None

    def _runtime_context(self, world_snapshot: dict[str, Any] | None) -> dict[str, Any]:
        if world_snapshot is not None and isinstance(world_snapshot.get("runtime"), dict):
            return world_snapshot["runtime"]

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

    def _robot_context(
        self, errors: list[str], world_snapshot: dict[str, Any] | None
    ) -> dict[str, Any]:
        if world_snapshot is not None and isinstance(world_snapshot.get("robot_state"), dict):
            return world_snapshot["robot_state"]

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

    def _world_context(
        self,
        task: str,
        spatial_limit: int,
        errors: list[str],
        world_snapshot: dict[str, Any] | None,
    ) -> dict[str, Any]:
        if world_snapshot is not None:
            memory_state = world_snapshot.get("memory_state") or {}
            return {
                "source": "structured_world_state",
                "spatial": memory_state.get("spatial", {"available": False, "matches": []}),
                "temporal": memory_state.get("temporal", {"available": False}),
                "semantic_temporal": world_snapshot.get("semantic_temporal_map", {}),
                "snapshot": world_snapshot,
            }

        return {
            "spatial": self._spatial_context(task, spatial_limit, errors),
            "temporal": self._temporal_context(errors),
        }

    def _skill_context(self, errors: list[str]) -> dict[str, Any]:
        skill_interface = self._skill_interface_context(errors)
        if self._skill_outcomes is None:
            return {
                "available": False,
                "recent_outcomes": [],
                "interface": skill_interface,
            }
        try:
            recent = self._skill_outcomes.get_recent_outcomes(limit=5)
        except Exception as exc:
            logger.warning("Failed to read skill-outcome context", exc_info=True)
            errors.append(f"skill_outcomes: {exc}")
            return {
                "available": True,
                "recent_outcomes": [],
                "interface": skill_interface,
            }
        return {
            "available": True,
            "recent_outcomes": recent,
            "interface": skill_interface,
        }

    def _skill_interface_context(self, errors: list[str]) -> dict[str, Any]:
        if self._skill_interface is None:
            return {"available": False, "skills": []}
        try:
            return self._skill_interface.get_skill_interface_snapshot()
        except Exception as exc:
            logger.warning("Failed to read skill-interface context", exc_info=True)
            errors.append(f"skill_interface: {exc}")
            return {"available": True, "skills": []}

    def _causal_context(self, errors: list[str]) -> dict[str, Any]:
        if self._causal_world_model is None:
            return {"available": False, "recent_transitions": []}
        try:
            recent = self._causal_world_model.get_recent_transitions(limit=5)
        except Exception as exc:
            logger.warning("Failed to read causal-world context", exc_info=True)
            errors.append(f"causal_world_model: {exc}")
            return {"available": True, "recent_transitions": []}
        return {"available": True, "recent_transitions": recent}

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

        connection = robot_state.get("connection")
        if connection:
            lines.append(
                "Connection: "
                f"mode={connection.get('mode')}, available={connection.get('available')}"
            )

        safety = robot_state.get("safety")
        if safety:
            lines.append(
                "Safety: "
                f"body_pose_available={safety.get('body_pose_available')}, "
                f"obstacle_avoidance={safety.get('obstacle_avoidance_configured')}"
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

        skill_state = metadata.get("skill_state", {})
        recent_outcomes = skill_state.get("recent_outcomes") or []
        if recent_outcomes:
            failures = [outcome for outcome in recent_outcomes if not outcome.get("success")]
            lines.append(
                f"Skill outcomes: {len(recent_outcomes)} recent, {len(failures)} failure(s)"
            )
        elif skill_state.get("available"):
            lines.append("Skill outcomes: available, no recorded outcomes")
        else:
            lines.append("Skill outcomes: unavailable")

        skill_interface = skill_state.get("interface") or {}
        if skill_interface.get("available"):
            lines.append(
                f"Skill interface: {skill_interface.get('skill_count', 0)} contract(s)"
            )
        else:
            lines.append("Skill interface: unavailable")

        causal_state = metadata.get("causal_state", {})
        recent_transitions = causal_state.get("recent_transitions") or []
        if recent_transitions:
            failures = [
                transition
                for transition in recent_transitions
                if transition.get("outcome_success") is False
            ]
            lines.append(
                f"Causal memory: {len(recent_transitions)} recent, "
                f"{len(failures)} failure transition(s)"
            )
        elif causal_state.get("available"):
            lines.append("Causal memory: available, no recorded transitions")
        else:
            lines.append("Causal memory: unavailable")

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
