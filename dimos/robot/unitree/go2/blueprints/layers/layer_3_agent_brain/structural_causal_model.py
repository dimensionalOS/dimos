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


class StructuralCausalModel:
    """Hand-authored symbolic SCM for Go2 Layer 3 action decisions."""

    model_type = "structural_causal_model"

    def __init__(self) -> None:
        self._edges = [
            ["target_query_present", "target_resolvability"],
            ["spatial_available", "spatial_has_matches"],
            ["spatial_has_matches", "target_resolvability"],
            ["visual_detection", "target_resolvability"],
            ["odom_ready", "motion_safety"],
            ["obstacle_clearance", "motion_safety"],
            ["target_resolvability", "navigation_success"],
            ["motion_safety", "navigation_success"],
        ]

    def explain(
        self,
        snapshot: dict[str, Any],
        action: dict[str, Any],
        causal_attribution: dict[str, Any] | None = None,
    ) -> dict[str, Any]:
        variables = self.evaluate(snapshot, action)
        risk_features = _risk_features(causal_attribution or {})
        return {
            "type": self.model_type,
            "assumption": (
                "symbolic SCM; graph structure is hand-authored and updated from "
                "observed/intervened outcomes"
            ),
            "variables": variables,
            "edges": self.edges(),
            "counterfactuals": self.counterfactuals(variables, action, risk_features),
        }

    def evaluate(self, snapshot: dict[str, Any], action: dict[str, Any]) -> dict[str, Any]:
        skill_name = str(action.get("skill_name") or "")
        args = action.get("args") if isinstance(action.get("args"), dict) else {}
        robot_state = (
            snapshot.get("robot_state") if isinstance(snapshot.get("robot_state"), dict) else {}
        )
        memory_state = (
            snapshot.get("memory_state") if isinstance(snapshot.get("memory_state"), dict) else {}
        )
        spatial = (
            memory_state.get("spatial") if isinstance(memory_state.get("spatial"), dict) else {}
        )
        semantic_temporal_map = (
            snapshot.get("semantic_temporal_map")
            if isinstance(snapshot.get("semantic_temporal_map"), dict)
            else {}
        )
        safety = robot_state.get("safety") if isinstance(robot_state.get("safety"), dict) else {}

        target_query_present = bool(str(args.get("query") or args.get("target") or "").strip())
        spatial_available = bool(spatial.get("available"))
        matches = spatial.get("matches") if isinstance(spatial.get("matches"), list) else []
        spatial_has_matches = bool(matches)
        visual_detection = _has_visual_detection(semantic_temporal_map)
        odom_ready = bool(robot_state.get("odom"))
        obstacle_clearance = safety.get("obstacle_clearance") is not False
        target_resolvability = bool(
            target_query_present and (spatial_has_matches or visual_detection)
        )
        motion_safety = bool(odom_ready and obstacle_clearance)

        is_navigation = skill_name == "navigate_with_text"
        navigation_success = (
            bool(target_resolvability and motion_safety) if is_navigation else motion_safety
        )
        return {
            "skill_name": skill_name,
            "target_query_present": target_query_present,
            "spatial_available": spatial_available,
            "spatial_has_matches": spatial_has_matches,
            "visual_detection": visual_detection,
            "target_resolvability": target_resolvability,
            "odom_ready": odom_ready,
            "obstacle_clearance": obstacle_clearance,
            "motion_safety": motion_safety,
            "navigation_success": navigation_success,
        }

    def counterfactuals(
        self,
        variables: dict[str, Any],
        action: dict[str, Any],
        risk_features: list[str],
    ) -> list[dict[str, Any]]:
        counterfactuals: list[dict[str, Any]] = []
        skill_name = str(action.get("skill_name") or "")

        if skill_name == "navigate_with_text" and (
            variables.get("spatial_has_matches") is False
            or "spatial_has_matches:False" in risk_features
        ):
            counterfactuals.append(
                {
                    "intervention": "do(spatial_has_matches=True)",
                    "target_variable": "spatial_has_matches",
                    "current_value": variables.get("spatial_has_matches"),
                    "counterfactual_value": True,
                    "expected_changes": {
                        "spatial_has_matches": True,
                        "target_resolvability": bool(variables.get("target_query_present")),
                        "navigation_success": bool(
                            variables.get("target_query_present") and variables.get("motion_safety")
                        ),
                    },
                    "recommended_action": (
                        "Use look_out_for or tag the target before semantic navigation."
                    ),
                }
            )

        if variables.get("visual_detection") is False and skill_name == "navigate_with_text":
            counterfactuals.append(
                {
                    "intervention": "do(visual_detection=True)",
                    "target_variable": "visual_detection",
                    "current_value": False,
                    "counterfactual_value": True,
                    "expected_changes": {
                        "visual_detection": True,
                        "target_resolvability": bool(variables.get("target_query_present")),
                    },
                    "recommended_action": "Run a perception query for the target.",
                }
            )

        if variables.get("odom_ready") is False or "has_odom:False" in risk_features:
            counterfactuals.append(
                {
                    "intervention": "do(odom_ready=True)",
                    "target_variable": "odom_ready",
                    "current_value": variables.get("odom_ready"),
                    "counterfactual_value": True,
                    "expected_changes": {
                        "motion_safety": bool(variables.get("obstacle_clearance")),
                        "navigation_success": bool(
                            variables.get("target_resolvability")
                            and variables.get("obstacle_clearance")
                        ),
                    },
                    "recommended_action": "Wait for odometry before issuing motion.",
                }
            )

        if variables.get("target_query_present") is False:
            counterfactuals.append(
                {
                    "intervention": "do(target_query_present=True)",
                    "target_variable": "target_query_present",
                    "current_value": False,
                    "counterfactual_value": True,
                    "expected_changes": {
                        "target_query_present": True,
                        "target_resolvability": bool(
                            variables.get("spatial_has_matches")
                            or variables.get("visual_detection")
                        ),
                    },
                    "recommended_action": "Fill the missing navigation target argument.",
                }
            )

        return counterfactuals

    def snapshot(self) -> dict[str, Any]:
        return {
            "type": self.model_type,
            "edges": self.edges(),
            "node_count": len({node for edge in self._edges for node in edge}),
            "edge_count": len(self._edges),
        }

    def edges(self) -> list[list[str]]:
        return [list(edge) for edge in self._edges]


def _has_visual_detection(semantic_temporal_map: dict[str, Any]) -> bool:
    fused = (
        semantic_temporal_map.get("fused")
        if isinstance(semantic_temporal_map.get("fused"), dict)
        else {}
    )
    if fused.get("entry_count") or fused.get("spatial_match_count"):
        return True
    for key in ("detections", "observations", "entries"):
        value = semantic_temporal_map.get(key)
        if isinstance(value, list) and value:
            return True
    return False


def _risk_features(causal_attribution: dict[str, Any]) -> list[str]:
    return [
        str(factor.get("feature") or "")
        for factor in causal_attribution.get("risk_factors") or []
        if factor.get("feature")
    ]


__all__ = ["StructuralCausalModel"]
