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

from dimos.robot.unitree.go2.blueprints.layers.layer_3_agent_brain.context_evidence import (
    ContextEvidencePolicy,
    build_context_evidence,
)


def _metadata(task: str = "walk to the kitchen") -> dict[str, Any]:
    return {
        "task": task,
        "focus": "navigation",
        "runtime": {"mode": "hardware"},
        "robot_state": {
            "odom": {"position": {"x": 1.0, "y": 2.0}},
            "navigation": {"state": "idle", "goal_reached": True},
            "safety": {"body_pose_available": True},
        },
        "world_state": {
            "spatial": {
                "available": True,
                "matches": [
                    {
                        "distance": 0.2,
                        "metadata": {"label": "kitchen"},
                    }
                ],
            },
            "temporal": {"available": True, "rolling_summary": "Kitchen was seen."},
            "semantic_temporal": {"fused": {"available": True, "entry_count": 2}},
        },
        "skill_state": {
            "recent_outcomes": [{"skill_name": "navigate_with_text", "success": False}],
            "interface": {"available": True, "skill_count": 3},
        },
        "causal_state": {
            "recent_transitions": [{"skill_name": "navigate_with_text", "outcome_success": False}],
        },
        "world_model_state": {
            "available": True,
            "model": {"sample_count": 2},
            "recent_failure_count": 1,
            "top_failure_mode": "semantic_map_missing_target",
        },
    }


def test_default_context_evidence_preserves_v1_shape() -> None:
    evidence = build_context_evidence(_metadata(), ContextEvidencePolicy())

    assert evidence["version"] == "context_evidence.v1"
    assert evidence["selection_policy"] == "deterministic_source_coverage_v1"
    assert evidence["query"] == "walk to the kitchen"
    assert evidence["focus"] == "navigation"
    assert evidence["entry_count"] == len(evidence["entries"])
    assert "spatial_memory" in evidence["selected_sources"]
    assert "skill_outcome_store" in evidence["selected_sources"]


def test_policy_filters_low_relevance_spatial_evidence() -> None:
    metadata = _metadata()
    metadata["world_state"]["spatial"]["matches"][0]["distance"] = 0.95

    evidence = build_context_evidence(
        metadata,
        ContextEvidencePolicy(min_relevance_score=0.5),
    )

    assert "spatial_memory" not in evidence["selected_sources"]
    assert all(entry["source"] != "spatial_memory" for entry in evidence["entries"])


def test_policy_enforces_max_entries_deterministically() -> None:
    evidence = build_context_evidence(
        _metadata(),
        ContextEvidencePolicy(max_entries=4),
    )

    assert evidence["entry_count"] == 4
    assert [entry["source"] for entry in evidence["entries"]] == [
        "task",
        "runtime_config",
        "robot_state",
        "spatial_memory",
    ]


def test_motion_policy_retains_robot_state_evidence() -> None:
    evidence = build_context_evidence(
        _metadata(task="navigate to the hallway"),
        ContextEvidencePolicy(max_entries=3, require_robot_state_for_motion=True),
    )

    assert [entry["source"] for entry in evidence["entries"]] == [
        "task",
        "runtime_config",
        "robot_state",
    ]
