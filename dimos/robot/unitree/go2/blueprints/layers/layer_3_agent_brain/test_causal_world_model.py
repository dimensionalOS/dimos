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

from dimos.robot.unitree.go2.blueprints.layers.layer_3_agent_brain.causal_world_model import (
    _Go2CausalWorldModel,
)
from dimos.robot.unitree.go2.blueprints.layers.layer_3_agent_brain.skill_outcome_predictor import (
    _Go2SkillOutcomePredictor,
)
from dimos.robot.unitree.go2.blueprints.layers.layer_3_agent_brain.skill_outcome_store import (
    _Go2SkillOutcomeStore,
)


def test_record_causal_transition_infers_semantic_map_failure() -> None:
    model = _Go2CausalWorldModel()

    result = model.record_causal_transition(
        task="go to the kitchen",
        skill_name="navigate_with_text",
        args_json='{"query": "kitchen"}',
        before_context="Spatial memory: available, no relevant matches",
        prediction_json='{"risk": "medium", "failure_reasons": []}',
        outcome_json='{"success": false, "message": "No matching location found"}',
    )

    assert result.success is True
    transition = model.get_recent_transitions(limit=1)[0]
    assert transition["skill_name"] == "navigate_with_text"
    assert transition["outcome_success"] is False
    assert transition["inferred_cause"] == "semantic_map_missing_target"
    assert transition["domain"] == "navigation"


def test_summarize_causal_patterns_counts_repeated_failures() -> None:
    model = _Go2CausalWorldModel()

    for target in ("kitchen", "office"):
        model.record_causal_transition(
            task=f"go to the {target}",
            skill_name="navigate_with_text",
            args_json=f'{{"query": "{target}"}}',
            before_context="Spatial memory: available, no relevant matches",
            outcome_json='{"success": false, "message": "No matching location found"}',
        )

    summary = model.summarize_causal_patterns(skill_name="navigate_with_text")

    assert summary.success is True
    assert summary.metadata["patterns"][0]["cause"] == "semantic_map_missing_target"
    assert summary.metadata["patterns"][0]["count"] == 2


def test_record_causal_transition_can_use_latest_skill_outcome() -> None:
    store = _Go2SkillOutcomeStore()
    store.record_skill_outcome(
        skill_name="navigate_with_text",
        success=False,
        domain="navigation",
        message="No matching location found",
    )

    model = _Go2CausalWorldModel()
    model._skill_outcomes = store

    result = model.record_causal_transition(
        task="go to the kitchen",
        skill_name="navigate_with_text",
        args_json='{"query": "kitchen"}',
    )

    assert result.success is True
    transition = model.get_recent_transitions(limit=1)[0]
    assert transition["outcome_message"] == "No matching location found"
    assert transition["inferred_cause"] == "semantic_map_missing_target"


def test_record_causal_transition_rejects_invalid_json() -> None:
    model = _Go2CausalWorldModel()

    result = model.record_causal_transition(
        task="go to the kitchen",
        skill_name="navigate_with_text",
        args_json="[1, 2]",
    )

    assert result.success is False
    assert result.error_code == "INVALID_INPUT"


def test_predictor_uses_repeated_causal_failures() -> None:
    model = _Go2CausalWorldModel()
    for target in ("kitchen", "office"):
        model.record_causal_transition(
            task=f"go to the {target}",
            skill_name="navigate_with_text",
            args_json=f'{{"query": "{target}"}}',
            outcome_json='{"success": false, "message": "No matching location found"}',
        )

    predictor = _Go2SkillOutcomePredictor()
    predictor._causal_world_model = model

    result = predictor.predict_skill_outcome(
        "navigate_with_text",
        args_json='{"query": "kitchen"}',
        context="Robot pose: x=1.0, y=2.0",
    )

    assert result.success is True
    assert result.metadata["risk"] == "high"
    assert result.metadata["predicted_success"] is False
    assert any("causal cause" in reason for reason in result.metadata["failure_reasons"])
