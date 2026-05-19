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

from dimos.robot.unitree.go2.blueprints.layers.layer_3_agent_brain.skill_outcome_predictor import (
    _Go2SkillOutcomePredictor,
)
from dimos.robot.unitree.go2.blueprints.layers.layer_3_agent_brain.skill_outcome_store import (
    _Go2SkillOutcomeStore,
)


def test_record_and_summarize_skill_outcomes() -> None:
    store = _Go2SkillOutcomeStore()

    result = store.record_skill_outcome(
        skill_name="navigate_with_text",
        success=False,
        domain="navigation",
        error_code="EXECUTION_FAILED",
        message="No matching location",
        risk="high",
        recovery="call get_context",
    )

    assert result.success is True
    outcomes = store.get_recent_outcomes(limit=5)
    assert len(outcomes) == 1
    assert outcomes[0]["skill_name"] == "navigate_with_text"
    assert outcomes[0]["success"] is False

    summary = store.summarize_skill_outcomes(domain="navigation")
    assert summary.success is True
    assert summary.metadata["outcomes"][0]["error_code"] == "EXECUTION_FAILED"


def test_predictor_uses_recent_failures() -> None:
    store = _Go2SkillOutcomeStore()
    store.record_skill_outcome("navigate_with_text", False)
    store.record_skill_outcome("navigate_with_text", False)

    predictor = _Go2SkillOutcomePredictor()
    predictor._skill_outcomes = store

    result = predictor.predict_skill_outcome(
        "navigate_with_text",
        args_json='{"query": "kitchen"}',
        context="Robot pose: x=1.0, y=2.0",
    )

    assert result.success is True
    assert result.metadata["risk"] == "high"
    assert result.metadata["predicted_success"] is False
    assert "same skill failed repeatedly in recent outcomes" in result.metadata["failure_reasons"]


def test_predictor_rejects_invalid_args_json() -> None:
    predictor = _Go2SkillOutcomePredictor()

    result = predictor.predict_skill_outcome("navigate_with_text", args_json="[1, 2]")

    assert result.success is False
    assert result.error_code == "INVALID_INPUT"


def test_predictor_requires_skill_name() -> None:
    predictor = _Go2SkillOutcomePredictor()

    result = predictor.predict_skill_outcome(" ")

    assert result.success is False
    assert result.error_code == "INVALID_INPUT"
