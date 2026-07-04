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

from dimos.robot.unitree.go2.blueprints.layers.layer_3_agent_brain.world_model_contract import (
    WORLD_MODEL_PREDICTION_SCHEMA,
    WORLD_MODEL_PROVIDER_SCHEMA,
    provider_contract,
    validate_world_model_prediction,
)


def test_provider_contract_is_versioned_and_dimos_native() -> None:
    contract = provider_contract(
        name="go2_causal_world_model",
        model_type="symbolic_causal_online_transition",
        capabilities=["predict_next_state", "record_intervention"],
        output_schemas={"prediction": WORLD_MODEL_PREDICTION_SCHEMA},
    )

    assert contract["schema"] == WORLD_MODEL_PROVIDER_SCHEMA
    assert contract["name"] == "go2_causal_world_model"
    assert contract["model_type"] == "symbolic_causal_online_transition"
    assert contract["output_schemas"]["prediction"] == WORLD_MODEL_PREDICTION_SCHEMA
    assert "predict_next_state" in contract["capabilities"]
    assert contract["provider"] == "dimos"


def test_validate_world_model_prediction_accepts_canonical_payload() -> None:
    prediction = {
        "schema": WORLD_MODEL_PREDICTION_SCHEMA,
        "goal": "go to the kitchen",
        "horizon": 1,
        "action": {"skill_name": "navigate_with_text", "domain": "navigation", "args": {}},
        "snapshot_summary": {"has_robot_state": True},
        "prediction": {
            "risk": "medium",
            "predicted_success": True,
            "score": 0.5,
            "confidence": "medium",
            "predicted_state_delta": {},
            "failure_modes": [],
            "reasons": [],
            "evidence": {},
        },
    }

    assert validate_world_model_prediction(prediction) == []


def test_validate_world_model_prediction_rejects_unversioned_payload() -> None:
    errors = validate_world_model_prediction(
        {
            "goal": "go to the kitchen",
            "horizon": 1,
            "action": {"skill_name": "navigate_with_text"},
            "prediction": {"risk": "medium"},
        }
    )

    assert "schema must be dimos.world_model_prediction.v1" in errors
