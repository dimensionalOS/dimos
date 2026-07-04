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

WORLD_MODEL_PROVIDER_SCHEMA = "dimos.world_model_provider.v1"
WORLD_MODEL_PREDICTION_SCHEMA = "dimos.world_model_prediction.v1"
WORLD_MODEL_DASHBOARD_SCHEMA = "dimos.world_model_dashboard_state.v1"


def provider_contract(
    name: str,
    model_type: str,
    capabilities: list[str] | tuple[str, ...] | None = None,
    output_schemas: dict[str, str] | None = None,
) -> dict[str, Any]:
    """Return a versioned DimOS-native contract for a world-model provider."""
    schemas = dict(output_schemas or {})
    schemas.setdefault("prediction", WORLD_MODEL_PREDICTION_SCHEMA)
    schemas.setdefault("dashboard", WORLD_MODEL_DASHBOARD_SCHEMA)
    return {
        "schema": WORLD_MODEL_PROVIDER_SCHEMA,
        "name": name.strip(),
        "model_type": model_type.strip(),
        "capabilities": sorted(
            {capability.strip() for capability in capabilities or () if capability.strip()}
        ),
        "output_schemas": schemas,
        "provider": "dimos",
    }


def validate_world_model_prediction(prediction: dict[str, Any]) -> list[str]:
    """Return contract errors for a world-model prediction payload."""
    errors: list[str] = []
    if prediction.get("schema") != WORLD_MODEL_PREDICTION_SCHEMA:
        errors.append(f"schema must be {WORLD_MODEL_PREDICTION_SCHEMA}")

    if not isinstance(prediction.get("goal"), str):
        errors.append("goal must be a string")
    if not isinstance(prediction.get("horizon"), int):
        errors.append("horizon must be an integer")

    action = prediction.get("action")
    if not isinstance(action, dict):
        errors.append("action must be a JSON object")
    elif not isinstance(action.get("skill_name"), str) or not action.get("skill_name"):
        errors.append("action.skill_name must be a non-empty string")

    if not isinstance(prediction.get("snapshot_summary"), dict):
        errors.append("snapshot_summary must be a JSON object")

    body = prediction.get("prediction")
    if not isinstance(body, dict):
        errors.append("prediction must be a JSON object")
        return errors

    if body.get("risk") not in {"low", "medium", "high"}:
        errors.append("prediction.risk must be low, medium, or high")
    if not isinstance(body.get("predicted_success"), bool):
        errors.append("prediction.predicted_success must be a boolean")
    score = body.get("score")
    if not isinstance(score, int | float) or isinstance(score, bool):
        errors.append("prediction.score must be a number")
    if body.get("confidence") not in {"low", "medium", "high"}:
        errors.append("prediction.confidence must be low, medium, or high")
    if not isinstance(body.get("predicted_state_delta"), dict):
        errors.append("prediction.predicted_state_delta must be a JSON object")
    if not isinstance(body.get("failure_modes"), list):
        errors.append("prediction.failure_modes must be a list")
    if not isinstance(body.get("reasons"), list):
        errors.append("prediction.reasons must be a list")
    if not isinstance(body.get("evidence"), dict):
        errors.append("prediction.evidence must be a JSON object")
    return errors


__all__ = [
    "WORLD_MODEL_DASHBOARD_SCHEMA",
    "WORLD_MODEL_PREDICTION_SCHEMA",
    "WORLD_MODEL_PROVIDER_SCHEMA",
    "provider_contract",
    "validate_world_model_prediction",
]
