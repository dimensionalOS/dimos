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
from dimos.robot.unitree.go2.blueprints.layers.layer_3_agent_brain.world_model_contract import (
    WORLD_MODEL_DASHBOARD_SCHEMA,
    WORLD_MODEL_PREDICTION_SCHEMA,
    WORLD_MODEL_PROVIDER_SCHEMA,
    validate_world_model_prediction,
)


def _stop_modules(*modules: object) -> None:
    for module in modules:
        stop = getattr(module, "stop", None)
        if stop is not None:
            stop()


class StubWebsocketVis:
    def __init__(self) -> None:
        self.states: list[dict] = []

    def set_world_model_state(self, state: dict) -> dict:
        self.states.append(state)
        return {"available": True}


def test_record_causal_transition_infers_semantic_map_failure() -> None:
    model = _Go2CausalWorldModel()
    try:
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
    finally:
        _stop_modules(model)


def test_summarize_causal_patterns_counts_repeated_failures() -> None:
    model = _Go2CausalWorldModel()
    try:
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
    finally:
        _stop_modules(model)


def test_record_causal_transition_can_use_latest_skill_outcome() -> None:
    store = _Go2SkillOutcomeStore()
    model = _Go2CausalWorldModel()
    try:
        store.record_skill_outcome(
            skill_name="navigate_with_text",
            success=False,
            domain="navigation",
            message="No matching location found",
            task="go to the kitchen",
        )

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
    finally:
        _stop_modules(model, store)


def test_record_causal_transition_rejects_invalid_json() -> None:
    model = _Go2CausalWorldModel()
    try:
        result = model.record_causal_transition(
            task="go to the kitchen",
            skill_name="navigate_with_text",
            args_json="[1, 2]",
        )

        assert result.success is False
        assert result.error_code == "INVALID_INPUT"
    finally:
        _stop_modules(model)


def test_record_causal_transition_preserves_legacy_positional_arguments() -> None:
    model = _Go2CausalWorldModel()
    try:
        result = model.record_causal_transition(
            "go to the kitchen",
            "navigate_with_text",
            '{"query": "kitchen"}',
            "Spatial memory: available, no relevant matches",
            "Navigation returned no matching location",
            '{"risk": "medium", "failure_reasons": []}',
            '{"success": false, "message": "No matching location found"}',
            "navigation",
        )

        assert result.success is True
        transition = model.get_recent_transitions(limit=1)[0]
        assert transition["before_context"] == "Spatial memory: available, no relevant matches"
        assert transition["after_context"] == "Navigation returned no matching location"
        assert transition["before_state"] == {}
    finally:
        _stop_modules(model)


def test_predictor_uses_repeated_causal_failures() -> None:
    model = _Go2CausalWorldModel()
    predictor = _Go2SkillOutcomePredictor()
    try:
        for target in ("kitchen", "office"):
            model.record_causal_transition(
                task=f"go to the {target}",
                skill_name="navigate_with_text",
                args_json=f'{{"query": "{target}"}}',
                outcome_json='{"success": false, "message": "No matching location found"}',
            )

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
    finally:
        _stop_modules(predictor, model)


def test_predict_next_state_uses_world_snapshot_and_recent_failures() -> None:
    model = _Go2CausalWorldModel()
    try:
        for target in ("kitchen", "office"):
            model.record_causal_transition(
                task=f"go to the {target}",
                skill_name="navigate_with_text",
                args_json=f'{{"query": "{target}"}}',
                before_state_json=(
                    '{"memory_state": {"spatial": {"available": true, "matches": []}}}'
                ),
                after_state_json=(
                    '{"memory_state": {"spatial": {"available": true, "matches": []}}}'
                ),
                outcome_json='{"success": false, "message": "No matching location found"}',
            )

        prediction = model.predict_next_state(
            snapshot_json=(
                '{"robot_state": {"navigation": {"state": "idle"}}, '
                '"memory_state": {"spatial": {"available": true, "matches": []}}}'
            ),
            action_json='{"skill_name": "navigate_with_text", "args": {"query": "kitchen"}}',
            goal="go to the kitchen",
        )

        assert prediction["action"]["skill_name"] == "navigate_with_text"
        assert prediction["schema"] == WORLD_MODEL_PREDICTION_SCHEMA
        assert validate_world_model_prediction(prediction) == []
        assert prediction["prediction"]["risk"] == "high"
        assert prediction["prediction"]["confidence"] == "medium"
        assert "navigation.state" in prediction["prediction"]["predicted_state_delta"]
        assert any(
            failure["cause"] == "semantic_map_missing_target"
            for failure in prediction["prediction"]["failure_modes"]
        )
        assert prediction["prediction"]["score"] < 0.5
    finally:
        _stop_modules(model)


def test_world_model_provider_contract_is_explicit() -> None:
    model = _Go2CausalWorldModel()
    try:
        contract = model.get_provider_contract()

        assert contract["schema"] == WORLD_MODEL_PROVIDER_SCHEMA
        assert contract["name"] == "go2_causal_world_model"
        assert contract["model_type"] == "symbolic_causal_online_transition"
        assert "predict_next_state" in contract["capabilities"]
        assert contract["output_schemas"]["prediction"] == WORLD_MODEL_PREDICTION_SCHEMA
        assert model.get_model_state()["provider_contract"] == contract
    finally:
        _stop_modules(model)


def test_predict_next_state_uses_online_transition_model() -> None:
    model = _Go2CausalWorldModel()
    snapshot_json = (
        '{"robot_state": {"odom": {"position": {"x": 1.0, "y": 2.0}}, '
        '"navigation": {"state": "idle"}}, '
        '"memory_state": {"spatial": {"available": true, "matches": [{"id": "kitchen"}]}, '
        '"temporal": {"available": true}}}'
    )
    try:
        for _ in range(2):
            model.record_causal_transition(
                task="go to the kitchen",
                skill_name="navigate_with_text",
                args_json='{"query": "kitchen"}',
                before_state_json=snapshot_json,
                after_state_json=(
                    '{"robot_state": {"navigation": {"state": "goal_reached"}}, '
                    '"memory_state": {"spatial": {"available": true, "matches": [{"id": "kitchen"}]}}}'
                ),
                outcome_json='{"success": true, "message": "goal reached"}',
            )

        prediction = model.predict_next_state(
            snapshot_json=snapshot_json,
            action_json='{"skill_name": "navigate_with_text", "args": {"query": "kitchen"}}',
            goal="go to the kitchen",
        )

        learned_model = prediction["prediction"]["model"]
        assert learned_model["type"] == "online_transition_outcome"
        assert learned_model["sample_count"] == 2
        assert learned_model["success_probability"] > 0.5
        assert prediction["prediction"]["score"] > 0.5
    finally:
        _stop_modules(model)


def test_predict_next_state_reports_causal_attribution_for_missing_spatial_evidence() -> None:
    model = _Go2CausalWorldModel()
    missing_snapshot_json = (
        '{"robot_state": {"odom": {"position": {"x": 1.0, "y": 2.0}}, '
        '"navigation": {"state": "idle"}}, '
        '"memory_state": {"spatial": {"available": true, "matches": []}, '
        '"temporal": {"available": true}}}'
    )
    matched_snapshot_json = (
        '{"robot_state": {"odom": {"position": {"x": 1.0, "y": 2.0}}, '
        '"navigation": {"state": "idle"}}, '
        '"memory_state": {"spatial": {"available": true, "matches": [{"id": "kitchen"}]}, '
        '"temporal": {"available": true}}}'
    )
    try:
        for _ in range(3):
            model.record_causal_transition(
                task="go to the kitchen",
                skill_name="navigate_with_text",
                args_json='{"query": "kitchen"}',
                before_state_json=missing_snapshot_json,
                after_state_json=missing_snapshot_json,
                outcome_json='{"success": false, "message": "No matching location found"}',
            )
            model.record_causal_transition(
                task="go to the kitchen",
                skill_name="navigate_with_text",
                args_json='{"query": "kitchen"}',
                before_state_json=matched_snapshot_json,
                after_state_json=(
                    '{"robot_state": {"navigation": {"state": "goal_reached"}}, '
                    '"memory_state": {"spatial": {"available": true, '
                    '"matches": [{"id": "kitchen"}]}}}'
                ),
                outcome_json='{"success": true, "message": "goal reached"}',
            )

        prediction = model.predict_next_state(
            snapshot_json=missing_snapshot_json,
            action_json='{"skill_name": "navigate_with_text", "args": {"query": "kitchen"}}',
            goal="go to the kitchen",
        )

        attribution = prediction["prediction"]["causal_attribution"]
        assert attribution["method"] == "observational_feature_effect"
        assert attribution["risk_factors"][0]["feature"] == "spatial_has_matches:False"
        assert attribution["risk_factors"][0]["effect_on_success"] < 0
        assert any("perception" in suggestion for suggestion in attribution["interventions"])
    finally:
        _stop_modules(model)


def test_record_intervention_log_is_returned_and_informs_prediction() -> None:
    model = _Go2CausalWorldModel()
    missing_snapshot_json = (
        '{"robot_state": {"odom": {"position": {"x": 1.0, "y": 2.0}}, '
        '"navigation": {"state": "idle"}}, '
        '"memory_state": {"spatial": {"available": true, "matches": []}, '
        '"temporal": {"available": true}}}'
    )
    matched_snapshot_json = (
        '{"robot_state": {"odom": {"position": {"x": 1.0, "y": 2.0}}, '
        '"navigation": {"state": "idle"}}, '
        '"memory_state": {"spatial": {"available": true, "matches": [{"id": "kitchen"}]}, '
        '"temporal": {"available": true}}}'
    )
    try:
        for _ in range(2):
            model.record_causal_transition(
                task="go to the kitchen",
                skill_name="navigate_with_text",
                args_json='{"query": "kitchen"}',
                before_state_json=missing_snapshot_json,
                after_state_json=missing_snapshot_json,
                outcome_json='{"success": false, "message": "No matching location found"}',
            )
            model.record_causal_transition(
                task="go to the kitchen",
                skill_name="navigate_with_text",
                args_json='{"query": "kitchen"}',
                before_state_json=matched_snapshot_json,
                after_state_json=matched_snapshot_json,
                outcome_json='{"success": true, "message": "goal reached"}',
            )

        result = model.record_intervention(
            task="go to the kitchen",
            intervention_name="look_out_for_then_tag_target",
            target_variable="spatial_has_matches",
            before_value_json="false",
            after_value_json="true",
            action_json='{"skill_name": "look_out_for", "args": {"description_of_things": ["kitchen"]}}',
            snapshot_before_json=missing_snapshot_json,
            snapshot_after_json=matched_snapshot_json,
            outcome_json='{"success": true, "message": "target tagged"}',
            causal_hypothesis="spatial_has_matches:False blocks semantic navigation",
        )

        assert result.success is True
        interventions = model.get_intervention_log(limit=1)
        assert interventions[0]["intervention_name"] == "look_out_for_then_tag_target"
        assert interventions[0]["target_variable"] == "spatial_has_matches"

        prediction = model.predict_next_state(
            snapshot_json=missing_snapshot_json,
            action_json='{"skill_name": "navigate_with_text", "args": {"query": "kitchen"}}',
            goal="go to the kitchen",
        )

        evidence = prediction["prediction"]["intervention_evidence"]
        assert evidence["suggestions"][0]["intervention_name"] == "look_out_for_then_tag_target"
        assert evidence["suggestions"][0]["target_variable"] == "spatial_has_matches"
    finally:
        _stop_modules(model)


def test_predict_next_state_includes_scm_counterfactuals() -> None:
    model = _Go2CausalWorldModel()
    snapshot_json = (
        '{"robot_state": {"odom": {"position": {"x": 1.0, "y": 2.0}}, '
        '"navigation": {"state": "idle"}}, '
        '"memory_state": {"spatial": {"available": true, "matches": []}, '
        '"temporal": {"available": true}}}'
    )
    try:
        prediction = model.predict_next_state(
            snapshot_json=snapshot_json,
            action_json='{"skill_name": "navigate_with_text", "args": {"query": "kitchen"}}',
            goal="go to the kitchen",
        )

        scm = prediction["prediction"]["structural_causal_model"]
        assert scm["variables"]["spatial_has_matches"] is False
        assert scm["variables"]["target_resolvability"] is False
        assert ["spatial_has_matches", "target_resolvability"] in scm["edges"]
        assert any(
            counterfactual["intervention"] == "do(spatial_has_matches=True)"
            and counterfactual["expected_changes"]["target_resolvability"] is True
            for counterfactual in scm["counterfactuals"]
        )
    finally:
        _stop_modules(model)


def test_world_model_state_persists_to_json_file(tmp_path) -> None:
    state_path = tmp_path / "world_model_state.json"
    missing_snapshot_json = (
        '{"robot_state": {"odom": {"position": {"x": 1.0, "y": 2.0}}, '
        '"navigation": {"state": "idle"}}, '
        '"memory_state": {"spatial": {"available": true, "matches": []}, '
        '"temporal": {"available": true}}}'
    )
    matched_snapshot_json = (
        '{"robot_state": {"odom": {"position": {"x": 1.0, "y": 2.0}}, '
        '"navigation": {"state": "idle"}}, '
        '"memory_state": {"spatial": {"available": true, "matches": [{"id": "kitchen"}]}, '
        '"temporal": {"available": true}}}'
    )
    model = _Go2CausalWorldModel()
    restored = _Go2CausalWorldModel()
    try:
        model.record_causal_transition(
            task="go to the kitchen",
            skill_name="navigate_with_text",
            args_json='{"query": "kitchen"}',
            before_state_json=missing_snapshot_json,
            after_state_json=missing_snapshot_json,
            outcome_json='{"success": false, "message": "No matching location found"}',
        )
        model.record_causal_transition(
            task="go to the kitchen",
            skill_name="navigate_with_text",
            args_json='{"query": "kitchen"}',
            before_state_json=matched_snapshot_json,
            after_state_json=matched_snapshot_json,
            outcome_json='{"success": true, "message": "goal reached"}',
        )
        model.record_intervention(
            task="go to the kitchen",
            intervention_name="look_out_for_then_tag_target",
            target_variable="spatial_has_matches",
            before_value_json="false",
            after_value_json="true",
            action_json='{"skill_name": "look_out_for", "args": {"description_of_things": ["kitchen"]}}',
            snapshot_before_json=missing_snapshot_json,
            snapshot_after_json=matched_snapshot_json,
            outcome_json='{"success": true, "message": "target tagged"}',
            causal_hypothesis="spatial_has_matches:False blocks semantic navigation",
        )

        saved = model.save_world_model_state(str(state_path))
        assert saved.success is True
        assert state_path.exists()

        loaded = restored.load_world_model_state(str(state_path))
        assert loaded.success is True
        assert restored.get_model_state()["sample_count"] == 2
        assert restored.get_recent_transitions(limit=10)[0]["skill_name"] == "navigate_with_text"
        assert restored.get_intervention_log(limit=1)[0]["intervention_name"] == (
            "look_out_for_then_tag_target"
        )

        prediction = restored.predict_next_state(
            snapshot_json=missing_snapshot_json,
            action_json='{"skill_name": "navigate_with_text", "args": {"query": "kitchen"}}',
            goal="go to the kitchen",
        )

        assert prediction["prediction"]["model"]["sample_count"] == 2
        assert (
            prediction["prediction"]["intervention_evidence"]["suggestions"][0]["intervention_name"]
            == "look_out_for_then_tag_target"
        )
    finally:
        _stop_modules(model, restored)


def test_predict_next_state_publishes_dashboard_world_model_state() -> None:
    model = _Go2CausalWorldModel()
    vis = StubWebsocketVis()
    model._websocket_vis = vis  # type: ignore[assignment]
    snapshot_json = (
        '{"robot_state": {"odom": {"position": {"x": 1.0, "y": 2.0}}, '
        '"navigation": {"state": "idle"}}, '
        '"memory_state": {"spatial": {"available": true, "matches": []}, '
        '"temporal": {"available": true}}}'
    )
    try:
        prediction = model.predict_next_state(
            snapshot_json=snapshot_json,
            action_json='{"skill_name": "navigate_with_text", "args": {"query": "kitchen"}}',
            goal="go to the kitchen",
        )

        assert vis.states
        dashboard_state = vis.states[-1]
        assert dashboard_state["schema"] == WORLD_MODEL_DASHBOARD_SCHEMA
        assert dashboard_state["prediction"]["risk"] == prediction["prediction"]["risk"]
        assert (
            dashboard_state["prediction"]["predicted_state_delta"]
            == prediction["prediction"]["predicted_state_delta"]
        )
        assert (
            dashboard_state["prediction"]["structural_causal_model"]
            == prediction["prediction"]["structural_causal_model"]
        )
        assert dashboard_state["action_role"] == "predicted_future_action"
        assert dashboard_state["action_label"] == "Predicted future action"
        assert dashboard_state["model_state"]["available"] is True
    finally:
        _stop_modules(model)


def test_start_publishes_initial_dashboard_world_model_state() -> None:
    model = _Go2CausalWorldModel()
    vis = StubWebsocketVis()
    model._websocket_vis = vis  # type: ignore[assignment]
    try:
        model.start()

        assert vis.states
        state = vis.states[-1]
        assert state["schema"] == WORLD_MODEL_DASHBOARD_SCHEMA
        assert state["available"] is True
        assert state["event"] == "world_model_start"
        assert state["model_state"]["available"] is True
        assert state["prediction"] == {}
    finally:
        _stop_modules(model)
