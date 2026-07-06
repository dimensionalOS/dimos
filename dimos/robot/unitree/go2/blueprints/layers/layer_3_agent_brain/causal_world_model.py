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

from collections import Counter, deque
from dataclasses import dataclass
import json
import os
from pathlib import Path
import time
from typing import Any, Protocol

from dimos.agents.annotation import skill
from dimos.agents.skill_result import SkillResult
from dimos.core.core import rpc
from dimos.core.module import Module
from dimos.robot.unitree.go2.blueprints.layers.layer_3_agent_brain.causal_effect_estimator import (
    CausalEffectEstimator,
)
from dimos.robot.unitree.go2.blueprints.layers.layer_3_agent_brain.intervention_log import (
    InterventionLog,
)
from dimos.robot.unitree.go2.blueprints.layers.layer_3_agent_brain.online_transition_model import (
    OnlineTransitionOutcomeModel,
)
from dimos.robot.unitree.go2.blueprints.layers.layer_3_agent_brain.skill_outcome_store import (
    SkillOutcomeStoreSpec,
)
from dimos.robot.unitree.go2.blueprints.layers.layer_3_agent_brain.structural_causal_model import (
    StructuralCausalModel,
)
from dimos.robot.unitree.go2.blueprints.layers.layer_3_agent_brain.world_model_contract import (
    WORLD_MODEL_DASHBOARD_SCHEMA,
    WORLD_MODEL_PREDICTION_SCHEMA,
    provider_contract,
)
from dimos.spec.utils import Spec
from dimos.web.websocket_vis_spec import WebsocketVisSpec

_STATE_SCHEMA = "go2_causal_world_model_state.v1"
_STATE_PATH_ENV = "DIMOS_GO2_WORLD_MODEL_STATE"


class CausalWorldModelSpec(Spec, Protocol):
    """RPC surface for Layer 3 modules that need world-model predictions."""

    def get_recent_transitions(
        self,
        limit: int = 5,
        skill_name: str = "",
        domain: str = "",
        cause: str = "",
    ) -> list[dict[str, Any]]: ...

    def predict_next_state(
        self,
        snapshot_json: str = "",
        action_json: str = "",
        goal: str = "",
        horizon: int = 1,
    ) -> dict[str, Any]: ...

    def score_action(
        self,
        snapshot_json: str = "",
        action_json: str = "",
        goal: str = "",
    ) -> dict[str, Any]: ...

    def get_intervention_log(
        self,
        limit: int = 10,
        target_variable: str = "",
        intervention_name: str = "",
    ) -> list[dict[str, Any]]: ...

    def save_world_model_state(self, path: str = "") -> SkillResult: ...

    def load_world_model_state(self, path: str = "") -> SkillResult: ...

    def get_model_state(self) -> dict[str, Any]: ...

    def get_provider_contract(self) -> dict[str, Any]: ...


@dataclass(frozen=True)
class _WorldTransition:
    """One observed world-state transition with inferred failure cause."""

    timestamp: float
    task: str
    domain: str
    skill_name: str
    args: dict[str, Any]
    before_state: dict[str, Any]
    before_context: str
    prediction_risk: str
    prediction_reasons: list[str]
    outcome_success: bool | None
    outcome_error_code: str
    outcome_message: str
    after_state: dict[str, Any]
    after_context: str
    state_delta: dict[str, Any]
    inferred_cause: str
    recovery: str
    confidence: str

    def to_dict(self) -> dict[str, Any]:
        return {
            "timestamp": round(self.timestamp, 3),
            "task": self.task,
            "domain": self.domain,
            "skill_name": self.skill_name,
            "args": self.args,
            "before_state": _bounded_jsonable(self.before_state),
            "before_context": self.before_context,
            "prediction_risk": self.prediction_risk,
            "prediction_reasons": self.prediction_reasons,
            "outcome_success": self.outcome_success,
            "outcome_error_code": self.outcome_error_code,
            "outcome_message": self.outcome_message,
            "after_state": _bounded_jsonable(self.after_state),
            "after_context": self.after_context,
            "state_delta": _bounded_jsonable(self.state_delta),
            "inferred_cause": self.inferred_cause,
            "recovery": self.recovery,
            "confidence": self.confidence,
        }


class _Go2CausalWorldModel(Module):
    """In-memory Layer 3 predictive world model for Go2.

    The model consumes Layer 4 JSON world snapshots plus candidate skill actions.
    It does not learn continuous dynamics yet; it provides an agent-friendly
    transition memory, risk estimate, predicted symbolic state delta, and action
    score so the coding/LLM agent can run a physical-autoresearch style loop.
    """

    _skill_outcomes: SkillOutcomeStoreSpec | None = None
    _websocket_vis: WebsocketVisSpec | None = None
    _max_transitions = 200

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._transitions: deque[_WorldTransition] = deque(maxlen=self._max_transitions)
        self._outcome_model = OnlineTransitionOutcomeModel()
        self._causal_estimator = CausalEffectEstimator()
        self._intervention_log = InterventionLog()
        self._scm = StructuralCausalModel()
        self._persistence_path = _configured_persistence_path()
        if self._persistence_path is not None:
            self._load_world_model_state_from_path(self._persistence_path, missing_ok=True)

    @rpc
    def start(self) -> None:
        super().start()
        self._publish_dashboard_state(event="world_model_start")

    @skill
    def record_causal_transition(
        self,
        task: str,
        skill_name: str,
        args_json: str = "",
        before_context: str = "",
        after_context: str = "",
        prediction_json: str = "",
        outcome_json: str = "",
        domain: str = "",
        before_state_json: str = "",
        after_state_json: str = "",
    ) -> SkillResult:
        """Record one before/action/result/after world-state transition.

        Use this after an important physical or recovery-sensitive tool call.
        The tool stores compact JSON state snapshots, infers a coarse cause,
        and returns a recovery suggestion. It does not execute or retry robot
        actions.

        Args:
            task: User goal or agent subtask that led to the action.
            skill_name: Tool or skill that was executed.
            args_json: Optional JSON object string with the executed arguments.
            before_state_json: Optional Layer 4 world snapshot before action.
            before_context: Context summary captured before the action.
            after_state_json: Optional Layer 4 world snapshot after action.
            after_context: Context summary captured after the action.
            prediction_json: Optional JSON object from predict_skill_outcome.
            outcome_json: Optional JSON object from the tool result or outcome
                store. If omitted, the latest same-skill outcome is used when
                SkillOutcomeStore is wired.
            domain: Optional expert domain override.
        """
        task = task.strip()
        skill_name = skill_name.strip()
        domain = domain.strip()
        before_context = before_context.strip()
        after_context = after_context.strip()

        if not task:
            return SkillResult.fail("INVALID_INPUT", "task is required")
        if not skill_name:
            return SkillResult.fail("INVALID_INPUT", "skill_name is required")

        parsed_args = _parse_json_object(args_json, "args_json")
        if isinstance(parsed_args, str):
            return SkillResult.fail("INVALID_INPUT", parsed_args)

        before_state = _parse_json_object(before_state_json, "before_state_json")
        if isinstance(before_state, str):
            return SkillResult.fail("INVALID_INPUT", before_state)

        after_state = _parse_json_object(after_state_json, "after_state_json")
        if isinstance(after_state, str):
            return SkillResult.fail("INVALID_INPUT", after_state)

        prediction = _parse_json_object(prediction_json, "prediction_json")
        if isinstance(prediction, str):
            return SkillResult.fail("INVALID_INPUT", prediction)

        outcome = _parse_json_object(outcome_json, "outcome_json")
        if isinstance(outcome, str):
            return SkillResult.fail("INVALID_INPUT", outcome)
        if not outcome:
            outcome = self._latest_outcome(skill_name)

        prediction_view = _metadata_view(prediction)
        outcome_view = _metadata_view(outcome)
        prediction_reasons = _prediction_reasons(prediction_view)
        outcome_success = _optional_bool(outcome_view.get("success"))
        outcome_error_code = str(outcome_view.get("error_code") or "")
        outcome_message = str(outcome_view.get("message") or "")
        prediction_risk = str(prediction_view.get("risk") or "unknown")
        resolved_domain = domain or str(prediction_view.get("domain") or _infer_domain(skill_name))
        action = {
            "skill_name": skill_name,
            "domain": resolved_domain,
            "args": parsed_args,
        }

        cause, recovery, confidence = _infer_cause(
            skill_name=skill_name,
            before_context=before_context,
            after_context=after_context,
            prediction_reasons=prediction_reasons,
            outcome_success=outcome_success,
            outcome_message=outcome_message,
            outcome_error_code=outcome_error_code,
        )
        state_delta = _diff_world_state(before_state, after_state)
        self._outcome_model.update(before_state, action, outcome_success)
        self._causal_estimator.update(before_state, action, outcome_success, cause)

        transition = _WorldTransition(
            timestamp=time.time(),
            task=task,
            domain=resolved_domain,
            skill_name=skill_name,
            args=parsed_args,
            before_state=before_state,
            before_context=before_context[:500],
            prediction_risk=prediction_risk,
            prediction_reasons=prediction_reasons,
            outcome_success=outcome_success,
            outcome_error_code=outcome_error_code,
            outcome_message=outcome_message[:500],
            after_state=after_state,
            after_context=after_context[:500],
            state_delta=state_delta,
            inferred_cause=cause,
            recovery=recovery,
            confidence=confidence,
        )
        self._transitions.append(transition)
        autosave_error = self._autosave_if_configured()
        dashboard_error = self._publish_dashboard_state(event="record_causal_transition")

        return SkillResult.ok(
            f"Recorded causal transition for {skill_name}: {cause}",
            transition=transition.to_dict(),
            total_transitions=len(self._transitions),
            autosave_error=autosave_error,
            dashboard_error=dashboard_error,
        )

    @skill
    def predict_world_transition(
        self,
        snapshot_json: str = "",
        action_json: str = "",
        goal: str = "",
        horizon: int = 1,
    ) -> SkillResult:
        """Predict symbolic state delta, risk, and failure modes for an action.

        Args:
            snapshot_json: Layer 4 world snapshot JSON object.
            action_json: Candidate action JSON object. Expected keys are
                skill_name and args.
            goal: Optional user goal for scoring.
            horizon: Number of symbolic action steps to roll out.
        """
        prediction = self.predict_next_state(
            snapshot_json=snapshot_json,
            action_json=action_json,
            goal=goal,
            horizon=horizon,
        )
        if prediction.get("error"):
            return SkillResult.fail("INVALID_INPUT", str(prediction["error"]))

        model = prediction["prediction"]
        message = (
            f"Predicted world risk={model['risk']}, "
            f"score={model['score']}, confidence={model['confidence']}"
        )
        return SkillResult.ok(message, **prediction)

    @skill
    def summarize_causal_patterns(
        self, skill_name: str = "", domain: str = "", limit: int = 10
    ) -> SkillResult:
        """Summarize recent causal failure patterns.

        Args:
            skill_name: Optional skill-name filter.
            domain: Optional expert-domain filter.
            limit: Maximum number of recent transitions to inspect.
        """
        transitions = self.get_recent_transitions(
            limit=limit,
            skill_name=skill_name,
            domain=domain,
        )
        if not transitions:
            return SkillResult.ok("No recorded causal transitions", patterns=[], transitions=[])

        patterns = _summarize_patterns(transitions)
        if not patterns:
            return SkillResult.ok(
                f"{len(transitions)} transition(s), no repeated failure pattern",
                patterns=[],
                transitions=transitions,
            )

        top = patterns[0]
        message = (
            f"{len(transitions)} transition(s), top failure cause "
            f"{top['cause']} occurred {top['count']} time(s)"
        )
        return SkillResult.ok(message, patterns=patterns, transitions=transitions)

    @skill
    def record_intervention(
        self,
        task: str,
        intervention_name: str,
        target_variable: str,
        before_value_json: str = "",
        after_value_json: str = "",
        action_json: str = "",
        snapshot_before_json: str = "",
        snapshot_after_json: str = "",
        outcome_json: str = "",
        causal_hypothesis: str = "",
    ) -> SkillResult:
        """Record an explicit intervention and its observed result.

        Use this when the agent intentionally changes one world variable, such
        as adding a spatial tag, waiting for odometry, or running perception,
        then observes whether the downstream task improved.
        """
        task = task.strip()
        intervention_name = intervention_name.strip()
        target_variable = target_variable.strip()
        causal_hypothesis = causal_hypothesis.strip()
        if not task:
            return SkillResult.fail("INVALID_INPUT", "task is required")
        if not intervention_name:
            return SkillResult.fail("INVALID_INPUT", "intervention_name is required")
        if not target_variable:
            return SkillResult.fail("INVALID_INPUT", "target_variable is required")

        before_value = _parse_json_value(before_value_json, "before_value_json")
        if isinstance(before_value, _JsonParseError):
            return SkillResult.fail("INVALID_INPUT", before_value.message)
        after_value = _parse_json_value(after_value_json, "after_value_json")
        if isinstance(after_value, _JsonParseError):
            return SkillResult.fail("INVALID_INPUT", after_value.message)

        action = _parse_json_object(action_json, "action_json")
        if isinstance(action, str):
            return SkillResult.fail("INVALID_INPUT", action)
        snapshot_before = _parse_json_object(snapshot_before_json, "snapshot_before_json")
        if isinstance(snapshot_before, str):
            return SkillResult.fail("INVALID_INPUT", snapshot_before)
        snapshot_after = _parse_json_object(snapshot_after_json, "snapshot_after_json")
        if isinstance(snapshot_after, str):
            return SkillResult.fail("INVALID_INPUT", snapshot_after)
        outcome = _parse_json_object(outcome_json, "outcome_json")
        if isinstance(outcome, str):
            return SkillResult.fail("INVALID_INPUT", outcome)

        outcome_view = _metadata_view(outcome)
        record = self._intervention_log.record(
            task=task,
            intervention_name=intervention_name,
            target_variable=target_variable,
            before_value=before_value,
            after_value=after_value,
            action=_normalize_action(action),
            snapshot_before=snapshot_before,
            snapshot_after=snapshot_after,
            outcome_success=_optional_bool(outcome_view.get("success")),
            outcome_message=str(outcome_view.get("message") or ""),
            causal_hypothesis=causal_hypothesis,
        )
        autosave_error = self._autosave_if_configured()
        dashboard_error = self._publish_dashboard_state(event="record_intervention")
        return SkillResult.ok(
            f"Recorded intervention {intervention_name} on {target_variable}",
            intervention=record,
            total_interventions=self._intervention_log.snapshot()["record_count"],
            autosave_error=autosave_error,
            dashboard_error=dashboard_error,
        )

    @skill
    def save_world_model_state(self, path: str = "") -> SkillResult:
        """Persist transitions, interventions, and learned model weights to JSON."""
        state_path = self._resolve_state_path(path)
        if state_path is None:
            return SkillResult.fail(
                "INVALID_INPUT",
                f"path is required unless {_STATE_PATH_ENV} is set",
            )
        try:
            summary = self._save_world_model_state_to_path(state_path)
        except OSError as exc:
            return SkillResult.fail("IO_ERROR", str(exc))
        summary["dashboard_error"] = self._publish_dashboard_state(event="save_world_model_state")
        return SkillResult.ok(f"Saved world model state to {state_path}", **summary)

    @skill
    def load_world_model_state(self, path: str = "") -> SkillResult:
        """Load transitions, interventions, and learned model weights from JSON."""
        state_path = self._resolve_state_path(path)
        if state_path is None:
            return SkillResult.fail(
                "INVALID_INPUT",
                f"path is required unless {_STATE_PATH_ENV} is set",
            )
        try:
            summary = self._load_world_model_state_from_path(state_path)
        except (OSError, ValueError, json.JSONDecodeError) as exc:
            return SkillResult.fail("IO_ERROR", str(exc))
        summary["dashboard_error"] = self._publish_dashboard_state(event="load_world_model_state")
        return SkillResult.ok(f"Loaded world model state from {state_path}", **summary)

    @rpc
    def get_recent_transitions(
        self,
        limit: int = 5,
        skill_name: str = "",
        domain: str = "",
        cause: str = "",
    ) -> list[dict[str, Any]]:
        """Return newest causal transitions first, optionally filtered."""
        limit = max(0, min(limit, self._max_transitions))
        skill_name = skill_name.strip()
        domain = domain.strip()
        cause = cause.strip()

        transitions = [
            transition
            for transition in reversed(self._transitions)
            if (not skill_name or transition.skill_name == skill_name)
            and (not domain or transition.domain == domain)
            and (not cause or transition.inferred_cause == cause)
        ]
        return [transition.to_dict() for transition in transitions[:limit]]

    @rpc
    def get_intervention_log(
        self,
        limit: int = 10,
        target_variable: str = "",
        intervention_name: str = "",
    ) -> list[dict[str, Any]]:
        """Return newest recorded interventions first, optionally filtered."""
        return self._intervention_log.recent(
            limit=limit,
            target_variable=target_variable,
            intervention_name=intervention_name,
        )

    @rpc
    def predict_next_state(
        self,
        snapshot_json: str = "",
        action_json: str = "",
        goal: str = "",
        horizon: int = 1,
    ) -> dict[str, Any]:
        """Predict a symbolic next-state delta for a candidate action."""
        snapshot = _parse_json_object(snapshot_json, "snapshot_json")
        if isinstance(snapshot, str):
            return {"error": snapshot}

        action = _parse_json_object(action_json, "action_json")
        if isinstance(action, str):
            return {"error": action}

        normalized_action = _normalize_action(action)
        skill_name = normalized_action["skill_name"]
        domain = _infer_domain(skill_name)
        horizon = max(1, min(horizon, 10))

        recent = self.get_recent_transitions(limit=20, skill_name=skill_name, domain=domain)
        failure_modes = _failure_modes(recent)
        reasons = _world_model_risk_reasons(snapshot, normalized_action, failure_modes)
        rule_risk = _risk_level(reasons)
        model_prediction = self._outcome_model.predict(snapshot, normalized_action)
        causal_attribution = self._causal_estimator.estimate(snapshot, normalized_action)
        structural_causal_model = self._scm.explain(
            snapshot,
            normalized_action,
            causal_attribution,
        )
        intervention_evidence = self._intervention_log.evidence_for(
            causal_attribution,
            structural_causal_model,
        )
        reasons.extend(_causal_risk_reasons(causal_attribution))
        rule_risk = _risk_level(reasons)
        risk = _combined_risk(rule_risk, str(model_prediction["risk"]))
        score = _combined_score(
            rule_score=_action_score(rule_risk, reasons, failure_modes),
            model_score=float(model_prediction["score"]),
            has_model_samples=int(model_prediction["sample_count"]) > 0,
            risk=risk,
        )
        predicted_delta = _predict_symbolic_delta(
            snapshot=snapshot,
            action=normalized_action,
            risk=risk,
            horizon=horizon,
        )
        confidence = _prediction_confidence(snapshot, normalized_action, failure_modes)
        evidence = _prediction_evidence(snapshot, normalized_action, recent, reasons)

        result = {
            "schema": WORLD_MODEL_PREDICTION_SCHEMA,
            "goal": goal.strip(),
            "horizon": horizon,
            "action": normalized_action,
            "snapshot_summary": _snapshot_summary(snapshot),
            "prediction": {
                "risk": risk,
                "predicted_success": risk != "high",
                "score": score,
                "confidence": _combined_confidence(confidence, model_prediction),
                "predicted_state_delta": predicted_delta,
                "failure_modes": failure_modes,
                "reasons": reasons,
                "evidence": evidence,
                "model": model_prediction,
                "causal_attribution": causal_attribution,
                "structural_causal_model": structural_causal_model,
                "intervention_evidence": intervention_evidence,
            },
        }
        dashboard_error = self._publish_dashboard_state(
            prediction=result,
            event="predict_next_state",
        )
        if dashboard_error:
            result["dashboard_error"] = dashboard_error
        return result

    @rpc
    def score_action(
        self,
        snapshot_json: str = "",
        action_json: str = "",
        goal: str = "",
    ) -> dict[str, Any]:
        """Return a compact action score derived from predict_next_state."""
        prediction = self.predict_next_state(
            snapshot_json=snapshot_json,
            action_json=action_json,
            goal=goal,
        )
        if prediction.get("error"):
            return prediction
        model = prediction["prediction"]
        return {
            "goal": prediction["goal"],
            "action": prediction["action"],
            "score": model["score"],
            "risk": model["risk"],
            "confidence": model["confidence"],
            "predicted_success": model["predicted_success"],
            "failure_modes": model["failure_modes"],
            "reasons": model["reasons"],
            "model": model["model"],
            "causal_attribution": model["causal_attribution"],
            "structural_causal_model": model["structural_causal_model"],
            "intervention_evidence": model["intervention_evidence"],
        }

    @rpc
    def get_provider_contract(self) -> dict[str, Any]:
        """Return the explicit world-model provider contract for this module."""
        return provider_contract(
            name="go2_causal_world_model",
            model_type="symbolic_causal_online_transition",
            capabilities=[
                "predict_next_state",
                "score_action",
                "record_causal_transition",
                "record_intervention",
                "get_model_state",
            ],
            output_schemas={
                "prediction": WORLD_MODEL_PREDICTION_SCHEMA,
                "dashboard": WORLD_MODEL_DASHBOARD_SCHEMA,
            },
        )

    @rpc
    def get_model_state(self) -> dict[str, Any]:
        """Return compact metadata for the learned local transition model."""
        state = self._outcome_model.snapshot()
        state["available"] = True
        state["provider_contract"] = self.get_provider_contract()
        state["causal_estimator"] = self._causal_estimator.snapshot()
        state["intervention_log"] = self._intervention_log.snapshot()
        state["structural_causal_model"] = self._scm.snapshot()
        state["persistence"] = {
            "schema": _STATE_SCHEMA,
            "path": str(self._persistence_path) if self._persistence_path else "",
            "autosave": self._persistence_path is not None,
        }
        return state

    def _resolve_state_path(self, path: str) -> Path | None:
        path = path.strip()
        if path:
            return Path(path).expanduser()
        return self._persistence_path

    def _autosave_if_configured(self) -> str:
        if self._persistence_path is None:
            return ""
        try:
            self._save_world_model_state_to_path(self._persistence_path)
        except OSError as exc:
            return str(exc)
        return ""

    def _save_world_model_state_to_path(self, path: Path) -> dict[str, Any]:
        path.parent.mkdir(parents=True, exist_ok=True)
        payload = self._state_payload()
        content = json.dumps(payload, indent=2, sort_keys=True)
        # Atomic write via temp file + rename: prevents truncation on crash
        # and ensures readers always see a complete file.  Still last-writer-
        # wins across concurrent instances, but no interleaved corruption.
        tmp = path.with_name("." + path.name + ".tmp")
        try:
            tmp.write_text(content, encoding="utf-8")
            os.replace(tmp, path)
        finally:
            if tmp.exists():
                try:
                    tmp.unlink()
                except OSError:
                    pass
        return _state_summary(payload, str(path))

    def _load_world_model_state_from_path(
        self,
        path: Path,
        missing_ok: bool = False,
    ) -> dict[str, Any]:
        if not path.exists():
            if missing_ok:
                return {
                    "schema": _STATE_SCHEMA,
                    "path": str(path),
                    "loaded": False,
                    "reason": "state file does not exist",
                }
            raise FileNotFoundError(path)

        payload = json.loads(path.read_text(encoding="utf-8"))
        if not isinstance(payload, dict):
            raise ValueError("world model state must be a JSON object")
        if payload.get("schema") != _STATE_SCHEMA:
            raise ValueError(f"unsupported world model state schema: {payload.get('schema')}")

        # Build new state in temporaries BEFORE mutating in-memory state.
        # If any step fails the running model is untouched.
        raw_transitions = (
            payload.get("transitions") if isinstance(payload.get("transitions"), list) else []
        )
        new_transitions: deque[_WorldTransition] = deque(maxlen=self._max_transitions)
        for item in raw_transitions[-self._max_transitions :]:
            if isinstance(item, dict):
                new_transitions.append(_transition_from_dict(item))

        outcome_model_data = (
            payload.get("outcome_model") if isinstance(payload.get("outcome_model"), dict) else {}
        )
        causal_estimator_data = (
            payload.get("causal_estimator")
            if isinstance(payload.get("causal_estimator"), dict)
            else {}
        )
        intervention_log_data = (
            payload.get("intervention_log")
            if isinstance(payload.get("intervention_log"), dict)
            else {}
        )

        # Swap in only after all parsing succeeded.
        self._transitions = new_transitions
        self._outcome_model.load_dict(outcome_model_data)
        self._causal_estimator.load_dict(causal_estimator_data)
        self._intervention_log.load_dict(intervention_log_data)
        return _state_summary(payload, str(path)) | {"loaded": True}

    def _state_payload(self) -> dict[str, Any]:
        return {
            "schema": _STATE_SCHEMA,
            "saved_at": round(time.time(), 3),
            "transitions": [transition.to_dict() for transition in self._transitions],
            "outcome_model": self._outcome_model.to_dict(),
            "causal_estimator": self._causal_estimator.to_dict(),
            "intervention_log": self._intervention_log.to_dict(),
            "structural_causal_model": self._scm.snapshot(),
        }

    def _publish_dashboard_state(
        self,
        prediction: dict[str, Any] | None = None,
        event: str = "",
    ) -> str:
        if self._websocket_vis is None:
            return ""
        try:
            self._websocket_vis.set_world_model_state(
                self._dashboard_state(prediction=prediction, event=event)
            )
        except Exception as exc:
            return str(exc)
        return ""

    def _dashboard_state(
        self,
        prediction: dict[str, Any] | None = None,
        event: str = "",
    ) -> dict[str, Any]:
        dashboard_state: dict[str, Any] = {
            "schema": WORLD_MODEL_DASHBOARD_SCHEMA,
            "available": True,
            "event": event,
            "updated_at": round(time.time(), 3),
            "model_state": self.get_model_state(),
            "recent_transitions": self.get_recent_transitions(limit=5),
            "recent_interventions": self.get_intervention_log(limit=5),
            "prediction": {},
            "action_role": "none",
            "action_label": "No action",
        }
        if prediction is not None:
            dashboard_state["goal"] = prediction.get("goal") or ""
            dashboard_state["horizon"] = prediction.get("horizon")
            dashboard_state["action"] = prediction.get("action") or {}
            dashboard_state["action_role"] = "predicted_future_action"
            dashboard_state["action_label"] = "Predicted future action"
            dashboard_state["snapshot_summary"] = prediction.get("snapshot_summary") or {}
            dashboard_state["prediction"] = prediction.get("prediction") or {}
        return _bounded_jsonable(dashboard_state)

    def _latest_outcome(self, skill_name: str) -> dict[str, Any]:
        if self._skill_outcomes is None:
            return {}
        outcomes = self._skill_outcomes.get_recent_outcomes(limit=1, skill_name=skill_name)
        return outcomes[0] if outcomes else {}


def _configured_persistence_path() -> Path | None:
    raw_path = os.environ.get(_STATE_PATH_ENV, "").strip()
    if not raw_path:
        return None
    return Path(raw_path).expanduser()


def _state_summary(payload: dict[str, Any], path: str) -> dict[str, Any]:
    transitions = payload.get("transitions") if isinstance(payload.get("transitions"), list) else []
    outcome_model = (
        payload.get("outcome_model") if isinstance(payload.get("outcome_model"), dict) else {}
    )
    intervention_log = (
        payload.get("intervention_log") if isinstance(payload.get("intervention_log"), dict) else {}
    )
    interventions = (
        intervention_log.get("records") if isinstance(intervention_log.get("records"), list) else []
    )
    return {
        "schema": str(payload.get("schema") or ""),
        "path": path,
        "transition_count": len(transitions),
        "intervention_count": len(interventions),
        "sample_count": int(outcome_model.get("sample_count") or 0),
        "saved_at": payload.get("saved_at"),
    }


def _transition_from_dict(value: dict[str, Any]) -> _WorldTransition:
    args = value.get("args") if isinstance(value.get("args"), dict) else {}
    before_state = value.get("before_state") if isinstance(value.get("before_state"), dict) else {}
    after_state = value.get("after_state") if isinstance(value.get("after_state"), dict) else {}
    state_delta = value.get("state_delta") if isinstance(value.get("state_delta"), dict) else {}
    prediction_reasons = (
        value.get("prediction_reasons") if isinstance(value.get("prediction_reasons"), list) else []
    )
    outcome_success = value.get("outcome_success")
    if not isinstance(outcome_success, bool):
        outcome_success = None
    return _WorldTransition(
        timestamp=float(value.get("timestamp") or time.time()),
        task=str(value.get("task") or ""),
        domain=str(value.get("domain") or ""),
        skill_name=str(value.get("skill_name") or ""),
        args=args,
        before_state=before_state,
        before_context=str(value.get("before_context") or ""),
        prediction_risk=str(value.get("prediction_risk") or "unknown"),
        prediction_reasons=[str(reason) for reason in prediction_reasons],
        outcome_success=outcome_success,
        outcome_error_code=str(value.get("outcome_error_code") or ""),
        outcome_message=str(value.get("outcome_message") or ""),
        after_state=after_state,
        after_context=str(value.get("after_context") or ""),
        state_delta=state_delta,
        inferred_cause=str(value.get("inferred_cause") or "unknown_transition"),
        recovery=str(value.get("recovery") or ""),
        confidence=str(value.get("confidence") or "low"),
    )


def _normalize_action(action: dict[str, Any]) -> dict[str, Any]:
    skill_name = str(
        action.get("skill_name")
        or action.get("tool_name")
        or action.get("action")
        or action.get("name")
        or ""
    ).strip()
    args = action.get("args") or action.get("arguments") or {}
    if not isinstance(args, dict):
        args = {"value": args}
    return {
        "skill_name": skill_name,
        "domain": str(action.get("domain") or _infer_domain(skill_name)),
        "args": args,
    }


def _failure_modes(transitions: list[dict[str, Any]]) -> list[dict[str, Any]]:
    counts: Counter[str] = Counter()
    examples: dict[str, dict[str, Any]] = {}
    for transition in transitions:
        if transition.get("outcome_success") is not False:
            continue
        cause = str(transition.get("inferred_cause") or "unknown_failure")
        if cause == "success_no_failure":
            continue
        counts[cause] += 1
        examples.setdefault(cause, transition)

    modes: list[dict[str, Any]] = []
    for cause, count in counts.most_common():
        example = examples[cause]
        modes.append(
            {
                "cause": cause,
                "count": count,
                "recovery": str(example.get("recovery") or ""),
                "latest_message": str(example.get("outcome_message") or ""),
                "confidence": "high" if count >= 2 else "medium",
            }
        )
    return modes


def _world_model_risk_reasons(
    snapshot: dict[str, Any],
    action: dict[str, Any],
    failure_modes: list[dict[str, Any]],
) -> list[str]:
    reasons: list[str] = []
    skill_name = str(action.get("skill_name") or "")
    name = skill_name.casefold()
    args = action.get("args") if isinstance(action.get("args"), dict) else {}

    for mode in failure_modes:
        cause = str(mode.get("cause") or "unknown_failure")
        count = int(mode.get("count") or 0)
        if count >= 2:
            reasons.append(f"same skill repeatedly failed from causal cause: {cause}")
        else:
            reasons.append(f"same skill has a recent causal failure: {cause}")

    robot_state = (
        snapshot.get("robot_state") if isinstance(snapshot.get("robot_state"), dict) else {}
    )
    memory_state = (
        snapshot.get("memory_state") if isinstance(snapshot.get("memory_state"), dict) else {}
    )
    spatial = memory_state.get("spatial") if isinstance(memory_state.get("spatial"), dict) else {}
    temporal = (
        memory_state.get("temporal") if isinstance(memory_state.get("temporal"), dict) else {}
    )

    if name in {"navigate_with_text", "relative_move", "follow_person"}:
        if not robot_state.get("odom"):
            reasons.append("robot pose or odometry may be unavailable")

    if name == "navigate_with_text":
        query = str(args.get("query") or "").strip()
        if not query:
            reasons.append("navigation query is missing")
        if spatial.get("available") is False:
            reasons.append("spatial memory is unavailable for semantic navigation")
        elif spatial.get("available") is True and not spatial.get("matches"):
            reasons.append("spatial memory has no relevant matches")

    if name == "follow_person":
        query = str(args.get("query") or "").strip()
        if not query:
            reasons.append("person-follow query is missing")
        if spatial.get("available") is False and temporal.get("available") is False:
            reasons.append("no memory source is available to help identify the target")

    if name == "look_out_for" and not args.get("description_of_things"):
        reasons.append("look_out_for descriptions are missing")

    return reasons


def _predict_symbolic_delta(
    snapshot: dict[str, Any],
    action: dict[str, Any],
    risk: str,
    horizon: int,
) -> dict[str, Any]:
    skill_name = str(action.get("skill_name") or "").casefold()
    args = action.get("args") if isinstance(action.get("args"), dict) else {}
    blocked = risk == "high"

    if skill_name == "navigate_with_text":
        return {
            "navigation.state": "blocked_before_navigation" if blocked else "following_path",
            "navigation.target_query": str(args.get("query") or ""),
            "navigation.horizon_steps": horizon,
            "memory_state.spatial.required_match": "missing_or_untrusted"
            if blocked
            else "used_for_target",
        }

    if skill_name == "relative_move":
        return {
            "robot_state.odom": "expected_motion_delta" if not blocked else "unchanged",
            "robot_state.motion_command": _bounded_jsonable(args),
            "safety.constraint_status": "must_pass_before_motion",
        }

    if skill_name == "follow_person":
        return {
            "navigation.state": "blocked_before_follow" if blocked else "following_person",
            "tracking.target_query": str(args.get("query") or ""),
            "memory_state.temporal.expected_update": "target_track_observation",
        }

    if skill_name == "look_out_for":
        return {
            "perception.query": _bounded_jsonable(args.get("description_of_things")),
            "memory_state.temporal.expected_update": "visual_observation",
            "semantic_temporal_map.expected_evidence": "perception",
        }

    if skill_name.startswith("stop_"):
        return {
            "navigation.state": "stopping_or_idle",
            "control.mode": "safe_stop_requested",
        }

    return {
        "world_state": "unknown_symbolic_delta",
        "action": str(action.get("skill_name") or ""),
    }


def _prediction_confidence(
    snapshot: dict[str, Any],
    action: dict[str, Any],
    failure_modes: list[dict[str, Any]],
) -> str:
    has_snapshot = bool(snapshot)
    has_action = bool(action.get("skill_name"))
    repeated_failures = any(int(mode.get("count") or 0) >= 2 for mode in failure_modes)
    if has_snapshot and has_action and not failure_modes:
        return "high"
    if has_action and (has_snapshot or repeated_failures):
        return "medium"
    return "low"


def _prediction_evidence(
    snapshot: dict[str, Any],
    action: dict[str, Any],
    recent: list[dict[str, Any]],
    reasons: list[str],
) -> dict[str, Any]:
    sources = snapshot.get("sources") if isinstance(snapshot.get("sources"), dict) else {}
    return {
        "snapshot_sources": _bounded_jsonable(sources),
        "skill_name": action.get("skill_name") or "",
        "recent_transition_count": len(recent),
        "risk_reason_count": len(reasons),
        "has_robot_state": isinstance(snapshot.get("robot_state"), dict),
        "has_memory_state": isinstance(snapshot.get("memory_state"), dict),
    }


def _action_score(
    risk: str,
    reasons: list[str],
    failure_modes: list[dict[str, Any]],
) -> float:
    score_by_risk = {"low": 0.85, "medium": 0.55, "high": 0.25}
    score = score_by_risk.get(risk, 0.4)
    score -= min(0.2, 0.04 * len(reasons))
    score -= min(0.2, 0.05 * sum(int(mode.get("count") or 0) for mode in failure_modes))
    return round(max(0.0, min(1.0, score)), 3)


def _combined_score(
    rule_score: float,
    model_score: float,
    has_model_samples: bool,
    risk: str,
) -> float:
    if not has_model_samples:
        return rule_score
    score = 0.55 * rule_score + 0.45 * model_score
    if risk == "high":
        score = min(score, 0.45)
    return round(max(0.0, min(1.0, score)), 3)


def _combined_risk(rule_risk: str, model_risk: str) -> str:
    severity = {"low": 0, "medium": 1, "high": 2}
    if severity.get(model_risk, 0) > severity.get(rule_risk, 0):
        return model_risk
    return rule_risk


def _combined_confidence(rule_confidence: str, model_prediction: dict[str, Any]) -> str:
    model_confidence = str(model_prediction.get("confidence") or "low")
    if int(model_prediction.get("sample_count") or 0) == 0:
        return rule_confidence
    severity = {"low": 0, "medium": 1, "high": 2}
    return (
        model_confidence
        if severity.get(model_confidence, 0) > severity.get(rule_confidence, 0)
        else rule_confidence
    )


def _causal_risk_reasons(causal_attribution: dict[str, Any]) -> list[str]:
    reasons: list[str] = []
    for factor in causal_attribution.get("risk_factors") or []:
        feature = str(factor.get("feature") or "")
        effect = factor.get("effect_on_success")
        confidence = str(factor.get("confidence") or "low")
        reasons.append(f"causal attribution: {feature} changes success by {effect} ({confidence})")
    return reasons


def _risk_level(reasons: list[str]) -> str:
    high_markers = (
        "failed repeatedly",
        "repeatedly failed from causal cause",
        "navigation query is missing",
        "person-follow query is missing",
        "descriptions are missing",
    )
    if any(any(marker in reason for marker in high_markers) for reason in reasons):
        return "high"
    if reasons:
        return "medium"
    return "low"


def _snapshot_summary(snapshot: dict[str, Any]) -> dict[str, Any]:
    robot_state = (
        snapshot.get("robot_state") if isinstance(snapshot.get("robot_state"), dict) else {}
    )
    memory_state = (
        snapshot.get("memory_state") if isinstance(snapshot.get("memory_state"), dict) else {}
    )
    spatial = memory_state.get("spatial") if isinstance(memory_state.get("spatial"), dict) else {}
    temporal = (
        memory_state.get("temporal") if isinstance(memory_state.get("temporal"), dict) else {}
    )
    return {
        "sources": _bounded_jsonable(snapshot.get("sources") or {}),
        "has_robot_state": bool(robot_state),
        "navigation": _bounded_jsonable(robot_state.get("navigation") or {}),
        "spatial_available": spatial.get("available"),
        "spatial_match_count": len(spatial.get("matches") or []),
        "temporal_available": temporal.get("available"),
        "semantic_temporal_available": bool(snapshot.get("semantic_temporal_map")),
    }


def _diff_world_state(before: dict[str, Any], after: dict[str, Any]) -> dict[str, Any]:
    if not before or not after:
        return {}
    delta: dict[str, Any] = {}
    for path in (
        ("robot_state", "navigation", "state"),
        ("robot_state", "navigation", "goal_reached"),
        ("robot_state", "odom"),
        ("memory_state", "spatial", "available"),
        ("memory_state", "temporal", "rolling_summary"),
        ("semantic_temporal_map", "fused", "entry_count"),
    ):
        before_value = _get_path(before, path)
        after_value = _get_path(after, path)
        if before_value != after_value:
            delta[".".join(path)] = {
                "before": _bounded_jsonable(before_value),
                "after": _bounded_jsonable(after_value),
            }
    return delta


def _get_path(value: dict[str, Any], path: tuple[str, ...]) -> Any:
    current: Any = value
    for key in path:
        if not isinstance(current, dict) or key not in current:
            return None
        current = current[key]
    return current


@dataclass(frozen=True)
class _JsonParseError:
    message: str


def _parse_json_value(value: str, field_name: str) -> Any | _JsonParseError:
    value = value.strip()
    if not value:
        return None
    try:
        return json.loads(value)
    except json.JSONDecodeError as exc:
        return _JsonParseError(f"{field_name} must be valid JSON: {exc}")


def _parse_json_object(value: str, field_name: str) -> dict[str, Any] | str:
    value = value.strip()
    if not value:
        return {}
    try:
        parsed = json.loads(value)
    except json.JSONDecodeError as exc:
        return f"{field_name} must be a JSON object: {exc}"
    if not isinstance(parsed, dict):
        return f"{field_name} must be a JSON object"
    return parsed


def _bounded_jsonable(value: Any, max_string_length: int = 500) -> Any:
    if value is None or isinstance(value, bool | int | float):
        return value
    if isinstance(value, str):
        return value[:max_string_length]
    if isinstance(value, dict):
        return {
            str(key): _bounded_jsonable(item, max_string_length)
            for key, item in value.items()
            if not str(key).startswith("_")
        }
    if isinstance(value, list | tuple):
        return [_bounded_jsonable(item, max_string_length) for item in value[:10]]
    return str(value)[:max_string_length]


def _metadata_view(value: dict[str, Any]) -> dict[str, Any]:
    metadata = value.get("metadata")
    if isinstance(metadata, dict):
        merged = dict(value)
        merged.update(metadata)
        return merged
    return value


def _prediction_reasons(prediction: dict[str, Any]) -> list[str]:
    reasons = prediction.get("failure_reasons") or prediction.get("reasons") or []
    if isinstance(reasons, list):
        return [str(reason)[:300] for reason in reasons]
    if isinstance(reasons, str):
        return [reasons[:300]]
    return []


def _optional_bool(value: Any) -> bool | None:
    if isinstance(value, bool):
        return value
    if value is None:
        return None
    if isinstance(value, str):
        lowered = value.casefold()
        if lowered == "true":
            return True
        if lowered == "false":
            return False
    return None


def _infer_cause(
    skill_name: str,
    before_context: str,
    after_context: str,
    prediction_reasons: list[str],
    outcome_success: bool | None,
    outcome_message: str,
    outcome_error_code: str,
) -> tuple[str, str, str]:
    text = " ".join([before_context, after_context, outcome_message]).casefold()
    reasons = " ".join(prediction_reasons).casefold()

    if outcome_success is True:
        return ("success_no_failure", "", "high")

    if "query is missing" in reasons or "descriptions are missing" in reasons:
        return (
            "invalid_or_missing_arguments",
            "Retry with complete, specific tool arguments.",
            "high",
        )
    if "failed repeatedly" in reasons:
        return (
            "repeated_failure_pattern",
            "Call summarize_causal_patterns before retrying this skill.",
            "high",
        )
    if "robot pose: unavailable" in text or "odometry may be unavailable" in reasons:
        return (
            "robot_state_unavailable",
            "Call get_context with focus='navigation' and wait for odometry.",
            "high",
        )
    if "no relevant matches" in text or "no matching location" in text:
        return (
            "semantic_map_missing_target",
            "Use perception, ask the user, or tag the location before navigating.",
            "medium",
        )
    if "spatial memory is unavailable" in reasons:
        return (
            "world_state_unavailable",
            "Wait for world-state modules or use a direct perception skill.",
            "medium",
        )
    if "could not find" in text or "no image available" in text:
        return (
            "visual_target_unavailable",
            "Call look_out_for or ask for a more specific visual description.",
            "medium",
        )
    if outcome_error_code:
        return (
            f"tool_error:{outcome_error_code}",
            "Inspect the tool error and gather context before retrying.",
            "medium",
        )
    if outcome_success is False:
        return (
            "unknown_failure",
            "Gather context and retry with more specific arguments.",
            "low",
        )
    return ("unknown_transition", "Record a clearer outcome before reasoning from it.", "low")


def _summarize_patterns(transitions: list[dict[str, Any]]) -> list[dict[str, Any]]:
    failures = [
        transition
        for transition in transitions
        if transition.get("outcome_success") is False
        and transition.get("inferred_cause") not in {"success_no_failure"}
    ]
    counts = Counter(str(transition.get("inferred_cause")) for transition in failures)
    patterns: list[dict[str, Any]] = []
    for cause, count in counts.most_common():
        examples = [
            transition for transition in failures if transition.get("inferred_cause") == cause
        ]
        skills = sorted({str(transition.get("skill_name")) for transition in examples})
        recovery = str(examples[0].get("recovery") or "")
        patterns.append(
            {
                "cause": cause,
                "count": count,
                "skill_names": skills,
                "latest_message": str(examples[0].get("outcome_message") or ""),
                "recovery": recovery,
            }
        )
    return patterns


def _infer_domain(skill_name: str) -> str:
    if skill_name in {"navigate_with_text", "stop_navigation"}:
        return "navigation"
    if skill_name in {"relative_move", "execute_sport_command"}:
        return "robot_motion"
    if skill_name in {"follow_person", "stop_following"}:
        return "person_follow"
    if skill_name in {"look_out_for", "stop_looking_out"}:
        return "perception"
    if skill_name in {"start_security_patrol", "stop_security_patrol"}:
        return "security"
    if skill_name == "speak":
        return "speech"
    return ""


__all__ = ["CausalWorldModelSpec", "_Go2CausalWorldModel"]
