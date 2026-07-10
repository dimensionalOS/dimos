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

from dataclasses import dataclass
from typing import Any


@dataclass(frozen=True)
class _CausalObservation:
    features: set[str]
    outcome_success: bool
    skill_name: str
    inferred_cause: str


class CausalEffectEstimator:
    """Observational feature-effect estimator for symbolic world transitions."""

    method = "observational_feature_effect"

    def __init__(self, max_observations: int = 500) -> None:
        self._max_observations = max_observations
        self._observations: list[_CausalObservation] = []

    def update(
        self,
        snapshot: dict[str, Any],
        action: dict[str, Any],
        outcome_success: bool | None,
        inferred_cause: str,
    ) -> None:
        if outcome_success is None:
            return
        self._observations.append(
            _CausalObservation(
                features=_causal_features(snapshot, action),
                outcome_success=outcome_success,
                skill_name=str(action.get("skill_name") or ""),
                inferred_cause=inferred_cause,
            )
        )
        if len(self._observations) > self._max_observations:
            self._observations = self._observations[-self._max_observations :]

    def estimate(self, snapshot: dict[str, Any], action: dict[str, Any]) -> dict[str, Any]:
        active = _causal_features(snapshot, action)
        skill_name = str(action.get("skill_name") or "")
        observations = [
            observation
            for observation in self._observations
            if not skill_name or observation.skill_name == skill_name
        ]
        if not observations:
            return self._empty(active)

        effects: list[dict[str, Any]] = []
        for feature in sorted(active):
            treated = [obs for obs in observations if feature in obs.features]
            control = [obs for obs in observations if feature not in obs.features]
            if not treated or not control:
                continue
            treated_rate = _smoothed_success_rate(treated)
            control_rate = _smoothed_success_rate(control)
            effect = treated_rate - control_rate
            effects.append(
                {
                    "feature": feature,
                    "effect_on_success": round(effect, 3),
                    "treated_success_rate": round(treated_rate, 3),
                    "control_success_rate": round(control_rate, 3),
                    "treated_count": len(treated),
                    "control_count": len(control),
                    "confidence": _effect_confidence(len(treated), len(control), effect),
                }
            )

        effects.sort(key=lambda item: float(item["effect_on_success"]))
        risk_factors = [effect for effect in effects if float(effect["effect_on_success"]) < -0.1]
        supports = [
            effect for effect in reversed(effects) if float(effect["effect_on_success"]) > 0.1
        ]
        return {
            "method": self.method,
            "assumption": "observational estimate; unobserved confounders are not controlled",
            "sample_count": len(observations),
            "active_features": sorted(active),
            "risk_factors": risk_factors[:5],
            "supporting_factors": supports[:5],
            "interventions": _interventions(risk_factors[:5]),
        }

    def snapshot(self) -> dict[str, Any]:
        return {
            "method": self.method,
            "sample_count": len(self._observations),
            "max_observations": self._max_observations,
        }

    def to_dict(self) -> dict[str, Any]:
        return {
            "method": self.method,
            "max_observations": self._max_observations,
            "observations": [
                {
                    "features": sorted(observation.features),
                    "outcome_success": observation.outcome_success,
                    "skill_name": observation.skill_name,
                    "inferred_cause": observation.inferred_cause,
                }
                for observation in self._observations
            ],
        }

    def load_dict(self, state: dict[str, Any]) -> None:
        self._max_observations = max(
            1,
            int(state.get("max_observations") or self._max_observations),
        )
        raw_observations = (
            state.get("observations") if isinstance(state.get("observations"), list) else []
        )
        observations: list[_CausalObservation] = []
        for item in raw_observations[-self._max_observations :]:
            if not isinstance(item, dict):
                continue
            features = item.get("features") if isinstance(item.get("features"), list) else []
            outcome_success = item.get("outcome_success")
            if not isinstance(outcome_success, bool):
                continue
            observations.append(
                _CausalObservation(
                    features={str(feature) for feature in features},
                    outcome_success=outcome_success,
                    skill_name=str(item.get("skill_name") or ""),
                    inferred_cause=str(item.get("inferred_cause") or ""),
                )
            )
        self._observations = observations

    def _empty(self, active: set[str]) -> dict[str, Any]:
        return {
            "method": self.method,
            "assumption": "observational estimate; unobserved confounders are not controlled",
            "sample_count": 0,
            "active_features": sorted(active),
            "risk_factors": [],
            "supporting_factors": [],
            "interventions": [],
        }


def _causal_features(snapshot: dict[str, Any], action: dict[str, Any]) -> set[str]:
    features: set[str] = set()
    skill_name = str(action.get("skill_name") or "")
    domain = str(action.get("domain") or "")
    args = action.get("args") if isinstance(action.get("args"), dict) else {}
    if skill_name:
        features.add(f"skill:{skill_name}")
    if domain:
        features.add(f"domain:{domain}")

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

    features.add(f"has_odom:{bool(robot_state.get('odom'))}")
    if "available" in spatial:
        features.add(f"spatial_available:{bool(spatial.get('available'))}")
    matches = spatial.get("matches") if isinstance(spatial.get("matches"), list) else []
    features.add(f"spatial_has_matches:{bool(matches)}")
    if "available" in temporal:
        features.add(f"temporal_available:{bool(temporal.get('available'))}")

    for key, value in args.items():
        features.add(f"arg_present:{key}:{bool(str(value).strip())}")
    return features


def _smoothed_success_rate(observations: list[_CausalObservation]) -> float:
    successes = sum(1 for observation in observations if observation.outcome_success)
    return (successes + 1.0) / (len(observations) + 2.0)


def _effect_confidence(treated_count: int, control_count: int, effect: float) -> str:
    support = min(treated_count, control_count)
    if support >= 5 and abs(effect) >= 0.25:
        return "high"
    if support >= 2 and abs(effect) >= 0.15:
        return "medium"
    return "low"


def _interventions(risk_factors: list[dict[str, Any]]) -> list[str]:
    suggestions: list[str] = []
    for factor in risk_factors:
        feature = str(factor.get("feature") or "")
        if feature == "spatial_has_matches:False":
            suggestions.append("Use perception or tag the target before semantic navigation.")
        elif feature == "spatial_available:False":
            suggestions.append("Wait for spatial memory or use direct perception.")
        elif feature == "has_odom:False":
            suggestions.append("Wait for odometry before motion-sensitive skills.")
        elif feature.startswith("arg_present:") and feature.endswith(":False"):
            suggestions.append("Fill the missing action argument before execution.")
    return suggestions


__all__ = ["CausalEffectEstimator"]
