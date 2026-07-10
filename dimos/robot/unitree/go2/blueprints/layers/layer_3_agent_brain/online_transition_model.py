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

from collections import Counter
import math
from typing import Any


class OnlineTransitionOutcomeModel:
    """Small online model for predicting action outcome from world-state features."""

    model_type = "online_transition_outcome"

    def __init__(self, learning_rate: float = 0.35, l2: float = 0.001) -> None:
        self._learning_rate = learning_rate
        self._l2 = l2
        self._weights: dict[str, float] = {}
        self._sample_count = 0
        self._positive_count = 0
        self._negative_count = 0
        self._skill_counts: Counter[str] = Counter()

    def update(
        self,
        snapshot: dict[str, Any],
        action: dict[str, Any],
        outcome_success: bool | None,
    ) -> None:
        if outcome_success is None:
            return
        features = self._features(snapshot, action)
        label = 1.0 if outcome_success else 0.0
        probability = self._probability(features)
        error = label - probability

        for name, value in features.items():
            current = self._weights.get(name, 0.0)
            self._weights[name] = current + self._learning_rate * (
                error * value - self._l2 * current
            )

        self._sample_count += 1
        if outcome_success:
            self._positive_count += 1
        else:
            self._negative_count += 1
        skill_name = str(action.get("skill_name") or "")
        if skill_name:
            self._skill_counts[skill_name] += 1

    def predict(self, snapshot: dict[str, Any], action: dict[str, Any]) -> dict[str, Any]:
        features = self._features(snapshot, action)
        probability = self._probability(features)
        risk = self._risk(probability)
        return {
            "type": self.model_type,
            "sample_count": self._sample_count,
            "positive_count": self._positive_count,
            "negative_count": self._negative_count,
            "success_probability": round(probability, 3),
            "score": round(probability, 3),
            "risk": risk,
            "confidence": self._confidence(action),
            "top_features": self._top_features(features),
        }

    def snapshot(self) -> dict[str, Any]:
        return {
            "type": self.model_type,
            "sample_count": self._sample_count,
            "positive_count": self._positive_count,
            "negative_count": self._negative_count,
            "skill_counts": dict(self._skill_counts),
            "weight_count": len(self._weights),
        }

    def to_dict(self) -> dict[str, Any]:
        return {
            "type": self.model_type,
            "learning_rate": self._learning_rate,
            "l2": self._l2,
            "weights": dict(self._weights),
            "sample_count": self._sample_count,
            "positive_count": self._positive_count,
            "negative_count": self._negative_count,
            "skill_counts": dict(self._skill_counts),
        }

    def load_dict(self, state: dict[str, Any]) -> None:
        weights = state.get("weights") if isinstance(state.get("weights"), dict) else {}
        skill_counts = (
            state.get("skill_counts") if isinstance(state.get("skill_counts"), dict) else {}
        )
        self._learning_rate = float(state.get("learning_rate") or self._learning_rate)
        self._l2 = float(state.get("l2") or self._l2)
        self._weights = {str(key): float(value) for key, value in weights.items()}
        self._sample_count = max(0, int(state.get("sample_count") or 0))
        self._positive_count = max(0, int(state.get("positive_count") or 0))
        self._negative_count = max(0, int(state.get("negative_count") or 0))
        self._skill_counts = Counter(
            {str(key): max(0, int(value)) for key, value in skill_counts.items()}
        )

    def _probability(self, features: dict[str, float]) -> float:
        logit = sum(self._weights.get(name, 0.0) * value for name, value in features.items())
        return 1.0 / (1.0 + math.exp(-max(-20.0, min(20.0, logit))))

    def _confidence(self, action: dict[str, Any]) -> str:
        skill_count = self._skill_counts.get(str(action.get("skill_name") or ""), 0)
        if self._sample_count >= 10 and skill_count >= 4:
            return "high"
        if self._sample_count >= 2 and skill_count >= 1:
            return "medium"
        return "low"

    def _top_features(self, features: dict[str, float]) -> list[dict[str, Any]]:
        contributions = [
            {
                "feature": name,
                "value": round(value, 3),
                "weight": round(self._weights.get(name, 0.0), 3),
                "contribution": round(self._weights.get(name, 0.0) * value, 3),
            }
            for name, value in features.items()
            if self._weights.get(name, 0.0)
        ]
        contributions.sort(key=lambda item: abs(float(item["contribution"])), reverse=True)
        return contributions[:8]

    def _features(self, snapshot: dict[str, Any], action: dict[str, Any]) -> dict[str, float]:
        features: dict[str, float] = {"bias": 1.0}
        skill_name = str(action.get("skill_name") or "")
        domain = str(action.get("domain") or "")
        args = action.get("args") if isinstance(action.get("args"), dict) else {}
        if skill_name:
            features[f"skill:{skill_name}"] = 1.0
        if domain:
            features[f"domain:{domain}"] = 1.0

        robot_state = (
            snapshot.get("robot_state") if isinstance(snapshot.get("robot_state"), dict) else {}
        )
        memory_state = (
            snapshot.get("memory_state") if isinstance(snapshot.get("memory_state"), dict) else {}
        )
        spatial = (
            memory_state.get("spatial") if isinstance(memory_state.get("spatial"), dict) else {}
        )
        temporal = (
            memory_state.get("temporal") if isinstance(memory_state.get("temporal"), dict) else {}
        )
        navigation = (
            robot_state.get("navigation") if isinstance(robot_state.get("navigation"), dict) else {}
        )

        features["has_odom"] = 1.0 if robot_state.get("odom") else 0.0
        nav_state = str(navigation.get("state") or "")
        if nav_state:
            features[f"navigation_state:{nav_state}"] = 1.0
        if "available" in spatial:
            features[f"spatial_available:{bool(spatial.get('available'))}"] = 1.0
        spatial_matches = spatial.get("matches") if isinstance(spatial.get("matches"), list) else []
        features["spatial_match_count"] = min(5.0, math.log1p(len(spatial_matches)))
        if "available" in temporal:
            features[f"temporal_available:{bool(temporal.get('available'))}"] = 1.0

        for key, value in args.items():
            features[f"arg_present:{key}"] = 1.0 if str(value).strip() else 0.0
            if key in {"query", "target", "description"} and str(value).strip():
                normalized = str(value).strip().casefold()[:64]
                features[f"arg_value:{key}:{normalized}"] = 1.0

        return features

    @staticmethod
    def _risk(probability: float) -> str:
        if probability < 0.35:
            return "high"
        if probability < 0.65:
            return "medium"
        return "low"


__all__ = ["OnlineTransitionOutcomeModel"]
