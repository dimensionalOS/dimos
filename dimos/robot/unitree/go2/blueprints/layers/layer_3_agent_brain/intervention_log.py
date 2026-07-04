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
import time
from typing import Any


@dataclass(frozen=True)
class InterventionRecord:
    """One explicit do-operation and the observed result."""

    timestamp: float
    task: str
    intervention_name: str
    target_variable: str
    before_value: Any
    after_value: Any
    action: dict[str, Any]
    snapshot_before: dict[str, Any]
    snapshot_after: dict[str, Any]
    outcome_success: bool | None
    outcome_message: str
    causal_hypothesis: str

    def to_dict(self) -> dict[str, Any]:
        return {
            "timestamp": round(self.timestamp, 3),
            "task": self.task,
            "intervention_name": self.intervention_name,
            "target_variable": self.target_variable,
            "before_value": _bounded_jsonable(self.before_value),
            "after_value": _bounded_jsonable(self.after_value),
            "action": _bounded_jsonable(self.action),
            "snapshot_before": _bounded_jsonable(self.snapshot_before),
            "snapshot_after": _bounded_jsonable(self.snapshot_after),
            "outcome_success": self.outcome_success,
            "outcome_message": self.outcome_message,
            "causal_hypothesis": self.causal_hypothesis,
        }


class InterventionLog:
    """Bounded memory of agent interventions for causal reuse."""

    def __init__(self, max_records: int = 200) -> None:
        self._records: deque[InterventionRecord] = deque(maxlen=max_records)

    def record(
        self,
        *,
        task: str,
        intervention_name: str,
        target_variable: str,
        before_value: Any,
        after_value: Any,
        action: dict[str, Any],
        snapshot_before: dict[str, Any],
        snapshot_after: dict[str, Any],
        outcome_success: bool | None,
        outcome_message: str,
        causal_hypothesis: str,
    ) -> dict[str, Any]:
        record = InterventionRecord(
            timestamp=time.time(),
            task=task,
            intervention_name=intervention_name,
            target_variable=target_variable,
            before_value=before_value,
            after_value=after_value,
            action=action,
            snapshot_before=snapshot_before,
            snapshot_after=snapshot_after,
            outcome_success=outcome_success,
            outcome_message=outcome_message[:500],
            causal_hypothesis=causal_hypothesis[:500],
        )
        self._records.append(record)
        return record.to_dict()

    def recent(
        self,
        *,
        limit: int = 10,
        target_variable: str = "",
        intervention_name: str = "",
    ) -> list[dict[str, Any]]:
        limit = max(0, min(limit, self._records.maxlen or limit))
        target_variable = target_variable.strip()
        intervention_name = intervention_name.strip()
        records = [
            record
            for record in reversed(self._records)
            if (not target_variable or record.target_variable == target_variable)
            and (not intervention_name or record.intervention_name == intervention_name)
        ]
        return [record.to_dict() for record in records[:limit]]

    def evidence_for(
        self,
        causal_attribution: dict[str, Any],
        structural_causal_model: dict[str, Any],
    ) -> dict[str, Any]:
        risk_features = [
            str(factor.get("feature") or "")
            for factor in causal_attribution.get("risk_factors") or []
        ]
        variables = structural_causal_model.get("variables")
        if isinstance(variables, dict):
            risk_features.extend(_risk_features_from_variables(variables))

        target_variables = [_feature_variable(feature) for feature in risk_features]
        target_variables = [variable for variable in target_variables if variable]
        suggestions = self._suggestions(target_variables, risk_features)
        return {
            "type": "intervention_log",
            "record_count": len(self._records),
            "risk_features": sorted({feature for feature in risk_features if feature}),
            "matched_count": len(suggestions),
            "suggestions": suggestions[:5],
        }

    def snapshot(self) -> dict[str, Any]:
        successes = sum(1 for record in self._records if record.outcome_success is True)
        failures = sum(1 for record in self._records if record.outcome_success is False)
        target_counts = Counter(record.target_variable for record in self._records)
        intervention_counts = Counter(record.intervention_name for record in self._records)
        return {
            "type": "intervention_log",
            "record_count": len(self._records),
            "positive_count": successes,
            "negative_count": failures,
            "target_variables": dict(target_counts),
            "intervention_names": dict(intervention_counts),
        }

    def to_dict(self) -> dict[str, Any]:
        return {
            "type": "intervention_log",
            "max_records": self._records.maxlen,
            "records": [record.to_dict() for record in self._records],
        }

    def load_dict(self, state: dict[str, Any]) -> None:
        raw_records = state.get("records") if isinstance(state.get("records"), list) else []
        max_records = self._records.maxlen or len(raw_records)
        self._records.clear()
        for item in raw_records[-max_records:]:
            if not isinstance(item, dict):
                continue
            action = item.get("action") if isinstance(item.get("action"), dict) else {}
            snapshot_before = (
                item.get("snapshot_before") if isinstance(item.get("snapshot_before"), dict) else {}
            )
            snapshot_after = (
                item.get("snapshot_after") if isinstance(item.get("snapshot_after"), dict) else {}
            )
            outcome_success = item.get("outcome_success")
            if not isinstance(outcome_success, bool):
                outcome_success = None
            self._records.append(
                InterventionRecord(
                    timestamp=float(item.get("timestamp") or time.time()),
                    task=str(item.get("task") or ""),
                    intervention_name=str(item.get("intervention_name") or ""),
                    target_variable=str(item.get("target_variable") or ""),
                    before_value=item.get("before_value"),
                    after_value=item.get("after_value"),
                    action=action,
                    snapshot_before=snapshot_before,
                    snapshot_after=snapshot_after,
                    outcome_success=outcome_success,
                    outcome_message=str(item.get("outcome_message") or ""),
                    causal_hypothesis=str(item.get("causal_hypothesis") or ""),
                )
            )

    def _suggestions(
        self,
        target_variables: list[str],
        risk_features: list[str],
    ) -> list[dict[str, Any]]:
        variables = set(target_variables)
        grouped: dict[tuple[str, str], list[InterventionRecord]] = {}
        for record in self._records:
            if record.target_variable not in variables:
                continue
            grouped.setdefault((record.target_variable, record.intervention_name), []).append(
                record
            )

        suggestions: list[dict[str, Any]] = []
        for (target_variable, intervention_name), records in grouped.items():
            successful = [record for record in records if record.outcome_success is True]
            if not successful:
                continue
            latest = successful[-1]
            suggestions.append(
                {
                    "intervention_name": intervention_name,
                    "target_variable": target_variable,
                    "before_value": _bounded_jsonable(latest.before_value),
                    "after_value": _bounded_jsonable(latest.after_value),
                    "success_count": len(successful),
                    "total_count": len(records),
                    "success_rate": round(len(successful) / len(records), 3),
                    "latest_message": latest.outcome_message,
                    "causal_hypothesis": latest.causal_hypothesis,
                    "matched_risk_features": [
                        feature
                        for feature in risk_features
                        if _feature_variable(feature) == target_variable
                    ],
                }
            )

        suggestions.sort(
            key=lambda item: (float(item["success_rate"]), int(item["success_count"])),
            reverse=True,
        )
        return suggestions


def _feature_variable(feature: str) -> str:
    if not feature:
        return ""
    return feature.split(":", 1)[0]


def _risk_features_from_variables(variables: dict[str, Any]) -> list[str]:
    risk_features: list[str] = []
    if variables.get("spatial_has_matches") is False:
        risk_features.append("spatial_has_matches:False")
    if variables.get("spatial_available") is False:
        risk_features.append("spatial_available:False")
    if variables.get("odom_ready") is False:
        risk_features.append("has_odom:False")
    if variables.get("target_query_present") is False:
        risk_features.append("arg_present:query:False")
    return risk_features


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


__all__ = ["InterventionLog", "InterventionRecord"]
