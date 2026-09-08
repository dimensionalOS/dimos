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

"""Flexible scoring for the SF-office PoseStamped autoresearch suite."""

from __future__ import annotations

from collections.abc import Callable, Sequence
import json
import math
from pathlib import Path
from typing import Any

import numpy as np
from scipy.optimize import linear_sum_assignment

from dimos.evals.types import Outcome

_ANSWERS = json.loads(Path(__file__).with_name("sf_office_pose_answers.json").read_text())[
    "answers"
]


def _clamp(value: float) -> float:
    return min(1.0, max(0.0, value)) if math.isfinite(value) else 0.0


def _falloff(error: float, full: float, zero: float) -> float:
    if not math.isfinite(error) or error >= zero:
        return 0.0
    if error <= full:
        return 1.0
    return (zero - error) / (zero - full)


def _number(value: Any) -> float | None:
    if isinstance(value, bool) or not isinstance(value, int | float):
        return None
    try:
        result = float(value)
    except OverflowError:
        return None
    return result if math.isfinite(result) else None


def _point(value: Any) -> tuple[float, float] | None:
    if not isinstance(value, list) or len(value) != 2:
        return None
    coordinates = tuple(_number(item) for item in value)
    if any(item is None for item in coordinates):
        return None
    x, y = coordinates
    assert x is not None and y is not None
    return x, y


def _interval(value: Any) -> tuple[float, float] | None:
    if isinstance(value, dict):
        value = [value.get("start_s"), value.get("end_s")]
    result = _point(value)
    return result if result is not None and result[0] <= result[1] else None


def _parse_json(text: str) -> dict[str, Any] | None:
    text = text.strip()
    if text.startswith("```") and text.endswith("```"):
        text = text[3:-3].strip()
        if text.startswith("json"):
            text = text[4:].strip()
    try:
        result = json.loads(text)
    except (json.JSONDecodeError, TypeError):
        return None
    return result if isinstance(result, dict) else None


def _numeric_score(
    answer: dict[str, Any], key: str, expected: float, full: float, zero: float
) -> float:
    value = _number(answer.get(key))
    return 0.0 if value is None else _falloff(abs(value - expected), full, zero)


def _point_score(
    answer: dict[str, Any], key: str, expected: Sequence[float], full: float, zero: float
) -> float:
    value = _point(answer.get(key))
    if value is None:
        return 0.0
    return _falloff(math.dist(value, expected), full, zero)


def _interval_score(value: Any, expected: Sequence[float], full: float, zero: float) -> float:
    interval = _interval(value)
    if interval is None:
        return 0.0
    error = (abs(interval[0] - expected[0]) + abs(interval[1] - expected[1])) / 2.0
    return _falloff(error, full, zero)


def _match_score(
    predicted: Sequence[Any],
    expected: Sequence[Any],
    similarity: Callable[[Any, Any], float],
) -> float:
    if not predicted or not expected:
        return float(not predicted and not expected)
    scores = np.array(
        [[similarity(candidate, reference) for reference in expected] for candidate in predicted]
    )
    rows, columns = linear_sum_assignment(-scores)
    return float(scores[rows, columns].sum() / max(len(predicted), len(expected)))


def _return_distance(answer: dict[str, Any]) -> float:
    expected = _ANSWERS["sf_office_pose_return_distance"]
    return _numeric_score(
        answer, "remaining_distance_m", expected["remaining_distance_m"], 0.03, 0.3
    )


def _stationary(answer: dict[str, Any]) -> float:
    expected = _ANSWERS["sf_office_pose_stationary_percentage"]
    duration = _numeric_score(answer, "stationary_time_s", expected["stationary_time_s"], 2.0, 25.0)
    percentage = _numeric_score(
        answer, "stationary_percent", expected["stationary_percent"], 0.5, 6.0
    )
    return (duration + percentage) / 2.0


def _backward_intervals(answer: dict[str, Any]) -> float:
    predicted = answer.get("backward_intervals_s")
    if not isinstance(predicted, list):
        return 0.0
    expected = _ANSWERS["sf_office_pose_backward_intervals"]["backward_intervals_s"]
    return _match_score(
        predicted,
        expected,
        lambda candidate, reference: _interval_score(
            candidate, _interval(reference) or (), 0.7, 4.0
        ),
    )


def _self_intersections(answer: dict[str, Any]) -> float:
    expected = _ANSWERS["sf_office_pose_self_intersections"]
    count = _numeric_score(answer, "intersection_count", expected["intersection_count"], 0.0, 6.0)
    predicted = answer.get("crossing_times_s")
    if not isinstance(predicted, list):
        times = 0.0
    else:
        times = _match_score(
            predicted,
            expected["crossing_times_s"],
            lambda candidate, reference: (
                0.0
                if (value := _number(candidate)) is None
                else _falloff(abs(value - reference), 0.8, 5.0)
            ),
        )
    return 0.6 * count + 0.4 * times


def _path_compression(answer: dict[str, Any]) -> float:
    expected = _ANSWERS["sf_office_pose_path_compression"]
    scores = [
        _numeric_score(answer, "original_length_m", expected["original_length_m"], 0.5, 8.0),
        _numeric_score(answer, "optimized_length_m", expected["optimized_length_m"], 0.5, 8.0),
        _numeric_score(answer, "reduction_percent", expected["reduction_percent"], 0.5, 5.0),
        _numeric_score(answer, "line_segments", expected["line_segments"], 1.0, 12.0),
        _numeric_score(answer, "turns", expected["turns"], 2.0, 12.0),
        _numeric_score(answer, "reverse_segments", expected["reverse_segments"], 0.0, 4.0),
        _numeric_score(answer, "in_place_rotations", expected["in_place_rotations"], 1.0, 6.0),
    ]
    return sum(scores) / len(scores)


def _opposite_retrace(answer: dict[str, Any]) -> float:
    expected = _ANSWERS["sf_office_pose_opposite_retrace"]
    current = _interval_score(answer.get("interval_s"), expected["interval_s"], 1.0, 6.0)
    earlier = _interval_score(
        answer.get("earlier_interval_s"), expected["earlier_interval_s"], 1.0, 6.0
    )
    length = _numeric_score(answer, "retrace_length_m", expected["retrace_length_m"], 0.3, 2.0)
    return 0.4 * current + 0.35 * earlier + 0.25 * length


def _longest_return(answer: dict[str, Any]) -> float:
    expected = _ANSWERS["sf_office_pose_longest_elapsed_return"]
    scores = [
        _point_score(answer, "location_m", expected["location_m"], 0.1, 1.0),
        _numeric_score(answer, "previous_time_s", expected["previous_time_s"], 1.0, 10.0),
        _numeric_score(answer, "return_time_s", expected["return_time_s"], 1.0, 10.0),
        _numeric_score(answer, "elapsed_s", expected["elapsed_s"], 1.0, 10.0),
    ]
    return sum(scores) / len(scores)


def _least_aligned(answer: dict[str, Any]) -> float:
    expected = _ANSWERS["sf_office_pose_least_aligned"]
    interval = _interval_score(answer.get("interval_s"), expected["interval_s"], 0.8, 5.0)
    center = _numeric_score(answer, "center_time_s", expected["center_time_s"], 0.8, 5.0)
    angle = _numeric_score(answer, "misalignment_deg", expected["misalignment_deg"], 3.0, 20.0)
    position = _point_score(answer, "position_m", expected["position_m"], 0.15, 1.0)
    return (interval + center + angle + position) / 4.0


def _repeated_cycle(answer: dict[str, Any]) -> float:
    expected = _ANSWERS["sf_office_pose_repeated_patrol_cycle"]
    value = answer.get("repeated_cycle")
    if not isinstance(value, bool) or value is not expected["repeated_cycle"]:
        return 0.0
    scores = [
        _numeric_score(answer, "start_time_s", expected["start_time_s"], 1.0, 6.0),
        _numeric_score(answer, "duration_s", expected["duration_s"], 1.0, 6.0),
        _numeric_score(answer, "length_m", expected["length_m"], 0.5, 3.0),
    ]
    return 0.25 + 0.75 * sum(scores) / len(scores)


_SCORERS: dict[str, Callable[[dict[str, Any]], float]] = {
    "sf_office_pose_return_distance": _return_distance,
    "sf_office_pose_stationary_percentage": _stationary,
    "sf_office_pose_backward_intervals": _backward_intervals,
    "sf_office_pose_self_intersections": _self_intersections,
    "sf_office_pose_path_compression": _path_compression,
    "sf_office_pose_opposite_retrace": _opposite_retrace,
    "sf_office_pose_longest_elapsed_return": _longest_return,
    "sf_office_pose_least_aligned": _least_aligned,
    "sf_office_pose_repeated_patrol_cycle": _repeated_cycle,
}


def score_answer(case_id: str, answer: dict[str, Any]) -> float:
    """Score one parsed answer, returning partial credit where meaningful."""
    return _clamp(_SCORERS[case_id](answer))


def grader(case_id: str) -> Callable[[Outcome], float]:
    """Create an EvalCase grader for one frozen pose question."""
    if case_id not in _SCORERS:
        raise KeyError(f"unknown pose autoresearch case: {case_id}")

    def grade(outcome: Outcome) -> float:
        answer = _parse_json(outcome.trajectory.final_answer)
        return 0.0 if answer is None else score_answer(case_id, answer)

    return grade
