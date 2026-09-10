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

"""Flexible, hidden scoring for the SF-office occupancy research suite."""

from __future__ import annotations

from collections.abc import Callable, Sequence
from dataclasses import dataclass
from functools import lru_cache
from itertools import pairwise
import json
import math
from pathlib import Path
from typing import Any

import numpy as np
from numpy.typing import NDArray
from scipy import ndimage
from scipy.optimize import linear_sum_assignment

from dimos.evals.types import Outcome
from dimos.memory.store.sqlite import SqliteStore

_ANSWERS = json.loads(Path(__file__).with_name("sf_office_occupancy_answers.json").read_text())[
    "answers"
]
_MAP_CASES = frozenset(
    {
        "sf_office_occupancy_three_point_loop",
        "sf_office_occupancy_largest_free_circle",
        "sf_office_occupancy_hide_location",
    }
)


@dataclass(frozen=True)
class MapData:
    clearance_m: NDArray[np.float64]
    reachable_025: NDArray[np.bool_]
    resolution_m: float
    origin_m: tuple[float, float]

    def cell(self, point: Sequence[float]) -> tuple[int, int] | None:
        column = round((point[0] - self.origin_m[0]) / self.resolution_m - 0.5)
        row = round((point[1] - self.origin_m[1]) / self.resolution_m - 0.5)
        if 0 <= row < self.clearance_m.shape[0] and 0 <= column < self.clearance_m.shape[1]:
            return row, column
        return None


def _clamp(value: float) -> float:
    return min(1.0, max(0.0, value)) if math.isfinite(value) else 0.0


def _falloff(error: float, full: float, zero: float) -> float:
    if not math.isfinite(error) or error >= zero:
        return 0.0
    if error <= full:
        return 1.0
    return (zero - error) / (zero - full)


def _number(value: Any) -> float | None:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return None
    result = float(value)
    return result if math.isfinite(result) else None


def _point(value: Any) -> tuple[float, float] | None:
    if not isinstance(value, list) or len(value) != 2:
        return None
    values = tuple(_number(item) for item in value)
    if values[0] is None or values[1] is None:
        return None
    return values[0], values[1]


def _distance(left: Sequence[float], right: Sequence[float]) -> float:
    return math.hypot(left[0] - right[0], left[1] - right[1])


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


@lru_cache(maxsize=2)
def _map_data(recording: str) -> MapData:
    with SqliteStore(path=recording, must_exist=True) as store:
        message = store.streams.global_costmap.last().data
        grid = message.grid.copy()
        resolution = float(message.info.resolution)
        origin_x = float(message.info.origin.position.x)
        origin_y = float(message.info.origin.position.y)
    free = (grid >= 0) & (grid < 50)
    clearance = ndimage.distance_transform_edt(free) * resolution
    safe = clearance >= 0.25
    source = (
        round((-4.2 - origin_y) / resolution - 0.5),
        round((0.0 - origin_x) / resolution - 0.5),
    )
    labels, _ = ndimage.label(safe, np.ones((3, 3), dtype=np.uint8))
    reachable = labels == labels[source] if labels[source] else np.zeros_like(safe)
    return MapData(clearance, reachable, resolution, (origin_x, origin_y))


def _boolean(answer: dict[str, Any], key: str, expected: bool) -> float:
    value = answer.get(key)
    return float(isinstance(value, bool) and value is expected)


def _numeric(answer: dict[str, Any], key: str, expected: float, full: float, zero: float) -> float:
    value = _number(answer.get(key))
    return 0.0 if value is None else _falloff(abs(value - expected), full, zero)


def _point_score(
    answer: dict[str, Any], key: str, expected: Sequence[float], full: float, zero: float
) -> float:
    value = _point(answer.get(key))
    return 0.0 if value is None else _falloff(_distance(value, expected), full, zero)


def _path_score(answer: dict[str, Any], map_data: MapData) -> float:
    raw = answer.get("waypoints_m")
    if not isinstance(raw, list):
        return 0.0
    maybe_points = [_point(item) for item in raw]
    if len(maybe_points) < 4 or any(point is None for point in maybe_points):
        return 0.0
    points = [point for point in maybe_points if point is not None]
    required = ((0.0, -4.2), (-7.0, -10.3), (3.3, -1.7))
    visits = sum(min(_distance(point, target) for point in points) <= 0.3 for target in required)
    required_score = (visits + (_distance(points[0], points[-1]) <= 0.3)) / 4

    valid_samples = 0
    total_samples = 0
    length = 0.0
    for start, end in pairwise(points):
        segment_length = _distance(start, end)
        length += segment_length
        count = max(1, math.ceil(segment_length / (map_data.resolution_m / 2)))
        for fraction in np.linspace(0.0, 1.0, count + 1):
            point = (
                start[0] + (end[0] - start[0]) * fraction,
                start[1] + (end[1] - start[1]) * fraction,
            )
            cell = map_data.cell(point)
            valid_samples += int(cell is not None and map_data.clearance_m[cell] >= 0.25)
            total_samples += 1
    collision_score = valid_samples / total_samples if total_samples else 0.0
    optimum = float(_ANSWERS["sf_office_occupancy_three_point_loop"]["optimal_length_m"])
    efficiency = _falloff(max(0.0, length - optimum), 0.5, 5.0)
    reported = _number(answer.get("length_m"))
    report_score = 0.0 if reported is None else _falloff(abs(reported - length), 0.25, 2.0)
    return 0.25 * required_score + 0.4 * collision_score + 0.25 * efficiency + 0.1 * report_score


def _circle_score(answer: dict[str, Any], map_data: MapData) -> float:
    center = _point(answer.get("center_m"))
    radius = _number(answer.get("radius_m"))
    if center is None or radius is None or radius <= 0:
        return 0.0
    cell = map_data.cell(center)
    if cell is None:
        return 0.0
    available = float(map_data.clearance_m[cell])
    validity = _falloff(max(0.0, radius - available), map_data.resolution_m, 0.5)
    optimum = float(_ANSWERS["sf_office_occupancy_largest_free_circle"]["radius_m"])
    quality = _clamp(radius / optimum)
    return validity * (0.25 + 0.75 * quality)


def _hide_score(answer: dict[str, Any], map_data: MapData) -> float:
    point = _point(answer.get("position_m"))
    if point is None:
        return 0.0
    cell = map_data.cell(point)
    if cell is None:
        return 0.0
    clearance = float(map_data.clearance_m[cell])
    traversable = _falloff(max(0.0, 0.25 - clearance), 0.0, 0.2)
    reachable = float(map_data.reachable_025[cell])
    reference = _ANSWERS["sf_office_occupancy_hide_location"]["position_m"]
    anchor = _falloff(_distance(point, reference), 0.75, 4.0)
    radius = max(1, round(2.0 / map_data.resolution_m))
    row, column = cell
    window = map_data.clearance_m[
        max(0, row - radius) : row + radius + 1,
        max(0, column - radius) : column + radius + 1,
    ]
    obstacle_density = float(np.count_nonzero(window == 0.0)) / window.size
    concealment = _clamp(obstacle_density / 0.35)
    return traversable * reachable * (0.4 + 0.35 * anchor + 0.25 * concealment)


def _doorway_score(answer: dict[str, Any]) -> float:
    raw = answer.get("doorways_m")
    if not isinstance(raw, list) or not raw:
        return 0.0
    predictions: list[tuple[tuple[float, float], float]] = []
    for item in raw:
        if not isinstance(item, list) or len(item) != 2:
            continue
        left, right = _point(item[0]), _point(item[1])
        if left is None or right is None:
            continue
        midpoint = ((left[0] + right[0]) / 2, (left[1] + right[1]) / 2)
        width_score = _falloff(abs(_distance(left, right) - 0.9), 0.3, 0.8)
        predictions.append((midpoint, width_score))
    if not predictions:
        return 0.0
    references = _ANSWERS["sf_office_occupancy_doorways"]["reference_centers_m"]
    distances = np.array(
        [
            [_distance(prediction[0], reference) for reference in references]
            for prediction in predictions
        ]
    )
    rows, columns = linear_sum_assignment(distances)
    credit = sum(
        _falloff(float(distances[row, column]), 0.35, 1.75) * predictions[row][1]
        for row, column in zip(rows, columns, strict=True)
    )
    recall = credit / len(references)
    precision = credit / len(predictions)
    return float(0.7 * recall + 0.3 * precision)


def _interval_score(answer: dict[str, Any]) -> float:
    raw = answer.get("intervals_s")
    if not isinstance(raw, list) or not raw:
        return 0.0
    predicted: list[tuple[float, float]] = []
    for item in raw:
        if not isinstance(item, list) or len(item) != 2:
            continue
        start, end = _number(item[0]), _number(item[1])
        if start is not None and end is not None and 440 <= start < end <= 464:
            predicted.append((start, end))
    if not predicted:
        return 0.0
    references = _ANSWERS["sf_office_occupancy_possible_person_motion"]["intervals_s"]

    def overlap(left: Sequence[float], right: Sequence[float]) -> float:
        intersection = max(0.0, min(left[1], right[1]) - max(left[0], right[0]))
        union = max(left[1], right[1]) - min(left[0], right[0])
        iou = intersection / union if union else 0.0
        boundary = _falloff(abs(left[0] - right[0]) + abs(left[1] - right[1]), 1.0, 6.0)
        return max(iou, 0.8 * boundary)

    quality = np.array(
        [[overlap(item, reference) for reference in references] for item in predicted]
    )
    rows, columns = linear_sum_assignment(-quality)
    credit = sum(float(quality[row, column]) for row, column in zip(rows, columns, strict=True))
    recall = credit / len(references)
    precision = credit / len(predicted)
    return 0.7 * recall + 0.3 * precision


def _bottleneck_score(answer: dict[str, Any]) -> float:
    raw = answer.get("opening_m")
    if not isinstance(raw, list) or len(raw) != 2:
        return 0.0
    left, right = _point(raw[0]), _point(raw[1])
    if left is None or right is None:
        return 0.0
    expected = _ANSWERS["sf_office_occupancy_doorway_bottleneck"]["opening_m"]
    expected_left, expected_right = expected
    midpoint = ((left[0] + right[0]) / 2, (left[1] + right[1]) / 2)
    expected_midpoint = (
        (expected_left[0] + expected_right[0]) / 2,
        (expected_left[1] + expected_right[1]) / 2,
    )
    location = _falloff(_distance(midpoint, expected_midpoint), 0.35, 2.0)
    angle = math.atan2(right[1] - left[1], right[0] - left[0])
    expected_angle = math.atan2(
        expected_right[1] - expected_left[1], expected_right[0] - expected_left[0]
    )
    angle_error = abs(math.degrees(angle - expected_angle)) % 180
    angle_error = min(angle_error, 180 - angle_error)
    orientation = _falloff(angle_error, 15.0, 70.0)
    width = _falloff(abs(_distance(left, right) - 0.9), 0.3, 0.8)
    return 0.6 * location + 0.25 * orientation + 0.15 * width


def _collision_score(answer: dict[str, Any]) -> float:
    if answer.get("collision") is not True:
        return 0.0
    reference = _ANSWERS["sf_office_occupancy_constant_twist_collision"]
    time_score = _numeric(answer, "time_s", float(reference["time_s"]), 0.15, 1.0)
    position_score = _point_score(answer, "position_m", reference["position_m"], 0.15, 0.75)
    return 0.25 + 0.4 * time_score + 0.35 * position_score


def score_answer(case_id: str, answer: dict[str, Any], map_data: MapData | None = None) -> float:
    """Score one parsed answer, with smooth credit for useful approximations."""
    reference = _ANSWERS[case_id]
    if case_id == "sf_office_occupancy_room_count":
        count = _number(answer.get("room_count"))
        if count is None:
            return 0.0
        error = abs(count - float(reference["room_count"]))
        return 1.0 if error < 0.5 else 0.85 if error < 1.5 else 0.35 if error < 2.5 else 0.0
    if case_id == "sf_office_occupancy_movement_square":
        return _point_score(answer, "center_m", reference["center_m"], 0.5, 1.5)
    if case_id == "sf_office_occupancy_three_point_loop":
        return _path_score(answer, _require_map(map_data))
    if case_id == "sf_office_occupancy_max_robot_radius":
        return _numeric(answer, "max_radius_m", float(reference["max_radius_m"]), 0.05, 0.25)
    if case_id == "sf_office_occupancy_blocked_opening_reachability":
        return _boolean(answer, "reachable", bool(reference["reachable"]))
    if case_id == "sf_office_occupancy_doorways":
        return _doorway_score(answer)
    if case_id == "sf_office_occupancy_largest_free_circle":
        return _circle_score(answer, _require_map(map_data))
    if case_id == "sf_office_occupancy_possible_person_motion":
        return _interval_score(answer)
    if case_id == "sf_office_occupancy_hide_location":
        return _hide_score(answer, _require_map(map_data))
    if case_id == "sf_office_occupancy_independent_routes":
        return _boolean(answer, "two_independent_routes", bool(reference["two_independent_routes"]))
    if case_id == "sf_office_occupancy_doorway_bottleneck":
        return _bottleneck_score(answer)
    if case_id == "sf_office_occupancy_constant_twist_collision":
        return _collision_score(answer)
    if case_id == "sf_office_occupancy_first_reachable_time":
        return _numeric(answer, "time_s", float(reference["time_s"]), 0.75, 4.0)
    if case_id == "sf_office_occupancy_known_free_area":
        return _numeric(
            answer, "known_free_area_m2", float(reference["known_free_area_m2"]), 1.0, 12.0
        )
    if case_id == "sf_office_occupancy_forward_clearance":
        return _boolean(answer, "clear", bool(reference["clear"]))
    raise KeyError(case_id)


def _require_map(map_data: MapData | None) -> MapData:
    if map_data is None:
        raise ValueError("this occupancy grader requires map data")
    return map_data


def grader(case_id: str) -> Callable[[Outcome], float]:
    """Build an Outcome grader without exposing references in the case prompt."""
    if case_id not in _ANSWERS:
        raise KeyError(case_id)

    def grade(outcome: Outcome) -> float:
        answer = _parse_json(outcome.trajectory.final_answer)
        if answer is None:
            return 0.0
        map_data = None
        if case_id in _MAP_CASES:
            map_data = _map_data(str(outcome.artifacts["recording"]))
        return _clamp(score_answer(case_id, answer, map_data))

    return grade
