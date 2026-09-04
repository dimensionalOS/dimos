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

import json
from types import SimpleNamespace

import numpy as np

from dimos.evals.suites.sf_office_occupancy_grading import MapData, grader, score_answer


def _open_map() -> MapData:
    return MapData(
        clearance_m=np.full((100, 100), 2.0),
        reachable_025=np.ones((100, 100), dtype=np.bool_),
        resolution_m=0.1,
        origin_m=(-5.0, -5.0),
    )


def test_room_count_gives_strong_credit_to_ambiguous_adjacent_counts() -> None:
    case_id = "sf_office_occupancy_room_count"

    assert score_answer(case_id, {"room_count": 4}) == 1.0
    assert score_answer(case_id, {"room_count": 3}) == 0.85
    assert score_answer(case_id, {"room_count": 5}) == 0.85
    assert score_answer(case_id, {"room_count": 20}) == 0.0


def test_numeric_answers_degrade_smoothly() -> None:
    case_id = "sf_office_occupancy_known_free_area"
    exact = score_answer(case_id, {"known_free_area_m2": 161.455})
    close = score_answer(case_id, {"known_free_area_m2": 166.0})
    far = score_answer(case_id, {"known_free_area_m2": 190.0})

    assert exact == 1.0
    assert 0.0 < close < exact
    assert far == 0.0


def test_valid_alternate_free_circle_receives_partial_credit() -> None:
    case_id = "sf_office_occupancy_largest_free_circle"
    score = score_answer(case_id, {"center_m": [3.0, 3.0], "radius_m": 1.5}, _open_map())

    assert 0.75 < score < 1.0


def test_doorway_matching_rewards_nearby_segments() -> None:
    case_id = "sf_office_occupancy_doorways"
    centers = [(-6.1, -1.2), (-1.5, 0.4), (2.7, -1.9), (5.4, -3.9)]
    exact = [[[x - 0.45, y], [x + 0.45, y]] for x, y in centers]
    nearby = [[[x - 0.25, y + 0.8], [x + 0.65, y + 0.8]] for x, y in centers]

    exact_score = score_answer(case_id, {"doorways_m": exact})
    nearby_score = score_answer(case_id, {"doorways_m": nearby})

    assert 0.0 < nearby_score < exact_score < 1.0


def test_malformed_json_scores_zero_without_raising() -> None:
    outcome = SimpleNamespace(trajectory=SimpleNamespace(final_answer="not json"), artifacts={})

    assert grader("sf_office_occupancy_room_count")(outcome) == 0.0
    fenced = SimpleNamespace(
        trajectory=SimpleNamespace(final_answer=f"```json\n{json.dumps({'room_count': 4})}\n```"),
        artifacts={},
    )
    assert grader("sf_office_occupancy_room_count")(fenced) == 1.0
