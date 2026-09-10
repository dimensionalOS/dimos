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
from pathlib import Path
from types import SimpleNamespace
from typing import cast

from dimos.evals.suites.sf_office_pose_grading import grader, score_answer
from dimos.evals.types import Outcome

_ANSWERS = {
    record["id"]: record["answer"]
    for record in json.loads(Path(__file__).with_name("sf_office_pose_qa.json").read_text())[
        "cases"
    ]
}


def test_all_reference_answers_receive_full_credit() -> None:
    for case_id, answer in _ANSWERS.items():
        assert score_answer(case_id, answer) == 1.0


def test_nearby_scalar_and_interval_answers_receive_partial_credit() -> None:
    distance = score_answer("sf_office_pose_return_distance", {"remaining_distance_m": 0.35})
    retrace = score_answer(
        "sf_office_pose_opposite_retrace",
        {
            "interval_s": [424.8, 433.2],
            "earlier_interval_s": [367.4, 374.9],
            "retrace_length_m": 4.5,
        },
    )

    assert 0.0 < distance < 1.0
    assert 0.0 < retrace < 1.0


def test_interval_matching_penalizes_missing_and_extra_predictions() -> None:
    reference = _ANSWERS["sf_office_pose_backward_intervals"]["backward_intervals_s"]
    exact = score_answer("sf_office_pose_backward_intervals", {"backward_intervals_s": reference})
    missing = score_answer(
        "sf_office_pose_backward_intervals", {"backward_intervals_s": reference[:-1]}
    )
    extra = score_answer(
        "sf_office_pose_backward_intervals",
        {"backward_intervals_s": [*reference, {"start_s": 400.0, "end_s": 410.0}]},
    )

    assert 0.0 < missing < exact
    assert 0.0 < extra < exact


def test_malformed_json_scores_zero_without_raising() -> None:
    outcome = cast(
        "Outcome",
        SimpleNamespace(trajectory=SimpleNamespace(final_answer="not json"), artifacts={}),
    )

    assert grader("sf_office_pose_return_distance")(outcome) == 0.0


def test_extreme_json_integer_scores_zero_without_raising() -> None:
    answer = {"remaining_distance_m": 10**1000}

    assert score_answer("sf_office_pose_return_distance", answer) == 0.0
