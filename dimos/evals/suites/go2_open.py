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

"""Open-phrase questions over go2 replays whose answers are already verified.

The references are the same facts as ``go2_smoke`` and the hand labels in
``go2_vqa``: a person at the end of ``go2_short``, chairs at the start,
straight-line displacement 1.7 m and path length 37.9 m, a couch and potted
plants in ``go2_hongkong_office``, and no swimming pool. Letters are the
episodic catalog: D count, F presence, G path, L metric.

Replies are a short phrase. ``open_answer_checks.json`` holds paraphrases and
wrong answers with human marks for the judge; it is not part of this suite.
"""

from __future__ import annotations

from collections.abc import Callable
from typing import Any

from dimos.evals.environments.dataset import Dataset
from dimos.evals.open_answer import JUDGE_MODEL, OPEN_ANSWER_PASS, grade_open_answer
from dimos.evals.types import EvalCase, Suite

_PHRASE = "Answer in a short phrase. There are no choices."


def _open(
    case_id: str,
    question: str,
    reference: str,
    dataset: str,
    select: Callable[[Any], Any],
    letter: str,
) -> EvalCase:
    return EvalCase(
        id=case_id,
        inputs=f"{question}\n{_PHRASE}",
        environment=Dataset(dataset, select=(select,)),
        grade=grade_open_answer(question, reference),
        tags=frozenset({"open", letter}),
        threshold=OPEN_ANSWER_PASS,
        judge_model=JUDGE_MODEL,
    )


SUITE: Suite = [
    _open(
        "short_person_visible",
        "Is a person visible in any of these images?",
        "Yes, a person is visible.",
        "go2_short",
        lambda store: store.streams.color_image.range_time(40, 61),
        "F",
    ),
    _open(
        "short_start_furniture",
        "At the start of these observations, which furniture is most numerous?",
        "chairs",
        "go2_short",
        lambda store: store.streams.color_image.range_time(0, 8),
        "D",
    ),
    _open(
        "short_displacement",
        "How far in a straight line is your final position from your first shown position, in meters?",
        "about 1.7 meters",
        "go2_short",
        lambda store: store.streams.odom,
        "L",
    ),
    _open(
        "short_path_length",
        "Roughly how many meters did you travel in total, counting path length rather than straight-line displacement?",
        "about 37.9 meters",
        "go2_short",
        lambda store: store.streams.odom,
        "G",
    ),
    _open(
        "hk_couch_seen",
        "Did you see a couch or sofa at any point?",
        "Yes.",
        "go2_hongkong_office",
        lambda store: store.streams.color_image.range_time(150, 250),
        "F",
    ),
    _open(
        "hk_plants_seen",
        "Did you see any potted plants?",
        "Yes.",
        "go2_hongkong_office",
        lambda store: store.streams.color_image.range_time(0, 60),
        "F",
    ),
    _open(
        "hk_pool_absent",
        "Did you see a swimming pool anywhere in the observations?",
        "No.",
        "go2_hongkong_office",
        lambda store: store.streams.color_image,
        "F",
    ),
]
