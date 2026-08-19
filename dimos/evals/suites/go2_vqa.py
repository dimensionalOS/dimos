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

"""Generated VQA suite over the go2 replays.

Rows (``go2_vqa.json``) are pure data emitted by :func:`rows` — ground truth
computed analytically from odom, quizzing the encoded odom summary. Scoring
lives in :func:`dimos.evals.suites.lib.generate.cases`; the JSON stays
behavior-free.

Regenerate::

    uv run python -m dimos.evals.suites.go2_vqa
"""

from __future__ import annotations

import json
from pathlib import Path

from dimos.evals.environments.dataset import Dataset
from dimos.evals.scorers import exact, yes_no
from dimos.evals.suites.lib import generate
from dimos.evals.types import EvalCase, Suite

_JSON = Path(__file__).parent / "go2_vqa.json"
_SHORT = [(0.0, 60.0), (0.0, 30.0), (30.0, 60.0)]
_HONGKONG = [(0.0, 558.0), (100.0, 300.0)]

# Hand-labeled presence questions, verified against the recording imagery.
_hand: list[EvalCase] = [
    EvalCase(
        id="hk_couch_seen",
        inputs="Did you see a couch or sofa at any point? Answer yes or no.",
        environment=Dataset(
            "go2_hongkong_office",
            select=(lambda s: s.streams.color_image.range_time(150, 250),),
        ),
        grade=lambda o: exact("yes", yes_no(o.trajectory.final_answer)),
        tags=frozenset({"image", "presence"}),
    ),
    EvalCase(
        id="hk_plants_seen",
        inputs="Did you see any potted plants? Answer yes or no.",
        environment=Dataset(
            "go2_hongkong_office", select=(lambda s: s.streams.color_image.range_time(0, 60),)
        ),
        grade=lambda o: exact("yes", yes_no(o.trajectory.final_answer)),
        tags=frozenset({"image", "presence"}),
    ),
]

SUITE: Suite = [
    *generate.cases(json.loads(_JSON.read_text()), tags=frozenset({"generated", "odom"})),
    *_hand,
]


def rows() -> list[generate.Row]:
    """The generator calls behind the committed JSON."""
    return [
        *generate.displacement_rows("go2_short", _SHORT),
        *generate.path_length_rows("go2_short", _SHORT[:1]),
        *generate.displacement_rows("go2_hongkong_office", _HONGKONG),
        *generate.path_length_rows("go2_hongkong_office", _HONGKONG[:1]),
    ]


if __name__ == "__main__":
    _JSON.write_text(json.dumps(rows(), indent=2) + "\n")
