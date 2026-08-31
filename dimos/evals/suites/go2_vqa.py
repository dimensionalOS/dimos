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

Rows (``go2_vqa.json``) are pure data emitted by
:mod:`dimos.evals.suites.lib.generate` — ground truth computed analytically
from odom, quizzing the encoded odom summary. Typing and scoring live here;
the JSON stays behavior-free.
"""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any

from dimos.evals.environments.dataset import Dataset
from dimos.evals.scorers import exact, first_number, within, yes_no
from dimos.evals.types import EvalCase, Suite

_ROWS = json.loads((Path(__file__).parent / "go2_vqa.json").read_text())


def _generated(row: dict[str, Any]) -> EvalCase:
    stream, window = str(row["stream"]), tuple(row["window"])
    expected, band = float(row["a"]), float(row["band"])
    return EvalCase(
        id=str(row["id"]),
        inputs=str(row["q"]),
        environment=Dataset(
            str(row["dataset"]), select=(lambda s: s.streams[stream].range_time(*window),)
        ),
        grade=lambda o: within(band)(expected, first_number(o.trajectory.final_answer)),
        tags=frozenset({"generated", "odom", "numeric"}),
    )


# Hand-labeled presence questions, verified against the recording imagery.
_hand: list[EvalCase] = [
    EvalCase(
        id="hk_couch_seen",
        inputs="Did you see a couch or sofa at any point?",
        environment=Dataset(
            "go2_hongkong_office",
            select=(lambda s: s.streams.color_image.range_time(150, 250),),
        ),
        grade=lambda o: exact("yes", yes_no(o.trajectory.final_answer)),
        tags=frozenset({"image", "presence"}),
    ),
    EvalCase(
        id="hk_plants_seen",
        inputs="Did you see any potted plants?",
        environment=Dataset(
            "go2_hongkong_office", select=(lambda s: s.streams.color_image.range_time(0, 60),)
        ),
        grade=lambda o: exact("yes", yes_no(o.trajectory.final_answer)),
        tags=frozenset({"image", "presence"}),
    ),
]

SUITE: Suite = [*(_generated(row) for row in _ROWS), *_hand]
