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

Rows (``go2_vqa.json``) are pure data emitted by :mod:`dimos.evals.generate` —
ground truth computed analytically from odom, quizzing the encoded odom
summary. Typing and scoring live here; the JSON stays behavior-free.
"""

from __future__ import annotations

import json
from pathlib import Path

from dimos.evals.scorers import exact, first_number, within, yes_no
from dimos.evals.types import PassiveEval, Suite

_ROWS = json.loads((Path(__file__).parent / "go2_vqa.json").read_text())

_generated: list[PassiveEval[float]] = [
    PassiveEval(
        id=str(row["id"]),
        inputs=str(row["q"]),
        expected=float(row["a"]),  # type: ignore[arg-type]
        parse=first_number,
        score=within(float(row["band"])),  # type: ignore[arg-type]
        context=(
            lambda s, name=str(row["stream"]), w=tuple(row["window"]):  # type: ignore[misc]
            s.streams[name].range_time(*w),
        ),
        dataset=str(row["dataset"]),
        tags=frozenset({"generated", "odom", "numeric"}),
    )
    for row in _ROWS
]

# Hand-labeled presence questions, verified against the recording imagery.
_hand: list[PassiveEval[str]] = [
    PassiveEval(
        id="hk_couch_seen",
        inputs="Did you see a couch or sofa at any point?",
        expected="yes",
        parse=yes_no,
        score=exact,
        context=(lambda s: s.streams.color_image.range_time(150, 250),),
        dataset="go2_hongkong_office",
        tags=frozenset({"image", "presence"}),
    ),
    PassiveEval(
        id="hk_plants_seen",
        inputs="Did you see any potted plants?",
        expected="yes",
        parse=yes_no,
        score=exact,
        context=(lambda s: s.streams.color_image.range_time(0, 60),),
        dataset="go2_hongkong_office",
        tags=frozenset({"image", "presence"}),
    ),
]

SUITE: Suite = [*_generated, *_hand]
