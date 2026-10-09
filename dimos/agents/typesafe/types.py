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
"""TypeSafe System One wire types: questions in, typed answers out."""

from __future__ import annotations

from collections.abc import Mapping, Sequence
from typing import Literal, TypedDict

from typing_extensions import NotRequired

Text = str | Mapping[str, object] | list[object]


class ChoiceQuestion(TypedDict):
    type: Literal["choice"]
    instructions: Text
    criteria: Mapping[str, Text | None]


class ScoreQuestion(TypedDict):
    type: Literal["score"]
    instructions: Text
    criteria: Sequence[Text]


class NoulQuestion(TypedDict):
    type: Literal["noul"]
    instructions: Text
    criteria: NotRequired[Mapping[str, Text]]


Question = ChoiceQuestion | ScoreQuestion | NoulQuestion


class ChoiceAnswer(TypedDict):
    type: Literal["choice"]
    choice: str
    confidence: float
    probabilities: dict[str, float]


class ScoreAnswer(TypedDict):
    type: Literal["score"]
    score: float
    confidence: float
    legend: dict[str, Text]
    probabilities: dict[str, float]


class NoulAnswer(TypedDict):
    type: Literal["noul"]
    noul: float


Answer = ChoiceAnswer | ScoreAnswer | NoulAnswer
Answers = dict[str, Answer]


def choice(instructions: Text, criteria: Mapping[str, Text | None]) -> ChoiceQuestion:
    return {"type": "choice", "instructions": instructions, "criteria": criteria}


def score(instructions: Text, criteria: Sequence[Text]) -> ScoreQuestion:
    return {"type": "score", "instructions": instructions, "criteria": criteria}


def noul(instructions: Text, criteria: Mapping[str, Text] | None = None) -> NoulQuestion:
    q: NoulQuestion = {"type": "noul", "instructions": instructions}
    if criteria is not None:
        q["criteria"] = criteria
    return q
