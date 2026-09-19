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
"""TypeSafe System One over its one HTTP endpoint: `POST /v1/systemone`."""

from __future__ import annotations

from collections.abc import Mapping
import os
import time
from typing import Literal, TypedDict

import requests
from typing_extensions import NotRequired

DEFAULT_MODEL = "jev-latest"
API_KEY_ENV = "TYPESAFE_API_KEY"
_RETRY_STATUSES = frozenset({429, 500, 502, 503, 504, 529})

Text = str | Mapping[str, object] | list[object]


class ChoiceQuestion(TypedDict):
    type: Literal["choice"]
    instructions: Text
    criteria: Mapping[str, Text | None]


class NoulQuestion(TypedDict):
    type: Literal["noul"]
    instructions: Text
    criteria: NotRequired[Mapping[str, Text]]


Question = ChoiceQuestion | NoulQuestion


class ChoiceAnswer(TypedDict):
    type: Literal["choice"]
    choice: str
    confidence: float
    probabilities: dict[str, float]


class NoulAnswer(TypedDict):
    type: Literal["noul"]
    noul: float


Answer = ChoiceAnswer | NoulAnswer
Answers = dict[str, Answer]


def choice(instructions: Text, criteria: Mapping[str, Text | None]) -> ChoiceQuestion:
    return {"type": "choice", "instructions": instructions, "criteria": criteria}


def noul(instructions: Text, criteria: Mapping[str, Text]) -> NoulQuestion:
    return {"type": "noul", "instructions": instructions, "criteria": criteria}


class SystemOne:
    def __init__(
        self, api_key: str, *, model: str = DEFAULT_MODEL, timeout_s: float = 10.0
    ) -> None:
        self._url = os.environ.get("TYPESAFE_BASE_URL", "https://api.typesafe.ai") + "/v1/systemone"
        self._model = model
        self._timeout_s = timeout_s
        self._session = requests.Session()
        self._session.headers["Authorization"] = f"Bearer {api_key}"
        self.last_usage: dict[str, int] = {}
        self.last_model = ""

    def __call__(self, state: object, questions: Mapping[str, Question]) -> Answers:
        body = {"state": state, "model": self._model, "questions": questions}
        for attempt in range(3):
            resp = self._session.post(self._url, json=body, timeout=self._timeout_s)
            if resp.status_code in _RETRY_STATUSES and attempt < 2:
                time.sleep(float(resp.headers.get("retry-after", 0.2 * 2**attempt)))
                continue
            if resp.status_code >= 400:
                raise RuntimeError(f"TypeSafe {resp.status_code}: {resp.text[:300]}")
            data = resp.json()
            self.last_usage = data.get("usage") or {}
            self.last_model = data.get("model", "")
            answers: Answers = data["answers"]
            return answers
        raise AssertionError("unreachable")

    def close(self) -> None:
        self._session.close()
