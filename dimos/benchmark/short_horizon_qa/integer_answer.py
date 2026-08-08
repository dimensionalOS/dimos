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

"""Integer-answer decoding and private oracle loading for frozen-memory questions."""

from __future__ import annotations

from pathlib import Path
import re

from dimos.benchmark.short_horizon_qa.models import (
    ExactIntegerOracle,
    FrozenIntegerQaCase,
    IntegerPrediction,
)

_ANSWER_LINE = re.compile(r"(?m)^ANSWER:\s*")
_TERMINAL_INTEGER = re.compile(r"(?:^|\n)ANSWER:\s*(-?\d+)\s*\Z")


def load_exact_integer_oracle(case: FrozenIntegerQaCase, case_root: Path) -> ExactIntegerOracle:
    reference = case.validator
    root = case_root.resolve()
    path = (root / reference.private_path).resolve()
    if root not in path.parents:
        raise ValueError("private oracle path escapes the case directory")
    return ExactIntegerOracle.model_validate_json(path.read_bytes())


def parse_integer_prediction(final_text: str) -> IntegerPrediction:
    match = _TERMINAL_INTEGER.search(final_text)
    if len(_ANSWER_LINE.findall(final_text)) != 1 or match is None:
        return IntegerPrediction(status="invalid")
    return IntegerPrediction(status="parsed", integer_answer=int(match.group(1)))
