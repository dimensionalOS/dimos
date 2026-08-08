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

import pytest

from dimos.benchmark.short_horizon_qa.eval import parse_integer_prediction


@pytest.mark.parametrize(
    ("text", "status", "answer"),
    [
        ("I counted them.\nANSWER: 4", "parsed", 4),
        ("ANSWER: -2", "parsed", -2),
        ("The answer is 4", "invalid", None),
        ("ANSWER: 3\nthen maybe\nANSWER: 4", "invalid", None),
        ("ANSWER: 4\nextra", "invalid", None),
    ],
)
def test_marked_integer_parser(text: str, status: str, answer: int | None) -> None:
    prediction = parse_integer_prediction(text)
    assert prediction.status == status
    assert prediction.integer_answer == answer
