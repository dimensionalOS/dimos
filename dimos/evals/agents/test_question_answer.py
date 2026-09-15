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

from dataclasses import dataclass
import json
from typing import Any, cast

from dimos.evals.agents.question_answer import QuestionAnswer
from dimos.evals.types import RunningEnvironment


@dataclass
class _Projection:
    """Minimal encoder that renders calibrated views, as image-bearing types do."""

    AGENT_ENCODE_LEGEND = "format under test"

    def agent_encode(self) -> dict[str, Any]:
        return {
            "num_points": 2,
            "views": [
                {
                    "coordinate_min_m": [1.125, -2.25, 3.5],
                    "coordinate_max_m": [2.75, 1.0, 4.0],
                    "png_base64": "iVBORw0KGgo=",
                }
            ],
        }


@dataclass
class _Observation:
    ts: float
    data: _Projection


@dataclass
class _Stream:
    name: str
    observations: list[_Observation]

    def __iter__(self):  # type: ignore[no-untyped-def]
        return iter(self.observations)


def test_image_ablation_retains_exact_calibration_and_all_other_text() -> None:
    stream = _Stream("cloud", [_Observation(1.0, _Projection())])
    env = RunningEnvironment(mcp_url="", streams=(stream,), artifacts={})  # type: ignore[arg-type]

    visual = cast("list[dict[str, Any]]", QuestionAnswer()._observation_blocks(env))
    numeric = cast(
        "list[dict[str, Any]]", QuestionAnswer(include_images=False)._observation_blocks(env)
    )

    assert len(visual) == len(numeric) + 1
    assert all(block["type"] == "text" for block in numeric)
    assert numeric == [block for block in visual if block["type"] == "text"]
    calibration = json.loads(numeric[-1]["text"])
    assert calibration["coordinate_min_m"] == [1.125, -2.25, 3.5]
    assert calibration["coordinate_max_m"] == [2.75, 1.0, 4.0]
    assert "png_base64" not in calibration
