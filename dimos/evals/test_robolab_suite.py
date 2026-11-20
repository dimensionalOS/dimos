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
from typing import Any

import pytest

from dimos.evals.suites.robolab import SUITE, case_id, succeeded
from dimos.evals.types import Outcome


def test_suite_has_one_case_per_robolab_task() -> None:
    assert len(SUITE) == 120
    assert len({case.id for case in SUITE}) == 120
    difficulty = [t for case in SUITE for t in case.tags if t in {"simple", "moderate", "complex"}]
    assert (
        difficulty.count("simple"),
        difficulty.count("moderate"),
        difficulty.count("complex"),
    ) == (
        64,
        39,
        17,
    )


def test_case_ids_and_prompts() -> None:
    case = next(c for c in SUITE if c.id == "robolab_banana_in_bowl")

    assert case_id("BananaInBowlTask") == "robolab_banana_in_bowl"
    assert case.inputs.startswith("Pick up the banana and place it in the bowl. You control")
    assert "robolab_banana_in_bowl" in case.tags
    assert all(".." not in c.inputs for c in SUITE)


def _outcome(tmp_path: Path, rows: list[dict[str, Any]]) -> Outcome:
    results = tmp_path / "episode_results.jsonl"
    results.write_text("".join(json.dumps(row) + "\n" for row in rows))
    return Outcome(trajectory=None, artifacts={"robolab_results": results})  # type: ignore[arg-type]


@pytest.mark.parametrize(("success", "score"), [(True, 1.0), (False, 0.0)])
def test_grade_is_robolabs_success(tmp_path: Path, success: bool, score: float) -> None:
    assert succeeded(_outcome(tmp_path, [{"success": success, "score": 0.5}])) == score


def test_grade_without_a_result_row_is_a_fail(tmp_path: Path) -> None:
    assert succeeded(_outcome(tmp_path, [])) == 0.0
