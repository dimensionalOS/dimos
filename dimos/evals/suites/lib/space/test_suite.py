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

from pathlib import Path

import pytest

from dimos.evals.agents.blind import Blind
from dimos.evals.agents.lib.trajectory_builder import TrajectoryBuilder
from dimos.evals.agents.text_question import TextQuestion
from dimos.evals.suites.lib.space.data import Example
from dimos.evals.suites.lib.space.suite import Grade, SpacePrompt
from dimos.evals.suites.lib.space.upstream import OfficialSpace
from dimos.evals.types import Outcome


def test_space_rejects_prompt_augmentation_before_execution() -> None:
    environment = SpacePrompt()
    with pytest.raises(ValueError, match="without an added system prompt"):
        environment.preflight(TextQuestion(system_prompt="Extra guidance"))
    with pytest.raises(ValueError, match="TextQuestion"):
        environment.preflight(Blind())
    environment.preflight(TextQuestion())


def test_unfinished_response_is_never_scored_as_a_wrong_answer(tmp_path: Path) -> None:
    grade = Grade(OfficialSpace(tmp_path / "unprepared"), Example(0, {"question": "q"}))
    trajectory = TrajectoryBuilder("q", name="TextQuestion", model="test").build("timeout")
    with pytest.raises(RuntimeError, match="did not complete: timeout"):
        grade(Outcome(trajectory=trajectory, artifacts={}))
