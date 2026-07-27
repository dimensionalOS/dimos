# Copyright 2025-2026 Dimensional Inc.
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

from collections.abc import Iterator
from typing import Any

import pytest

from dimos.perception.perceive_loop_skill import PerceiveLoopSkill


@pytest.fixture
def perceive_loop_skill(mocker: Any) -> Iterator[tuple[PerceiveLoopSkill, Any]]:
    model = mocker.Mock()
    mocker.patch("dimos.perception.perceive_loop_skill.create", return_value=model)
    skill = PerceiveLoopSkill()
    yield skill, model
    skill.stop()


def test_lookout_model_stays_ready_between_sessions(
    perceive_loop_skill: tuple[PerceiveLoopSkill, Any],
) -> None:
    skill, model = perceive_loop_skill

    skill.start()
    stopped = skill.stop_looking_out()

    assert stopped == "Stopped looking out for []"
    model.start.assert_called_once_with()
    model.stop.assert_not_called()

    skill.stop()
    model.stop.assert_called_once_with()
