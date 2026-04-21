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

from collections.abc import Callable
from typing import Any

from dimos.core.module import Module
from dimos.robot.unitree.g1.skill_container import UnitreeG1SkillContainer


def _mocked_move_rpc(calls: list[dict[str, Any]]) -> Callable[..., None]:
    def _move_rpc(*args: Any, **kwargs: Any) -> None:
        calls.append({"args": args, "kwargs": kwargs})

    return _move_rpc


def _setup_skill(calls: list[dict[str, Any]]) -> UnitreeG1SkillContainer:
    skill = UnitreeG1SkillContainer()
    Module.__init__(skill)
    move_rpc = _mocked_move_rpc(calls)
    skill.get_rpc_calls = lambda _: move_rpc  # type: ignore[method-assign]
    return skill


def test_navigate_3d_uses_planar_and_stair_segments() -> None:
    calls: list[dict[str, Any]] = []
    skill = _setup_skill(calls)

    result = skill.navigate_3d(x=1.0, y=1.0, z=0.34, speed=0.25)

    assert "3D navigation completed" in result
    assert len(calls) >= 4
    # First call should be yaw alignment with angular velocity only.
    first_twist = calls[0]["args"][0]
    assert first_twist.linear.x == 0.0
    assert first_twist.angular.z != 0.0
    # Second call should be planar motion.
    second_twist = calls[1]["args"][0]
    assert second_twist.linear.x > 0.0
    # Remaining calls are stair segments.
    for call in calls[2:]:
        assert call["args"][0].linear.x > 0.0


def test_climb_stairs_handles_zero_steps() -> None:
    calls: list[dict[str, Any]] = []
    skill = _setup_skill(calls)

    result = skill.climb_stairs(steps=0)

    assert result == "No stair traversal requested because steps=0."
    assert calls == []


def test_climb_stairs_negative_steps_moves_down() -> None:
    calls: list[dict[str, Any]] = []
    skill = _setup_skill(calls)

    result = skill.climb_stairs(steps=-3, step_height=0.17, speed=0.2)

    assert "down" in result
    assert len(calls) == 3
    assert all(call["args"][0].linear.x < 0.0 for call in calls)


def test_climb_stairs_rejects_unrealistic_height() -> None:
    calls: list[dict[str, Any]] = []
    skill = _setup_skill(calls)

    result = skill.climb_stairs(steps=2, step_height=0.4, speed=0.2)

    assert "Refusing stair traversal" in result
    assert calls == []
