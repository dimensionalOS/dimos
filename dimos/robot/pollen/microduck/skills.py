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


"""Agent skills for the MicroDuck: navigation goals plus the policy task's
tricks (kicks, roulade, sit/stand), each one RPC into ControlCoordinator."""

from __future__ import annotations

from typing import Any

from dimos.agents.annotation import skill
from dimos.agents.skill_result import SkillResult
from dimos.control.coordinator import ControlCoordinator
from dimos.core.module import Module
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.std_msgs.Bool import Bool
from dimos.robot.pollen.microduck.blueprints.simulation import MICRODUCK_POLICY_TASK


class MicroduckSkills(Module):
    """go_to / stop / where_am_i on the nav stack; tricks on the policy task."""

    goal_request: Out[PoseStamped]
    stop_movement: Out[Bool]
    odom: In[PoseStamped]

    _control_coordinator: ControlCoordinator

    def _task(self, method: str, **kwargs: Any) -> str | SkillResult:
        result = self._control_coordinator.task_invoke(MICRODUCK_POLICY_TASK, method, kwargs)
        if isinstance(result, dict) and result.get("accepted") is False:
            return SkillResult.fail("REJECTED", str(result.get("reason", method)))
        return f"{method} {kwargs or ''}: ok".strip()

    @skill
    def go_to(self, x: float, y: float) -> str:
        """Walk to world coordinates (x, y) in metres. Returns immediately; the
        duck walks about 0.1 m/s, so use where_am_i to check progress."""
        self.goal_request.publish(PoseStamped(position=(x, y, 0.0), frame_id="world"))
        return f"navigating to ({x:.2f}, {y:.2f})"

    @skill
    def stop_moving(self) -> str:
        """Cancel navigation and stop walking."""
        self.stop_movement.publish(Bool(data=True))
        self._task("stop_motion")
        return "stopped"

    @skill
    def where_am_i(self) -> str | SkillResult:
        """Current world position (x, y) in metres."""
        try:
            pose = self.odom.get_next(timeout=2.0)
        except Exception:
            return SkillResult.fail("NO_ODOM", "no odometry received")
        return f"at ({pose.position.x:.2f}, {pose.position.y:.2f})"

    @skill
    def list_tricks(self) -> str:
        """Names of the one-shot motions perform() accepts."""
        skills = self._control_coordinator.task_invoke(MICRODUCK_POLICY_TASK, "list_skills")
        return ", ".join(s["name"] for s in skills or []) or "none"

    @skill
    def perform(self, name: str) -> str | SkillResult:
        """Run a one-shot motion by name (see list_tricks); the duck must be standing."""
        return self._task("run_skill", name=name)

    @skill
    def sit(self) -> str | SkillResult:
        """Sit down. The duck cannot walk while seated."""
        return self._task("set_posture", posture="sit")

    @skill
    def stand_up(self) -> str | SkillResult:
        """Stand up from sitting."""
        return self._task("set_posture", posture="stand")
