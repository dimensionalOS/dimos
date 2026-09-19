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

"""Point-goal navigation tools over a planner that speaks ``goal`` / ``goal_reached``."""

from dimos_lcm.std_msgs import Bool

from dimos.agents.annotation import skill
from dimos.agents.capabilities import CAP_MOVEMENT
from dimos.core.core import rpc
from dimos.core.module import Module
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.PointStamped import PointStamped
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped

# Under the MCP client's 120 s call timeout.
MAX_WAIT_S = 100.0


class NavSkills(Module):
    goal: Out[PointStamped]
    stop_movement: Out[Bool]
    finished: Out[Bool]
    goal_reached: In[Bool]
    odom: In[PoseStamped]

    frame_id: str = "world"
    _z: float = 0.0  # the floor the robot stands on; goals go there, not to z = 0

    @rpc
    def start(self) -> None:
        super().start()
        self.odom.subscribe(self._on_odom)

    def _on_odom(self, pose: PoseStamped) -> None:
        self._z = pose.z

    @skill(uses=[CAP_MOVEMENT])
    def go_to(self, x: float, y: float, wait_s: float = 90.0) -> str:
        """Drive to a point in the world frame with the planner and wait for arrival.

        Args:
            x: target x in metres, world frame.
            y: target y in metres, world frame.
            wait_s: how long to wait for arrival before returning (max 100). The robot keeps
                driving after a timeout; call go_to again to keep waiting, or stop_navigation().
        """
        self.goal.publish(PointStamped(x, y, self._z, frame_id=self.frame_id))
        try:
            self.goal_reached.get_next(timeout=min(wait_s, MAX_WAIT_S))
        except Exception:
            return f"not at ({x:.2f}, {y:.2f}) after {wait_s:.0f}s; still driving"
        return f"reached ({x:.2f}, {y:.2f})"

    @skill
    def stop_navigation(self) -> str:
        """Cancel the current goal and stop moving."""
        self.stop_movement.publish(Bool(True))
        return "stopped"

    @skill
    def finish(self, note: str = "") -> str:
        """Declare the task complete; the evaluation stops timing here.

        Args:
            note: one line on what was achieved.
        """
        self.finished.publish(Bool(True))
        return f"finished: {note}" if note else "finished"
