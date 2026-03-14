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

"""
Booster K1 skill container for the new agents framework.
Provides movement skills for the K1 humanoid robot.
"""

from dimos.agents.annotation import skill
from dimos.core.core import rpc
from dimos.core.module import Module
from dimos.msgs.geometry_msgs import Twist, Vector3
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class BoosterK1SkillContainer(Module):
    rpc_calls: list[str] = [
        "K1Connection.move",
        "K1Connection.standup",
        "K1Connection.sit",
    ]

    @rpc
    def start(self) -> None:
        super().start()

    @rpc
    def stop(self) -> None:
        super().stop()

    @skill
    def move(self, x: float, y: float = 0.0, yaw: float = 0.0, duration: float = 0.0) -> str:
        """Move the robot using direct velocity commands. Determine duration required based on user distance instructions.

        Example call:
            args = { "x": 0.5, "y": 0.0, "yaw": 0.0, "duration": 2.0 }
            move(**args)

        Args:
            x: Forward velocity (m/s)
            y: Left/right velocity (m/s)
            yaw: Rotational velocity (rad/s)
            duration: How long to move (seconds)
        """
        move_rpc = self.get_rpc_calls("K1Connection.move")
        twist = Twist(linear=Vector3(x, y, 0), angular=Vector3(0, 0, yaw))
        move_rpc(twist, duration=duration)
        return f"Started moving with velocity=({x}, {y}, {yaw}) for {duration} seconds"

    @skill
    def standup(self) -> str:
        """Make the robot stand up from a sitting or damping position.

        Example call:
            standup()
        """
        standup_rpc = self.get_rpc_calls("K1Connection.standup")
        success = standup_rpc()
        if success:
            return "Robot is now standing."
        return "Failed to stand up."

    @skill
    def sit(self) -> str:
        """Make the robot sit down (lie down).

        Example call:
            sit()
        """
        sit_rpc = self.get_rpc_calls("K1Connection.sit")
        success = sit_rpc()
        if success:
            return "Robot is now sitting."
        return "Failed to sit down."


booster_k1_skills = BoosterK1SkillContainer.blueprint

__all__ = ["BoosterK1SkillContainer", "booster_k1_skills"]
