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

"""Skill container for the Deep Robotics M20 quadruped."""

from __future__ import annotations

import datetime
import math
import time

from dimos.agents.annotation import skill
from dimos.core.core import rpc
from dimos.core.module import Module
from dimos.msgs.geometry_msgs import PoseStamped, Quaternion, Vector3
from dimos.navigation.base import NavigationState
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class M20SkillContainer(Module):
    """LLM-callable skills for the Deep Robotics M20 quadruped."""

    rpc_calls: list[str] = [
        "NavigationInterface.set_goal",
        "NavigationInterface.get_state",
        "NavigationInterface.is_goal_reached",
        "NavigationInterface.cancel_goal",
        "M20Connection.standup",
        "M20Connection.sitdown",
        "M20Connection.set_gait",
        "M20Connection.emergency_stop",
        "M20Connection.move",
    ]

    @rpc
    def start(self) -> None:
        super().start()
        _ = self.tf

    @rpc
    def stop(self) -> None:
        super().stop()

    @skill
    def relative_move(
        self, forward: float = 0.0, left: float = 0.0, degrees: float = 0.0
    ) -> str:
        """Move the robot relative to its current position.

        Args:
            forward: Meters forward (negative = backward)
            left: Meters left (negative = right)
            degrees: Rotation in degrees (positive = left, negative = right)

        Examples:
            relative_move(forward=2, left=-1, degrees=0)   # 2m forward, 1m right
            relative_move(forward=-1)                       # 1m backward
            relative_move(degrees=-90)                      # Turn 90 degrees right
        """
        forward, left, degrees = float(forward), float(left), float(degrees)

        tf = self.tf.get("world", "base_link")
        if tf is None:
            return "Failed to get the position of the robot."

        try:
            set_goal_rpc, get_state_rpc, is_goal_reached_rpc = self.get_rpc_calls(
                "NavigationInterface.set_goal",
                "NavigationInterface.get_state",
                "NavigationInterface.is_goal_reached",
            )
        except Exception:
            logger.error("Navigation module not connected properly")
            return "Failed to connect to navigation module."

        set_goal_rpc(self._generate_new_goal(tf.to_pose(), forward, left, degrees))

        time.sleep(1.0)

        start_time = time.monotonic()
        timeout = 100.0
        while get_state_rpc() == NavigationState.FOLLOWING_PATH:
            if time.monotonic() - start_time > timeout:
                return "Navigation timed out"
            time.sleep(0.1)

        time.sleep(1.0)

        if not is_goal_reached_rpc():
            return "Navigation was cancelled or failed"
        else:
            return "Navigation goal reached"

    def _generate_new_goal(
        self,
        current_pose: PoseStamped,
        forward: float,
        left: float,
        degrees: float,
    ) -> PoseStamped:
        local_offset = Vector3(forward, left, 0)
        global_offset = current_pose.orientation.rotate_vector(local_offset)
        goal_position = current_pose.position + global_offset

        current_euler = current_pose.orientation.to_euler()
        goal_yaw = current_euler.yaw + math.radians(degrees)
        goal_euler = Vector3(current_euler.roll, current_euler.pitch, goal_yaw)
        goal_orientation = Quaternion.from_euler(goal_euler)

        return PoseStamped(position=goal_position, orientation=goal_orientation)

    @skill
    def standup(self) -> str:
        """Command the M20 robot to stand up."""
        try:
            standup_rpc = self.get_rpc_calls("M20Connection.standup")
            standup_rpc()
            return "Robot is standing up."
        except Exception:
            return "Failed to send standup command."

    @skill
    def sitdown(self) -> str:
        """Command the M20 robot to sit down."""
        try:
            sitdown_rpc = self.get_rpc_calls("M20Connection.sitdown")
            sitdown_rpc()
            return "Robot is sitting down."
        except Exception:
            return "Failed to send sitdown command."

    @skill
    def switch_gait(self, gait_name: str) -> str:
        """Switch the M20's walking gait.

        Args:
            gait_name: One of "standard", "high_obstacle", or "stairs"

        Examples:
            switch_gait("standard")       # Normal walking
            switch_gait("stairs")         # Stair climbing mode
            switch_gait("high_obstacle")  # High obstacle traversal
        """
        gait_map = {
            "standard": 0x1001,
            "high_obstacle": 0x1002,
            "stairs": 0x1003,
        }
        gait_name = gait_name.lower().strip()
        if gait_name not in gait_map:
            return f"Unknown gait '{gait_name}'. Choose: standard, high_obstacle, stairs"

        try:
            set_gait_rpc = self.get_rpc_calls("M20Connection.set_gait")
            set_gait_rpc(gait_map[gait_name])
            return f"Gait switched to {gait_name}."
        except Exception:
            return "Failed to switch gait."

    @skill
    def emergency_stop(self) -> str:
        """EMERGENCY STOP — immediately halt all robot motion."""
        try:
            estop_rpc = self.get_rpc_calls("M20Connection.emergency_stop")
            estop_rpc(True)
            return "EMERGENCY STOP ENGAGED. Call release_emergency_stop to resume."
        except Exception:
            return "Failed to engage emergency stop."

    @skill
    def release_emergency_stop(self) -> str:
        """Release the emergency stop, allowing robot motion to resume."""
        try:
            estop_rpc = self.get_rpc_calls("M20Connection.emergency_stop")
            estop_rpc(False)
            return "Emergency stop released. Robot can now move."
        except Exception:
            return "Failed to release emergency stop."

    @skill
    def wait(self, seconds: float) -> str:
        """Wait for a specified amount of time.

        Args:
            seconds: Seconds to wait
        """
        time.sleep(seconds)
        return f"Wait completed with length={seconds}s"

    @skill
    def current_time(self) -> str:
        """Provides current time."""
        return str(datetime.datetime.now())


m20_skills = M20SkillContainer.blueprint

__all__ = ["M20SkillContainer", "m20_skills"]
