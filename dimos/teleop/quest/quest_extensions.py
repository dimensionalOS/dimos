#!/usr/bin/env python3
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

"""Quest teleop module extensions and subclasses.

This module contains various subclasses of QuestTeleopModule that customize
output formats or add additional functionality.

Available subclasses:
    - ToggleEngageTeleopModule: Press X to toggle engage (instead of hold)
    - TwistTeleopModule: Outputs Twist instead of PoseStamped
    - VisualizingTeleopModule: Adds Rerun visualization (uses toggle engage)
"""

from typing import Any

from dimos.core import Out
from dimos.msgs.geometry_msgs import PoseStamped, TwistStamped
from dimos.teleop.quest.quest_teleop_module import Hand, QuestTeleopModule
from dimos.teleop.utils.teleop_visualization import (
    init_rerun_visualization,
    visualize_buttons,
    visualize_pose,
)


class TwistTeleopModule(QuestTeleopModule):
    """Quest teleop that outputs TwistStamped instead of PoseStamped.
    Outputs:
        - left_twist: TwistStamped (linear + angular velocity)
        - right_twist: TwistStamped (linear + angular velocity)
        - buttons: QuestButtons (inherited)
    """

    left_twist: Out[TwistStamped]
    right_twist: Out[TwistStamped]

    def _publish_msg(self, hand: Hand, output_msg: PoseStamped) -> None:
        """Convert PoseStamped to TwistStamped and publish."""
        twist = TwistStamped(
            ts=output_msg.ts,
            frame_id=output_msg.frame_id,
            linear=output_msg.position,
            angular=output_msg.orientation.to_euler(),
        )
        if hand == Hand.LEFT:
            self.left_twist.publish(twist)
        else:
            self.right_twist.publish(twist)


class ArmTeleop(QuestTeleopModule):
    """Quest teleop with toggle-based engage.
    Press X button once to engage, press again to disengage.
    Outputs:
        - left_controller_output: PoseStamped (inherited)
        - right_controller_output: PoseStamped (inherited)
        - buttons: QuestButtons (inherited)
    """

    def __init__(self, *args: Any, **kwargs: Any) -> None:
        super().__init__(*args, **kwargs)
        self._prev_x_pressed = False

    def _handle_engage(self) -> None:
        """Toggle engage state on X button rising edge."""
        left = self._controllers.get(Hand.LEFT)
        if left is None:
            return
        x_pressed = left.primary

        # Rising edge detection: was not pressed, now pressed
        if x_pressed and not self._prev_x_pressed:
            if self._is_engaged:
                self.disengage()
            else:
                self.engage()
        self._prev_x_pressed = x_pressed


class VisualizingTeleopModule(ArmTeleop):
    """Quest teleop with Rerun visualization.

    Adds visualization of controller poses and trigger values to Rerun.
    Useful for debugging and development.
    """

    def start(self) -> None:
        """Start module and initialize Rerun visualization."""
        super().start()
        init_rerun_visualization()

    def _get_output_pose(self, hand: Hand) -> PoseStamped | None:
        """Get output pose and visualize in Rerun."""
        output_pose = super()._get_output_pose(hand)

        if output_pose is not None:
            current_pose = self._current_poses.get(hand)
            if current_pose is not None:
                label = "left" if hand == Hand.LEFT else "right"
                visualize_pose(current_pose, label)

                controller = self._controllers.get(hand)
                if controller:
                    visualize_buttons(
                        label,
                        primary=controller.primary,
                        secondary=controller.secondary,
                        grip=controller.grip,
                        trigger=controller.trigger,
                    )

        return output_pose


# Module blueprints for easy instantiation
twist_teleop_module = TwistTeleopModule.blueprint
arm_teleop_module = ArmTeleop.blueprint
visualizing_teleop_module = VisualizingTeleopModule.blueprint

__all__ = [
    "ArmTeleop",
    "TwistTeleopModule",
    "VisualizingTeleopModule",
    "arm_teleop_module",
    "twist_teleop_module",
    "visualizing_teleop_module",
]
