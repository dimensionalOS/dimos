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

Available subclasses:
    - ArmTeleopModule: Per-hand toggle engage (X/A press to toggle)
    - TwistTeleopModule: Outputs Twist instead of PoseStamped
    - VisualizingTeleopModule: Adds Rerun visualization (uses toggle engage)
"""

from dataclasses import dataclass
from typing import Any

from dimos.core import Out
from dimos.msgs.geometry_msgs import PoseStamped, TwistStamped
from dimos.teleop.quest.quest_teleop_module import Hand, QuestTeleopConfig, QuestTeleopModule
from dimos.teleop.utils.teleop_visualization import (
    visualize_buttons,
    visualize_pose,
)


@dataclass
class TwistTeleopConfig(QuestTeleopConfig):
    """Configuration for TwistTeleopModule."""

    linear_scale: float = 1.0
    angular_scale: float = 1.0


# Example implementation to show how to extend QuestTeleopModule for different teleop behaviors and outputs.
class TwistTeleopModule(QuestTeleopModule):
    """Quest teleop that outputs TwistStamped instead of PoseStamped.

    Config:
        - linear_scale: Scale factor for linear (position) values. Default 1.0.
        - angular_scale: Scale factor for angular (orientation) values. Default 1.0.

    Outputs:
        - left_twist: TwistStamped (linear + angular velocity)
        - right_twist: TwistStamped (linear + angular velocity)
        - buttons: QuestButtons (inherited)
    """

    default_config = TwistTeleopConfig

    left_twist: Out[TwistStamped]
    right_twist: Out[TwistStamped]

    def _publish_msg(self, hand: Hand, output_msg: PoseStamped) -> None:
        """Convert PoseStamped to TwistStamped, apply scaling, and publish."""
        cfg: TwistTeleopConfig = self.config  # type: ignore[assignment]
        twist = TwistStamped(
            ts=output_msg.ts,
            frame_id=output_msg.frame_id,
            linear=output_msg.position * cfg.linear_scale,
            angular=output_msg.orientation.to_euler() * cfg.angular_scale,
        )
        if hand == Hand.LEFT:
            self.left_twist.publish(twist)
        else:
            self.right_twist.publish(twist)


class ArmTeleopModule(QuestTeleopModule):
    """Quest teleop with per-hand toggle engage.

    Each controller's primary button (X for left, A for right)
    toggles that hand's engage state independently.

    Outputs:
        - left_controller_output: PoseStamped (inherited)
        - right_controller_output: PoseStamped (inherited)
        - buttons: QuestButtons (inherited)
    """

    def __init__(self, *args: Any, **kwargs: Any) -> None:
        super().__init__(*args, **kwargs)
        self._prev_primary: dict[Hand, bool] = {Hand.LEFT: False, Hand.RIGHT: False}

    def _handle_engage(self) -> None:
        """Toggle per-hand engage on primary button rising edge."""
        for hand in Hand:
            controller = self._controllers.get(hand)
            if controller is None:
                continue

            pressed = controller.primary
            if pressed and not self._prev_primary[hand]:
                if self._is_engaged[hand]:
                    self._disengage(hand)
                else:
                    self._engage(hand)
            self._prev_primary[hand] = pressed


class VisualizingTeleopModule(ArmTeleopModule):
    """Quest teleop with Rerun visualization.

    Adds visualization of controller poses and trigger values to Rerun.
    Useful for debugging and development.

    Outputs:
        - left_controller_output: PoseStamped (inherited)
        - right_controller_output: PoseStamped (inherited)
        - buttons: QuestButtons (inherited)
    """

    def _get_output_pose(self, hand: Hand) -> PoseStamped | None:
        """Get output pose and visualize in Rerun."""
        output_pose = super()._get_output_pose(hand)

        if output_pose is not None:
            current_pose = self._current_poses.get(hand)
            controller = self._controllers.get(hand)
            if current_pose is not None:
                label = "left" if hand == Hand.LEFT else "right"
                visualize_pose(current_pose, label)

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
arm_teleop_module = ArmTeleopModule.blueprint
visualizing_teleop_module = VisualizingTeleopModule.blueprint

__all__ = [
    "ArmTeleopModule",
    "TwistTeleopModule",
    "VisualizingTeleopModule",
    "arm_teleop_module",
    "twist_teleop_module",
    "visualizing_teleop_module",
]
