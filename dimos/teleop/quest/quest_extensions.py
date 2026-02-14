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
    - ArmTeleopModule: Per-hand press-and-hold engage (X/A hold to track), task name routing
    - TwistTeleopModule: Outputs Twist instead of PoseStamped
    - VisualizingTeleopModule: Adds Rerun visualization (inherits press-and-hold engage)
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import TYPE_CHECKING, Any

from dimos.msgs.geometry_msgs import PoseStamped, TwistStamped
from dimos.teleop.quest.quest_teleop_module import Hand, QuestTeleopConfig, QuestTeleopModule
from dimos.teleop.utils.teleop_visualization import (
    visualize_buttons,
    visualize_pose,
)

if TYPE_CHECKING:
    from dimos.core import Out


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
        - buttons: Buttons (inherited)
    """

    default_config = TwistTeleopConfig
    config: TwistTeleopConfig

    left_twist: Out[TwistStamped]
    right_twist: Out[TwistStamped]

    def _publish_msg(self, hand: Hand, output_msg: PoseStamped) -> None:
        """Convert PoseStamped to TwistStamped, apply scaling, and publish."""
        twist = TwistStamped(
            ts=output_msg.ts,
            frame_id=output_msg.frame_id,
            linear=output_msg.position * self.config.linear_scale,
            angular=output_msg.orientation.to_euler() * self.config.angular_scale,
        )
        if hand == Hand.LEFT:
            self.left_twist.publish(twist)
        else:
            self.right_twist.publish(twist)


@dataclass
class ArmTeleopConfig(QuestTeleopConfig):
    """Configuration for ArmTeleopModule.

    Attributes:
        task_names: Mapping of Hand -> coordinator task name. Used to set
            frame_id on output PoseStamped so the coordinator routes each
            hand's commands to the correct TeleopIKTask.
    """

    task_names: dict[str, str] = field(default_factory=dict)


class ArmTeleopModule(QuestTeleopModule):
    """Quest teleop with per-hand press-and-hold engage and task name routing.

    Each controller's primary button (X for left, A for right)
    engages that hand while held, disengages on release.

    When task_names is configured, output PoseStamped messages have their
    frame_id set to the task name, enabling the coordinator to route
    each hand's commands to the correct TeleopIKTask.

    Outputs:
        - left_controller_output: PoseStamped (inherited)
        - right_controller_output: PoseStamped (inherited)
        - buttons: Buttons (inherited)
    """

    default_config = ArmTeleopConfig
    config: ArmTeleopConfig

    def __init__(self, *args: Any, **kwargs: Any) -> None:
        super().__init__(*args, **kwargs)

        self._task_names: dict[Hand, str] = {
            Hand[k.upper()]: v for k, v in self.config.task_names.items()
        }

    def _publish_msg(self, hand: Hand, output_msg: PoseStamped) -> None:
        """Stamp frame_id with task name and publish."""
        task_name = self._task_names.get(hand)
        if task_name:
            output_msg = PoseStamped(
                position=output_msg.position,
                orientation=output_msg.orientation,
                ts=output_msg.ts,
                frame_id=task_name,
            )
        super()._publish_msg(hand, output_msg)


class VisualizingTeleopModule(ArmTeleopModule):
    """Quest teleop with Rerun visualization.

    Adds visualization of controller poses and trigger values to Rerun.
    Useful for debugging and development.

    Outputs:
        - left_controller_output: PoseStamped (inherited)
        - right_controller_output: PoseStamped (inherited)
        - buttons: Buttons (inherited)
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
    "ArmTeleopConfig",
    "ArmTeleopModule",
    "TwistTeleopModule",
    "VisualizingTeleopModule",
    "arm_teleop_module",
    "twist_teleop_module",
    "visualizing_teleop_module",
]
