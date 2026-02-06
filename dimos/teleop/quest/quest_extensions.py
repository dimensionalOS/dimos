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
    - TwistTeleopModule: Outputs Twist instead of PoseStamped
    - VisualizingTeleopModule: Adds Rerun visualization
"""

from dimos.core import Out
from dimos.msgs.geometry_msgs import PoseStamped, Twist
from dimos.teleop.quest.quest_teleop_module import Hand, QuestTeleopModule
from dimos.teleop.utils.teleop_visualization import (
    init_rerun_visualization,
    visualize_pose,
    visualize_trigger_value,
)


class TwistTeleopModule(QuestTeleopModule):
    """Quest teleop that outputs Twist instead of PoseStamped.

    Converts pose deltas to velocity commands by treating position as
    linear velocity and orientation (converted to euler) as angular velocity.

    Outputs:
        - left_twist: Twist (linear + angular velocity)
        - right_twist: Twist (linear + angular velocity)
        - buttons: QuestButtons (inherited)

    Note: Inherited left_controller_delta and right_controller_delta
    still exist but are not published to (dormant).
    """

    left_twist: Out[Twist]
    right_twist: Out[Twist]

    def _publish_pose(self, hand: Hand, pose: PoseStamped) -> None:
        """Convert PoseStamped to Twist and publish."""
        twist = Twist(
            linear=pose.position,
            angular=pose.orientation,  # Quaternion -> euler via Twist dispatch
        )

        if hand == Hand.LEFT:
            self.left_twist.publish(twist)
        else:
            self.right_twist.publish(twist)


class VisualizingTeleopModule(QuestTeleopModule):
    """Quest teleop with Rerun visualization.

    Adds visualization of controller poses and trigger values to Rerun.
    Useful for debugging and development.

    Outputs:
        - left_controller_delta: PoseStamped (inherited)
        - right_controller_delta: PoseStamped (inherited)
        - buttons: QuestButtons (inherited)
        - Rerun visualization of poses and triggers
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
                trigger_value = controller.trigger if controller else 0.0
                visualize_trigger_value(trigger_value, label)

        return output_pose


# Module blueprints for easy instantiation
twist_teleop_module = TwistTeleopModule.blueprint
visualizing_teleop_module = VisualizingTeleopModule.blueprint

__all__ = [
    "TwistTeleopModule",
    "VisualizingTeleopModule",
    "twist_teleop_module",
    "visualizing_teleop_module",
]
