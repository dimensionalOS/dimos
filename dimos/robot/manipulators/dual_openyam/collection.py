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

"""The scaled WebXR teleop module for dual OpenYAM data collection."""

from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.teleop.webxr.controller_types import Hand
from dimos.teleop.webxr.extensions import ArmTeleopModule

# ArmTeleopConfig has no position scale (TwistTeleopConfig has linear_scale); a field there
# replaces this subclass.
DUAL_OPENYAM_COLLECTION_MOTION_SCALE = 0.75


def scale_controller_pose(pose: PoseStamped, scale: float) -> PoseStamped:
    """The same pose with its position scaled about the headset origin."""
    return PoseStamped(
        position=pose.position * scale,
        orientation=pose.orientation,
        ts=pose.ts,
        frame_id=pose.frame_id,
    )


class DualOpenYamCollectionTeleopModule(ArmTeleopModule):
    """ArmTeleopModule whose controller positions are scaled for a slower arm."""

    def _get_output_pose(self, hand: Hand) -> PoseStamped | None:
        pose = super()._get_output_pose(hand)
        if pose is None:
            return None
        return scale_controller_pose(pose, DUAL_OPENYAM_COLLECTION_MOTION_SCALE)
