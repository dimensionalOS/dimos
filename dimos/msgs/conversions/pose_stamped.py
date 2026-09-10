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

"""Agent-facing stamped-pose encoding."""

from __future__ import annotations

import math
from typing import TYPE_CHECKING, Any

if TYPE_CHECKING:
    from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped


def pose_stamped_agent_encode(pose: PoseStamped) -> dict[str, Any]:
    """Encode a stamped pose with explicit frames, component order, and units."""
    euler = pose.orientation.to_euler()
    return {
        "frame_id": pose.frame_id,
        "timestamp_s": pose.ts,
        "position_m": [pose.x, pose.y, pose.z],
        "quaternion_xyzw": pose.orientation.to_list(),
        "roll_pitch_yaw_deg": [
            math.degrees(euler.roll),
            math.degrees(euler.pitch),
            math.degrees(euler.yaw),
        ],
    }
