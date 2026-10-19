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

"""Geometry access shared by SVG and Rerun scene renderers."""

from dimos_generated.geometry_msgs.msg import Point, Pose, PoseStamped

from dimos.msgs.geometry import yaw


def message_position(message: Point | Pose | PoseStamped) -> Point:
    if isinstance(message, PoseStamped):
        return message.pose.position
    if isinstance(message, Pose):
        return message.position
    return message


def message_yaw(message: Pose | PoseStamped) -> float:
    pose = message.pose if isinstance(message, PoseStamped) else message
    return yaw(pose.orientation)
