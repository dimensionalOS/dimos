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

from dimos.message_utils import time as message_time
from dimos.messages.geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion


def point_to_pose_stamped(point: tuple[float, float]) -> PoseStamped:
    pose = PoseStamped(
        header=message_time.header_now(),
        pose=Pose(position=Point(0.0, 0.0, 0.0), orientation=Quaternion(0.0, 0.0, 0.0, 1.0)),
    )
    pose.pose.position.x = point[0]
    pose.pose.position.y = point[1]
    return pose


def pose_stamped_to_point(pose: PoseStamped) -> tuple[float, float]:
    return (pose.pose.position.x, pose.pose.position.y)
