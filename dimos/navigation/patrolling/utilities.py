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

from dimos_generated.geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
from dimos_generated.std_msgs.msg import Header


def point_to_pose_stamped(point: Point, header: Header) -> PoseStamped:
    return PoseStamped(header=header, pose=Pose(position=point, orientation=Quaternion(w=1)))


def pose_stamped_to_point(pose: PoseStamped) -> tuple[float, float]:
    return (pose.pose.position.x, pose.pose.position.y)
