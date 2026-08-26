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

"""Planner contract used to reject unsafe learned grasp proposals."""

from typing import Protocol

from dimos.manipulation.planning.spec.models import IKResult, RobotName
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.spec.utils import Spec


class GraspCandidateFilterSpec(Spec, Protocol):
    def inverse_kinematics_single(
        self,
        pose: Pose,
        robot_name: RobotName | None = None,
        seed: JointState | None = None,
        check_collision: bool = True,
    ) -> IKResult: ...
