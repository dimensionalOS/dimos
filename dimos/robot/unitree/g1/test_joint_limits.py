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

from dimos.robot.unitree.g1.joint_limits import g1_velocity_limits


def test_g1_velocity_limits_cover_all_controlled_joints() -> None:
    limits = g1_velocity_limits()

    assert len(limits) == 29
    assert limits["g1/left_shoulder_pitch"] == 37.0
    assert limits["g1/right_wrist_yaw"] == 22.0
