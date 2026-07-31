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

from dimos.robot.manipulators.xarm.config import make_xarm7_sim_robot_config


def test_xarm7_sim_pre_grasp_offset_keeps_gripper_clear_of_target() -> None:
    config = make_xarm7_sim_robot_config()

    assert config.pre_grasp_offset == 0.10
