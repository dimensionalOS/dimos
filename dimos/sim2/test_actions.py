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


from dimos.sim2.actions import JointCommand, read_actions, world_action
from dimos.sim2.world_observations import WorldMirror, WorldWriter


def test_a_joint_command_to_a_robot_mounted_deep_in_a_scene_reads_back_as_that_robot_s() -> None:
    # A robot's id is its namespace, the mount frame it is on, so it can have several segments.
    robot = "office_555/left"
    writer, mirror = WorldWriter(), WorldMirror()
    writer.start_ns = 0
    writer.add(robot, model="g1.usda")
    writer.add(f"{robot}/left_knee_joint")
    mirror.apply(writer.end_frame(1.0))
    command = JointCommand(robot, ("left_knee_joint",), position=(0.3,), kp=(40.0,), kd=(1.0,))

    assert read_actions(world_action(mirror, command), writer) == [command]
