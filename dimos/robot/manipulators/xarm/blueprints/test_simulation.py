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

from dimos.control.task import CoordinatorState, JointStateSnapshot
from dimos.control.tasks.eef_twist_task.eef_twist_task import create_task
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.TwistStamped import TwistStamped
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.robot.manipulators.xarm.blueprints.simulation import _xarm7_sim_twist_task

# A raw-interface run froze here: joint5 had drifted 1.1 mrad past the IK model's -3.1102 limit
# (the MJCF leaves it unbounded), and the twist task then dropped every command.
_PAST_LIMIT = [-1.153, 0.767, 1.056, 1.396, -3.1113, -0.385, 2.46]


def test_sim_twist_task_steers_a_wrist_back_from_just_past_its_limit() -> None:
    config = _xarm7_sim_twist_task(PoseStamped(frame_id="world", position=Vector3(z=0.912)))
    task = create_task(config, {})
    names = list(config.joint_names)
    positions = dict(zip(names, _PAST_LIMIT, strict=True))

    def state(t: float) -> CoordinatorState:
        return CoordinatorState(
            joints=JointStateSnapshot(joint_positions=dict(positions)), t_now=t, dt=0.01
        )

    task.compute(state(1.0))
    twist = TwistStamped(frame_id="link_tcp", linear=[0.0, 0.0, 0.02], angular=[0.0, 0.0, 0.3])
    assert task.on_ee_twist_command(twist, t_now=1.0)
    commands = 0
    for tick in range(1, 51):
        output = task.compute(state(1.0 + 0.01 * tick))
        if output is not None:
            commands += 1
            positions.update(zip(output.joint_names, output.positions, strict=True))

    assert commands == 50
    assert positions[names[4]] >= -3.1102
