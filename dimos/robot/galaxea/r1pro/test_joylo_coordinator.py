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

import threading

from dimos.control.task import ControlMode, CoordinatorState, JointStateSnapshot
from dimos.control.tasks.velocity_task.twist_velocity_task import create_task
from dimos.control.tick_loop import TickLoop
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.robot.galaxea.r1pro.config import R1PRO_BODY_AXES
from dimos.robot.galaxea.r1pro.joylo_coordinator import joylo_tasks


def test_desktop_drive_wins_over_continuous_joylo_zero_until_it_expires():
    configs = {cfg.name: cfg for cfg in joylo_tasks() if cfg.type == "twist_velocity"}
    joylo = create_task(configs["vel_chassis"], None)
    desktop = create_task(configs["desktop_chassis"], None)
    loop = TickLoop(100, {}, threading.Lock(), {}, threading.Lock(), {})
    desktop.on_twist_command(Twist(linear=[0.4, 0, 0]), 1.0)
    joylo.on_twist_command(Twist.zero(), 1.1)

    def winners(now):
        state = CoordinatorState(joints=JointStateSnapshot(), t_now=now)
        return loop._arbitrate(
            [(task, task.claim(), task.compute(state)) for task in (joylo, desktop)]
        )[0]

    assert winners(1.1)[R1PRO_BODY_AXES[0]] == (0.4, ControlMode.VELOCITY, "desktop_chassis")
    assert winners(1.21)[R1PRO_BODY_AXES[0]] == (0.0, ControlMode.VELOCITY, "vel_chassis")
    joylo.on_twist_command(Twist(linear=[0.2, 0, 0]), 1.22)
    assert winners(1.22)[R1PRO_BODY_AXES[0]] == (0.2, ControlMode.VELOCITY, "vel_chassis")
