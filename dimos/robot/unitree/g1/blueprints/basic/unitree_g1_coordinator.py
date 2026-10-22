#!/usr/bin/env python3
# Copyright 2025-2026 Dimensional Inc.
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

"""G1 joint trajectory coordinator using the same robot connection as GR00T."""

from dimos.control.tasks.trajectory_task.trajectory_task import joint_trajectory_task
from dimos.core.coordination.blueprints import autoconnect
from dimos.hardware.whole_body.spec import WholeBodyConfig
from dimos.robot.unitree.g1.connection import G1Connection
from dimos.robot.unitree.g1.controllers.groot.config import G1_GROOT_KD, G1_GROOT_KP
from dimos.robot.unitree.g1.controllers.groot.coordinator import G1GrootCoordinator
from dimos.robot.unitree.g1.model.joints import g1_joints

unitree_g1_coordinator = autoconnect(
    G1Connection.blueprint(controller="groot"),
    G1GrootCoordinator.blueprint(
        instance_name="ControlCoordinator",
        publish_robot_joint_states=True,
        motor_gains={"g1": WholeBodyConfig(kp=tuple(G1_GROOT_KP), kd=tuple(G1_GROOT_KD))},
        tasks=[joint_trajectory_task(g1_joints, velocity_limits={j: 1.0 for j in g1_joints})],
    ),
).remappings(
    [
        (G1Connection, "motor_command", "g1_motor_command"),
        (G1Connection, "motor_states", "g1_motor_states"),
        (G1Connection, "imu", "g1_imu"),
    ]
)
