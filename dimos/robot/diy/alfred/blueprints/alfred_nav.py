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

"""Alfred: click-and-go navigation + pillar lift + both OpenArms planned from viser.

    dimos run alfred-nav

Composition (nothing here re-implements navigation):

* ``alfred_mls_nav_lidar`` — Jeff's stack: mast D455 (IR pair + IMU) + Mid-360 (raw Livox,
  lidar IMU) into dimSLAM, ``RayTracingVoxelMap`` → ``MLSPlannerNative`` → ``DanLocalPlanner``
  → ``DanHolonomicTC`` → ``MovementManager`` (rerun click = goal, teleop/nav mux),
  ``AlfredMountTf`` (sensor mount tree off ``alfred.urdf``), and ``AlfredHighLevel`` as the
  ONLY writer to the FlowBase (Portal RPC + wheel odometry).
* ``PillarConnection`` + ``ControlCoordinator`` with the pillar (LCM transport adapter) and the
  OpenArms (Damiao CAN when both ports are given, mock otherwise): one joint trajectory task
  for lift + arms. No base hardware in the coordinator on purpose.
* ``ManipulationModule`` on the full ``alfred_v1`` model (viser on :8095): plan the ``lift``,
  ``left_manipulator`` and ``right_manipulator`` groups and execute through the coordinator.
* ``KeyboardTeleop`` (pygame WASD/QE) publishes ``tele_cmd_vel`` so the operator overrides
  navigation through the MovementManager mux.

Frames: navigation runs ``map → base_link`` from dimSLAM; the planner keeps its own scene
(``tf_extra_links`` empty) so nothing publishes a second ``world`` root onto tf.

Hardware pins: the OpenArm CAN ports come from ``OPENARM_LEFT_CAN`` / ``OPENARM_RIGHT_CAN``
(both or neither); the pillar serial device from ``PillarConnection`` config (``/dev/ttyUSB0``).
"""

from __future__ import annotations

import os

from dimos.control.components import HardwareComponent
from dimos.control.coordinator import ControlCoordinator, TaskConfig
from dimos.control.tasks.trajectory_task.trajectory_task import joint_trajectory_task
from dimos.core.coordination.blueprints import autoconnect
from dimos.robot.diy.alfred.alfred_model import alfred_arm_joints, alfred_model_config
from dimos.robot.diy.alfred.blueprints.alfred_mls_nav_lidar import alfred_mls_nav_lidar
from dimos.robot.diy.alfred.blueprints.pillar import (
    PILLAR_LIFT_VELOCITY_LIMIT_M_S,
    PILLAR_MOTOR_TRANSPORTS,
)
from dimos.robot.diy.alfred.pillar_connection import (
    PILLAR_LIFT_JOINT,
    PillarConnection,
    pillar_hardware,
)
from dimos.robot.manipulators.common.blueprints import planner
from dimos.robot.manipulators.openarm.config import openarm_hardware
from dimos.robot.unitree.keyboard_teleop import KeyboardTeleop

OPENARM_LEFT_CAN_ENV = "OPENARM_LEFT_CAN"
OPENARM_RIGHT_CAN_ENV = "OPENARM_RIGHT_CAN"
ARM_VELOCITY_LIMIT_RAD_S = 1.0


def _openarm_hardware_from_env() -> HardwareComponent:
    """Real Damiao arms only when both CAN ports are set; the mock adapter otherwise."""
    return openarm_hardware(
        left_can_port=os.environ.get(OPENARM_LEFT_CAN_ENV) or None,
        right_can_port=os.environ.get(OPENARM_RIGHT_CAN_ENV) or None,
    )


def alfred_manipulation_tasks() -> list[TaskConfig]:
    """The coordinator's single trajectory task: planner executions for arms + lift, and any
    streamed ``joint_command`` (velocity-bounded; the lift at the pillar's safe 0.1 m/s)."""
    return [
        joint_trajectory_task(
            [*alfred_arm_joints(), PILLAR_LIFT_JOINT],
            # The task wants a limit for every joint once any is given: arms keep the
            # task's own 1 rad/s default, the lift gets the pillar's safe speed.
            velocity_limits={
                **dict.fromkeys(alfred_arm_joints(), ARM_VELOCITY_LIMIT_RAD_S),
                PILLAR_LIFT_JOINT: PILLAR_LIFT_VELOCITY_LIMIT_M_S,
            },
        ),
    ]


alfred_nav = (
    autoconnect(
        alfred_mls_nav_lidar,
        PillarConnection.blueprint(),
        planner(
            model=alfred_model_config(),
            visualization={"backend": "viser"},
        ),
        ControlCoordinator.blueprint(
            instance_name="ControlCoordinator",
            hardware=[pillar_hardware(), _openarm_hardware_from_env()],
            tasks=alfred_manipulation_tasks(),
        ),
        KeyboardTeleop.blueprint(),
    )
    .transports(dict(PILLAR_MOTOR_TRANSPORTS))
    .remappings(
        [
            # Operator twist goes through MovementManager's teleop/nav mux, not to the base.
            (KeyboardTeleop, "cmd_vel", "tele_cmd_vel"),
        ]
    )
    .global_config(n_workers=14, robot_model="alfred")
)
