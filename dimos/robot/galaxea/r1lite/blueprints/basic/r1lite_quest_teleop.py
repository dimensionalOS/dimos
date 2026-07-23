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

"""R1 Lite quest teleop: bimanual arm tracking, grippers and chassis driving.

Runs the full R1 Lite control stack (connection, servo holder, chassis
velocity) plus two TeleopIKTask instances that track the Quest controllers
while the primary buttons are held. Replaces r1lite-coordinator while it runs;
never run both at once. Chassis software control needs RC mode 5.

The sim variant swaps the connection for mock adapters and renders the two
arms in viser; drive it with a real headset or scripts/r1lite_test/
fake_quest_stream.py. The torso is not rendered.

    dimos run r1lite-quest-teleop
    dimos run r1lite-quest-teleop-sim
"""

from __future__ import annotations

from dimos.control.components import HardwareComponent, HardwareType, make_twist_base_joints
from dimos.control.coordinator import ControlCoordinator, TaskConfig
from dimos.core.coordination.blueprints import autoconnect
from dimos.manipulation.manipulation_module import ManipulationModule
from dimos.manipulation.planning.spec.config import RobotModelConfig
from dimos.robot.galaxea.r1lite.blueprints.basic.r1lite_coordinator import (
    r1lite_control_base,
    r1lite_standard_tasks,
    r1lite_vis,
)
from dimos.robot.galaxea.r1lite.config import (
    R1LITE_ARM_DOF,
    R1LITE_LEFT_ARM_MODEL,
    R1LITE_RIGHT_ARM_MODEL,
)
from dimos.robot.galaxea.r1lite.connection import R1LITE_UPPER_BODY_JOINTS
from dimos.robot.manipulators._modeling import base_pose, joint_names
from dimos.robot.manipulators.a1z.config import (
    A1Z_COLLISION_EXCLUSIONS,
    A1Z_FLANGE_MODEL_PATH,
    A1Z_PACKAGE_PATHS,
)
from dimos.teleop.quest.quest_extensions import R1LiteQuestTeleopModule

_ARM_DOF = R1LITE_ARM_DOF

R1LITE_LEFT_ARM_JOINTS = R1LITE_UPPER_BODY_JOINTS[4:10]
R1LITE_RIGHT_ARM_JOINTS = R1LITE_UPPER_BODY_JOINTS[10:16]

_TASK_NAMES = {"left": "teleop_left_arm", "right": "teleop_right_arm"}
_TELEOP_PRIORITY = 20  # preempts the servo holder (10) on the arm joints while engaged

# The arms start folded, where small cartesian targets need more than the
# default 5 degree gate of joint motion, so solutions are rejected forever and
# the arm wedges. Accept locally sane solutions (30 degree hard reject) and
# execute them as bounded 0.5 degree steps per 100 Hz tick (50 deg/s cap).
_ARM_IK_LIMITS = {"max_joint_delta_deg": 30.0, "max_step_deg_per_tick": 0.5}


def _teleop_tasks() -> list[TaskConfig]:
    # Inline TaskConfig: joint_names must be the 6-joint arm slice, and the IK
    # solution maps onto them positionally. teleop_ik_task() would claim all 16.
    return [
        TaskConfig(
            name=_TASK_NAMES["left"],
            type="teleop_ik",
            joint_names=R1LITE_LEFT_ARM_JOINTS,
            priority=_TELEOP_PRIORITY,
            params={
                "model_path": R1LITE_LEFT_ARM_MODEL,
                "ee_joint_id": _ARM_DOF,
                "hand": "left",
                **_ARM_IK_LIMITS,
            },
        ),
        TaskConfig(
            name=_TASK_NAMES["right"],
            type="teleop_ik",
            joint_names=R1LITE_RIGHT_ARM_JOINTS,
            priority=_TELEOP_PRIORITY,
            params={
                "model_path": R1LITE_RIGHT_ARM_MODEL,
                "ee_joint_id": _ARM_DOF,
                "hand": "right",
                **_ARM_IK_LIMITS,
            },
        ),
    ]


r1lite_quest_teleop = autoconnect(
    R1LiteQuestTeleopModule.blueprint(task_names=_TASK_NAMES),
    r1lite_control_base(extra_tasks=_teleop_tasks()),
    r1lite_vis(),
).remappings(
    [
        (R1LiteQuestTeleopModule, "left_controller_output", "coordinator_cartesian_command"),
        (R1LiteQuestTeleopModule, "right_controller_output", "coordinator_cartesian_command"),
        (R1LiteQuestTeleopModule, "color_image", "head_left_color"),
    ]
)


def _sim_hardware() -> list[HardwareComponent]:
    return [
        HardwareComponent(
            hardware_id="r1lite",
            hardware_type=HardwareType.MANIPULATOR,
            joints=list(R1LITE_UPPER_BODY_JOINTS),
            adapter_type="mock",
        ),
        HardwareComponent(
            hardware_id="chassis",
            hardware_type=HardwareType.BASE,
            joints=make_twist_base_joints("chassis"),
            adapter_type="mock_twist_base",
        ),
    ]


def _sim_arm_model(side: str, y_offset: float) -> RobotModelConfig:
    # Viser needs meshes; the captured per-arm URDFs are geometry-stripped, so
    # the viewer renders the A1Z model as a visual stand-in. IK does not use it.
    return RobotModelConfig(
        name=f"{side}_arm",
        model_path=A1Z_FLANGE_MODEL_PATH,
        base_pose=base_pose(y=y_offset),
        joint_names=joint_names(_ARM_DOF, prefix="arm_joint"),
        end_effector_link="arm_link6",
        base_link="base_link",
        package_paths=A1Z_PACKAGE_PATHS,
        auto_convert_meshes=True,
        collision_exclusion_pairs=A1Z_COLLISION_EXCLUSIONS,
        joint_name_mapping={
            f"r1lite/{side}_arm_joint{i}": f"arm_joint{i}" for i in range(1, _ARM_DOF + 1)
        },
    )


r1lite_quest_teleop_sim = autoconnect(
    R1LiteQuestTeleopModule.blueprint(task_names=_TASK_NAMES),
    ControlCoordinator.blueprint(
        hardware=_sim_hardware(),
        tasks=[*r1lite_standard_tasks(), *_teleop_tasks()],
    ),
    ManipulationModule.blueprint(
        robots=[_sim_arm_model("left", 0.25), _sim_arm_model("right", -0.25)],
        visualization={"backend": "viser"},
    ),
).remappings(
    [
        (R1LiteQuestTeleopModule, "left_controller_output", "coordinator_cartesian_command"),
        (R1LiteQuestTeleopModule, "right_controller_output", "coordinator_cartesian_command"),
        (R1LiteQuestTeleopModule, "cmd_vel", "twist_command"),
    ]
)
