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

"""Alfred whole-robot planner on mock hardware, visualised in viser.

Same composition as ``alfred`` (base + pillar + both arms under one coordinator, planner with
viser) but every hardware component is in-memory, and the lift joins the trajectory task so
``plan_to_joints`` can drive lift + arms together against the full Alfred URDF (alfred_v2, with
caster joints animated from cmd_vel by CasterKinematics).

    dimos run alfred-sim          # viser at http://127.0.0.1:8095, WASD in the pygame window drives the base
"""

from __future__ import annotations

from dimos.control.blueprints.mobile import _mock_twist_base
from dimos.control.components import HardwareComponent, HardwareType, make_twist_base_joints
from dimos.control.coordinator import ControlCoordinator, TaskConfig
from dimos.control.tasks.trajectory_task.trajectory_task import joint_trajectory_task
from dimos.core.coordination.blueprints import autoconnect
from dimos.hardware.whole_body.spec import WholeBodyConfig
from dimos.robot.diy.alfred.alfred_model import alfred_sim_model_config
from dimos.robot.diy.alfred.blueprints.pillar import PILLAR_LIFT_VELOCITY_LIMIT_M_S
from dimos.robot.diy.alfred.caster_kinematics import (
    CASTER_HARDWARE_ID,
    CasterKinematics,
    caster_coordinator_joints,
)
from dimos.robot.diy.alfred.pillar_connection import PILLAR_HARDWARE_ID, PILLAR_LIFT_JOINT
from dimos.robot.manipulators.common.blueprints import planner
from dimos.robot.manipulators.openarm.config import OPENARM_ARM_JOINTS, openarm_hardware
from dimos.robot.unitree.keyboard_teleop import KeyboardTeleop

_base_joints = make_twist_base_joints("base")


# lift_joint follows the pillar firmware: 0 = top limit switch, range [-0.5, -0.002] m.
# With the lift at its bottom stop (-0.5) and the arms hanging straight down, the right gripper
# overlaps the lidar/ZED module box on the base (CAD: ~3 cm). Start the sim lift raised so
# the home configuration is collision-free; the planner refuses to plan into that zone.
SIM_LIFT_START_M = -0.25
CASTER_STREAM_VELOCITY_LIMIT = 100.0  # rad/s; display joints, not a controller
ARM_VELOCITY_LIMIT = 1.0  # rad/s, the trajectory task's own default


def mock_pillar_hardware() -> HardwareComponent:
    """In-memory stand-in for the LCM pillar: one metre-valued lift joint."""
    return HardwareComponent(
        hardware_id=PILLAR_HARDWARE_ID,
        hardware_type=HardwareType.WHOLE_BODY,
        joints=[PILLAR_LIFT_JOINT],
        adapter_type="mock_whole_body",
        auto_enable=True,
        adapter_kwargs={"initial_positions": [SIM_LIFT_START_M]},
        wb_config=WholeBodyConfig(kp=(0.0,), kd=(0.0,)),
    )


def mock_caster_hardware() -> HardwareComponent:
    """Eight caster joints (steer/drive per corner) driven by CasterKinematics for display."""
    joints = caster_coordinator_joints()
    return HardwareComponent(
        hardware_id=CASTER_HARDWARE_ID,
        hardware_type=HardwareType.WHOLE_BODY,
        joints=joints,
        adapter_type="mock_whole_body",
        auto_enable=True,
        wb_config=WholeBodyConfig(kp=(0.0,) * len(joints), kd=(0.0,) * len(joints)),
    )


alfred_sim = (
    autoconnect(
        planner(
            model=alfred_sim_model_config(wheels=True),
            visualization={"backend": "viser"},
        ),
        ControlCoordinator.blueprint(
            instance_name="ControlCoordinator",
            # openarm_hardware() is the mock whole-body adapter unless both CAN ports are set.
            hardware=[
                _mock_twist_base(),
                mock_pillar_hardware(),
                mock_caster_hardware(),
                openarm_hardware(),
            ],
            tasks=[
                TaskConfig(
                    name="vel_base",
                    type="velocity",
                    joint_names=_base_joints,
                    priority=10,
                    params={"timeout": 0.2, "zero_on_timeout": True},
                ),
                # One canonical trajectory task: planner executions (arms + lift) and the
                # streamed caster joint_command from CasterKinematics both land here. Casters
                # get a high velocity bound so the drive angle's ±π wrap is a one-tick jump.
                joint_trajectory_task(
                    [*OPENARM_ARM_JOINTS, PILLAR_LIFT_JOINT, *caster_coordinator_joints()],
                    # The task wants a limit for every joint once any is given.
                    velocity_limits={
                        **dict.fromkeys(OPENARM_ARM_JOINTS, ARM_VELOCITY_LIMIT),
                        PILLAR_LIFT_JOINT: PILLAR_LIFT_VELOCITY_LIMIT_M_S,
                        **dict.fromkeys(caster_coordinator_joints(), CASTER_STREAM_VELOCITY_LIMIT),
                    },
                ),
            ],
        ),
        KeyboardTeleop.blueprint(),  # WASD pygame window -> cmd_vel -> vel_base task
        CasterKinematics.blueprint(),  # cmd_vel -> caster steer/drive joint_command -> trajectory task (viser wheels)
    )
    .remappings([(ControlCoordinator, "twist_command", "cmd_vel")])
    .global_config(n_workers=4)
)
