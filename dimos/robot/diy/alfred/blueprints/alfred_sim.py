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

    dimos run alfred-sim    # viser at http://127.0.0.1:8095, WASD in the pygame window

Lift and both arms share one coordinator and one trajectory task against the full alfred_v2
URDF; the caster joints are animated from cmd_vel by CasterKinematics.
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
    CasterKinematics,
    caster_coordinator_joints,
    caster_mock_hardware,
)
from dimos.robot.diy.alfred.pillar_connection import PILLAR_HARDWARE_ID, PILLAR_LIFT_JOINT
from dimos.robot.manipulators.common.blueprints import planner
from dimos.robot.manipulators.openarm.config import OPENARM_ARM_JOINTS, openarm_hardware
from dimos.robot.unitree.keyboard_teleop import KeyboardTeleop

_base_joints = make_twist_base_joints("base")


# Raised so the all-zero arm pose is collision-free (see ALFRED_LIFT_SAFE_MIN_M).
SIM_LIFT_START_M = -0.25
CASTER_STREAM_VELOCITY_LIMIT = 100.0  # rad/s; display joints, not a controller
ARM_VELOCITY_LIMIT = 0.5  # rad/s


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


alfred_sim = (
    autoconnect(
        planner(
            model=alfred_sim_model_config(wheels=True),
            visualization={"backend": "viser"},
        ),
        ControlCoordinator.blueprint(
            instance_name="ControlCoordinator",
            hardware=[
                _mock_twist_base(),
                mock_pillar_hardware(),
                caster_mock_hardware(),
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
                # Casters stream through the same task; the bound lets the drive wrap in one tick.
                joint_trajectory_task(
                    [*OPENARM_ARM_JOINTS, PILLAR_LIFT_JOINT, *caster_coordinator_joints()],
                    velocity_limits={
                        **dict.fromkeys(OPENARM_ARM_JOINTS, ARM_VELOCITY_LIMIT),
                        PILLAR_LIFT_JOINT: PILLAR_LIFT_VELOCITY_LIMIT_M_S,
                        **dict.fromkeys(caster_coordinator_joints(), CASTER_STREAM_VELOCITY_LIMIT),
                    },
                ),
            ],
        ),
        KeyboardTeleop.blueprint(),
        CasterKinematics.blueprint(),
    )
    .remappings([(ControlCoordinator, "twist_command", "cmd_vel")])
    .global_config(n_workers=4)
)
