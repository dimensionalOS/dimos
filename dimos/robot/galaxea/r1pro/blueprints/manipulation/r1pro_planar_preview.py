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

"""R1 Pro whole-body planning and execution on mock hardware.

The chassis is a mock twist base that integrates its commands into
odometry, so base plans run through the same base trajectory task as on
the robot.
"""

import math

from dimos.control.components import HardwareComponent, HardwareType, make_twist_base_joints
from dimos.control.coordinator import TaskConfig
from dimos.core.coordination.blueprints import autoconnect
from dimos.robot.galaxea.r1pro.config import (
    R1PRO_PLANAR_BASE,
    R1PRO_UPPER_BODY_PLANNING_JOINTS,
    make_r1pro_planar_model_config,
)
from dimos.robot.manipulators.common.blueprints import coordinator, planner, trajectory_task

_r1pro_hardware = HardwareComponent(
    hardware_id="r1pro",
    hardware_type=HardwareType.WHOLE_BODY,
    joints=list(R1PRO_UPPER_BODY_PLANNING_JOINTS),
    adapter_type="mock_whole_body",
)
_chassis_hardware = HardwareComponent(
    hardware_id="chassis",
    hardware_type=HardwareType.BASE,
    joints=make_twist_base_joints("chassis"),
    adapter_type="mock_twist_base",
    adapter_kwargs={"integrate_odometry": True},
)
_vx_limit, _vy_limit, _wz_limit = R1PRO_PLANAR_BASE.velocity_limits

r1pro_planar_preview = autoconnect(
    planner(
        model=make_r1pro_planar_model_config(),
        visualization={"backend": "viser"},
        joint_state_aliases=dict(
            zip(_chassis_hardware.joints, R1PRO_PLANAR_BASE.joint_names, strict=True)
        ),
        trajectory_tasks={
            "joint_trajectory": list(R1PRO_UPPER_BODY_PLANNING_JOINTS),
            "base_trajectory": list(R1PRO_PLANAR_BASE.joint_names),
        },
    ),
    coordinator(
        hardware=[_r1pro_hardware, _chassis_hardware],
        tasks=[
            trajectory_task(_r1pro_hardware),
            TaskConfig(
                name="base_trajectory",
                type="planar_base_trajectory",
                joint_names=_chassis_hardware.joints,
                # The planner limits x and y separately, so a diagonal is faster.
                params={
                    "max_linear": math.hypot(_vx_limit, _vy_limit),
                    "max_angular": _wz_limit,
                },
            ),
        ],
    ),
)
