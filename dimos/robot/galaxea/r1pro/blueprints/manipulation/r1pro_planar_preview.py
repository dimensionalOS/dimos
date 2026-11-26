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

A mock connection stands in for ``R1ProConnection``: the upper body takes
positions, the chassis takes body speeds and adds them up into odometry, so
base plans run through the same base trajectory task as on the robot.
"""

import math

from dimos.control.connection.connection_module import ConnectionConfig
from dimos.control.connection.mock_base_connection import MockBaseConnection
from dimos.control.contract.keys import Interface
from dimos.control.coordinator import ControlCoordinator, TaskConfig
from dimos.control.tasks.trajectory_task.trajectory_task import joint_trajectory_task
from dimos.core.coordination.blueprints import autoconnect
from dimos.core.stream import In, Out
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.robot.galaxea.r1pro.config import (
    R1PRO_BODY_AXES,
    R1PRO_CONTROL_MODEL,
    R1PRO_PLANAR_BASE,
    R1PRO_UPPER_BODY_PLANNING_JOINTS,
    make_r1pro_planar_model_config,
)
from dimos.robot.manipulators.common.blueprints import planner


class _PreviewCoordinator(ControlCoordinator):
    """The mock R1 Pro's ports, robot name "r1pro", plus the base's odometry."""

    r1pro_position_command: Out[JointState]
    r1pro_velocity_command: Out[JointState]
    r1pro_joint_state: In[JointState]
    odometry: In[Odometry]


_r1pro = ConnectionConfig(
    source="r1pro",
    model=R1PRO_CONTROL_MODEL,
    joint_commands={
        **dict.fromkeys(R1PRO_UPPER_BODY_PLANNING_JOINTS, (Interface.POSITION,)),
        **dict.fromkeys(R1PRO_BODY_AXES, (Interface.VELOCITY,)),
    },
)
_vx_limit, _vy_limit, _wz_limit = R1PRO_PLANAR_BASE.velocity_limits

r1pro_planar_preview = autoconnect(
    planner(
        model=make_r1pro_planar_model_config(),
        visualization={"backend": "viser"},
        trajectory_tasks={
            "joint_trajectory": list(R1PRO_UPPER_BODY_PLANNING_JOINTS),
            "base_trajectory": list(R1PRO_PLANAR_BASE.joint_names),
        },
    ),
    _PreviewCoordinator.blueprint(
        instance_name="ControlCoordinator",
        connections=[_r1pro],
        tasks=[
            joint_trajectory_task(R1PRO_UPPER_BODY_PLANNING_JOINTS),
            TaskConfig(
                name="base_trajectory",
                type="planar_base_trajectory",
                joint_names=list(R1PRO_BODY_AXES),
                # The planner limits x and y separately, so a diagonal is faster.
                params={
                    "max_linear": math.hypot(_vx_limit, _vy_limit),
                    "max_angular": _wz_limit,
                },
            ),
        ],
    ),
    MockBaseConnection.blueprint(**{**_r1pro.model_dump(), "instance_name": "r1pro"}),
).remappings(
    [
        ("r1pro", "position_command", "r1pro_position_command"),
        ("r1pro", "velocity_command", "r1pro_velocity_command"),
        ("r1pro", "joint_state", "r1pro_joint_state"),
    ]
)
