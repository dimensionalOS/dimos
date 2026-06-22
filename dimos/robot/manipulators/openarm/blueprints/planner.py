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

"""OpenArm planner + coordinator blueprints."""

from __future__ import annotations

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.transport import LCMTransport
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.robot.manipulators.common.blueprints import coordinator, planner
from dimos.robot.manipulators.openarm.blueprints.basic import openarm_task
from dimos.robot.manipulators.openarm.config import (
    LEFT_CAN,
    OPENARM_ADAPTER_KWARGS,
    RIGHT_CAN,
    openarm_hardware,
    openarm_model_config,
)

_mock_planner_left = openarm_hardware(side="left", scoped_joints=True)
_mock_planner_right = openarm_hardware(side="right", scoped_joints=True)

_planner_left_hw = openarm_hardware(
    side="left",
    address=LEFT_CAN,
    adapter_type="openarm",
    adapter_kwargs=OPENARM_ADAPTER_KWARGS,
    scoped_joints=True,
)
_planner_right_hw = openarm_hardware(
    side="right",
    address=RIGHT_CAN,
    adapter_type="openarm",
    adapter_kwargs=OPENARM_ADAPTER_KWARGS,
    scoped_joints=True,
)

openarm_mock_planner_coordinator = autoconnect(
    planner(
        robots=[
            openarm_model_config("left"),
            openarm_model_config("right"),
        ],
    ),
    coordinator(
        hardware=[_mock_planner_left, _mock_planner_right],
        tasks=[
            openarm_task(_mock_planner_left),
            openarm_task(_mock_planner_right),
        ],
    ),
).transports(
    {
        ("coordinator_joint_state", JointState): LCMTransport(
            "/coordinator/joint_state", JointState
        ),
    }
)

openarm_planner_coordinator = autoconnect(
    planner(
        robots=[
            openarm_model_config("left"),
            openarm_model_config("right"),
        ],
    ),
    coordinator(
        hardware=[_planner_left_hw, _planner_right_hw],
        tasks=[
            openarm_task(_planner_left_hw),
            openarm_task(_planner_right_hw),
        ],
    ),
).transports(
    {
        ("coordinator_joint_state", JointState): LCMTransport(
            "/coordinator/joint_state", JointState
        ),
    }
)
