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

"""The OpenYAM stacks on the MuJoCo rig instead of the robot.

Each blueprint is the real one with its connection switched off and the
simulated rig in its place, so the coordinator, its tasks, the planner and the
teleop modules are the ones the robot runs.
"""

from typing import Any

from dimos.control.connection.mock_joint_connection import MockJointConnection
from dimos.core.coordination.blueprints import Blueprint, autoconnect
from dimos.robot.manipulators.damiao.damiao_blueprint import arm_declaration
from dimos.robot.manipulators.openyam.blueprints.basic import openyam_planner_coordinator
from dimos.robot.manipulators.openyam.blueprints.teleop import teleop_webxr_openyam
from dimos.robot.manipulators.openyam.connection import OpenYamConnection
from dimos.robot.manipulators.openyam.sim_connection import OpenYamSimConnection

# What the real blueprints drive the robot through: the Damiao connection, or
# under ``--simulation`` the mock that echoes its commands.
_ROBOT_CONNECTIONS = (OpenYamConnection, MockJointConnection)


def openyam_sim_connection(**config: Any) -> Blueprint:
    """The MuJoCo rig, declared and remapped as the connection it stands in for.

    ``config`` sets the rig's own options, e.g. ``enable_pointcloud=True``.
    """
    return OpenYamSimConnection.blueprint(
        **arm_declaration(OpenYamConnection).model_dump(), **config
    ).remappings([(OpenYamSimConnection, "position_command", "arm_position_command")])


openyam_sim = autoconnect(
    openyam_planner_coordinator.disabled_modules(*_ROBOT_CONNECTIONS),
    openyam_sim_connection(),
)
teleop_webxr_openyam_sim = autoconnect(
    teleop_webxr_openyam.disabled_modules(*_ROBOT_CONNECTIONS),
    openyam_sim_connection(),
)
