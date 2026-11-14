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

"""The Dual OpenYAM stacks on the MuJoCo rig instead of the robot.

Each blueprint is the real one with its connection switched off and the
simulated rig in its place, so the coordinator, its tasks, the planner and the
teleop modules are the ones the robot runs. The grasp stack has its three
RealSense cameras switched off as well: the simulated rig renders them.
"""

from typing import Any

from dimos.control.connection.mock_joint_connection import MockJointConnection
from dimos.core.coordination.blueprints import Blueprint, autoconnect
from dimos.hardware.sensors.camera.realsense.camera import RealSenseCamera
from dimos.robot.manipulators.damiao.damiao_blueprint import arm_declaration
from dimos.robot.manipulators.dual_openyam.blueprints.basic import (
    dual_openyam_planner_coordinator,
)
from dimos.robot.manipulators.dual_openyam.blueprints.grasp import dual_openyam_grasp
from dimos.robot.manipulators.dual_openyam.blueprints.teleop import teleop_webxr_dual_openyam
from dimos.robot.manipulators.dual_openyam.connection import DualOpenYamConnection
from dimos.robot.manipulators.dual_openyam.sim_connection import DualOpenYamSimConnection

# What the real blueprints drive the robot through: the Damiao connection, or
# under ``--simulation`` the mock that echoes its commands.
_ROBOT_CONNECTIONS = (DualOpenYamConnection, MockJointConnection)


def dual_openyam_sim_connection(**options: Any) -> Blueprint:
    """The MuJoCo rig, declared and remapped as the connection it stands in for.

    Args:
        **options: Settings of the simulated rig, e.g. ``enable_pointcloud``.
    """
    return DualOpenYamSimConnection.blueprint(
        **arm_declaration(DualOpenYamConnection).model_dump(), **options
    ).remappings([(DualOpenYamSimConnection, "position_command", "arm_position_command")])


dual_openyam_sim = autoconnect(
    dual_openyam_planner_coordinator.disabled_modules(*_ROBOT_CONNECTIONS),
    dual_openyam_sim_connection(),
)
teleop_webxr_dual_openyam_sim = autoconnect(
    teleop_webxr_dual_openyam.disabled_modules(*_ROBOT_CONNECTIONS),
    dual_openyam_sim_connection(),
)
dual_openyam_grasp_sim = autoconnect(
    dual_openyam_grasp.disabled_modules(*_ROBOT_CONNECTIONS, RealSenseCamera),
    # The planner's map is built from the overhead cloud, and the planner
    # publishes that camera's mount itself, as a static transform.
    dual_openyam_sim_connection(enable_pointcloud=True, publish_overhead_mount=False),
)
