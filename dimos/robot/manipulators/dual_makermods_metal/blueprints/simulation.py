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

"""The Dual Metal stacks on the MuJoCo rig instead of the robot.

Each blueprint is the real one with its connection switched off and the
simulated rig in its place, so the coordinator, its tasks, the planner and the
teleop modules are the ones the robot runs. dual-metal-sim adds Scan and Pick
on the rig's overhead camera, as dual-openyam-grasp-sim has them.
"""

from typing import Any

from dimos.control.connection.mock_joint_connection import MockJointConnection
from dimos.core.coordination.blueprints import Blueprint, autoconnect
from dimos.manipulation.grasp_verification import GraspVerificationConfig
from dimos.robot.manipulators.damiao.damiao_blueprint import arm_declaration
from dimos.robot.manipulators.dual_makermods_metal.blueprints.basic import (
    dual_metal_control,
    dual_metal_planner,
)
from dimos.robot.manipulators.dual_makermods_metal.blueprints.teleop import (
    teleop_webxr_dual_metal,
)
from dimos.robot.manipulators.dual_makermods_metal.config import (
    FINGER_JOINTS,
    SIDES,
    dual_metal_model_config,
)
from dimos.robot.manipulators.dual_makermods_metal.connection import DualMetalConnection
from dimos.robot.manipulators.dual_makermods_metal.sim_connection import DualMetalSimConnection
from dimos.robot.manipulators.dual_openyam.blueprints.grasp import DUAL_OPENYAM_VOXEL_SIZE
from dimos.robot.manipulators.dual_openyam.blueprints.simulation import rig_pick_modules
from dimos.robot.manipulators.makermods_metal.config import METAL_HEURISTIC_AIM

# What the real blueprints drive the robot through: the Damiao connection, or
# under ``--simulation`` the mock that echoes its commands.
_ROBOT_CONNECTIONS = (DualMetalConnection, MockJointConnection)


def dual_metal_sim_connection(**options: Any) -> Blueprint:
    """The MuJoCo rig, declared and remapped as the connection it stands in for.

    Args:
        **options: Settings of the simulated rig, e.g. ``enable_pointcloud``.
    """
    return DualMetalSimConnection.blueprint(
        **arm_declaration(DualMetalConnection).model_dump(), **options
    ).remappings([(DualMetalSimConnection, "position_command", "arm_position_command")])


dual_metal_sim = autoconnect(
    dual_metal_control().disabled_modules(*_ROBOT_CONNECTIONS),
    dual_metal_planner(voxel_map_resolution=DUAL_OPENYAM_VOXEL_SIZE),
    *rig_pick_modules(
        # The joint state has no position for joint8, which mirrors joint7, so
        # the fingers stay fixed.
        self_filter_model=dual_metal_model_config(robot="arm").model.with_fixed_joints(
            *(joint for side in SIDES for joint in FINGER_JOINTS[side])
        ),
        heuristic_aim=METAL_HEURISTIC_AIM,
        # The single Metal sim's thresholds.
        grasp_verification=GraspVerificationConfig(),
    ),
    dual_metal_sim_connection(enable_pointcloud=True),
)
teleop_webxr_dual_metal_sim = autoconnect(
    teleop_webxr_dual_metal.disabled_modules(*_ROBOT_CONNECTIONS),
    dual_metal_sim_connection(),
)
