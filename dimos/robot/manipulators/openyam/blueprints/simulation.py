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
teleop modules are the ones the robot runs. openyam-sim adds Scan and Pick on
the rig's overhead camera, as dual-openyam-grasp-sim has them.
"""

from dataclasses import replace
import math
from typing import Any

from dimos.control.connection.mock_joint_connection import MockJointConnection
from dimos.core.coordination.blueprints import Blueprint, autoconnect
from dimos.manipulation.grasp_verification import GraspVerificationConfig
from dimos.manipulation.planning.spec.config import RobotModelConfig
from dimos.robot.manipulators.common.blueprints import planner
from dimos.robot.manipulators.damiao.damiao_blueprint import arm_declaration
from dimos.robot.manipulators.dual_openyam.blueprints.grasp import DUAL_OPENYAM_VOXEL_SIZE
from dimos.robot.manipulators.dual_openyam.blueprints.simulation import rig_pick_modules
from dimos.robot.manipulators.openyam.blueprints.basic import openyam_control
from dimos.robot.manipulators.openyam.blueprints.teleop import teleop_webxr_openyam
from dimos.robot.manipulators.openyam.config import (
    OPENYAM_FINGER_JOINTS,
    make_openyam_model_config,
)
from dimos.robot.manipulators.openyam.connection import OpenYamConnection
from dimos.robot.manipulators.openyam.sim_connection import OpenYamSimConnection
from dimos.robot.urdf_manager import UrdfManager

# What the real blueprints drive the robot through: the Damiao connection, or
# under ``--simulation`` the mock that echoes its commands.
_ROBOT_CONNECTIONS = (OpenYamConnection, MockJointConnection)

#: The pick's tool tip, on arm_link_6 where the fingers end, less 1 mm: z
#: along the fingers, y along the jaw, as the heuristic grasp aims a tip, and
#: x out of the hand's top: a straight-down grasp then has the top facing away
#: from the base, which the arm reaches.
OPENYAM_TCP = "tcp"
_TCP_OUT = 0.149
# Where the gripper's parts sit along the approach from the tcp, metres, read
# off the collision meshes in arm_link_6: the pads' centre (the 2 cm nose
# behind the tip), the fingertips (0.1498) and the hand body's front (0.068).
OPENYAM_HEURISTIC_AIM = {
    "pad_behind_tcp": 0.009,
    "fingertip_reach": 0.001,
    "palm_behind_tcp": 0.081,
}


def openyam_grasp_model_config() -> RobotModelConfig:
    """The OpenYAM planned to its fingertip tcp, the tip a heuristic grasp aims."""
    config = make_openyam_model_config(robot="arm")
    config.model = config.model.with_fixed_frame(
        OPENYAM_TCP, "arm_link_6", xyz=(_TCP_OUT, 0.0, 0.0), rpy=(0.0, -math.pi / 2, math.pi)
    )
    config.planning_groups = [
        replace(group, tip_link=OPENYAM_TCP) for group in config.planning_groups
    ]
    return config


def openyam_sim_connection(**config: Any) -> Blueprint:
    """The MuJoCo rig, declared and remapped as the connection it stands in for.

    ``config`` sets the rig's own options, e.g. ``enable_pointcloud=True``.
    """
    return OpenYamSimConnection.blueprint(
        **arm_declaration(OpenYamConnection).model_dump(), **config
    ).remappings([(OpenYamSimConnection, "position_command", "arm_position_command")])


_GRASP_MODEL = openyam_grasp_model_config()

# UrdfManager serves the URDF to web 3D views. The planner already puts every
# link on tf, so the manager leaves joint tf off (on, links would get two parents).
openyam_sim = autoconnect(
    planner(
        model=_GRASP_MODEL,
        gripper_tasks={"manipulator": "arm_gripper"},
        default_speed_scale=0.25,
        voxel_map_resolution=DUAL_OPENYAM_VOXEL_SIZE,
    ),
    openyam_control().disabled_modules(*_ROBOT_CONNECTIONS),
    UrdfManager.blueprint(model=_GRASP_MODEL.model, publish_joint_tf=False),
    *rig_pick_modules(
        # The joint state's joint7 is the jaw motor's reading, which runs
        # opposite to the fingers' travel, so the fingers stay fixed.
        # with_fixed_joints takes the URDF's names, before the robot's prefix.
        self_filter_model=make_openyam_model_config(robot="arm").model.with_fixed_joints(
            *OPENYAM_FINGER_JOINTS
        ),
        heuristic_aim=OPENYAM_HEURISTIC_AIM,
        # The Dual OpenYAM's jaws: empty closes read 0.01 to 0.02, a held object
        # 0.05 or more, and a close must hold still for 1.5 s.
        grasp_verification=GraspVerificationConfig(
            empty_epsilon=0.035, settle_samples=30, timeout=5.0
        ),
    ),
    openyam_sim_connection(enable_pointcloud=True),
)
teleop_webxr_openyam_sim = autoconnect(
    teleop_webxr_openyam.disabled_modules(*_ROBOT_CONNECTIONS),
    openyam_sim_connection(),
)
