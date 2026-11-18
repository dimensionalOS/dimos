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
``rig_pick_modules`` gives another robot on the rig the same Scan and Pick.
"""

from typing import Any

from dimos.control.connection.mock_joint_connection import MockJointConnection
from dimos.core.coordination.blueprints import Blueprint, autoconnect
from dimos.hardware.sensors.camera.realsense.camera import RealSenseCamera
from dimos.manipulation.grasp_verification import GraspVerificationConfig
from dimos.manipulation.grasping.heuristic_grasp import HeuristicGraspModule
from dimos.manipulation.manipulation_localize import ManipulationLocalize
from dimos.manipulation.manipulation_skills import ManipulationSkills
from dimos.manipulation.pick_and_place_module import PickAndPlaceModule
from dimos.robot.assets.model import RobotModel
from dimos.robot.manipulators.common.blueprints import manipulation_mapping
from dimos.robot.manipulators.damiao.damiao_blueprint import arm_declaration
from dimos.robot.manipulators.dual_openyam.blueprints.basic import (
    dual_openyam_planner_coordinator,
)
from dimos.robot.manipulators.dual_openyam.blueprints.grasp import (
    DUAL_OPENYAM_LOCALIZE_PARAMS,
    DUAL_OPENYAM_MAP_PADDING,
    DUAL_OPENYAM_MAP_TF_TOLERANCE,
    DUAL_OPENYAM_VOXEL_SIZE,
    dual_openyam_grasp,
)
from dimos.robot.manipulators.dual_openyam.blueprints.teleop import teleop_webxr_dual_openyam
from dimos.robot.manipulators.dual_openyam.connection import DualOpenYamConnection
from dimos.robot.manipulators.dual_openyam.sim_connection import DualOpenYamSimConnection
from dimos.robot.urdf_manager_point_clearing import UrdfManagerPointClearing

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


def rig_pick_modules(
    *,
    self_filter_model: RobotModel,
    heuristic_aim: dict[str, float],
    grasp_verification: GraspVerificationConfig,
) -> tuple[Blueprint, ...]:
    """Scan and Pick for a robot on the MuJoCo rig, with the grasp stack's settings.

    Localize finds objects on the overhead camera, a top-down heuristic grasp
    picks them, and the overhead cloud becomes the planner's voxel map. Turn
    on the rig's ``enable_pointcloud``, end the planner's groups at a tool tip
    with z along the fingers and y along the jaw, and give the planner
    ``voxel_map_resolution=DUAL_OPENYAM_VOXEL_SIZE``. The pick tries the grasp
    without the grasp stack's contact check.

    Args:
        self_filter_model: The robot as the map removes it from the cloud, with
            the joints the coordinator's joint state does not carry, such as
            the fingers, fixed.
        heuristic_aim: Where the gripper's parts sit along its approach from the
            tool tip, metres, as ``HeuristicGraspModule`` takes them.
        grasp_verification: How the gripper's readback tells a held object
            from an empty close.
    """
    return (
        ManipulationSkills.blueprint(),
        PickAndPlaceModule.blueprint(
            planning_frame="world",
            pregrasp_offset=0.05,
            grasp_verification=grasp_verification,
        ),
        HeuristicGraspModule.blueprint(**heuristic_aim),
        # A UrdfManager of the stack serves the URDF to web views; the self
        # filter's copy has its fingers fixed, so it answers on other topics.
        manipulation_mapping(
            self_filter_model=self_filter_model,
            voxel_size=DUAL_OPENYAM_VOXEL_SIZE,
            self_filter_padding=DUAL_OPENYAM_MAP_PADDING,
            tf_tolerance_s=DUAL_OPENYAM_MAP_TF_TOLERANCE,
        ).remappings(
            [
                (UrdfManagerPointClearing, "render_file_request", "self_filter_file_request"),
                (UrdfManagerPointClearing, "file_info", "self_filter_file_info"),
            ]
        ),
        ManipulationLocalize.blueprint(world_frame="world", params=DUAL_OPENYAM_LOCALIZE_PARAMS),
    )


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
