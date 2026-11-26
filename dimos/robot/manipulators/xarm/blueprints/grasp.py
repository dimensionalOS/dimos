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

"""The xArm grasping stack, on hardware by default.

``dimos run xarm-grasp --xarmconnection.ip 192.168.1.185``   heuristic grasps
``dimos run xarm-grasp-graspgenx --xarmconnection.ip 192.168.1.185`` learned grasps
``dimos run xarm-grasp --simulation mujoco``      the same stack in MuJoCo
``dimos run xarm6-grasp --xarmconnection.ip 192.168.1.185``  an xArm6, hardware only

Only the grasp provider separates xarm-grasp from xarm-grasp-graspgenx. The
arm-versus-sim split is decided here at import time, because composition runs
before module config is applied. On hardware the arm is driven through
``XArmConnection``; in MuJoCo through ``MujocoArmConnection``. The split also
swaps the base pose, the camera and the engine behind it, the
detector backends and the home pose. Everything else -- pick-and-place, scene
registration -- is the same stack either way.
"""

from __future__ import annotations

from typing import Any

from dimos.core.coordination.blueprints import Blueprint, MetaBlueprint, autoconnect
from dimos.core.global_config import global_config
from dimos.hardware.sensors.camera.realsense.camera import RealSenseCamera
from dimos.manipulation.grasping.grasp_gen_x.module import GraspGenXModule
from dimos.manipulation.grasping.heuristic_grasp import HeuristicGraspModule
from dimos.manipulation.manipulation_localize import ManipulationLocalize
from dimos.manipulation.manipulation_module import ManipulationModule
from dimos.manipulation.manipulation_skills import ManipulationSkills
from dimos.manipulation.pick_and_place_module import PickAndPlaceModule
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.robot.manipulators.common.blueprints import manipulation_mapping
from dimos.robot.manipulators.xarm.blueprints.basic import xarm_coordinator
from dimos.robot.manipulators.xarm.config import (
    XARM_ARMS,
    XARM_HEURISTIC_AIM,
    XArmType,
    make_xarm7_sim_module_kwargs,
    make_xarm7_sim_robot_config,
    make_xarm_model_config,
    xarm_model,
    xarm_wrist_camera_transform,
)
from dimos.utils.data import LfsPath
from dimos.visualization.rerun.bridge import RerunBridgeModule

SIMULATED = bool(global_config.simulation)

XARM_GRASP_SCENE_PATH = LfsPath("xarm_grasp_sim/scene.xml")
# The stock xArm home points the narrow wrist-camera frustum between the widely
# spaced targets. This collision-free top-down pose raises the camera enough to
# put every mesh in one frame, without changing the configured base pose.
XARM_GRASP_SCAN_JOINTS = [0.0, -0.04609, 0.0, 1.83940, 0.0, 1.87106, 0.0]
# The mapper and planner octree use the same resolution.
XARM_GRASP_VOXEL_SIZE = 0.01
# The arm's reach round link_base, gripper included: the map drops what lies farther.
XARM_GRASP_WORKSPACE_RADIUS = 0.9

XARM_GRASP_PROMPTS = [
    "black bottle",
    "gray can",
    "red cup",
    "green tape roll",
    "blue marker",
    "brown box",
    # The wrist camera sees the tape almost directly from above, where it reads
    # as a ring instead of a roll. Keep a shape-word fallback for that view.
    "green ring",
]

# Measured off data/xarm_grasp_sim (mj_forward at the driver joint limits) and the
# gripper URDF, expressed in GraspGenX's convention -- approach along +Z, jaws
# closing along X -- with the origin on xarm_gripper_base_link, the frame
# GraspGenX predicts into. The geometry is the real gripper's, so it holds on
# hardware too.
XARM_GRIPPER_SWEEP_VOLUME = {
    "extents_open": (0.0889, 0.030, 0.0370),
    "offset_open": (0.0, 0.0, 0.1421),
    "extents_half_open": (0.0479, 0.030, 0.0370),
    "offset_half_open": (0.0, 0.0, 0.1530),
    "fingertip_depth": 0.1606,
}
# xarm_gripper_base_link -> link_tcp, the planning tip frame: +0.172 m along the
# approach axis (xarm_gripper.urdf.xacro joint_tcp) plus the quarter turn that
# takes GraspGenX's X closing axis onto the xArm gripper's Y.
XARM_GRASP_FRAME_TO_TCP = (
    (0.0, 1.0, 0.0, 0.0),
    (-1.0, 0.0, 0.0, 0.0),
    (0.0, 0.0, 1.0, 0.172),
    (0.0, 0.0, 0.0, 1.0),
)


def _sim_module_kwargs() -> dict[str, Any]:
    """The MuJoCo grasping scene, with the wrist camera's colour, depth and points."""
    return {
        **make_xarm7_sim_module_kwargs(XARM_GRASP_SCENE_PATH),
        "headless": True,
        # Publish the simulated camera pose directly in the planning
        # frame. A wrist-relative TF would need a second
        # asynchronously stamped world->link7 edge and can make an
        # otherwise valid scan unregistrable.
        "base_frame_id": "world",
        "reset_joint_positions": XARM_GRASP_SCAN_JOINTS,
        # Off by default, and the voxel chain has nothing to map
        # without it. Localize reads colour and depth.
        "enable_pointcloud": True,
    }


def _grasp_modules(arm: XArmType, simulated: bool) -> tuple[Blueprint | MetaBlueprint, ...]:
    """Everything in the grasping stack but the grasp provider.

    Exactly one provider may be composed in: PickAndPlaceModule resolves its
    generator by spec, so two would be ambiguous.

    Args:
        arm: Which arm, "xarm7" or "xarm6", with the xArm Gripper.
        simulated: Run the arm in the MuJoCo grasping scene, through
            ``MujocoArmConnection``, whose wrist camera stands in for the
            RealSense; only the xArm7 has one. Otherwise drive the real arm
            through ``XArmConnection``, with a RealSense on its wrist.
    """
    dof, robot_type = XARM_ARMS[arm]
    if simulated and arm != "xarm7":
        raise ValueError(f"only the xArm7 has a simulated grasping scene, not the {arm}")
    if simulated:
        # data/xarm_grasp_sim/xarm7.xml bolts link_base to the world origin instead
        # of the 12 cm pedestal data/xarm7 uses. Inheriting that offset would put the
        # planning model 12 cm above the arm MuJoCo simulates.
        model = make_xarm7_sim_robot_config(
            base_pose=PoseStamped(frame_id="world"),
            tf_extra_links=[],
            robot="arm",
        )
        sensing: tuple[Blueprint | MetaBlueprint, ...] = ()
    else:
        model = make_xarm_model_config(
            dof,
            robot_type=robot_type,
            robot="arm",
            base_pose=PoseStamped(frame_id="world"),
            tf_extra_links=[f"link{dof}"],
        )
        # enable_pointcloud is off by default, and the voxel chain has nothing to map without it.
        sensing = (RealSenseCamera.blueprint(enable_pointcloud=True),)
    control = xarm_coordinator(
        arm,
        simulated=simulated,
        sim_module_kwargs=_sim_module_kwargs() if simulated else None,
    )
    return (
        ManipulationModule.blueprint(
            gripper_tasks={"manipulator": "arm_gripper"},
            model=model,
            static_transforms=[] if simulated else [xarm_wrist_camera_transform(dof)],
            planning_timeout=10.0,
            visualization={"backend": "viser"},
            world_frame="world",
            voxel_map_resolution=XARM_GRASP_VOXEL_SIZE,
        ),
        ManipulationSkills.blueprint(),
        PickAndPlaceModule.blueprint(planning_frame="world", robot=model),
        *sensing,
        # Finds the objects pick_object takes by id, and the dense scene its contact check needs.
        ManipulationLocalize.blueprint(world_frame="world"),
        manipulation_mapping(
            # Filter only the arm; gripper returns may remain in the map.
            self_filter_model=xarm_model(arm, robot="arm"),
            voxel_size=XARM_GRASP_VOXEL_SIZE,
            workspace_radius_m=XARM_GRASP_WORKSPACE_RADIUS,
        ),
        RerunBridgeModule.blueprint(),
        control,
    )


def _graspgenx() -> Blueprint:
    """Learned grasps for the xArm Gripper."""
    return GraspGenXModule.blueprint(
        gripper=XARM_GRIPPER_SWEEP_VOLUME,
        grasp_frame_to_tcp=XARM_GRASP_FRAME_TO_TCP,
        # Return every sample: the pick's contact check chooses among them.
        max_candidates=500,
    )


_XARM7_MODULES = _grasp_modules("xarm7", SIMULATED)
# Hardware only: there is no simulated xArm6 grasping scene.
_XARM6_MODULES = _grasp_modules("xarm6", simulated=False)

xarm_grasp = autoconnect(*_XARM7_MODULES, HeuristicGraspModule.blueprint(**XARM_HEURISTIC_AIM))
xarm_grasp_graspgenx = autoconnect(*_XARM7_MODULES, _graspgenx())
xarm6_grasp = autoconnect(*_XARM6_MODULES, HeuristicGraspModule.blueprint(**XARM_HEURISTIC_AIM))
xarm6_grasp_graspgenx = autoconnect(*_XARM6_MODULES, _graspgenx())
