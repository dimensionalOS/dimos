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
``XArmConnection``; in MuJoCo through the coordinator's simulated adapter. The
split also swaps the base pose, the camera and the engine behind it, the
detector backends and the home pose. Everything else -- pick-and-place, scene
registration -- is the same stack either way.
"""

from __future__ import annotations

from dimos.control.coordinator import TaskConfig
from dimos.core.coordination.blueprints import Blueprint, autoconnect
from dimos.core.global_config import global_config
from dimos.core.module import ModuleBase
from dimos.hardware.sensors.camera.realsense.camera import RealSenseCamera
from dimos.manipulation.grasping.grasp_gen_x.module import GraspGenXModule
from dimos.manipulation.grasping.heuristic_grasp import HeuristicGraspModule
from dimos.manipulation.manipulation_module import ManipulationModule
from dimos.manipulation.manipulation_skills import ManipulationSkills
from dimos.manipulation.pick_and_place_module import PickAndPlaceModule
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.perception.experimental.object_scene_registration import ObjectSceneRegistrationModule
from dimos.robot.manipulators.common.blueprints import (
    coordinator,
    manipulation_mapping,
    trajectory_task,
)
from dimos.robot.manipulators.xarm.blueprints.basic import XARM_REMAPS, xarm_coordinator
from dimos.robot.manipulators.xarm.config import (
    XARM_ARMS,
    XARM_HEURISTIC_AIM,
    XArmType,
    make_xarm7_model_config,
    make_xarm7_sim_hardware,
    make_xarm7_sim_module_kwargs,
    make_xarm7_sim_robot_config,
    make_xarm_model_config,
    xarm_model,
)
from dimos.simulation.engines.mujoco_sim_module import MujocoSimModule
from dimos.utils.data import LfsPath
from dimos.visualization.rerun.bridge import RerunBridgeModule

SIMULATED = bool(global_config.simulation)

XARM_GRASP_SCENE_PATH = LfsPath("xarm_grasp_sim/scene.xml")
# The stock xArm home points the narrow wrist-camera frustum between the widely
# spaced targets. This collision-free top-down pose raises the camera enough to
# put every mesh in one frame, without changing the configured base pose.
XARM_GRASP_SCAN_JOINTS = [0.0, -0.04609, 0.0, 1.83940, 0.0, 1.87106, 0.0]
# The mapper and planner octree use the same resolution.
XARM_GRASP_VOXEL_SIZE = 0.025

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


def _wrist_camera(dof: int) -> Transform:
    """Where the wrist RealSense sits on the arm's last link, from hand-eye calibration.

    Measured on the xArm7's link7; the xArm6 gets the same numbers on link6, not
    measured there.

    Args:
        dof: The arm's joint count: 7 for the xArm7, 6 for the xArm6.
    """
    # RealSenseCamera publishes only its own subtree, so without this edge
    # camera_link has no parent, nothing resolves into world, and every cloud the
    # camera produces is silently unusable. The last link is a frame the model
    # already publishes, so ManipulationModule emits the whole chain from one
    # loop at one rate.
    return Transform(
        translation=Vector3(x=0.06693724, y=-0.0309563, z=0.00691482),
        rotation=Quaternion(0.70513398, 0.00535696, 0.70897578, -0.01052180),  # xyzw
        frame_id=f"link{dof}",
        child_frame_id="camera_link",
    )


def _sensing(simulated: bool) -> tuple[Blueprint, ...]:
    """The camera, and in sim the engine that produces its frames."""
    if simulated:
        return (
            MujocoSimModule.blueprint(
                **{
                    **make_xarm7_sim_module_kwargs(XARM_GRASP_SCENE_PATH),
                    "headless": True,
                    # Publish the simulated camera pose directly in the planning
                    # frame. A wrist-relative TF would need a second
                    # asynchronously stamped world->link7 edge and can make an
                    # otherwise valid scan unregistrable.
                    "base_frame_id": "world",
                    "reset_joint_positions": XARM_GRASP_SCAN_JOINTS,
                    # Off by default, and the voxel chain has nothing to map
                    # without it. Scene registration reads colour and depth.
                    "enable_pointcloud": True,
                }
            ),
        )
    return (
        # enable_pointcloud is off by default here too, and the voxel chain has
        # nothing to map without it.
        RealSenseCamera.blueprint(enable_pointcloud=True),
    )


def _scene_registration(simulated: bool) -> Blueprint:
    """Detector settings differ: synthetic renders score far below real images."""
    if simulated:
        return ObjectSceneRegistrationModule.blueprint(
            target_frame="world",
            detector_backend="owlv2",
            # OWLv2 is box-only; YOLO-E visual prompts refine its boxes into masks.
            segmentation_backend="yolo",
            detector_confidence=0.07,
            segmentation_confidence=0.05,
            # Keep adjacent tabletop targets distinct instead of merging by label.
            distance_threshold=0.05,
            detect_on_request=True,
            # The obstacle stream carries permanent objects only, so one explicit
            # scan must promote its first sightings immediately.
            min_detections_for_permanent=1,
        )
    return ObjectSceneRegistrationModule.blueprint(
        target_frame="world",
        detector_backend="moondream",
        segmentation_backend="edgetam",
        detect_on_request=True,
        distance_threshold=0.08,
        min_detections_for_permanent=3,
        max_distance=1.0,
        use_aabb=True,
        max_obstacle_width=0.06,
    )


def _grasp_modules(arm: XArmType, simulated: bool) -> tuple[Blueprint, ...]:
    """Everything in the grasping stack but the grasp provider.

    Exactly one provider may be composed in: PickAndPlaceModule resolves its
    generator by spec, so two would be ambiguous. On hardware, wire it up with
    ``.remappings(XARM_REMAPS)``.

    Args:
        arm: Which arm, "xarm7" or "xarm6", with the xArm Gripper.
        simulated: Run the arm in the MuJoCo grasping scene, through the
            coordinator's simulated adapter; only the xArm7 has one. Otherwise
            drive the real arm through ``XArmConnection``, with a RealSense on
            its wrist.
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
        )
        hardware = make_xarm7_sim_hardware(home_joints=XARM_GRASP_SCAN_JOINTS)
        self_filter_model = make_xarm7_model_config(add_gripper=False).model
        control = coordinator(
            hardware=[hardware],
            tasks=[
                trajectory_task(hardware),
                TaskConfig(
                    name="arm_gripper",
                    type="gripper",
                    joint_names=["arm/gripper"],
                    priority=20,
                ),
            ],
        )
    else:
        model = make_xarm_model_config(
            dof,
            robot_type=robot_type,
            robot="arm",
            gripper_hardware_id="arm",
            base_pose=PoseStamped(frame_id="world"),
            tf_extra_links=[f"link{dof}"],
        )
        self_filter_model = xarm_model(arm, robot="arm")
        control = xarm_coordinator(arm)
    return (
        ManipulationModule.blueprint(
            model=model,
            static_transforms=[] if simulated else [_wrist_camera(dof)],
            planning_timeout=10.0,
            visualization={"backend": "viser"},
            world_frame="world",
            voxel_map_resolution=XARM_GRASP_VOXEL_SIZE,
        ),
        ManipulationSkills.blueprint(),
        PickAndPlaceModule.blueprint(planning_frame="world", robot=model),
        *_sensing(simulated),
        _scene_registration(simulated),
        manipulation_mapping(
            # Filter only the arm; gripper returns may remain in the map.
            self_filter_model=self_filter_model,
            voxel_size=XARM_GRASP_VOXEL_SIZE,
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
_XARM7_REMAPS: list[tuple[type[ModuleBase], str, str]] = [] if SIMULATED else XARM_REMAPS
# Hardware only: there is no simulated xArm6 grasping scene.
_XARM6_MODULES = _grasp_modules("xarm6", simulated=False)

xarm_grasp = autoconnect(
    *_XARM7_MODULES, HeuristicGraspModule.blueprint(**XARM_HEURISTIC_AIM)
).remappings(_XARM7_REMAPS)
xarm_grasp_graspgenx = autoconnect(*_XARM7_MODULES, _graspgenx()).remappings(_XARM7_REMAPS)
xarm6_grasp = autoconnect(
    *_XARM6_MODULES, HeuristicGraspModule.blueprint(**XARM_HEURISTIC_AIM)
).remappings(XARM_REMAPS)
xarm6_grasp_graspgenx = autoconnect(*_XARM6_MODULES, _graspgenx()).remappings(XARM_REMAPS)
