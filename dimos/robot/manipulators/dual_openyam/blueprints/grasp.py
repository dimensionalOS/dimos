# Copyright 2025-2026 Dimensional Inc.
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

"""The xArm grasp stack on the dual OpenYAM workcell, both arms in one blueprint.

``dimos run dual-openyam-grasp --config workcell.json --left-can-port follower_l --right-can-port follower_r``

The planning model is the calibrated workcell model with both arms, a fingertip
tcp per arm and a gripper task bound to each planning group. Everything else is
the stack as it exists: pick_object and place_at from the pick-and-place module,
one top-down heuristic grasp, scene registration from the overhead D405, the
planner with the workcell as its collision world, and a Rerun layout for a
headless rig. The camera serials come from the run config.
"""

from __future__ import annotations

from dataclasses import replace
import math
from typing import Any

from dimos.core.coordination.blueprints import Blueprint, autoconnect
from dimos.hardware.sensors.camera.realsense.camera import RealSenseCamera
from dimos.manipulation.grasp_verification import GraspVerificationConfig
from dimos.manipulation.grasping.grasp_gen_x.module import GraspGenXModule
from dimos.manipulation.grasping.heuristic_grasp import HeuristicGraspModule
from dimos.manipulation.manipulation_skills import ManipulationSkills
from dimos.manipulation.pick_and_place_module import PickAndPlaceModule
from dimos.manipulation.planning.spec.config import RobotModelConfig
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.robot.assets.model import RobotModel
from dimos.robot.manipulators.common.blueprints import manipulation_mapping, planner
from dimos.robot.manipulators.dual_openyam.blueprints.basic import (
    DUAL_OPENYAM_GRIPPER_TASKS,
    DualOpenYamCoordinator,
    dual_openyam_gripper_task,
    dual_openyam_trajectory_task,
)
from dimos.robot.manipulators.dual_openyam.blueprints.teleop import DUAL_OPENYAM_PINK
from dimos.robot.manipulators.dual_openyam.config import (
    DUAL_OPENYAM_SIDES,
    dual_openyam_model_config,
)
from dimos.robot.manipulators.dual_openyam.scene_overlay import SceneOverlayModule
from dimos.robot.manipulators.dual_openyam.scene_registration import (
    SelfClearedSceneRegistrationModule,
)
from dimos.visualization.rerun.bridge import RerunBridgeModule

# Table top in the workcell frame (rig calibration 2026-10-15); the model's slab sits 1 cm lower.
DUAL_OPENYAM_TABLE_TOP_Z = -0.0346
# Planner map cell, the xArm grasp stack's value. 1 cm was tried (2026-10-30, run 9c56):
# 68k cells, octree rebuilds under the world lock slowed the planner's tf to 4 Hz, a third
# of the clouds dropped and scans took 13 s.
DUAL_OPENYAM_VOXEL_SIZE = 0.025
# How far past the model a return still counts as the robot in the check's scene. This
# camera's arm returns sit 1 to 2 cm off the model (1.3 deg extrinsic roll at 0.5 m plus
# depth noise; scene 56 replay 2026-10-29).
DUAL_OPENYAM_SELF_PADDING = 0.02
# The same for the planner's map: a cell must not straddle a static link, so the padding
# covers the slop plus one cell. Replay of run 77c6: 2 cm left the init pose in
# collision with table cells at the post and base feet, 4 cm cleared it.
DUAL_OPENYAM_MAP_PADDING = 0.04
# One period of the planner's tf (about 110 ms here), so a cloud is not dropped for
# landing between two publishes.
DUAL_OPENYAM_MAP_TF_TOLERANCE = 0.2
# Pad centre 3.7 cm beyond the grasp frame; the printed fingers match stock (calipers 2026-10-15).
DUAL_OPENYAM_PAD_CENTRE_OFFSET = 0.037
DUAL_OPENYAM_FINGER_EXTENSION = {"left": 0.009, "right": 0.009}
# Where the gripper's parts sit along the approach from the tcp, metres, as the xArm
# stack gives its heuristic: the closed pads' centre behind the tcp (the finger
# extension above), the fingertips' reach past it and the gripper body behind it, the
# last two read off the v6 collision meshes (0.8 mm and 82.5 mm).
DUAL_OPENYAM_HEURISTIC_AIM = {
    "pad_behind_tcp": DUAL_OPENYAM_FINGER_EXTENSION["right"],
    "fingertip_reach": 0.001,
    "palm_behind_tcp": 0.0825,
}

# The same gripper in GraspGenX's convention: origin on the gripper link, approach
# along +Z, jaws closing along X. Pads meet 0.137 m out and the open jaws span 94 mm.
DUAL_OPENYAM_GRIPPER_SWEEP_VOLUME = {
    "extents_open": (0.094, 0.028, 0.020),
    "offset_open": (0.0, 0.0, DUAL_OPENYAM_PAD_CENTRE_OFFSET + 0.10),
    "extents_half_open": (0.047, 0.028, 0.020),
    "offset_half_open": (0.0, 0.0, DUAL_OPENYAM_PAD_CENTRE_OFFSET + 0.10),
    "fingertip_depth": 0.10
    + DUAL_OPENYAM_PAD_CENTRE_OFFSET
    + DUAL_OPENYAM_FINGER_EXTENSION["left"],
}
# GraspGenX frame -> {side}_tcp: the quarter turn that takes GraspGenX's X closing axis
# onto the gripper's Y, then out along the approach to the fingertip the tcp sits on.
DUAL_OPENYAM_GRASP_FRAME_TO_TCP = (
    (0.0, 1.0, 0.0, 0.0),
    (-1.0, 0.0, 0.0, 0.0),
    (0.0, 0.0, 1.0, DUAL_OPENYAM_GRIPPER_SWEEP_VOLUME["fingertip_depth"]),
    (0.0, 0.0, 0.0, 1.0),
)
# Overhead colour optical pose from the 2026-10-15 calibration, expressed as camera_link.
DUAL_OPENYAM_CAMERA_TRANSFORM = Transform(
    translation=Vector3(x=-0.01684, y=0.01262, z=0.41508),
    rotation=Quaternion(-0.02168, 0.47733, -0.00152, 0.87845),
    frame_id="world",
    child_frame_id="camera_link",
)
# State and the 100 Hz joint targets (the action for training), the cameras and tf.
DUAL_OPENYAM_RECORD_TOPICS = ",".join(
    [
        "coordinator_joint_state",
        "coordinator_joint_command",
        "color_image",
        "depth_image",
        "camera_info",
        "tf",
        # The planner's obstacle map, so a refused start pose can be replayed.
        "voxel_map",
        *(
            f"{side}_wrist_{stream}"
            for side in DUAL_OPENYAM_SIDES
            for stream in ("color_image", "camera_info")
        ),
    ]
)


def _check_side(side: str) -> None:
    if side not in DUAL_OPENYAM_SIDES:
        raise ValueError(f"side must be 'left' or 'right', got {side!r}")


def dual_openyam_grasp_model_config() -> RobotModelConfig:
    """The workcell model in the world frame with a fingertip tcp per arm."""
    config = dual_openyam_model_config()
    model = config.model
    for arm in DUAL_OPENYAM_SIDES:
        # The grasp frame's +Z leaves the palm; the tcp turns it to point at the object.
        model = model.with_fixed_frame(
            f"{arm}_tcp",
            f"{arm}_grasp_frame",
            xyz=(0.0, 0.0, -(DUAL_OPENYAM_PAD_CENTRE_OFFSET + DUAL_OPENYAM_FINGER_EXTENSION[arm])),
            rpy=(0.0, math.pi, 0.0),
        )
    config.model = model
    config.base_pose = PoseStamped(frame_id="world")
    # The self filter lifts the overhead cloud into the model's root frame; the planner
    # publishes tips and extra links only, and this root is a frame of its own.
    config.tf_extra_links = [model.load().root_link]
    config.planning_groups = [
        replace(group, tip_link=f"{group.name.split('_')[0]}_tcp")
        for group in config.planning_groups
    ]
    # The pads meet at zero and the wrist nests in the gripper body, so these hulls always touch.
    config.collision_exclusion_pairs = [
        *config.collision_exclusion_pairs,
        *((f"{arm}_tip_left", f"{arm}_tip_right") for arm in DUAL_OPENYAM_SIDES),
        *((f"{arm}_link4", f"{arm}_gripper") for arm in DUAL_OPENYAM_SIDES),
    ]
    return config


def dual_openyam_self_filter_model() -> RobotModel:
    """The whole rig, fingers fixed: what the map and the check's scene drop as the robot.

    The xArm stack filters only its arm because a wrist camera never sees the rest.
    The overhead camera sees the grippers, the wrist cameras and the post, and their
    cells pin the arm at its start pose. The fingers are fixed because the joint state
    carries no finger positions.
    """
    return dual_openyam_model_config().model.with_fixed_joints(
        *(f"{arm}_finger_{jaw}_joint" for arm in DUAL_OPENYAM_SIDES for jaw in ("left", "right"))
    )


def dual_openyam_grasp_view() -> Any:
    """Rerun layout: the three cameras as tiles on the left, the scene on the right."""
    # Heavy optional dependency, loaded only when the bridge builds its layout.
    import rerun as rr
    import rerun.blueprint as rrb

    cameras = [
        ("world/color_image", "Overhead"),
        ("world/overlay_image", "Scene overlay"),
        ("world/left_wrist_color_image", "Left wrist"),
        ("world/right_wrist_color_image", "Right wrist"),
    ]
    overhead, *others = cameras
    tiles = [
        rrb.Spatial2DView(
            origin=overhead[0],
            name=overhead[1],
            contents=["$origin/**", "/world/detections_3d/**"],
        ),
        *[rrb.Spatial2DView(origin=path, name=name) for path, name in others],
    ]
    return rrb.Blueprint(
        rrb.Horizontal(
            rrb.Vertical(*tiles),
            rrb.Spatial3DView(
                origin="world",
                name="Scene",
                background=rrb.Background(kind="SolidColor", color=[0, 0, 0]),
                line_grid=rrb.LineGrid3D(plane=rr.components.Plane3D.XY.with_distance(0.0)),
                overrides={path: rrb.EntityBehavior(visible=False) for path, _ in cameras},
            ),
            column_shares=[1, 2],
        ),
        rrb.TimePanel(state="collapsed"),
        rrb.SelectionPanel(state="collapsed"),
    )


# The scene cloud stays off the bridge: streamed to a remote viewer it saturates the link.
DUAL_OPENYAM_VIEW_TOPICS = [
    "color_image",
    "overlay_image",
    "camera_info",
    "left_wrist_color_image",
    "right_wrist_color_image",
    "tf",
    "detections_3d",
]


# Frames per second the bridge forwards per image entity. The cameras run at 30 Hz;
# four such streams kept a laptop viewer at two cores of JPEG decoding (2026-10-27).
DUAL_OPENYAM_VIEW_MAX_HZ = {
    "world/color_image": 10.0,
    "world/overlay_image": 10.0,
    "world/left_wrist_color_image": 3.0,
    "world/right_wrist_color_image": 3.0,
}


def dual_openyam_grasp_rerun() -> Blueprint:
    return RerunBridgeModule.blueprint(
        blueprint=dual_openyam_grasp_view,
        topics=DUAL_OPENYAM_VIEW_TOPICS,
        max_hz=DUAL_OPENYAM_VIEW_MAX_HZ,
        memory_limit="2GB",
        rerun_open="none",
    )


def dual_openyam_wrist_camera(side: str) -> Blueprint:
    """A wrist D405 with ``{side}_wrist_`` stream names; colour only, the D405s share USB 2 hubs."""
    _check_side(side)
    name = f"{side}_wrist_camera"
    camera = RealSenseCamera.blueprint(
        instance_name=name,
        frame_id_prefix=f"{side}_wrist",
        width=640,
        fps=30,
        enable_depth=False,
        align_depth_to_color=False,
        enable_pointcloud=False,
    )
    streams = [stream.name for stream in camera.blueprints[0].streams]
    return camera.remappings([(name, stream, f"{side}_wrist_{stream}") for stream in streams])


def dual_openyam_grasp_modules(grasp_provider: Blueprint | None = None) -> tuple[Blueprint, ...]:
    model = dual_openyam_grasp_model_config()
    return (
        planner(
            model=model,
            gripper_tasks=DUAL_OPENYAM_GRIPPER_TASKS,
            kinematics=DUAL_OPENYAM_PINK,
            default_speed_scale=0.25,
            voxel_map_resolution=DUAL_OPENYAM_VOXEL_SIZE,
            static_transforms=[DUAL_OPENYAM_CAMERA_TRANSFORM],
            visualization={"backend": "viser"},
            world_frame="world",
        ),
        ManipulationSkills.blueprint(),
        # 0.10 puts the pre-place above the fingers-down reach ceiling.
        PickAndPlaceModule.blueprint(
            planning_frame="world",
            # The contact check drops grasps the arm cannot reach or that would hit the
            # table, a neighbour, or the object off the fingertips, before any IK. Its
            # scene comes from the self-cleared registration below.
            robot=model,
            pregrasp_offset=0.05,
            # Empty closes read 0.01 to 0.02 on these jaws and the thinnest benchmark object,
            # a toothpaste carton end, settles at 0.05 to 0.07; the 0.10 default threw it away.
            grasp_verification=GraspVerificationConfig(empty_epsilon=0.035),
        ),
        grasp_provider or HeuristicGraspModule.blueprint(**DUAL_OPENYAM_HEURISTIC_AIM),
        # The D405 has no 848x480 at 15 fps mode.
        RealSenseCamera.blueprint(
            width=640,
            fps=30,
            enable_pointcloud=True,
        ),
        # The planner's obstacles are the overhead cloud, as on the xArm stack: table,
        # bin and neighbours come from the camera, not the model.
        manipulation_mapping(
            self_filter_model=dual_openyam_self_filter_model(),
            voxel_size=DUAL_OPENYAM_VOXEL_SIZE,
            padding_m=DUAL_OPENYAM_MAP_PADDING,
            tf_tolerance_s=DUAL_OPENYAM_MAP_TF_TOLERANCE,
        ),
        dual_openyam_wrist_camera("left"),
        dual_openyam_wrist_camera("right"),
        # Ghost of a saved scene over the overhead image, for rebuilding benchmark scenes;
        # pass-through until --sceneoverlaymodule.scene names a capture in scenes-dir.
        SceneOverlayModule.blueprint(),
        # The overhead camera sees the arms, so the scene the contact check reads has
        # the robot's returns removed, as the planner's map has.
        SelfClearedSceneRegistrationModule.blueprint(
            model=dual_openyam_self_filter_model(),
            padding_m=DUAL_OPENYAM_SELF_PADDING,
            target_frame="world",
            detector_backend="moondream",
            segmentation_backend="edgetam",
            detect_on_request=True,
            # Benchmark scenes put objects 9 cm apart; 8 cm merged neighbours into one id.
            distance_threshold=0.03,
            min_detections_for_permanent=1,
            max_distance=1.5,
            use_aabb=True,
            max_obstacle_width=0.06,
        ),
        dual_openyam_grasp_rerun(),
        DualOpenYamCoordinator.blueprint(
            instance_name="ControlCoordinator",
            tasks=[
                dual_openyam_trajectory_task(),
                dual_openyam_gripper_task("left"),
                dual_openyam_gripper_task("right"),
            ],
        ),
    )


# Assigned through autoconnect so the registry generator sees it.
dual_openyam_grasp = autoconnect(*dual_openyam_grasp_modules()).global_config(
    record="sqlite", record_topics=DUAL_OPENYAM_RECORD_TOPICS
)

dual_openyam_grasp_graspgenx = autoconnect(
    *dual_openyam_grasp_modules(
        GraspGenXModule.blueprint(
            gripper=DUAL_OPENYAM_GRIPPER_SWEEP_VOLUME,
            grasp_frame_to_tcp=DUAL_OPENYAM_GRASP_FRAME_TO_TCP,
            # Return every sample: the pick's contact check chooses among them.
            max_candidates=500,
        ),
    )
).global_config(record="sqlite", record_topics=DUAL_OPENYAM_RECORD_TOPICS)
