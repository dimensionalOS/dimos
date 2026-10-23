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

"""The xArm grasp stack on the dual OpenYAM workcell, one arm per blueprint.

``dimos run dual-openyam-grasp-right --left-can-port follower_l --right-can-port follower_r``
``dimos run dual-openyam-grasp-left ...``             the other arm
``dimos run dual-openyam-grasp-right-graspgenx ...``  learned grasps

The planning model is the dual workcell model with both arms and both planning
groups. The manipulation module binds one gripper per model, so each blueprint
benchmarks one arm and names that arm's gripper; the other arm rests on its
support under gravity compensation. Everything else is the stack as it exists:
pick_object and place_at from the pick-and-place module, one top-down heuristic
grasp or GraspGenX, scene registration from the overhead D405, the planner with
the table, post and overhead camera as collision geometry in the workcell
model. No self filter or voxel map: the planner's obstacles are the detected
objects and the bin added per scene.
"""

from __future__ import annotations

from dataclasses import replace
import math
from typing import Any

from dimos.control.coordinator import TaskConfig
from dimos.core.coordination.blueprints import Blueprint, autoconnect
from dimos.hardware.sensors.camera.realsense.camera import RealSenseCamera
from dimos.manipulation.grasping.grasp_gen_x.module import GraspGenXModule
from dimos.manipulation.grasping.heuristic_grasp import HeuristicGraspModule
from dimos.manipulation.manipulation_skills import ManipulationSkills
from dimos.manipulation.pick_and_place_module import PickAndPlaceModule
from dimos.manipulation.planning.kinematics.config import PinkKinematicsConfig
from dimos.manipulation.planning.spec.config import RobotModelConfig
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.perception.experimental.object_scene_registration import ObjectSceneRegistrationModule
from dimos.robot.manipulators.common.blueprints import planner
from dimos.robot.manipulators.dual_openyam.blueprints.basic import (
    DualOpenYamCoordinator,
    dual_openyam_trajectory_task,
)
from dimos.robot.manipulators.dual_openyam.config import (
    DUAL_OPENYAM_SIDES,
    dual_openyam_model_config,
)
from dimos.robot.manipulators.dual_openyam.joints import DUAL_OPENYAM_GRIPPER_JOINTS
from dimos.visualization.rerun.bridge import RerunBridgeModule

# Table top in the workcell frame (rig calibration 2026-10-15); the model's slab sits 1 cm lower.
DUAL_OPENYAM_TABLE_TOP_Z = -0.0346
# Pad centre 3.7 cm beyond the grasp frame; the printed fingers match stock (calipers 2026-10-15).
DUAL_OPENYAM_PAD_CENTRE_OFFSET = 0.037
DUAL_OPENYAM_FINGER_EXTENSION = {"left": 0.009, "right": 0.009}
# Pink's defaults do not converge on this model; the WebXR teleop uses these.
DUAL_OPENYAM_GRASP_PINK = PinkKinematicsConfig(
    dt=0.01,
    position_cost=8.0,
    orientation_cost=2.0,
    posture_cost=0.01,
    joint_limit_posture_margin=0.3,
    lm_damping=0.01,
    gain=1.0,
)
# Overhead colour optical pose from the 2026-10-15 calibration, expressed as camera_link.
DUAL_OPENYAM_CAMERA_TRANSFORM = Transform(
    translation=Vector3(x=-0.01684, y=0.01262, z=0.41508),
    rotation=Quaternion(-0.02168, 0.47733, -0.00152, 0.87845),
    frame_id="world",
    child_frame_id="camera_link",
)
# Override with --realsensecamera.serial-number and --{side}-wrist-camera.serial-number.
DUAL_OPENYAM_OVERHEAD_CAMERA_SERIAL = "230322272156"
DUAL_OPENYAM_WRIST_CAMERA_SERIALS = {"left": "260322272983", "right": "260322276650"}
# GraspGenX convention: origin on the gripper link, approach +Z, jaws close along X.
DUAL_OPENYAM_GRIPPER_SWEEP_VOLUME = {
    "extents_open": (0.094, 0.028, 0.020),
    "offset_open": (0.0, 0.0, 0.137),
    "extents_half_open": (0.047, 0.028, 0.020),
    "offset_half_open": (0.0, 0.0, 0.137),
    "fingertip_depth": 0.1468,
}
# GraspGenX frame -> {side}_tcp: swap X and Y, then 13.7 cm back along the approach.
DUAL_OPENYAM_GRASP_FRAME_TO_TCP = (
    (0.0, -1.0, 0.0, 0.0),
    (1.0, 0.0, 0.0, 0.0),
    (0.0, 0.0, 1.0, -0.137),
    (0.0, 0.0, 0.0, 1.0),
)
DUAL_OPENYAM_RECORD_TOPICS = ",".join(
    [
        "coordinator_joint_state",
        "color_image",
        "depth_image",
        "camera_info",
        "tf",
        *(
            f"{side}_wrist_{stream}"
            for side in DUAL_OPENYAM_SIDES
            for stream in ("color_image", "camera_info")
        ),
    ]
)
DUAL_OPENYAM_GRIPPER_TASKS = {
    f"{side}_manipulator": f"{side}_arm_gripper" for side in DUAL_OPENYAM_SIDES
}


def _check_side(side: str) -> None:
    if side not in DUAL_OPENYAM_SIDES:
        raise ValueError(f"side must be 'left' or 'right', got {side!r}")


def dual_openyam_gripper_task(side: str, *, priority: int = 20) -> TaskConfig:
    _check_side(side)
    return TaskConfig(
        name=DUAL_OPENYAM_GRIPPER_TASKS[f"{side}_manipulator"],
        type="gripper",
        joint_names=[DUAL_OPENYAM_GRIPPER_JOINTS[DUAL_OPENYAM_SIDES.index(side)]],
        priority=priority,
    )


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


def dual_openyam_grasp_provider(*, graspgen: bool) -> Blueprint:
    if graspgen:
        return GraspGenXModule.blueprint(
            gripper=DUAL_OPENYAM_GRIPPER_SWEEP_VOLUME,
            grasp_frame_to_tcp=DUAL_OPENYAM_GRASP_FRAME_TO_TCP,
        )
    return HeuristicGraspModule.blueprint()


def dual_openyam_grasp_view() -> Any:
    """Rerun layout: the three cameras as tiles on the left, the scene on the right."""
    import rerun as rr
    import rerun.blueprint as rrb

    cameras = [
        ("world/color_image", "Overhead"),
        ("world/left_wrist_color_image", "Left wrist"),
        ("world/right_wrist_color_image", "Right wrist"),
    ]
    overhead, *wrists = cameras
    tiles = [
        rrb.Spatial2DView(
            origin=overhead[0],
            name=overhead[1],
            contents=["$origin/**", "/world/detections_3d/**"],
        ),
        *[rrb.Spatial2DView(origin=path, name=name) for path, name in wrists],
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


DUAL_OPENYAM_VIEW_TOPICS = [
    "color_image",
    "camera_info",
    "left_wrist_color_image",
    "right_wrist_color_image",
    "pointcloud",
    "tf",
    "detections_3d",
]


def dual_openyam_grasp_rerun() -> Blueprint:
    return RerunBridgeModule.blueprint(
        blueprint=dual_openyam_grasp_view,
        topics=DUAL_OPENYAM_VIEW_TOPICS,
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
        height=480,
        fps=30,
        enable_depth=False,
        align_depth_to_color=False,
        enable_pointcloud=False,
        serial_number=DUAL_OPENYAM_WRIST_CAMERA_SERIALS[side],
    )
    streams = [stream.name for stream in camera.blueprints[0].streams]
    return camera.remappings([(name, stream, f"{side}_wrist_{stream}") for stream in streams])


def dual_openyam_grasp_modules(*, graspgen: bool) -> tuple[Blueprint, ...]:
    return (
        planner(
            model=dual_openyam_grasp_model_config(),
            gripper_tasks=DUAL_OPENYAM_GRIPPER_TASKS,
            kinematics=DUAL_OPENYAM_GRASP_PINK,
            default_speed_scale=0.25,
            static_transforms=[DUAL_OPENYAM_CAMERA_TRANSFORM],
            visualization={"backend": "viser"},
            world_frame="world",
        ),
        ManipulationSkills.blueprint(),
        # 0.10 puts the pre-place above the fingers-down reach ceiling.
        PickAndPlaceModule.blueprint(planning_frame="world", pregrasp_offset=0.05),
        dual_openyam_grasp_provider(graspgen=graspgen),
        # The D405 has no 848x480 at 15 fps mode.
        RealSenseCamera.blueprint(
            width=640,
            height=480,
            fps=30,
            enable_pointcloud=True,
            serial_number=DUAL_OPENYAM_OVERHEAD_CAMERA_SERIAL,
        ),
        dual_openyam_wrist_camera("left"),
        dual_openyam_wrist_camera("right"),
        ObjectSceneRegistrationModule.blueprint(
            target_frame="world",
            detector_backend="moondream",
            segmentation_backend="edgetam",
            detect_on_request=True,
            distance_threshold=0.08,
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


def dual_openyam_grasp_blueprint(*, graspgen: bool) -> Blueprint:
    return autoconnect(*dual_openyam_grasp_modules(graspgen=graspgen)).global_config(
        record="sqlite", record_topics=DUAL_OPENYAM_RECORD_TOPICS
    )


# Assigned through autoconnect so the registry generator sees them.
dual_openyam_grasp = autoconnect(*dual_openyam_grasp_modules(graspgen=False)).global_config(
    record="sqlite", record_topics=DUAL_OPENYAM_RECORD_TOPICS
)
dual_openyam_grasp_graspgenx = autoconnect(
    *dual_openyam_grasp_modules(graspgen=True)
).global_config(record="sqlite", record_topics=DUAL_OPENYAM_RECORD_TOPICS)
