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
"""The dual OpenYAM grasping stack, on hardware by default.

```bash
dimos run dual-openyam-grasp --left-can-port follower_l --right-can-port follower_r
dimos run dual-openyam-grasp ... --graspgen                  # GraspGenX grasps
dimos run dual-openyam-grasp ... --record=                   # no recording
dimos run dual-openyam-grasp                                 # in-memory arms, no CAN
```

The port of the xArm grasp stack: coordinator, planner, pick-and-place, scene
registration and a grasp provider, with a fixed depth camera over the table
feeding perception and one more on each wrist. Both arms are planning groups
with their own grippers, so ``pick_object`` takes ``left_manipulator`` or
``right_manipulator``.

Every run records the policy-training streams (joint states, planned and
accepted joint commands, all three cameras, TF) to
``recordings/<run-id>/memory.db`` unless ``--record=`` turns it off.

The grasp provider is chosen at import time from ``global_config.graspgen``,
as the xArm stack chooses sim or hardware: the heuristic top-down grasp by
default, GraspGenX with ``--graspgen``.
"""

from __future__ import annotations

from dataclasses import replace
import math
from typing import Any

from dimos.core.coordination.blueprints import Blueprint, autoconnect
from dimos.core.global_config import global_config
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
    dual_openyam_gripper_task,
    dual_openyam_trajectory_task,
)
from dimos.robot.manipulators.dual_openyam.config import (
    DUAL_OPENYAM_SIDES,
    dual_openyam_model_config,
)
from dimos.visualization.rerun.bridge import RerunBridgeModule

# Distance between the centres of the two base motors, tape-measured on the
# rig and confirmed 2026-10-07; the URDF carries the ABC bench's 0.62 m.
DUAL_OPENYAM_BASE_SPACING = 0.43

# Wrist camera: a D405 on a 6 cm bracket that leaves the top of the wrist tube
# at 45 deg, leaning toward the wrist (sketch of 2026-10-06). It sits on the
# gripper link's -X side, the side that points up at the rest pose. Boxes in
# the gripper frame with 1.5 cm of margin per side and 2 cm for the USB plug.
_WRIST_TUBE_HALF_WIDTH = 0.0325
_BRACKET_RPY = (0.0, -math.pi / 4, 0.0)
DUAL_OPENYAM_WRIST_CAMERA_BOXES = [
    # name, size, xyz: the bracket, then the camera body at its end.
    ("wrist_camera_bracket", (0.034, 0.055, 0.090), (-0.054, 0.0, 0.021)),
    ("wrist_camera", (0.072, 0.072, 0.073), (-0.090, 0.0, 0.058)),
]

# The workcell in the world frame: the table top is 3 cm below the arm base
# plates; the bin stands at the far edge, 20.5 cm ahead of the origin; the
# centre post carries the overhead camera 46 cm up, midway between the arms.
# Bin height and post footprint are placeholders until measured.
DUAL_OPENYAM_TABLE_TOP_Z = -0.03
DUAL_OPENYAM_STATIC_BOXES = [
    {
        "name": "table",
        "size": (0.60, 1.00, 0.10),
        "xyz": (0.185, 0.0, DUAL_OPENYAM_TABLE_TOP_Z - 0.055),
    },
    {
        "name": "bin",
        "size": (0.304, 0.231, 0.13),
        "xyz": (0.347, -0.024, DUAL_OPENYAM_TABLE_TOP_Z + 0.065),
    },
    {"name": "camera_post", "size": (0.08, 0.08, 0.50), "xyz": (0.0, 0.0, 0.22)},
    {"name": "overhead_camera", "size": (0.10, 0.10, 0.08), "xyz": (0.016, -0.006, 0.47)},
]

# {side}_grasp_frame is 10 cm below the gripper link on its axis. The finger
# pads (tip_left.stl, tip_right.stl at the URDF's closed zero position) meet on
# that axis from 12.7 to 14.7 cm below the gripper link; plan to the pad centre.
DUAL_OPENYAM_TCP_OFFSET = (0.0, 0.0, -0.037)

# The same gripper in GraspGenX's convention: origin on the gripper link,
# approach along +Z (the URDF's -Z), jaws closing along X (the URDF's Y). The
# fingers slide 4.7 cm each, so the open and half-open sweep volumes share
# their centre; the pads are up to 2.8 cm wide and 2 cm tall.
DUAL_OPENYAM_GRIPPER_SWEEP_VOLUME = {
    "extents_open": (0.094, 0.028, 0.020),
    "offset_open": (0.0, 0.0, 0.137),
    "extents_half_open": (0.047, 0.028, 0.020),
    "offset_half_open": (0.0, 0.0, 0.137),
    "fingertip_depth": 0.1468,
}
# GraspGenX frame -> {side}_tcp: swap the X and Y axes and flip Z, then move
# 13.7 cm along the approach to the pad centre.
DUAL_OPENYAM_GRASP_FRAME_TO_TCP = (
    (0.0, 1.0, 0.0, 0.0),
    (1.0, 0.0, 0.0, 0.0),
    (0.0, 0.0, -1.0, 0.137),
    (0.0, 0.0, 0.0, 1.0),
)

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

# Fixed camera on the centre post, in the frame midway between the arm bases
# (x forward, y toward the left arm, z up). Solved from four 60 mm AprilTags
# taped to the table at tape-measured positions, 3.7 px reprojection RMS.
DUAL_OPENYAM_CAMERA_TRANSFORM = Transform(
    translation=Vector3(x=0.0160, y=-0.0057, z=0.4598),
    rotation=Quaternion(-0.00254, 0.51604, 0.00190, 0.85656),  # xyzw, pitched 62 deg down
    frame_id="world",
    child_frame_id="camera_link",
)

# RealSense D405 serials on the benchmark rig; override per camera with
# --realsensecamera.serial-number, --left-wrist-camera.serial-number and
# --right-wrist-camera.serial-number.
DUAL_OPENYAM_OVERHEAD_CAMERA_SERIAL = "230322272156"
# Sides verified 2026-10-06 by covering the left wrist lens.
DUAL_OPENYAM_WRIST_CAMERA_SERIALS = {"left": "260322272983", "right": "260322276650"}

# Streams a run keeps for ACT and VLA training: the joint states, the plans
# the planner sent and the commands the hardware accepted, every camera's
# colour, depth and intrinsics, and TF. Globs on the stream names.
DUAL_OPENYAM_RECORD_TOPICS = ",".join(
    [
        "coordinator_joint_state",
        "planned_joint_trajectory",
        "applied_joint_position_command",
        "color_image",
        "depth_image",
        "camera_info",
        "tf",
        "detections_3d",
        "grasp_candidates",
        "grasp_target",
        "planned_tool_path",
        *(
            f"{side}_wrist_{stream}"
            for side in DUAL_OPENYAM_SIDES
            for stream in ("color_image", "depth_image", "camera_info", "tf")
        ),
    ]
)

DUAL_OPENYAM_GRASP_PROMPTS = [
    "soup can",
    "mustard bottle",
    "cracker box",
    "banana",
    "plate",
    "toothpaste",
    "toy block",
    "mug",
    "marker",
    "towel",
]


def dual_openyam_grasp_model_config() -> RobotModelConfig:
    """Planning model in the world frame: measured base spacing, collision
    hulls on every link, the wrist cameras as boxes, a fingertip TCP per arm."""
    config = dual_openyam_model_config(base_pose=PoseStamped(frame_id="world"))
    model = config.model.with_collision_from_visuals()
    for side, sign in zip(DUAL_OPENYAM_SIDES, (1.0, -1.0), strict=True):
        model = model.with_joint_origin(
            f"{side}_arm_fixed_joint", xyz=(0.0, sign * DUAL_OPENYAM_BASE_SPACING / 2, 0.0)
        )
        for name, size, xyz in DUAL_OPENYAM_WRIST_CAMERA_BOXES:
            model = model.with_collision_box(
                f"{side}_gripper", f"{side}_{name}", size=size, xyz=xyz, rpy=_BRACKET_RPY
            )
        model = model.with_fixed_frame(
            f"{side}_tcp", f"{side}_grasp_frame", xyz=DUAL_OPENYAM_TCP_OFFSET
        )
    config.model = model
    # The pads meet at the URDF's closed zero position, so the two fingertip
    # hulls always touch, and the wrist assembly nests inside the gripper body
    # so their hulls overlap by 6 cm at every pose; both pairs are rigidly
    # close and filtered. Everything else is filtered as adjacent links.
    config.collision_exclusion_pairs = [
        *config.collision_exclusion_pairs,
        *((f"{side}_tip_left", f"{side}_tip_right") for side in DUAL_OPENYAM_SIDES),
        *((f"{side}_link4", f"{side}_gripper") for side in DUAL_OPENYAM_SIDES),
    ]
    config.planning_groups = [
        replace(group, tip_link=f"{group.name.split('_')[0]}_tcp")
        for group in config.planning_groups
    ]
    return config


def dual_openyam_grasp_provider(graspgen: bool) -> Blueprint:
    if graspgen:
        return GraspGenXModule.blueprint(
            gripper=DUAL_OPENYAM_GRIPPER_SWEEP_VOLUME,
            grasp_frame_to_tcp=DUAL_OPENYAM_GRASP_FRAME_TO_TCP,
        )
    # The half turn about Y turns the top-down grasp into the OpenYAM grasp
    # frame, which points back at the wrist; the extra yaws matter because the
    # two arms accept different wrist bands over the same object.
    return HeuristicGraspModule.blueprint(tool_rotation_rpy=(0.0, math.pi, 0.0), yaw_candidates=8)


def dual_openyam_grasp_view() -> Any:
    """Rerun layout: the three cameras as tiles on the left, the scene on the right."""
    import rerun as rr
    import rerun.blueprint as rrb

    cameras = [
        ("world/color_image", "Overhead"),
        ("world/left_wrist_color_image", "Left wrist"),
        ("world/right_wrist_color_image", "Right wrist"),
    ]
    return rrb.Blueprint(
        rrb.Horizontal(
            rrb.Vertical(*[rrb.Spatial2DView(origin=path, name=name) for path, name in cameras]),
            rrb.Spatial3DView(
                origin="world",
                name="Scene",
                background=rrb.Background(kind="SolidColor", color=[0, 0, 0]),
                line_grid=rrb.LineGrid3D(plane=rr.components.Plane3D.XY.with_distance(0.0)),
                # The images already have their own tiles; in 3D they only clutter.
                overrides={path: rrb.EntityBehavior(visible=False) for path, _ in cameras},
            ),
            column_shares=[1, 2],
        ),
        rrb.TimePanel(state="collapsed"),
        rrb.SelectionPanel(state="collapsed"),
    )


# Streams the Rerun bridge forwards; everything else stays off the viewer link.
DUAL_OPENYAM_VIEW_TOPICS = [
    "color_image",
    "left_wrist_color_image",
    "right_wrist_color_image",
    "pointcloud",
    "detections_3d",
    "grasp_candidates",
    "grasp_target",
    "planned_tool_path",
    "tf",
]

# Jaw glyph in the TCP frame: pads 9.4 cm apart along Y, the approach along
# -Z, a stem up toward the wrist.
_JAW_HALF_OPENING = 0.047
_JAW_STRIPS = [
    [[0.0, -_JAW_HALF_OPENING, -0.01], [0.0, -_JAW_HALF_OPENING, 0.03]],
    [[0.0, _JAW_HALF_OPENING, -0.01], [0.0, _JAW_HALF_OPENING, 0.03]],
    [[0.0, -_JAW_HALF_OPENING, 0.03], [0.0, _JAW_HALF_OPENING, 0.03]],
    [[0.0, 0.0, 0.03], [0.0, 0.0, 0.09]],
]
_CANDIDATE_COLOR = [100, 190, 255]
_TARGET_COLOR = [255, 215, 0]
_PATH_COLOR = (255, 215, 0)


def _jaw_glyph(path: str, pose: Any, color: list[int], radius: float) -> list[tuple[str, Any]]:
    import rerun as rr

    return [
        (
            path,
            rr.Transform3D(
                translation=pose.position.to_tuple(),
                rotation=rr.Quaternion(xyzw=pose.orientation.to_tuple()),
            ),
        ),
        (
            f"{path}/jaws",
            rr.LineStrips3D(strips=_JAW_STRIPS, colors=[color] * 4, radii=[radius] * 4),
        ),
    ]


def grasp_candidates_to_rerun(msg: Any) -> list[tuple[str, Any]]:
    """The top proposals as jaw glyphs; rank 0 is the one tried first."""
    import rerun as rr

    root = "world/grasp_candidates"
    data: list[tuple[str, Any]] = [(root, rr.Clear(recursive=True))]
    for rank, pose in enumerate(msg.poses[:8]):
        data.extend(_jaw_glyph(f"{root}/{rank:02d}", pose, _CANDIDATE_COLOR, 0.0012))
    return data


def grasp_target_to_rerun(msg: Any) -> list[tuple[str, Any]]:
    """The grasp being attempted right now, in yellow."""
    return _jaw_glyph("world/grasp_target", msg, _TARGET_COLOR, 0.0025)


def planned_tool_path_to_rerun(msg: Any) -> Any:
    """The tip's planned path on the table, not the nav default half a metre up."""
    return msg.to_rerun(color=_PATH_COLOR, z_offset=0.0, radii=0.003)


def dual_openyam_grasp_rerun() -> Blueprint:
    """The bridge for a headless rig: no window on the box, watch it from dimos-viewer."""
    return RerunBridgeModule.blueprint(
        blueprint=dual_openyam_grasp_view,
        topics=DUAL_OPENYAM_VIEW_TOPICS,
        visual_override={
            "world/grasp_candidates": grasp_candidates_to_rerun,
            "world/grasp_target": grasp_target_to_rerun,
            "world/planned_tool_path": planned_tool_path_to_rerun,
        },
        memory_limit="2GB",
        rerun_open="none",
    )


def dual_openyam_wrist_camera(side: str) -> Blueprint:
    """A wrist D405 whose streams and frames carry a ``{side}_wrist_`` prefix, so
    they never collide with the overhead camera that feeds perception.

    A prefix with an underscore, not a namespace: the recorder uses stream
    names as SQL identifiers and rejects a slash.
    """
    if side not in DUAL_OPENYAM_SIDES:
        raise ValueError(f"side must be 'left' or 'right', got {side!r}")
    name = f"{side}_wrist_camera"
    # 640x480 at 30 fps is what the OpenArm ACT datasets were collected at.
    camera = RealSenseCamera.blueprint(
        instance_name=name,
        frame_id_prefix=f"{side}_wrist",
        width=640,
        height=480,
        fps=30,
        enable_pointcloud=False,
        serial_number=DUAL_OPENYAM_WRIST_CAMERA_SERIALS[side],
    )
    streams = [stream.name for stream in camera.blueprints[0].streams]
    return camera.remappings([(name, stream, f"{side}_wrist_{stream}") for stream in streams])


def dual_openyam_grasp_modules(*, graspgen: bool) -> tuple[Blueprint, ...]:
    return (
        planner(
            model=dual_openyam_grasp_model_config(),
            kinematics=DUAL_OPENYAM_GRASP_PINK,
            default_speed_scale=0.25,
            static_transforms=[DUAL_OPENYAM_CAMERA_TRANSFORM],
            static_boxes=DUAL_OPENYAM_STATIC_BOXES,
            visualization={"backend": "viser"},
            world_frame="world",
        ),
        ManipulationSkills.blueprint(),
        PickAndPlaceModule.blueprint(planning_frame="world", pregrasp_along_tool_z=True),
        dual_openyam_grasp_provider(graspgen),
        RealSenseCamera.blueprint(
            width=640,
            height=480,
            fps=30,
            enable_pointcloud=True,
            serial_number=DUAL_OPENYAM_OVERHEAD_CAMERA_SERIAL,
        ),
        dual_openyam_wrist_camera("left"),
        dual_openyam_wrist_camera("right"),
        dual_openyam_grasp_rerun(),
        ObjectSceneRegistrationModule.blueprint(
            target_frame="world",
            detector_backend="moondream",
            segmentation_backend="edgetam",
            detect_on_request=True,
            distance_threshold=0.08,
            min_detections_for_permanent=3,
            max_distance=1.5,
            use_aabb=True,
            max_obstacle_width=0.06,
        ),
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


# Assigned through autoconnect so the registry generator sees it.
dual_openyam_grasp = autoconnect(
    *dual_openyam_grasp_modules(graspgen=bool(global_config.graspgen))
).global_config(record="sqlite", record_topics=DUAL_OPENYAM_RECORD_TOPICS)
