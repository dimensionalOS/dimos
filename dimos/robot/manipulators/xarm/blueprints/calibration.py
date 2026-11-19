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

"""Hand-eye calibration of the xArm6 and xArm7 wrist cameras and of a fixed camera beside them.

``dimos run xarm6-handeye-calibrate --xarmconnection.ip <ip>`` brings up the planner, the
coordinator, the wrist RealSense and the calibration module on the real arm; then from
``dimos shell``: ``app.get_module("HandEyeCalibrationModule").calibrate()`` and ``approve()``
for each of the first moves. ``xarm6-handeye-calibrate-sim`` is the same stack over the MuJoCo
bench in data/xarm6/calibration_bench.xml, for testing the run without an arm. The xArm7
blueprints are the same over the kitting bench (data/xarm7_kitting_bench), where the board lies
anywhere on the table and the run finds it.
"""

from __future__ import annotations

import math

from dimos.control.coordinator import TaskConfig
from dimos.core.coordination.blueprints import Blueprint, autoconnect
from dimos.hardware.sensors.camera.realsense.camera import RealSenseCamera
from dimos.manipulation.calibration.hand_eye_module import BoxObstacle, HandEyeCalibrationModule
from dimos.manipulation.planning.spec.config import RobotModelConfig
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.robot.assets.model import RobotModel
from dimos.robot.manipulators.common.blueprints import coordinator, planner, trajectory_task
from dimos.robot.manipulators.xarm.blueprints.basic import XARM_REMAPS, xarm_coordinator
from dimos.robot.manipulators.xarm.config import (
    XARM6_JOINT_LIMITS,
    XARM7_JOINT_LIMITS,
    make_xarm7_sim_hardware,
    make_xarm7_sim_robot_config,
    make_xarm_hardware,
    make_xarm_model_config,
)
from dimos.simulation.engines.mujoco_sim_module import MujocoSimModule
from dimos.utils.data import LfsPath

# Tool down about 0.4 m over a board 0.45 m out; wrist at zero keeps the camera cable untwisted.
XARM6_CALIBRATION_HOME = [0.0, -0.247, -0.909, 0.0, 1.156, 0.0]
# Planner table box; pass the real top (highest surface + 15 mm) and edge on the command line.
XARM6_BENCH_TOP_Z = 0.0
XARM6_BENCH_X_M = (0.1, 1.0)

# Kit wrist mount: D435 screw line 87 mm off the flange axis, tilted 15 deg in, lens 26 mm forward.
XARM6_WRIST_CAMERA_NOMINAL_XYZ = (0.079, -0.041, 0.026)
XARM6_WRIST_CAMERA_NOMINAL_XYZW = (-0.0922963, -0.0922963, 0.7010586, 0.7010561)

XARM6_CALIBRATION_BENCH_PATH = LfsPath("xarm6/calibration_bench.xml")
# The planner keeps continuous joints within one turn, so the draw does too.
XARM6_PLANNER_JOINT_LIMITS: list[tuple[float, float]] = [
    (-math.pi, math.pi) if lo <= -2 * math.pi else (lo, hi) for lo, hi in XARM6_JOINT_LIMITS
]

# Gripper plate for a fixed camera: 5 x 4 ChArUco, 30 mm squares, 22.5 mm markers, ids from 100.
XARM6_CHEST_PLATE = {
    "squares_x": 5,
    "squares_y": 4,
    "square_m": 0.030,
    "marker_m": 0.0225,
    "ids_from": 100,
}
# Plate held out facing a camera on the arm's right (-y), 0.6 m up; solved on the bench twin.
XARM6_CHEST_CALIBRATION_HOME = [-0.212, -0.099, -1.161, -0.773, 1.028, 3.076]
# Tape-measured lens pose, base frame, OpenCV optical axes (z view, x right, y down).
XARM6_CHEST_CAMERA_NOMINAL_XYZ = (0.55, -0.60, 0.60)
XARM6_CHEST_CAMERA_NOMINAL_XYZW = (0.9108557, 0.0753847, -0.0334691, -0.4043994)
XARM6_CHEST_CAMERA_TOLERANCE_M = 0.15
XARM6_CHEST_CAMERA_TOLERANCE_DEG = 10.0
XARM6_CHEST_BENCH_PATH = LfsPath("xarm6/calibration_bench_chest.xml")
# The bench's camera, in the arm's base frame: the base plate sits 0.12 m up its pedestal there.
XARM6_CHEST_SIM_CAMERA_XYZ = (0.55, -0.60, 0.48)
# The simulated wrist camera sits 50 mm in front of the flange looking along the tool.
XARM6_SIM_WRIST_CAMERA_XYZ = (0.05, 0.0, 0.0)
XARM6_SIM_WRIST_CAMERA_XYZW = (0.0, 0.0, 0.7071068, 0.7071068)


def _xarm6_calibration_model() -> RobotModelConfig:
    # tf_extra_links publishes the flange, which the calibration reads its poses from.
    return make_xarm_model_config(6, robot="arm", tf_extra_links=["link6"])


xarm6_handeye_calibrate = autoconnect(
    # the viser view shows each plan before the operator approves it
    planner(model=_xarm6_calibration_model(), visualization={"backend": "viser"}),
    xarm_coordinator("xarm6"),
    RealSenseCamera.blueprint(),
    HandEyeCalibrationModule.blueprint(
        name="xarm6_wrist_camera",
        nominal_translation=XARM6_WRIST_CAMERA_NOMINAL_XYZ,
        nominal_rotation_xyzw=XARM6_WRIST_CAMERA_NOMINAL_XYZW,
        home_joints=XARM6_CALIBRATION_HOME,
        joint_limits_rad=XARM6_PLANNER_JOINT_LIMITS,
        table_top_z=XARM6_BENCH_TOP_Z,
        table_x_m=XARM6_BENCH_X_M,
    ),
).remappings(XARM_REMAPS)


# Simulated arm via the coordinator's MuJoCo adapter, joints named as XArmConnection names them.
_sim_hardware = make_xarm_hardware(
    "arm",
    6,
    adapter_type="sim_mujoco",
    gripper=True,
    home_joints=XARM6_CALIBRATION_HOME,
    canonical_joint_names=[f"arm/joint{i}" for i in range(1, 7)],
)

xarm6_handeye_calibrate_sim = autoconnect(
    planner(model=_xarm6_calibration_model()),
    coordinator(
        hardware=[_sim_hardware],
        tasks=[
            trajectory_task(_sim_hardware),
            TaskConfig(
                name="arm_gripper", type="gripper", joint_names=["arm/gripper"], priority=20
            ),
        ],
    ),
    MujocoSimModule.blueprint(
        address=XARM6_CALIBRATION_BENCH_PATH,
        headless=True,
        dof=len(_sim_hardware.joints),
        camera_name="wrist_camera",
        base_frame_id="link6",
        width=848,
        height=480,
        fps=5,
        reset_joint_positions=[*XARM6_CALIBRATION_HOME, 0.0],
    ),
    HandEyeCalibrationModule.blueprint(
        name="xarm6_wrist_camera_sim",
        nominal_translation=XARM6_SIM_WRIST_CAMERA_XYZ,
        nominal_rotation_xyzw=XARM6_SIM_WRIST_CAMERA_XYZW,
        home_joints=XARM6_CALIBRATION_HOME,
        joint_limits_rad=XARM6_PLANNER_JOINT_LIMITS,
        optical_frame="wrist_camera_color_optical_frame",
        camera_link_frame="wrist_camera_link",
        # the bench twin's table top, between x 0.15 and 0.85
        table_top_z=0.13,
        table_x_m=(0.15, 0.85),
        approve_first_moves=0,
        # the simulated arm creeps onto its target for seconds after the planner reports done
        settle_timeout_s=30.0,
    ),
)


xarm6_handeye_calibrate_chest = autoconnect(
    planner(model=_xarm6_calibration_model(), visualization={"backend": "viser"}),
    xarm_coordinator("xarm6"),
    RealSenseCamera.blueprint(),
    HandEyeCalibrationModule.blueprint(
        name="xarm6_chest_camera",
        camera_frame="base",
        nominal_translation=XARM6_CHEST_CAMERA_NOMINAL_XYZ,
        nominal_rotation_xyzw=XARM6_CHEST_CAMERA_NOMINAL_XYZW,
        nominal_tolerance_m=XARM6_CHEST_CAMERA_TOLERANCE_M,
        nominal_tolerance_deg=XARM6_CHEST_CAMERA_TOLERANCE_DEG,
        board=XARM6_CHEST_PLATE,
        home_joints=XARM6_CHEST_CALIBRATION_HOME,
        joint_limits_rad=XARM6_PLANNER_JOINT_LIMITS,
        table_top_z=XARM6_BENCH_TOP_Z,
        table_x_m=XARM6_BENCH_X_M,
    ),
).remappings(XARM_REMAPS)


_chest_sim_hardware = make_xarm_hardware(
    "arm",
    6,
    adapter_type="sim_mujoco",
    gripper=True,
    home_joints=XARM6_CHEST_CALIBRATION_HOME,
    canonical_joint_names=[f"arm/joint{i}" for i in range(1, 7)],
)

xarm6_handeye_calibrate_chest_sim = autoconnect(
    planner(model=_xarm6_calibration_model()),
    coordinator(
        hardware=[_chest_sim_hardware],
        tasks=[
            trajectory_task(_chest_sim_hardware),
            TaskConfig(
                name="arm_gripper", type="gripper", joint_names=["arm/gripper"], priority=20
            ),
        ],
    ),
    # the camera body sits in the world, so its tf is published against the world frame
    MujocoSimModule.blueprint(
        address=XARM6_CHEST_BENCH_PATH,
        headless=True,
        dof=len(_chest_sim_hardware.joints),
        camera_name="chest_camera",
        base_frame_id="world",
        width=1280,
        height=720,
        fps=5,
        reset_joint_positions=[*XARM6_CHEST_CALIBRATION_HOME, 0.0],
    ),
    HandEyeCalibrationModule.blueprint(
        name="xarm6_chest_camera_sim",
        camera_frame="base",
        nominal_translation=XARM6_CHEST_SIM_CAMERA_XYZ,
        nominal_rotation_xyzw=XARM6_CHEST_CAMERA_NOMINAL_XYZW,
        nominal_tolerance_m=XARM6_CHEST_CAMERA_TOLERANCE_M,
        nominal_tolerance_deg=XARM6_CHEST_CAMERA_TOLERANCE_DEG,
        board=XARM6_CHEST_PLATE,
        home_joints=XARM6_CHEST_CALIBRATION_HOME,
        joint_limits_rad=XARM6_PLANNER_JOINT_LIMITS,
        optical_frame="chest_camera_color_optical_frame",
        camera_link_frame="chest_camera_link",
        # the bench twin's table top, between x 0.15 and 0.85
        table_top_z=0.13,
        table_x_m=(0.15, 0.85),
        approve_first_moves=0,
        settle_timeout_s=30.0,
    ),
)


# xArm7 kitting bench, base frame: x toward the front table edge, y toward the OpenYAM end, z up.

# Home pose in joint degrees, as UFactory Studio shows it.
XARM7_CALIBRATION_HOME = [
    round(math.radians(v), 4) for v in (-5.1, -35.0, -0.8, 53.5, -1.8, 87.5, 1.5)
]
XARM7_PLANNER_JOINT_LIMITS: list[tuple[float, float]] = [
    (-math.pi, math.pi) if lo <= -2 * math.pi else (lo, hi) for lo, hi in XARM7_JOINT_LIMITS
]
# Table top 8 mm under the base, box top 15 mm above it for the taped board.
XARM7_BENCH_TOP_Z = 0.007
XARM7_BENCH_X_M = (0.10, 0.695)
XARM7_BENCH_Y_M = (-0.61, 0.914)
XARM7_BENCH_BOXES = [
    # the wall, 12 in behind the table's back edge
    BoxObstacle(name="hand_eye_wall", centre=(-0.422, 0.15, 0.5), size=(0.1, 2.6, 2.0)),
    # the dual OpenYAM at the right end of the table, its arms resting, the camera post included
    BoxObstacle(name="hand_eye_openyam", centre=(0.315, 0.775, 0.245), size=(0.67, 0.65, 0.5)),
]
# Where the board may lie: the done and empty-bin zones in front of the arm.
XARM7_SEARCH_X_M = (0.15, 0.55)
XARM7_SEARCH_Y_M = (-0.55, 0.35)
# Prior: the xArm6's measured kit-mount pose; link6 and link7 are both the tool flange.
XARM7_WRIST_CAMERA_NOMINAL_XYZ = (0.0770182, -0.0408053, 0.0285793)
XARM7_WRIST_CAMERA_NOMINAL_XYZW = (-0.0927751, -0.0807643, 0.7044101, 0.6990539)
XARM7_CALIBRATION_BENCH_PATH = LfsPath("xarm7_kitting_bench/calibration_bench.xml")

_XARM7_BENCH = {
    "table_top_z": XARM7_BENCH_TOP_Z,
    "table_x_m": XARM7_BENCH_X_M,
    "table_y_m": XARM7_BENCH_Y_M,
    "obstacle_boxes": XARM7_BENCH_BOXES,
    "find_board": True,
    "search_x_m": XARM7_SEARCH_X_M,
    "search_y_m": XARM7_SEARCH_Y_M,
}


def _xarm7_calibration_model() -> RobotModelConfig:
    # the flange, which the poses are read from, and the tip, which pose targets are given for
    return make_xarm_model_config(
        7,
        robot="arm",
        base_pose=PoseStamped(frame_id="world"),
        tf_extra_links=["link7", "link_tcp"],
    )


def _with_held_card(config: RobotModelConfig) -> RobotModelConfig:
    card = RobotModel.from_urdf_string(XARM7_HELD_CARD_URDF)
    return config.model_copy(
        update={
            "model": config.model.with_attached_model(
                card, parent_link="link7", joint_name="held_card_joint", xyz=XARM7_HELD_CARD_XYZ
            ),
            "collision_exclusion_pairs": [
                *config.collision_exclusion_pairs,
                *(("held_card", link) for link in XARM7_HELD_CARD_TOUCHES),
            ],
        }
    )


xarm7_handeye_calibrate = autoconnect(
    planner(
        model=_xarm7_calibration_model(),
        gripper_tasks={"manipulator": "arm_gripper"},
        visualization={"backend": "viser"},
    ),
    xarm_coordinator("xarm7"),
    RealSenseCamera.blueprint(),
    HandEyeCalibrationModule.blueprint(
        name="xarm7_wrist_camera",
        nominal_translation=XARM7_WRIST_CAMERA_NOMINAL_XYZ,
        nominal_rotation_xyzw=XARM7_WRIST_CAMERA_NOMINAL_XYZW,
        home_joints=XARM7_CALIBRATION_HOME,
        joint_limits_rad=XARM7_PLANNER_JOINT_LIMITS,
        flange_frame="link7",
        **_XARM7_BENCH,
    ),
).remappings(XARM_REMAPS)


def _xarm7_sim(
    *,
    camera_name: str,
    base_frame_id: str,
    width: int,
    height: int,
    address: LfsPath = XARM7_CALIBRATION_BENCH_PATH,
    held_card: bool = False,
) -> Blueprint:
    """The simulated xArm7 on the kitting bench twin, streaming one of its cameras."""
    hardware = make_xarm7_sim_hardware(home_joints=XARM7_CALIBRATION_HOME)
    model = make_xarm7_sim_robot_config(
        base_pose=PoseStamped(frame_id="world"), tf_extra_links=["link7", "link_tcp"]
    )
    return autoconnect(
        planner(
            model=_with_held_card(model) if held_card else model,
            gripper_tasks={"manipulator": "arm_gripper"},
        ),
        coordinator(
            hardware=[hardware],
            tasks=[
                trajectory_task(hardware),
                TaskConfig(
                    name="arm_gripper", type="gripper", joint_names=["arm/gripper"], priority=20
                ),
            ],
        ),
        MujocoSimModule.blueprint(
            address=address,
            headless=True,
            dof=7,
            camera_name=camera_name,
            base_frame_id=base_frame_id,
            width=width,
            height=height,
            fps=5,
            reset_joint_positions=XARM7_CALIBRATION_HOME,
        ),
    )


# The twin's camera sits on the measured mount; the run starts from the kit drawing's, mm off.
xarm7_handeye_calibrate_sim = autoconnect(
    _xarm7_sim(camera_name="wrist_camera", base_frame_id="link7", width=848, height=480),
    HandEyeCalibrationModule.blueprint(
        name="xarm7_wrist_camera_sim",
        nominal_translation=XARM6_WRIST_CAMERA_NOMINAL_XYZ,
        nominal_rotation_xyzw=XARM6_WRIST_CAMERA_NOMINAL_XYZW,
        home_joints=XARM7_CALIBRATION_HOME,
        joint_limits_rad=XARM7_PLANNER_JOINT_LIMITS,
        flange_frame="link7",
        optical_frame="wrist_camera_color_optical_frame",
        camera_link_frame="wrist_camera_link",
        approve_first_moves=0,
        settle_timeout_s=30.0,
        **_XARM7_BENCH,
    ),
)


# Tripod camera, position unmeasured: hold the plate at each hold point facing each aim until seen.
# Side holds because the card stops decoding about 1 m from the lens.
XARM7_SCENE_HOLDS = [(0.45, 0.0, 0.30), (0.50, 0.25, 0.38), (0.50, -0.25, 0.38)]
XARM7_SCENE_AIMS = [(x, y, 0.70) for x in (1.20, 1.50) for y in (0.0, -0.45, 0.45)]
# Middle of the tripod band; only the keep-out sphere and the default aim use it.
XARM7_SCENE_CAMERA_GUESS_XYZ = (1.35, 0.0, 0.70)
XARM7_SCENE_CAMERA_GUESS_XYZW = (0.6311099, 0.6311099, -0.3189048, -0.3189048)
# link7 -> board frame of the held plate, from the twin; only for aiming.
XARM7_PLATE_XYZ = (-0.075, 0.0, 0.277)
XARM7_PLATE_XYZW = (-0.7071068, 0.0, 0.0, 0.7071068)
# The held card as a planner body on link7, 36 cm out, so no move takes it into the table.
XARM7_HELD_CARD_URDF = """<robot name="held_card"><link name="held_card">
<visual><geometry><box size="0.19 0.03 0.193"/></geometry></visual>
<collision><geometry><box size="0.19 0.03 0.193"/></geometry></collision>
</link></robot>"""
XARM7_HELD_CARD_XYZ = (0.0, -0.007, 0.2635)
# The fingers clamp it, and the gripper base sits right behind it.
XARM7_HELD_CARD_TOUCHES = (
    "xarm_gripper_base_link",
    "left_finger",
    "right_finger",
    "left_inner_knuckle",
    "right_inner_knuckle",
    "left_outer_knuckle",
    "right_outer_knuckle",
    "link_tcp",
)
XARM7_SCENE_BENCH_PATH = LfsPath("xarm7_kitting_bench/calibration_bench_scene.xml")

_XARM7_SCENE = {
    "camera_frame": "base",
    "nominal_translation": XARM7_SCENE_CAMERA_GUESS_XYZ,
    "nominal_rotation_xyzw": XARM7_SCENE_CAMERA_GUESS_XYZW,
    # checked against the first view of the plate, which is good to a few millimetres
    "nominal_tolerance_m": 0.05,
    "nominal_tolerance_deg": 10.0,
    "board": XARM6_CHEST_PLATE,
    "home_joints": XARM7_CALIBRATION_HOME,
    "joint_limits_rad": XARM7_PLANNER_JOINT_LIMITS,
    "flange_frame": "link7",
    "table_top_z": XARM7_BENCH_TOP_Z,
    "table_x_m": XARM7_BENCH_X_M,
    "table_y_m": XARM7_BENCH_Y_M,
    "obstacle_boxes": XARM7_BENCH_BOXES,
    "find_camera": True,
    "hold_points": XARM7_SCENE_HOLDS,
    "aim_points": XARM7_SCENE_AIMS,
    "plate_translation": XARM7_PLATE_XYZ,
    "plate_rotation_xyzw": XARM7_PLATE_XYZW,
}

xarm7_handeye_calibrate_scene = autoconnect(
    planner(
        model=_with_held_card(_xarm7_calibration_model()),
        gripper_tasks={"manipulator": "arm_gripper"},
        visualization={"backend": "viser"},
    ),
    xarm_coordinator("xarm7"),
    RealSenseCamera.blueprint(width=1280, height=720),
    HandEyeCalibrationModule.blueprint(name="xarm7_scene_camera", **_XARM7_SCENE),
).remappings(XARM_REMAPS)

xarm7_handeye_calibrate_scene_sim = autoconnect(
    _xarm7_sim(
        camera_name="scene_camera",
        base_frame_id="world",
        width=1280,
        height=720,
        address=XARM7_SCENE_BENCH_PATH,
        held_card=True,
    ),
    HandEyeCalibrationModule.blueprint(
        name="xarm7_scene_camera_sim",
        optical_frame="scene_camera_color_optical_frame",
        camera_link_frame="scene_camera_link",
        approve_first_moves=0,
        settle_timeout_s=30.0,
        **_XARM7_SCENE,
    ),
)
