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

"""Hand-eye calibration of the xArm6 wrist camera.

``dimos run xarm6-handeye-calibrate --xarmconnection.ip <ip>`` brings up the planner, the
coordinator, the wrist RealSense and the calibration module on the real arm; then from
``dimos shell``: ``app.get_module("HandEyeCalibrationModule").calibrate()`` and ``approve()``
for each of the first moves. ``xarm6-handeye-calibrate-sim`` is the same stack over the MuJoCo
bench in data/xarm6/calibration_bench.xml, for testing the run without an arm.
"""

from __future__ import annotations

import math

from dimos.control.coordinator import TaskConfig
from dimos.core.coordination.blueprints import autoconnect
from dimos.hardware.sensors.camera.realsense.camera import RealSenseCamera
from dimos.manipulation.calibration.hand_eye_module import HandEyeCalibrationModule
from dimos.manipulation.planning.spec.config import RobotModelConfig
from dimos.robot.manipulators.common.blueprints import coordinator, planner, trajectory_task
from dimos.robot.manipulators.xarm.blueprints.basic import XARM_REMAPS, xarm_coordinator
from dimos.robot.manipulators.xarm.config import (
    XARM6_JOINT_LIMITS,
    make_xarm_hardware,
    make_xarm_model_config,
)
from dimos.simulation.engines.mujoco_sim_module import MujocoSimModule
from dimos.utils.data import LfsPath

# Tool pointing down about 0.4 m over a board 0.45 m in front of the base; the wrist at zero
# keeps the camera cable untwisted (the board fits the view either way round).
XARM6_CALIBRATION_HOME = [0.0, -0.247, -0.909, 0.0, 1.156, 0.0]
# The bench the arm shares with the board, as the planner box the module adds for the run: a
# table top level with the base plate reaching from just past the base to 1 m out. Set
# table_top_z to the highest surface within reach, the board included, plus 15 mm, and
# table_x_m to the measured edge, on the command line.
XARM6_BENCH_TOP_Z = 0.0
XARM6_BENCH_X_M = (0.1, 1.0)

# The wrist RealSense on the xArm6 kit mount: the camera's screw line 87 mm off the flange
# axis, tilted 15 degrees inward, looking along the tool. The colour lens sits at one end of
# the D435 body, 41 mm along it from the screw line, and 26 mm forward of the flange face
# (first hardware run, 2026-11-03; the kit drawing's screw line was 42 mm off as a prior).
XARM6_WRIST_CAMERA_NOMINAL_XYZ = (0.079, -0.041, 0.026)
XARM6_WRIST_CAMERA_NOMINAL_XYZW = (-0.0922963, -0.0922963, 0.7010586, 0.7010561)

XARM6_CALIBRATION_BENCH_PATH = LfsPath("xarm6/calibration_bench.xml")
# The planner's model keeps the continuous joints within one turn, so the draw stays there too
# or its targets would be refused at the limit.
XARM6_PLANNER_JOINT_LIMITS: list[tuple[float, float]] = [
    (-math.pi, math.pi) if lo <= -2 * math.pi else (lo, hi) for lo, hi in XARM6_JOINT_LIMITS
]

# A fixed camera beside the bench, the mirror run: the printed gripper plate rides in the closed
# fingers and the camera watches it. The plate: 5 by 4 ChArUco, 30 mm squares, 22.5 mm markers,
# ids from 100 (real/boards/gripper_plate_A4.pdf).
XARM6_CHEST_PLATE = {
    "squares_x": 5,
    "squares_y": 4,
    "square_m": 0.030,
    "marker_m": 0.0225,
    "ids_from": 100,
}
# Home for the plate run: the plate held out in front of the arm, its face square to a camera
# on the arm's right (negative y), 0.6 m up, 0.45 m from it, with the arm a tool's length above
# the bench and 0.39 m clear of the camera; solved on the bench twin for the nominal below.
XARM6_CHEST_CALIBRATION_HOME = [-0.212, -0.099, -1.161, -0.773, 1.028, 3.076]
# Where the lens is and where it looks, base frame, from a tape measure: base centre to the lens
# (0.55 m out, 0.60 m to the right, 0.60 m up) looking at the middle of the workspace. The
# verdict allows the solve 0.30 m from this, so a tape measure is enough. OpenCV optical axes:
# z along the view, x image right (horizontal), y image down.
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
    return make_xarm_model_config(
        6, robot="arm", gripper_hardware_id="arm", tf_extra_links=["link6"]
    )


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


# The simulated arm, through the coordinator's MuJoCo adapter, with its joints named the way
# XArmConnection names the real ones, which is what the planner model expects.
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
