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

"""Alfred's bench stack with all of its hardware simulated: rehearse a demo with no robot.

    dimos run alfred-manipulation-sim

The modules and the wiring are alfred-manipulation's (``alfred_bench_modules``), so the
Manipulation app and the Controller drive it as they drive the robot. Only what touches
hardware is swapped:

* arms: the OpenArms' mock adapter in the ready pose, whatever OPENARM_LEFT_CAN and
  OPENARM_RIGHT_CAN say (the robot's .env sets them): no CAN is opened. The grippers keep
  their coordinator tasks, so Open and Close work;
* lift: PillarConnection on ``PILLAR_SIM_DEVICE``, the Nano's serial protocol emulated
  in-process (``pillar_emulator``): home, set_position, stop_motion and get_status answer as on
  the robot, and the coordinator reaches the lift through the same transport adapter;
* base: the coordinator's mock twist base, integrating what it is told through Alfred's
  measured plant (``alfred_posedomain.json``: yaw 2.85x, translation 0.77x), so a plan's base
  segment runs on the same command gains as on the robot. ``CoordinatorBaseOdometry`` reads its
  pose back as the wheel odometry AlfredHighLevel publishes there, and the same
  ``WheelOdometryRelay`` puts ``odom -> base_link`` on tf and ``/flowbase/odom``;
* camera: ``MujocoSimModule`` rendering ``alfred_bench_scene.xml`` from the mast D455's pose,
  under its names and frames (``camera_front_color_optical_frame`` under AlfredMountTf's
  ``camera_front_link``), 848x480 at 15 Hz, a cloud of every third pixel to 3 m at 5 Hz. The
  scene's ``base_link`` follows ``/flowbase/odom``, so the picture moves when the base does;
* models: what this machine runs (``perception_device``, see ``SIM_MODELS``).

The scene is static: nothing a gripper closes on moves, and the robot is not in it (the arms
move in the Controller's view, not in the picture).

Never run it on a network shared with the robot while the robot's stack runs: zenoh finds
peers on the LAN, and this coordinator answers to the same RPC names as the real one.
"""

from __future__ import annotations

from pathlib import Path
from typing import Any

from dimos.control.blueprints.mobile import _mock_twist_base
from dimos.control.components import HardwareComponent
from dimos.control.coordinator import ControlCoordinatorConfig
from dimos.core.coordination.blueprints import autoconnect
from dimos.robot.diy.alfred.alfred_model import alfred_base_command_gains
from dimos.robot.diy.alfred.blueprints.alfred_full import (
    ALFRED_BASE_HARDWARE_ID,
    alfred_base_transports,
    alfred_openarm_hardware,
)
from dimos.robot.diy.alfred.blueprints.alfred_full_perception import (
    D455_FPS,
    D455_HEIGHT,
    D455_POINTCLOUD_DECIMATION,
    D455_POINTCLOUD_FPS,
    D455_WIDTH,
    DEPTH_TRUNC_M,
    GraspGenerator,
    PerceptionDevice,
    SceneSegmenter,
    alfred_perception_modules,
    perception_device,
)
from dimos.robot.diy.alfred.blueprints.alfred_manipulation import (
    BenchHardware,
    alfred_bench_modules,
)
from dimos.robot.diy.alfred.coordinator_base_odometry import CoordinatorBaseOdometry
from dimos.robot.diy.alfred.pillar_connection import PILLAR_SIM_DEVICE
from dimos.simulation.engines.mujoco_sim_module import MujocoSimModule
from dimos.simulation.mujoco.headless_gl import default_mujoco_gl

ALFRED_BENCH_SCENE = Path(__file__).with_name("alfred_bench_scene.xml")
# The D455's stem: its frames are camera_front_link, camera_front_color_optical_frame, ...
SIM_CAMERA = "camera_front"
# The scene body that stands for base_link, moved by the base's odometry.
SIM_BASE_BODY = "base_link"

# What the pick pipeline runs where: GraspGenX needs CUDA, EdgeTAM CUDA or Apple's MPS.
SIM_MODELS: dict[PerceptionDevice, tuple[GraspGenerator, SceneSegmenter]] = {
    "cuda": ("graspgenx", "edgetam"),
    "mps": ("heuristic", "edgetam"),
    "cpu": ("heuristic", "none"),
}
# OWLv2 on one rendered frame of the scene (2026-10-25): tennis ball 0.69, cardboard box 0.39,
# soda can 0.31, bottle 0.27, cucumber 0.16 (the app's "ball", "box", "cup" 0.60, 0.56, 0.25),
# the best false box 0.07. The bench's 0.3 would miss three of the five; EdgeTAM masked all
# five at IoU 0.99 against the scene's own segmentation.
SIM_SCENE_DETECTOR_CONFIDENCE = 0.1

# The worker that renders imports mujoco after this; on Linux with no display it needs EGL.
default_mujoco_gl()


def alfred_sim_flowbase() -> HardwareComponent:
    """The coordinator's mock twist base under the FlowBase's name (the tasks claim its joints),
    integrated once a tick through the measured plant the base task's command gains invert."""
    base = _mock_twist_base(ALFRED_BASE_HARDWARE_ID)
    tick_rate = ControlCoordinatorConfig.model_fields["tick_rate"].default
    base.adapter_kwargs = {
        "integrate_odometry": True,
        "step_dt": 1.0 / tick_rate,
        "plant_gains": tuple(1.0 / gain for gain in alfred_base_command_gains()),
    }
    return base


def alfred_sim_camera() -> Any:
    """The mast D455 as a MuJoCo camera: the same streams, names, frames and rates."""
    return MujocoSimModule.blueprint(
        address=str(ALFRED_BENCH_SCENE),
        headless=True,
        camera_name=SIM_CAMERA,
        # Mounted on a body named for the D455's link: the mount tree (AlfredMountTf) owns
        # base_link -> camera_front_link; the camera adds only its optical frames below it.
        base_frame_id=f"{SIM_CAMERA}_link",
        width=D455_WIDTH,
        height=D455_HEIGHT,
        fps=D455_FPS,
        enable_color=True,
        enable_depth=True,
        align_depth_to_color=True,
        enable_pointcloud=True,
        pointcloud_fps=D455_POINTCLOUD_FPS,
        pointcloud_decimation=D455_POINTCLOUD_DECIMATION,
        depth_trunc_m=DEPTH_TRUNC_M,
        mocap_body=SIM_BASE_BODY,
    ).remappings(
        [
            # base_link's pose in odom, as WheelOdometryRelay publishes it for the coordinator.
            (MujocoSimModule, "mocap_pose", "start_pose"),
            # Only a scene with a free-jointed robot publishes it: odom is tf's frame here.
            (MujocoSimModule, "odom", "mujoco_odom_unused"),
        ]
    )


def alfred_sim_hardware(device: PerceptionDevice) -> BenchHardware:
    """Every stand-in the bench stack needs, and the models ``device`` can run."""
    grasp, segmenter = SIM_MODELS[device]
    base = alfred_sim_flowbase()
    return BenchHardware(
        flowbase=CoordinatorBaseOdometry.blueprint(base_joints=list(base.joints)),
        pillar_device_path=PILLAR_SIM_DEVICE,
        arms=alfred_openarm_hardware(),
        base=base,
        perception=alfred_perception_modules(
            models=True,
            localize=False,
            grasp=grasp,
            camera=alfred_sim_camera(),
            scene_segmenter=segmenter,
            scene_detector_confidence=SIM_SCENE_DETECTOR_CONFIDENCE,
        ),
    )


def alfred_sim_modules(device: PerceptionDevice | None = None) -> tuple[Any, ...]:
    """alfred-manipulation's modules on simulated hardware; ``device`` overrides the probe."""
    return alfred_bench_modules(hardware=alfred_sim_hardware(device or perception_device()))


alfred_manipulation_sim = (
    autoconnect(*alfred_sim_modules())
    .transports(alfred_base_transports())
    .global_config(n_workers=10, robot_model="alfred")
)
