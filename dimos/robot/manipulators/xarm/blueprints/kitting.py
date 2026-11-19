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

"""Bin handling on the xArm7 kitting bench, on the arm and on its twin.

``dimos run xarm7-kitting --xarmconnection.ip <ip>``, then in ``dimos shell``::

    kit = app.get_module("XArm7KittingModule")
    kit.run_cycle(0.33, -0.14, 25.0)   # the empty bin's footprint centre (m) and yaw (deg)

``xarm7-kitting-scene`` adds the fixed scene camera (calibrated with ``xarm7-handeye-calibrate-scene``)
and the bin finder, so the empty bin's pose comes from the camera::

    dimos run xarm7-kitting-scene --xarmconnection.ip <ip> --realsensecamera.serial-number <serial>
    finder = app.get_module("XArm7BinFinder")
    finder.find_bins()        # every bin: zone, centre, yaw, and an annotated image
    finder.load_from_scan()   # load the empty bin it found
"""

from __future__ import annotations

import math

from dimos.control.coordinator import TaskConfig
from dimos.core.coordination.blueprints import autoconnect
from dimos.hardware.sensors.camera.realsense.camera import RealSenseCamera
from dimos.manipulation.planning.spec.config import RobotModelConfig
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.robot.manipulators.common.blueprints import coordinator, planner, trajectory_task
from dimos.robot.manipulators.xarm.bin_finder import XArm7BinFinder
from dimos.robot.manipulators.xarm.blueprints.basic import XARM_REMAPS, xarm_coordinator
from dimos.robot.manipulators.xarm.config import make_xarm7_sim_hardware, make_xarm_model_config
from dimos.robot.manipulators.xarm.kitting import XArm7KittingModule
from dimos.simulation.engines.mujoco_sim_module import MujocoSimModule
from dimos.utils.data import LfsPath

# The home Krishna set on the arm in UFactory Studio, 2026-11-16.
XARM7_KITTING_HOME = [round(math.radians(v), 4) for v in (-5.1, -35.0, -0.8, 53.5, -1.8, 87.5, 1.5)]
XARM7_KITTING_BENCH_PATH = LfsPath("xarm7_kitting_bench/kitting_bench.xml")
# The twin's free bodies, whose poses the twin publishes on body_poses.
XARM7_KITTING_BODIES = [
    "bin_loading",
    "bin_empty_1",
    "bin_empty_2",
    "bin_done",
    "bottle_clear",
    "bottle_black",
]


# A full bin hangs from one wall: gentle starts and stops keep it from swinging in the fingers.
XARM7_KITTING_ACCELERATION = 0.6  # rad/s^2, every joint; the model's default is 2.0


def _model(robot: str | None) -> RobotModelConfig:
    config = make_xarm_model_config(
        7,
        robot=robot,
        base_pose=PoseStamped(frame_id="world"),
        tf_extra_links=["link7", "link_tcp"],
        home_joints=XARM7_KITTING_HOME,
    )
    gentle = config.model.with_default_joint_acceleration_limit(XARM7_KITTING_ACCELERATION)
    return config.model_copy(update={"model": gentle})


xarm7_kitting = autoconnect(
    planner(
        model=_model("arm"),
        gripper_tasks={"manipulator": "arm_gripper"},
        visualization={"backend": "viser"},
    ),
    xarm_coordinator("xarm7"),
    XArm7KittingModule.blueprint(),
).remappings(XARM_REMAPS)

_sim_hardware = make_xarm7_sim_hardware(home_joints=XARM7_KITTING_HOME)

xarm7_kitting_sim = autoconnect(
    planner(model=_model(None), gripper_tasks={"manipulator": "arm_gripper"}),
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
        address=XARM7_KITTING_BENCH_PATH,
        headless=True,
        dof=7,
        camera_name="scene_camera",
        base_frame_id="world",
        width=640,
        height=360,
        fps=2,
        reset_joint_positions=XARM7_KITTING_HOME,
        tracked_bodies=XARM7_KITTING_BODIES,
    ),
    XArm7KittingModule.blueprint(),
)

xarm7_kitting_scene = autoconnect(
    xarm7_kitting, RealSenseCamera.blueprint(), XArm7BinFinder.blueprint()
)

# The twin's scene camera stands where the scene calibration twin's does, so its sim result is
# the camera pose here.
xarm7_kitting_scene_sim = autoconnect(
    xarm7_kitting_sim, XArm7BinFinder.blueprint(calibration="xarm7_scene_camera_sim")
)
