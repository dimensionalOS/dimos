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

"""Official-policy MicroDuck simulation through ControlCoordinator."""

from __future__ import annotations

from dimos.control.coordinator import ControlCoordinator, TaskConfig
from dimos.core.coordination.blueprints import autoconnect
from dimos.core.global_config import global_config
from dimos.core.stream import Out
from dimos.core.transport import LCMTransport, pSHMTransport
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.Imu import Imu
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.robot.pollen.microduck.config import (
    MICRODUCK_HOME,
    MICRODUCK_JOINTS,
    MICRODUCK_POLICY_DIR,
    MICRODUCK_SCENE,
    MICRODUCK_SIM_SPEC,
    make_microduck_sim_hardware,
)
from dimos.simulation.engines.mujoco_sim_module import MujocoSimModule


class _MicroDuckCoordinator(ControlCoordinator):
    microduck_joints: Out[JointState]


if global_config.simulation and global_config.simulation != "mujoco":
    raise ValueError("microduck-sim only supports --simulation mujoco")


_simulator = MujocoSimModule.blueprint(
    address=MICRODUCK_SCENE,
    headless=global_config.viewer == "none",
    dof=len(MICRODUCK_JOINTS),
    spawn_z=0.125,
    reset_joint_positions=list(MICRODUCK_HOME),
    camera_name="head_camera",
    base_frame_id="trunk_base",
    width=320,
    height=240,
    fps=15,
    enable_color=True,
    enable_depth=True,
    enable_pointcloud=False,
    robot_sim_spec=MICRODUCK_SIM_SPEC,
    imu_gyro_sensor_names=["imu_ang_vel", "angular-velocity"],
    imu_accel_sensor_names=["imu_accel"],
)

_coordinator = _MicroDuckCoordinator.blueprint(
    instance_name="ControlCoordinator",
    tick_rate=50.0,
    publish_robot_joint_states=True,
    hardware=[make_microduck_sim_hardware()],
    tasks=[
        TaskConfig(
            name="microduck_policy",
            type="microduck_policy",
            joint_names=list(MICRODUCK_JOINTS),
            priority=50,
            auto_start=True,
            params={
                "policy_dir": MICRODUCK_POLICY_DIR,
                "hardware_id": "microduck",
                "auto_arm": True,
            },
        )
    ],
)

microduck_sim = (
    autoconnect(_simulator, _coordinator)
    .remappings([(_MicroDuckCoordinator, "twist_command", "cmd_vel")])
    .transports(
        {
            ("cmd_vel", Twist): LCMTransport("/microduck/cmd_vel", Twist),
            ("microduck_joints", JointState): LCMTransport("/microduck/joints", JointState),
            ("imu", Imu): LCMTransport("/microduck/imu", Imu),
            ("odom", PoseStamped): LCMTransport("/microduck/odom", PoseStamped),
            ("color_image", Image): pSHMTransport("/microduck/color_image"),
            ("depth_image", Image): pSHMTransport("/microduck/depth_image"),
            ("camera_info", CameraInfo): LCMTransport("/microduck/camera_info", CameraInfo),
            ("depth_camera_info", CameraInfo): LCMTransport(
                "/microduck/depth_camera_info", CameraInfo
            ),
        }
    )
    .global_config(robot_model="microduck", simulation="mujoco", n_workers=2)
)
