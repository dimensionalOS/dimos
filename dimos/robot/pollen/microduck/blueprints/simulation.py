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

from pathlib import Path
from typing import Any

from dimos.control.coordinator import ControlCoordinator, TaskConfig
from dimos.core.coordination.blueprints import Blueprint, autoconnect
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
    MICRODUCK_MESHDIR,
    MICRODUCK_POLICY_DIR,
    MICRODUCK_ROBOT_MJCF,
    MICRODUCK_SCENE,
    MICRODUCK_SIM_SPEC,
    make_microduck_sim_hardware,
)
from dimos.robot.pollen.microduck.rerun import (
    MICRODUCK_RERUN_JOINTS,
    MICRODUCK_RERUN_ROOT,
    MICRODUCK_RERUN_SCENE,
    microduck_joint_state,
    microduck_static_robot,
    microduck_static_scene,
)
from dimos.simulation.engines.mujoco_sim_module import MujocoSimModule
from dimos.simulation.scenes.catalog import resolve_scene_package
from dimos.visualization.rerun.websocket_server import RerunWebSocketServer
from dimos.visualization.vis_module import vis_module
from dimos.web.websocket_vis.websocket_vis_module import WebsocketVisModule


class _MicroDuckCoordinator(ControlCoordinator):
    microduck_joints: Out[JointState]


if global_config.simulation and global_config.simulation != "mujoco":
    raise ValueError("microduck-sim only supports --simulation mujoco")


def _microduck_mujoco_backend(
    scene_package: str | Path | None,
) -> tuple[Blueprint, str | Path]:
    common: dict[str, Any] = {
        "headless": True,
        "dof": len(MICRODUCK_JOINTS),
        "reset_joint_positions": list(MICRODUCK_HOME),
        "camera_name": "head_camera",
        "base_frame_id": "trunk_base",
        "width": 320,
        "height": 240,
        "fps": 15,
        "enable_color": True,
        "enable_depth": True,
        "enable_pointcloud": False,
        "robot_sim_spec": MICRODUCK_SIM_SPEC,
        "imu_gyro_sensor_names": ["imu_ang_vel", "angular-velocity"],
        "imu_accel_sensor_names": ["imu_accel"],
    }
    package = resolve_scene_package(scene_package)
    if package is None:
        return (
            MujocoSimModule.blueprint(
                address=MICRODUCK_SCENE,
                spawn_z=0.125,
                **common,
            ),
            MICRODUCK_SCENE,
        )
    if package.mujoco_scene_path is None:
        raise ValueError(f"scene package has no MuJoCo scene artifact: {package.metadata_path}")

    return (
        MujocoSimModule.blueprint(
            scene_xml=package.mujoco_scene_path,
            robot_mjcf=MICRODUCK_ROBOT_MJCF,
            robot_meshdir=MICRODUCK_MESHDIR,
            robot_id="",
            scene_entities=package.entities,
            timestep=0.005,
            **common,
        ),
        MICRODUCK_ROBOT_MJCF,
    )


_simulator, _adapter_address = _microduck_mujoco_backend(global_config.scene_package)

_coordinator = _MicroDuckCoordinator.blueprint(
    instance_name="ControlCoordinator",
    tick_rate=50.0,
    publish_robot_joint_states=True,
    hardware=[make_microduck_sim_hardware(_adapter_address)],
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
    autoconnect(
        _simulator,
        _coordinator,
        vis_module(
            viewer_backend=global_config.viewer,
            rerun_config={
                "visual_override": {
                    MICRODUCK_RERUN_JOINTS: microduck_joint_state,
                },
                "static": {
                    MICRODUCK_RERUN_ROOT: microduck_static_robot,
                    MICRODUCK_RERUN_SCENE: microduck_static_scene,
                },
                "max_hz": {
                    MICRODUCK_RERUN_JOINTS: 20.0,
                },
            },
        ),
    )
    .remappings(
        [
            (_MicroDuckCoordinator, "twist_command", "cmd_vel"),
            (RerunWebSocketServer, "tele_cmd_vel", "cmd_vel"),
            (WebsocketVisModule, "tele_cmd_vel", "cmd_vel"),
        ]
    )
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
    .global_config(robot_model="microduck", simulation="mujoco", n_workers=3)
)
