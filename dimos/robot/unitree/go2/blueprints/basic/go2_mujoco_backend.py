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

"""MuJoCo sim backend for the go2 blueprint family.

Replaces the legacy ``mujoco_process.py`` subprocess sim: ``MujocoSimModule``
runs the physics in-process (robot MJCF composed into a scene package or a
flat floor) and ``ControlCoordinator`` runs an mjlab-trained rough-terrain
velocity policy (``quadruped_velocity`` task) at 50 Hz, commanding the
robot's MuJoCo position actuators over SHM. The module publishes the same
ports the rest of the go2 stack consumes from ``GO2Connection``:
``pointcloud`` (raycast lidar, remapped onto mappers' ``lidar`` inputs),
``odom``, ``color_image``/``camera_info``, and takes ``cmd_vel`` into the
coordinator.

Note the simulated body is a Unitree **Go1** for now - the policy
(``go1_heightscan_policy.onnx``, trained in mjlab with a terrain height
scan) and the vendored MJCF (``go1_mjlab.xml``, the exact spec it trained
against) are Go1 artifacts standing in for the Go2, exactly like the
legacy sim's go2->go1 remap did. Swap both files to move to a real Go2
policy; the wiring is morphology-agnostic.

Scene selection follows ``--scene-package`` (office / supermarket /
lowpoly_tdm / a package path); without it the robot walks a flat floor.
"""

from __future__ import annotations

from pathlib import Path
from typing import Any

from dimos.control.components import HardwareComponent, HardwareType
from dimos.control.coordinator import ControlCoordinator, TaskConfig
from dimos.control.tasks.quadruped_velocity_task.quadruped_velocity_task import (
    GO1_DEFAULT_POSITIONS,
    GO1_KD,
    GO1_KP,
    make_go1_joints,
)
from dimos.core.coordination.blueprints import Blueprint, autoconnect
from dimos.core.global_config import global_config
from dimos.core.transport import LCMTransport
from dimos.hardware.whole_body.spec import WholeBodyConfig
from dimos.mapping.voxels import VoxelGridMapper
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.simulation.engines.mujoco_sim_module import MujocoSimModule
from dimos.simulation.engines.robot_sim_binding import RobotSimSpec
from dimos.utils.data import LfsPath

_ASSETS_DIR = Path(__file__).resolve().parents[2] / "assets"
_ROBOT_MJCF = _ASSETS_DIR / "go1_mjlab.xml"
_FLAT_SCENE_XML = _ASSETS_DIR / "go1_flat_scene.xml"
_POLICY_ONNX = LfsPath("mujoco_sim/go1_heightscan_policy.onnx")

_JOINTS = make_go1_joints("go2")
_SPAWN_Z = 0.278  # trained init height above ground
_WIDTH_CLEARANCE = 0.35

# Known-good spawns inside cooked scene packages (world coordinates).
_SCENE_SPAWNS: dict[str, tuple[float, float]] = {
    "dimos_office": (-2.0, 1.6),
}

_LIDAR_CAMERAS = (
    "lidar_front_camera",
    "lidar_left_camera",
    "lidar_right_camera",
)
# Robot geoms occupy groups 0/1; legacy floors use group 2 and cooked scene
# packages/entities use group 3, so rays only see world geometry.
_WORLD_GEOM_GROUPS = [2, 3]

_sim_spec = RobotSimSpec(
    robot_id="go2",
    hardware_joints=tuple(_JOINTS),
    root_body_names=("trunk",),
    root_joint_names=("floating_base_joint",),
    require_floating_base=True,
    imu_gyro_names=("imu_ang_vel",),
    imu_accel_names=("imu_lin_acc",),
    imu_linvel_names=("imu_lin_vel",),
    require_imu=True,
)


def _sim_module() -> Any:
    scene_xml: Path | str = _FLAT_SCENE_XML
    scene_entities: list[dict[str, Any]] = []
    spawn_xy = (0.0, 0.0)

    if global_config.scene_package is not None:
        from dimos.simulation.scenes.catalog import resolve_scene_package

        package = resolve_scene_package(global_config.scene_package)
        if package is not None:
            if package.mujoco_scene_path is None:
                raise ValueError(
                    f"scene package has no MuJoCo scene artifact: {package.metadata_path}"
                )
            scene_xml = package.mujoco_scene_path
            scene_entities = package.entities
            spawn_xy = _SCENE_SPAWNS.get(package.package_dir.name, (0.0, 0.0))

    # Meshes are visual-only and byte-identical to the menagerie copies
    # bundled with mujoco_playground, so they aren't vendored in-repo.
    from mujoco_playground._src import mjx_env

    mjx_env.ensure_menagerie_exists()
    # str(): MENAGERIE_PATH is an etils GPath, which pydantic rejects.
    robot_meshdir = str(mjx_env.MENAGERIE_PATH / "unitree_go1" / "assets")

    return MujocoSimModule.blueprint(
        scene_xml=scene_xml,
        robot_mjcf=_ROBOT_MJCF,
        robot_meshdir=robot_meshdir,
        robot_id="",
        scene_entities=scene_entities,
        spawn_xy=spawn_xy,
        spawn_z=_SPAWN_Z,
        headless=True,
        dof=len(_JOINTS),
        robot_sim_spec=_sim_spec,
        reset_joint_positions=list(GO1_DEFAULT_POSITIONS),
        camera_name="head_camera",
        width=640,
        height=360,
        fps=5,
        enable_color=True,
        enable_depth=False,
        enable_pointcloud=True,
        pointcloud_fps=1.0,
        enable_mujoco_lidar=True,
        mujoco_lidar_camera_names=list(_LIDAR_CAMERAS),
        mujoco_lidar_geom_groups=list(_WORLD_GEOM_GROUPS),
        mujoco_lidar_raycast_width=64,
        mujoco_lidar_raycast_height=32,
        mujoco_lidar_robot_exclusion_radius=_WIDTH_CLEARANCE,
        enable_height_scan=True,
        height_scan_geom_groups=list(_WORLD_GEOM_GROUPS),
    )


def go2_mujoco_backend() -> Blueprint:
    """Sim module + coordinator bundle, a drop-in for ``GO2Connection``."""
    coordinator = ControlCoordinator.blueprint(
        tick_rate=50.0,  # the policy's trained rate (decimation 1)
        hardware=[
            HardwareComponent(
                hardware_id="go2",
                hardware_type=HardwareType.WHOLE_BODY,
                joints=_JOINTS,
                adapter_type="sim_mujoco_quadruped",
                address=str(_ROBOT_MJCF),
                # Gains are baked into the MJCF position actuators; these
                # document the trained contract and keep parity if a real
                # adapter (on-board PD) replaces the sim one.
                wb_config=WholeBodyConfig(kp=tuple(GO1_KP), kd=tuple(GO1_KD)),
            ),
        ],
        tasks=[
            TaskConfig(
                name="velocity_policy",
                type="quadruped_velocity",
                joint_names=_JOINTS,
                priority=50,
                auto_start=True,
                params={
                    "policy_path": str(_POLICY_ONNX),
                    "hardware_id": "go2",
                    "auto_arm": True,
                    "decimation": 1,
                },
            ),
        ],
    ).transports(
        {
            ("cmd_vel", Twist): LCMTransport("/cmd_vel", Twist),
        }
    )

    return autoconnect(_sim_module(), coordinator).remappings(
        [
            # MujocoSimModule publishes ``pointcloud``; the go2 mappers
            # consume ``lidar`` (named after GO2Connection's port).
            (VoxelGridMapper, "lidar", "pointcloud"),
            (ControlCoordinator, "twist_command", "cmd_vel"),
        ]
    )


__all__ = ["go2_mujoco_backend"]
