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

"""Unitree G1 GR00T whole-body control, mapping, and navigation.

The module graph above the hardware boundary is identical in real and simulated
runs. A simulation provider replaces the G1 and MID360 devices without replacing
PointLIO, mapping, navigation, control, or visualization.

Usage:
    dimos run unitree-g1-groot-wbc
    dimos --simulation mujoco --simulation-provider pimsim \
        --scene-package office run unitree-g1-groot-wbc
"""

from __future__ import annotations

from pathlib import Path
from typing import Any, cast

from dimos.control.components import HardwareComponent, HardwareType
from dimos.control.coordinator import ControlCoordinator, TaskConfig
from dimos.control.tasks.g1_groot_wbc_task.g1_groot_wbc_task import (
    ARM_DEFAULT_POSE,
    G1_GROOT_KD,
    G1_GROOT_KP,
    g1_arms,
    g1_joints,
    g1_legs_waist,
)
from dimos.core.coordination.blueprints import autoconnect
from dimos.core.global_config import global_config
from dimos.core.transport import LCMTransport
from dimos.hardware.sensors.lidar.pointlio.module import PointLio
from dimos.hardware.whole_body.spec import WholeBodyConfig
from dimos.mapping.costmapper import CostMapper
from dimos.mapping.pointclouds.occupancy import HeightCostConfig
from dimos.mapping.ray_tracing.module import RayTracingVoxelMap
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.nav_msgs.Path import Path as NavPath
from dimos.msgs.sensor_msgs.Imu import Imu
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.sensor_msgs.MotorCommandArray import MotorCommandArray
from dimos.navigation.movement_manager.movement_manager import MovementManager
from dimos.navigation.replanning_a_star.module import ReplanningAStarPlanner
from dimos.robot.unitree.g1.blueprints.basic.groot_wbc_platform import (
    resolve_g1_groot_platform,
)
from dimos.robot.unitree.g1.config import G1
from dimos.robot.unitree.g1.g1_rerun import (
    g1_costmap,
    g1_urdf_joint_state,
    g1_urdf_static_robot,
)
from dimos.utils.data import LfsPath
from dimos.visualization.rerun.scene_package import scene_package_static_entities
from dimos.visualization.vis_module import vis_module

_GROOT_MODEL_DIR = LfsPath("groot")
_NAV_VOXEL_RESOLUTION = 0.08
_NAV_OVERHEAD_SAFETY_MARGIN = 0.2
_NAV_MAX_STEP_HEIGHT = 0.10
_NAV_ROTATION_DIAMETER = 0.8
_NAV_SAFE_RADIUS_MARGIN = 0.6
_RERUN_ROOT = "world/odometry/g1"
_URDF_PATH = Path(__file__).resolve().parents[2] / "g1.urdf"
_NOMINAL_PELVIS_Z = 0.74
_pelvis_mid360_cache: list[Any] = []

assert G1.height_clearance is not None and G1.width_clearance is not None

_platform = resolve_g1_groot_platform()

_navigation = autoconnect(
    PointLio.blueprint(**_platform.pointlio_config),
    RayTracingVoxelMap.blueprint(
        voxel_size=_NAV_VOXEL_RESOLUTION,
        emit_every=0,
        global_emit_every=4,
        max_health=10,
        graze_cos=0.85,
    ),
    CostMapper.blueprint(
        config=HeightCostConfig(
            resolution=_NAV_VOXEL_RESOLUTION,
            can_pass_under=G1.height_clearance + _NAV_OVERHEAD_SAFETY_MARGIN,
            can_climb=_NAV_MAX_STEP_HEIGHT,
        ),
        initial_safe_radius_meters=G1.width_clearance + _NAV_SAFE_RADIUS_MARGIN,
    ),
    ReplanningAStarPlanner.blueprint(
        robot_width=G1.width_clearance,
        robot_rotation_diameter=_NAV_ROTATION_DIAMETER,
    ),
    MovementManager.blueprint(),
)


def _rerun_blueprint() -> Any:
    import rerun as rr
    import rerun.blueprint as rrb

    return rrb.Blueprint(
        rrb.Spatial3DView(
            origin="world",
            name="G1 GR00T WBC",
            background=rrb.Background(kind="SolidColor", color=[0, 0, 0]),
            line_grid=rrb.LineGrid3D(
                plane=rr.components.Plane3D.XY.with_distance(0.0),
            ),
        ),
        rrb.TimePanel(state="collapsed"),
    )


def _nav_path(path: NavPath) -> Any:
    return path.to_rerun(z_offset=0.3)


def _pelvis_to_mid360() -> Any:
    if not _pelvis_mid360_cache:
        from importlib import import_module

        import numpy as np

        urdf = import_module("yourdfpy").URDF.load(str(_URDF_PATH), load_meshes=False)
        urdf.update_cfg(np.zeros(len(urdf.actuated_joint_names)))
        _pelvis_mid360_cache.append(urdf.get_transform("mid360_link", "pelvis"))
    return _pelvis_mid360_cache[0]


def _odometry_root(odometry: Any) -> Any:
    import numpy as np
    import rerun as rr

    from dimos.msgs.geometry_msgs.Quaternion import Quaternion

    world_mid360 = np.eye(4)
    world_mid360[:3, :3] = odometry.orientation.to_rotation_matrix() @ np.diag([1.0, -1.0, -1.0])
    world_mid360[:3, 3] = (odometry.x, odometry.y, odometry.z)
    world_pelvis = world_mid360 @ np.linalg.inv(_pelvis_to_mid360())
    quaternion = Quaternion.from_rotation_matrix(world_pelvis[:3, :3])
    return rr.Transform3D(
        translation=world_pelvis[:3, 3].tolist(),
        rotation=rr.Quaternion(xyzw=[quaternion.x, quaternion.y, quaternion.z, quaternion.w]),
    )


def _ground_z() -> float:
    return -(float(_pelvis_to_mid360()[2, 3]) + _NOMINAL_PELVIS_Z)


def _costmap(grid: Any) -> Any:
    return g1_costmap(grid, z_offset=_ground_z() + 0.02)


def _scene_root(rr: Any) -> Any:
    return rr.Transform3D(translation=[0.0, 0.0, _ground_z()])


_static_entities: dict[str, Any] = {
    _RERUN_ROOT: g1_urdf_static_robot(root_path=_RERUN_ROOT),
}
_scene_entities = scene_package_static_entities(
    global_config.scene_package,
    entity_path="world/scene/visual",
)
if _scene_entities:
    _static_entities["world/scene"] = _scene_root
    _static_entities.update(_scene_entities)

_rerun_config: dict[str, Any] = {
    "blueprint": _rerun_blueprint,
    "visual_override": {
        "world/color_image": None,
        "world/camera_info": None,
        "world/depth_image": None,
        "world/depth_camera_info": None,
        "world/lidar": None,
        "world/coordinator_joint_state": g1_urdf_joint_state(root_path=_RERUN_ROOT),
        "world/odometry": _odometry_root,
        "world/global_costmap": _costmap,
        "world/navigation_costmap": _costmap,
        "world/path": _nav_path,
    },
    "max_hz": {
        "world/coordinator_joint_state": 20.0,
        "world/g1/imu": 10.0,
        "world/g1/motor_states": 10.0,
        "world/g1/motor_command": 10.0,
        "world/odometry": 15.0,
        "world/global_map": 1.0,
        "world/global_costmap": 2.0,
        "world/navigation_costmap": 2.0,
        "world/path": 0,
    },
    "static": _static_entities,
}


def _viewer() -> Any:
    return vis_module(viewer_backend=global_config.viewer, rerun_config=_rerun_config)


_coordinator = ControlCoordinator.blueprint(
    tick_rate=_platform.tick_rate,
    hardware=[
        HardwareComponent(
            hardware_id="g1",
            hardware_type=HardwareType.WHOLE_BODY,
            joints=g1_joints,
            adapter_type=_platform.adapter_type,
            address=_platform.adapter_address,
            wb_config=WholeBodyConfig(kp=tuple(G1_GROOT_KP), kd=tuple(G1_GROOT_KD)),
        ),
    ],
    tasks=[
        TaskConfig(
            name="groot_wbc",
            type="g1_groot_wbc",
            joint_names=g1_legs_waist,
            priority=50,
            auto_start=True,
            params={
                "model_path": _GROOT_MODEL_DIR,
                "hardware_id": "g1",
                "auto_arm": _platform.auto_arm,
                "auto_dry_run": _platform.auto_dry_run,
                "default_ramp_seconds": _platform.ramp_seconds,
                "decimation": _platform.policy_decimation,
            },
        ),
        TaskConfig(
            name="servo_arms",
            type="servo",
            joint_names=g1_arms,
            priority=10,
            auto_start=True,
            params={"default_positions": ARM_DEFAULT_POSE},
        ),
    ],
).transports(
    {
        ("joint_command", JointState): LCMTransport("/g1/joint_command", JointState),
        ("cmd_vel", Twist): LCMTransport("/g1/cmd_vel", Twist),
        ("motor_states", JointState): LCMTransport("/g1/motor_states", JointState),
        ("imu", Imu): LCMTransport("/g1/imu", Imu),
        ("motor_command", MotorCommandArray): LCMTransport(
            "/g1/motor_command",
            MotorCommandArray,
        ),
    }
)

unitree_g1_groot_wbc = (
    autoconnect(_platform.backend, _coordinator, _navigation, _viewer())
    .remappings(cast("Any", [(ControlCoordinator, "twist_command", "cmd_vel")]))
    .global_config(robot_model="unitree_g1", n_workers=_platform.n_workers)
)
