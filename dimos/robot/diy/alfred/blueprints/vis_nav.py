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

"""Alfred's vision-only navigation stack, from camera streams down to wheel commands."""

from __future__ import annotations

from functools import partial
from pathlib import Path
from typing import Any

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.global_config import global_config
from dimos.mapping.dim_slam.dim_slam import DimSlam
from dimos.mapping.ray_tracing.module import RayTracingVoxelMap
from dimos.navigation.dannav.holonomic_tc.module import DanHolonomicTC
from dimos.navigation.dannav.local_planner.module import DanLocalPlanner
from dimos.navigation.movement_manager.movement_manager import MovementManager
from dimos.navigation.nav_3d.mls_planner.goal_relay import GoalRelay
from dimos.navigation.nav_3d.mls_planner.mls_planner_native import MLSPlannerNative
from dimos.visualization.rerun.urdf_robot import UrdfRobotStaticRerunFactory
from dimos.visualization.vis_module import vis_module

ALFRED_URDF = Path(__file__).resolve().parent.parent / "alfred.urdf"
ALFRED_RERUN_ROOT = "world/alfred"
CAMERA_RERUN_ROOT = "world/camera"

IR_ENTITY_BY_FRAME = {
    "camera_infra1_optical_frame": f"{CAMERA_RERUN_ROOT}/infra1",
    "camera_infra2_optical_frame": f"{CAMERA_RERUN_ROOT}/infra2",
}
"""Both imagers arrive on one topic, so the entity has to come from the message."""


def _image_at(msg: Any, entity_path: str) -> list[tuple[str, Any]]:
    return [(entity_path, msg.to_rerun())]


def _pinhole_at(msg: Any, entity_path: str) -> Any:
    return msg.to_rerun(image_topic=entity_path, optical_frame=msg.frame_id)


def _ir_image(msg: Any) -> list[tuple[str, Any]] | None:
    entity_path = IR_ENTITY_BY_FRAME.get(msg.frame_id)
    return _image_at(msg, entity_path) if entity_path else None


def _ir_pinhole(msg: Any) -> Any:
    entity_path = IR_ENTITY_BY_FRAME.get(msg.frame_id)
    return _pinhole_at(msg, entity_path) if entity_path else None


def _rerun_blueprint() -> Any:
    """The stock bridge blueprint is 3D only, so no image view exists to render into."""
    import rerun as rr
    import rerun.blueprint as rrb

    return rrb.Blueprint(
        rrb.Horizontal(
            rrb.Spatial3DView(
                origin="world",
                background=rrb.Background(kind="SolidColor", color=[0, 0, 0]),
                line_grid=rrb.LineGrid3D(plane=rr.components.Plane3D.XY.with_distance(0.5)),
            ),
            rrb.Vertical(
                rrb.Spatial2DView(origin=f"{CAMERA_RERUN_ROOT}/color", name="color"),
                rrb.Spatial2DView(origin=f"{CAMERA_RERUN_ROOT}/depth", name="depth"),
                rrb.Spatial2DView(origin=f"{CAMERA_RERUN_ROOT}/infra1", name="infra1"),
                rrb.Spatial2DView(origin=f"{CAMERA_RERUN_ROOT}/infra2", name="infra2"),
            ),
            column_shares=[3, 1],
        ),
        collapse_panels=True,
    )


def _alfred_urdf_static(rr: Any) -> list[tuple[str, Any]]:
    """Pinned to the live base_link so the meshes follow odometry."""
    factory = UrdfRobotStaticRerunFactory(urdf_path=ALFRED_URDF, root_path=ALFRED_RERUN_ROOT)
    return [
        *factory(rr),
        (ALFRED_RERUN_ROOT, rr.Transform3D(parent_frame="tf#/base_link")),
    ]


VOXEL_SIZE_METERS = 0.05
DEPTH_MAX_RANGE_METERS = 4.0
"""Stereo error grows as range squared; 4 m won the mapping grid against 6 m and against
depth2depth-densified clouds (top-down F1 .570 / .506 / .411 vs a lidar-raycast
reference on drive_2026-08-18_23-05-04.db)."""

ALFRED_BODY_HEIGHT_METERS = 0.5

vis_nav = autoconnect(
    # drive_2026-08-18_23-05-04.db vs lidar: wheel 2.66 m, wheel + gyro 1.33 m, floor 0.59 m.
    DimSlam.blueprint(
        # Alfred's computer has no GPU; the fork-built libcuvslam carries the CPU path.
        use_gpu=False,
        depth_units_per_meter=1000.0,
        depth_cloud_max_range=DEPTH_MAX_RANGE_METERS,
        # A full-resolution D455 cloud is ~400k points a frame at 30 Hz and drowns the mapper.
        depth_cloud_decimation=3,
        source_frames=["visual_odom", "wheel_odom"],
        # Fixed variances: the message covariances report accumulated drift, not the delta
        # fused. Wheel yaw is dropped, and visual z, which the planar constraint below pins.
        source_pose_variances=[
            *(0.01, 0.01, 0.0, 0.05, 0.05, 0.05),
            *(0.05, 0.05, 0.0, 0.0, 0.0, 0.0),
        ],
        # The CPU tracker's reported translation std starts above 1.0 and grows past 9
        # while driving normally, so no threshold separates good frames from bad.
        covariance_gate_translation_std=0.0,
        # Only the wheels measure velocity; cuVSLAM's twist is differentiated pose.
        source_twist_variances=[*(0.0,) * 6, *(0.02, 0.02, 0.0, 0.0, 0.0, 0.05)],
        # Alfred is holonomic in the plane; only out-of-plane directions are constrained.
        constraint_twist_variances=[0.0, 0.0, 0.01, 0.01, 0.01, 0.0],
        # Wheel odometry crosses the wifi link and can land seconds late.
        replay_buffer_seconds=2.0,
        # Bosch BMI055 datasheet figures, the part in the D455.
        imu_gyro_noise_density=0.0018,
        imu_gyro_random_walk=2e-5,
        imu_accel_noise_density=0.02,
        imu_accel_random_walk=3e-3,
    ).remappings([(DimSlam, "sources", "source_odometry")]),
    RayTracingVoxelMap.blueprint(
        voxel_size=VOXEL_SIZE_METERS,
        max_range=DEPTH_MAX_RANGE_METERS,
    ).remappings(
        # Alfred has no lidar.
        [(RayTracingVoxelMap, "lidar", "depth_cloud")]
    ),
    MLSPlannerNative.blueprint(
        world_frame="odom",
        voxel_size=VOXEL_SIZE_METERS,
        robot_height=ALFRED_BODY_HEIGHT_METERS,
        wall_clearance_m=0.2,
        step_penalty_weight=1.0,
    ).remappings(
        [
            (MLSPlannerNative, "path", "planner_path"),
            (MLSPlannerNative, "start_pose", "odom"),
        ]
    ),
    # Nothing else converts odometry into the PoseStamped every odom consumer wants.
    GoalRelay.blueprint().remappings([(GoalRelay, "start_pose", "odom")]),
    DanLocalPlanner.blueprint(resample_spacing_m=0.1),
    DanHolonomicTC.blueprint(),
    MovementManager.blueprint(),
    vis_module(
        global_config.viewer,
        rerun_config={
            "blueprint": _rerun_blueprint,
            "static": {ALFRED_RERUN_ROOT: _alfred_urdf_static},
            # An image only renders if it shares an entity with its Pinhole.
            "visual_override": {
                "world/color_image": partial(_image_at, entity_path=f"{CAMERA_RERUN_ROOT}/color"),
                "world/color_camera_info": partial(
                    _pinhole_at, entity_path=f"{CAMERA_RERUN_ROOT}/color"
                ),
                "world/depth_image": partial(_image_at, entity_path=f"{CAMERA_RERUN_ROOT}/depth"),
                "world/depth_camera_info": partial(
                    _pinhole_at, entity_path=f"{CAMERA_RERUN_ROOT}/depth"
                ),
                "world/image": _ir_image,
                "world/camera_info": _ir_pinhole,
            },
        },
    ),
)
