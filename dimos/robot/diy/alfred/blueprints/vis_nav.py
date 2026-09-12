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

"""Alfred's vision-only navigation fragment: odometry through to wheel commands.

Carries no image or odometry source, so it is composed into a blueprint that supplies
one rather than run on its own.
"""

from __future__ import annotations

from collections.abc import Sequence
from functools import partial
from typing import Any

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.global_config import global_config
from dimos.mapping.dim_slam.dim_slam import (
    CameraConfig,
    Covariance,
    DimSlam,
    ImuConfig,
    SourceConfig,
)
from dimos.mapping.ray_tracing.module import RayTracingVoxelMap
from dimos.navigation.dannav.holonomic_tc.module import DanHolonomicTC
from dimos.navigation.dannav.local_planner.module import DanLocalPlanner
from dimos.navigation.movement_manager.movement_manager import MovementManager
from dimos.navigation.nav_3d.mls_planner.mls_planner_native import MLSPlannerNative
from dimos.navigation.nav_3d.mls_planner.start_relay import StartRelay
from dimos.robot.diy.alfred.base_arbiter import alfred_base_arbiter_blueprint
from dimos.robot.diy.alfred.config import ALFRED, ALFRED_URDF
from dimos.visualization.rerun.urdf_robot import UrdfRobotStaticRerunFactory
from dimos.visualization.vis_module import vis_module

ALFRED_RERUN_ROOT = "world/alfred"
CAMERA_RERUN_ROOT = "world/camera"

IR_ENTITY_BY_FRAME = {
    "d455_infra1_optical_frame": f"{CAMERA_RERUN_ROOT}/infra1",
    "d455_infra2_optical_frame": f"{CAMERA_RERUN_ROOT}/infra2",
}
"""The IR pair, left first. Both imagers arrive on one topic, so the entity has to come
from the message, and cuVSLAM's rig has to be told the order."""

DEPTH_FRAME = "d455_depth_optical_frame"
"""Colour is off, so depth is unaligned and arrives in its own frame."""


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


def _path_colored(msg: Any, color: tuple[int, int, int]) -> Any:
    return msg.to_rerun(color=color)


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
                rrb.Spatial2DView(origin=f"{CAMERA_RERUN_ROOT}/depth", name="depth"),
                rrb.Spatial2DView(origin=f"{CAMERA_RERUN_ROOT}/infra1", name="infra1"),
                rrb.Spatial2DView(origin=f"{CAMERA_RERUN_ROOT}/infra2", name="infra2"),
            ),
            column_shares=[3, 1],
        ),
        collapse_panels=True,
    )


def _alfred_urdf_static(rr: Any) -> list[tuple[str, Any]]:
    factory = UrdfRobotStaticRerunFactory(urdf_path=ALFRED_URDF, root_path=ALFRED_RERUN_ROOT)
    return [
        *factory(rr),
        (ALFRED_RERUN_ROOT, rr.Transform3D(parent_frame="tf#/base_link")),
    ]


VOXEL_SIZE_METERS = 0.05
DEPTH_MAX_RANGE_METERS = 4.0
"""4 m won the mapping grid against 6 m (top-down F1 .570 vs .506 against a
lidar-raycast reference on drive_2026-08-18_23-05-04.db)."""


def vis_nav(
    *,
    imus: Sequence[ImuConfig] = (),
    map_cloud_topic: str = "depth_cloud",
    map_max_range: float = DEPTH_MAX_RANGE_METERS,
) -> Any:
    """Odometry through to wheel commands. `map_cloud_topic` is what the voxel map raytraces,
    so it also decides which sensor's origin the free space is carved from."""
    return autoconnect(
        DimSlam.blueprint(
            camera_mode="stereo",
            cameras=[
                *(CameraConfig(frame_id=frame) for frame in IR_ENTITY_BY_FRAME),
                CameraConfig(
                    frame_id=DEPTH_FRAME,
                    depth_cloud_max_range=DEPTH_MAX_RANGE_METERS,
                    # A full-resolution D455 cloud is ~400k points a frame at 30 Hz and drowns
                    # the mapper: voxel_ray_tracing handles one cloud at a time and its cost is
                    # linear in point count, so at 3 it sheds most of what it is sent.
                    depth_cloud_decimation=5,
                ),
            ],
            # Fixed variances: the message covariances report accumulated drift, not the delta
            # fused. Visual z is dropped; the wheels report z=roll=pitch=0 always, so those
            # are kept as absolute anchors - the zero-twist constraint below only damps the
            # velocities and let all three random-walk (a second floor in the map, then a
            # visibly rolled robot). With no IMU there is no other gravity reference.
            visual_odom_pose_variances=Covariance(x=0.01, y=0.01, roll=0.05, pitch=0.05, yaw=0.05),
            # Only the wheels measure velocity; the tracker publishes no twist at all.
            odom_sources=[
                SourceConfig(
                    parent_frame_id="wheel_odom",
                    child_frame_id="base_link",
                    pose_variances=Covariance(x=0.05, y=0.05, z=0.001, roll=0.001, pitch=0.001),
                    twist_variances=Covariance(x=0.02, y=0.02, yaw=0.05),
                ),
            ],
            # The CPU tracker's reported translation std starts above 1.0 and grows past 9
            # while driving normally, so no threshold separates good frames from bad.
            covariance_gate_translation_std=0.0,
            # Alfred is holonomic in the plane.
            per_dimension_error_variance=Covariance(z=0.01, roll=0.01, pitch=0.01),
            # Wheel odometry crosses the wifi link and can land seconds late.
            replay_buffer_seconds=2.0,
            # Every IMU publishes on the one imu topic; the filter sorts them by frame_id and
            # keeps a bias pair per entry, so an unlisted frame is dropped rather than fused.
            imus=list(imus),
        ),
        RayTracingVoxelMap.blueprint(
            voxel_size=VOXEL_SIZE_METERS,
            max_range=map_max_range,
        ).remappings([(RayTracingVoxelMap, "lidar", map_cloud_topic)]),
        MLSPlannerNative.blueprint(
            voxel_size=VOXEL_SIZE_METERS,
            robot_height=ALFRED.body_height,
            wall_clearance_m=0.2,
            step_penalty_weight=1.0,
        ).remappings([(MLSPlannerNative, "path", "planner_path")]),
        # Solely the tf-driven start_pose source for the dannav odom remaps below.
        StartRelay.blueprint(),
        DanLocalPlanner.blueprint(resample_spacing_m=0.1).remappings(
            [(DanLocalPlanner, "odom", "start_pose")]
        ),
        DanHolonomicTC.blueprint().remappings([(DanHolonomicTC, "odom", "start_pose")]),
        # Kept for the click->goal relay and the cancel it fires on teleop. Its own
        # teleop/nav mux is superseded by the arbiter, so its cmd_vel goes nowhere.
        MovementManager.blueprint().remappings(
            [(MovementManager, "cmd_vel", "_movement_manager_cmd_vel_unused")]
        ),
        alfred_base_arbiter_blueprint(),
        vis_module(
            global_config.viewer,
            rerun_config={
                "blueprint": _rerun_blueprint,
                "static": {ALFRED_RERUN_ROOT: _alfred_urdf_static},
                # Keyed by the topic's entity path, before any visual_override renames it.
                # Everything is held to 1 Hz: the viewer rides a wifi link, and the
                # uncapped clouds put it seconds behind live. tf is capped too, so the
                # whole scene ticks once a second rather than re-posing per odom update.
                "max_hz": {
                    "world/tf": 1.0,
                    "world/depth_image": 1.0,
                    "world/image": 1.0,
                    "world/depth_cloud": 1.0,
                    "world/lidar": 1.0,
                    "world/global_map": 1.0,
                    "world/local_map": 1.0,
                    "world/surface_map": 1.0,
                    "world/nodes": 1.0,
                    "world/node_edges": 1.0,
                },
                # An image only renders if it shares an entity with its Pinhole.
                "visual_override": {
                    # The MLS planner path; the local planner's path keeps the stock green.
                    "world/planner_path": partial(_path_colored, color=(170, 60, 220)),
                    "world/depth_image": partial(
                        _image_at, entity_path=f"{CAMERA_RERUN_ROOT}/depth"
                    ),
                    "world/depth_camera_info": partial(
                        _pinhole_at, entity_path=f"{CAMERA_RERUN_ROOT}/depth"
                    ),
                    "world/image": _ir_image,
                    "world/camera_info": _ir_pinhole,
                },
            },
        ),
    )
