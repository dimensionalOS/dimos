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

"""Alfred's vision-only navigation stack, from camera streams down to wheel commands.

Everything sensor-independent lives here: ``DimSlam`` tracking the infrared stereo
pair and fusing with wheel odometry and the IMU, its range-gated decimated depth
cloud feeding ``RayTracingVoxelMap``, MLS planning over that map, and Dan's local
planner and holonomic tracking controller driving the result. ``alfred-mls-nav``
composes this with the live ``RealSenseCamera`` and ``AlfredHighLevel`` drivers;
``alfred-replay`` composes the identical stack with a recording, so a replay
exercises exactly the code a real run does.

Depth2depth densification (the enhanced depth image) switches on when its weights are
present under ``$DIMOS_DEPTH2DEPTH_DIR`` (default ``~/.cache/dimos/depth2depth``):
``dinov2_vits14.safetensors`` and ``da2_head_vits.safetensors``.
"""

from __future__ import annotations

from functools import partial
import os
from pathlib import Path
from typing import Any

from dimos.constants import CACHE_DIR
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
    """The URDF meshes, pinned to the live base_link frame so they follow odometry."""
    factory = UrdfRobotStaticRerunFactory(urdf_path=ALFRED_URDF, root_path=ALFRED_RERUN_ROOT)
    return [
        *factory(rr),
        (ALFRED_RERUN_ROOT, rr.Transform3D(parent_frame="tf#/base_link")),
    ]


VOXEL_SIZE_METERS = 0.05
DEPTH_MAX_RANGE_METERS = 6.0
"""Beyond this the D455's stereo error grows past a voxel, so returns stop being
evidence. Its quadratic error model puts ~5 cm at 6 m for the 95 mm baseline."""

ALFRED_BODY_HEIGHT_METERS = 0.5

DEPTH2DEPTH_DIR = Path(os.environ.get("DIMOS_DEPTH2DEPTH_DIR", str(CACHE_DIR / "depth2depth")))


def _weights(filename: str) -> str:
    path = DEPTH2DEPTH_DIR / filename
    return str(path) if path.exists() else ""


vis_nav = autoconnect(
    # cuVSLAM drops out whenever the IR pair loses texture and recovers by jumping;
    # wheel odometry never drops out but its heading drifts without bound; the IMU
    # has the heading rate and no position. Offline on drive_2026-08-18_23-05-04.db,
    # wheel alone ends 2.66 m from the point-lio reference and wheel with a
    # bias-corrected gyro heading ends 1.33 m, against a 0.59 m ceiling set by
    # replaying the wheel steps along the reference's own heading.
    DimSlam.blueprint(
        # cuVSLAM's own inertial mode is implemented only on the CUDA path, so asking
        # for it here aborts the tracker with "CUDA driver version is insufficient" on
        # Alfred, which has no usable driver. The IMU goes to the fusion filter
        # instead, which is also where it belongs: the filter carries a gyro bias
        # state and cuVSLAM does not.
        cuvslam_enable_imu=False,
        # Alfred's computer has no GPU; the fork-built libcuvslam carries the CPU path.
        use_gpu=False,
        source_frames=["visual_odom", "wheel_odom"],
        # Fixed variances, not the message covariances: both sources report the drift
        # they have accumulated, which says nothing about the delta being fused.
        # Wheel yaw is dropped outright: the wheels' heading is a biased random walk
        # (~0.4 deg/s on drive_2026-08-18_23-05-04.db), and fusing it at any weight
        # drags the fused track toward that drift — replaying the drive with wheel
        # yaw trusted over visual landed 8.8 m yaw-fit rmse vs point-lio where raw
        # cuVSLAM alone fits at 1.3 m. Heading is visual deltas with the gyro in
        # between. Visual z is dropped: the CPU tracker's z drifts metres per
        # minute on this rig, and the planar twist constraint below already pins z.
        source_pose_variances=[
            *(0.01, 0.01, 0.0, 0.05, 0.05, 0.05),
            *(0.05, 0.05, 0.0, 0.0, 0.0, 0.0),
        ],
        # The CPU-built tracker reports covariance as identity plus accumulated
        # drift, so its translation std starts above 1.0 and grows past 9 during
        # normal driving: any threshold either rejects everything or nothing.
        # Off; the speed gate stays as the teleport backstop.
        covariance_gate_translation_std=0.0,
        # Only the wheels measure velocity; cuVSLAM's twist is differentiated pose.
        source_twist_variances=[*(0.0,) * 6, *(0.02, 0.02, 0.0, 0.0, 0.0, 0.05)],
        # Alfred is holonomic in the plane, so only the out-of-plane directions are
        # constrained: it cannot climb, roll or pitch.
        constraint_twist_variances=[0.0, 0.0, 0.01, 0.01, 0.01, 0.0],
        # Wheel odometry crosses the wifi link and can land seconds late; a message
        # older than the buffer is dropped instead of replayed into the filter.
        replay_buffer_seconds=2.0,
        # The mapper ignores everything past its own max_range, so gate the cloud
        # at the source and keep those points off the bus entirely.
        depth_cloud_max_range=DEPTH_MAX_RANGE_METERS,
        # A full-resolution cloud is ~400k points at 28 Hz and drowned the mapper
        # (378% CPU for ~15 Hz consumed, the rest dropped at the input queue). At
        # k=3 the sampling pitch is 3z/fx = 42 mm at the 6 m gate, still under the
        # 50 mm voxel, for 9x fewer points.
        depth_cloud_decimation=3,
        depth2depth_dinov2_weights=_weights("dinov2_vits14.safetensors"),
        depth2depth_head_weights=_weights("da2_head_vits.safetensors"),
        # Inference runs on the tracker's ingest thread, so at full resolution
        # (~60 ms/frame on an M-series GPU) a 30 Hz depth stream saturates the
        # module and starves the imu and image queues; vision then degrades from
        # the dropped frames. Half resolution keeps the thread ahead of the
        # sensor.
        depth2depth_quality=0.5,
        # Depth arrives aligned to the colour camera, which is not the rig camera;
        # the model reads its colour frames off the image stream.
        depth2depth_color_frame="camera_color_optical_frame",
    ).remappings([(DimSlam, "sources", "source_odometry")]),
    RayTracingVoxelMap.blueprint(
        voxel_size=VOXEL_SIZE_METERS,
        max_range=DEPTH_MAX_RANGE_METERS,
        world_frame="odom",
    ).remappings(
        # The tracker's range-gated depth cloud stands in for the lidar the mapper
        # normally consumes; there is none on Alfred.
        [(RayTracingVoxelMap, "lidar", "depth_cloud")]
    ),
    MLSPlannerNative.blueprint(
        # Nothing closes loops here, so map -> odom stays identity and odom is the
        # only consistent frame the voxel map and the planner share.
        world_frame="odom",
        voxel_size=VOXEL_SIZE_METERS,
        robot_height=ALFRED_BODY_HEIGHT_METERS,
        wall_clearance_m=0.2,
        wall_buffer_m=0.75,
        wall_buffer_weight=100.0,
        step_threshold_m=0.16,
        step_penalty_weight=1.0,
    ).remappings(
        [
            (MLSPlannerNative, "path", "planner_path"),
            (MLSPlannerNative, "start_pose", "odom"),
        ]
    ),
    # On Go2 the base pose comes off the robot connection. Alfred has no such module,
    # so GoalRelay's odometry-to-pose conversion is what feeds every consumer of odom.
    GoalRelay.blueprint().remappings([(GoalRelay, "start_pose", "odom")]),
    DanLocalPlanner.blueprint(resample_spacing_m=0.1),
    DanHolonomicTC.blueprint(run_profile="walk"),
    MovementManager.blueprint(),
    vis_module(
        global_config.viewer,
        rerun_config={
            "blueprint": _rerun_blueprint,
            "static": {ALFRED_RERUN_ROOT: _alfred_urdf_static},
            # Each image has to sit on the same entity as its Pinhole or rerun has
            # nothing to project it through, which is what made the camera read as
            # rotated and left the image views empty.
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
