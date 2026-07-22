#!/usr/bin/env python3
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

"""Go2 blueprints for a robot running the go2web zenoh bridge.

The ``unitree_go2_nav_3d`` stack minus the modules the robot now runs itself: no WebRTC
``GO2Connection``, no local ``PointLio``. Three layers, each a superset of the one above,
so a failure can be bisected by dropping down a level:

- ``go2-zenoh-basic`` — streams plus teleop; the bridge, tf and camera, no mapping.
- ``go2-zenoh-raycaster`` — adds :class:`RayTracingVoxelMap`.
- ``go2-zenoh-nav`` — the full stack: planner, goal relay and path follower.
- ``go2-zenoh-htc`` — ``go2-zenoh-nav`` with the follower swapped for the
  ``DanLocalPlanner`` + ``DanHolonomicTC`` pair from ``unitree-go2-mls-htc``.
- ``go2-zenoh-avoidance`` — ``go2-zenoh-htc`` with :class:`PathCorrector` in
  place of ``DanLocalPlanner``: plans are pushed off obstacles and stamped
  with a body yaw the TC then tracks.
"""

import itertools
import math
from typing import Any

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.global_config import global_config
from dimos.mapping.costmapper import CostMapper
from dimos.mapping.ray_tracing.module import RayTracingVoxelMap
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.navigation.basic_path_follower.module import BasicPathFollower
from dimos.navigation.dannav.holonomic_tc.module import DanHolonomicTC
from dimos.navigation.dannav.local_planner.module import DanLocalPlanner
from dimos.navigation.movement_manager.movement_manager import MovementManager
from dimos.navigation.nav_3d.mls_planner.goal_relay import GoalRelay
from dimos.navigation.nav_3d.mls_planner.mls_planner_native import MLSPlannerNative
from dimos.navigation.nav_3d.mls_planner.odom_body_frame import OdomBodyFrame
from dimos.navigation.nav_3d.mls_planner.viz import planner_visual_override
from dimos.navigation.obstacle_avoidance.path_correction import PathCorrector
from dimos.robot.unitree.go2.zenoh.zenohconnection import GO2Zenoh
from dimos.visualization.vis_module import vis_module

voxel_size = 0.08
# Raise above 0 (2.0 works) to draw what the planner searched over: surface, nodes and
# cost-coloured edges. Drives both its publishing and the rerun overrides.
planner_viz_hz = 2.0

# Feeds both the static tf GO2Zenoh publishes and the rotation that levels its odometry —
# they must agree or nav steers off-heading. Verified against Point-LIO's own attitude.
MID360_MOUNT_RPY_DEG = (-60.0, 0.0, -90.0)


def _mount_rotation() -> list[float]:
    """base_link <- lidar rotation, so nav reads odometry in the level body frame.

    base_link -> front_camera carries no rotation, so this is just the mount rpy above.
    """
    rpy = Vector3(*(math.radians(d) for d in MID360_MOUNT_RPY_DEG))
    return list(Quaternion.from_euler(rpy).to_tuple())


def _camera_info_to_pinhole(camera_info: Any) -> Any:
    """Log the pinhole onto the video's entity instead of camera_info's own.

    Entities are named after topics, so the two land on sibling paths — and a Pinhole only
    projects its own entity and its children, hence a frustum that draws but stays empty.
    No ``optical_frame``: the video's frame_id already anchors it, a second parent is
    rejected.
    """
    return camera_info.to_rerun(image_topic="world/video")


def _rerun_blueprint() -> Any:
    """Split layout: camera feed + 3D world, as the WebRTC go2 blueprint has.

    The 2D view sits on ``world/video``, not ``world/color_image`` — over zenoh the camera
    arrives as H.264 on the ``video`` port, which is also where the pinhole is logged.
    """
    import rerun as rr
    import rerun.blueprint as rrb

    return rrb.Blueprint(
        rrb.Horizontal(
            rrb.Spatial2DView(origin="world/video", name="Camera"),
            rrb.Spatial3DView(
                origin="world",
                name="3D",
                background=rrb.Background(kind="SolidColor", color=[0, 0, 0]),
                line_grid=rrb.LineGrid3D(plane=rr.components.Plane3D.XY.with_distance(0.5)),
                # Hidden rather than dropped: still in the entity tree, tickable in the
                # viewer.
                overrides={
                    "world/pointlio_map": rrb.EntityBehavior(visible=False),
                    "world/lidar": rrb.EntityBehavior(visible=False),
                    "world/nodes": rrb.EntityBehavior(visible=False),
                },
            ),
            column_shares=[1, 2],
        ),
        rrb.TimePanel(state="hidden"),
        rrb.SelectionPanel(state="hidden"),
    )


def _render_map(msg: Any) -> Any:
    return msg.to_rerun(voxel_size=0.01)


def _render_path(msg: Any) -> Any:
    # The planner emits an empty path when it finds no route to the goal.
    # Logging those would blank the line, so drop them and keep the last path.
    if len(msg.poses) == 0:
        return None
    return msg


_GO2_HALF_SIZES = (0.35, 0.155, 0.2)


def _sample_poses(poses: list[Any], spacing: float) -> list[Any]:
    """Every ~*spacing* meters of arc length, keeping the original poses.

    Unlike ``simple_resample_path`` this preserves each pose's stamped orientation
    instead of re-deriving it from the travel direction.
    """
    kept = [poses[0]]
    s = 0.0
    for prev, pose in itertools.pairwise(poses):
        s += math.hypot(pose.position.x - prev.position.x, pose.position.y - prev.position.y)
        if s >= spacing:
            kept.append(pose)
            s = 0.0
    if kept[-1] is not poses[-1]:
        kept.append(poses[-1])
    return kept


def _render_corrected_path(msg: Any) -> Any:
    """Corrected path as a chain of oriented go2-sized cubes.

    Shows where the body will be — position AND stamped yaw — along the path, not
    just the line. Orientations come verbatim from the corrector's waypoints.
    """
    if len(msg.poses) == 0:
        return None
    import rerun as rr

    boxes = _sample_poses(msg.poses, 0.35)
    return rr.Boxes3D(
        centers=[(p.position.x, p.position.y, p.position.z + _GO2_HALF_SIZES[2]) for p in boxes],
        half_sizes=[_GO2_HALF_SIZES] * len(boxes),
        quaternions=[
            rr.Quaternion(xyzw=[p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w])
            for p in boxes
        ],
        colors=[(255, 60, 60)] * len(boxes),
    )


def _rerun_config(visual_override: dict[str, Any] | None = None) -> dict[str, Any]:
    """The bridge's own view, plus whatever the layer above it adds."""
    return {
        "blueprint": _rerun_blueprint,
        "visual_override": {
            "world/camera_info": _camera_info_to_pinhole,
            "world/pointlio_map": _render_map,
            "world/lidar": None,
            "world/local_map": _render_map,
            "world/global_map": _render_map,
            "world/path": _render_path,
            **planner_visual_override(planner_viz_hz, voxel_size=voxel_size, wall_clearance_m=0.1),
            **(visual_override or {}),
        },
    }


# Streams + teleop only. cmd_vel still reaches the robot through MovementManager, so this
# is the layer to drive from when something upstream is suspect.
go2_zenoh_basic = autoconnect(
    vis_module(viewer_backend=global_config.viewer, rerun_config=_rerun_config()),
    GO2Zenoh.blueprint(mid360_mount_rpy_deg=MID360_MOUNT_RPY_DEG),
    MovementManager.blueprint(),
).global_config(transport="zenoh", n_workers=4, robot_model="unitree_go2")

# global_map is remapped off so the planner runs purely on the
# incremental local_map + region_bounds pair.
_mls_planner = MLSPlannerNative.blueprint(
    world_frame="odom",
    voxel_size=voxel_size,
    robot_height=0.3,
    surface_closing_radius=0.3,
    wall_clearance_m=0.1,
    wall_buffer_m=0.75,
    wall_buffer_weight=100.0,
    step_threshold_m=0.16,
    step_penalty_weight=4.0,
    viz_publish_hz=planner_viz_hz,
).remappings([(MLSPlannerNative, "global_map", "global_map_unused")])

# Consumes GO2Zenoh's lidar + odometry directly: the bridge stamps them exactly as
# PointLio does locally (frames odom / mid360_link, xyz+intensity at point_step 16).
go2_zenoh_raycaster = autoconnect(
    go2_zenoh_basic,
    # Re-declared with the pointlio map muted: the raytraced maps replace it here, and
    # autoconnect keeps the newest duplicate, so this vis module wins over basic's.
    vis_module(
        viewer_backend=global_config.viewer,
        rerun_config=_rerun_config({"world/pointlio_map": None, "world/lidar": None}),
    ),
    RayTracingVoxelMap.blueprint(
        voxel_size=voxel_size,
        emit_every=1,
        global_emit_every=50,
        min_health=-1,
        max_health=5,
        support_min=4,
    ),
).global_config(transport="zenoh", n_workers=6, robot_model="unitree_go2")


go2_zenoh_nav = autoconnect(
    go2_zenoh_raycaster,
    _mls_planner,
    OdomBodyFrame.blueprint(mount_rotation=_mount_rotation()),
    GoalRelay.blueprint(),
    BasicPathFollower.blueprint(speed=0.5, heading_gain=0.4, max_angular=0.6).remappings(
        [(BasicPathFollower, "odometry", "body_odometry")]
    ),
    MovementManager.blueprint(),
).global_config(transport="zenoh", n_workers=8, robot_model="unitree_go2")

# The nav stack with BasicPathFollower swapped for the DanLocalPlanner + DanHolonomicTC
# pair from unitree-go2-mls-htc. The raw planner stream moves to planner_path; the gate
# forwards committed paths on path, so world/planner_path is muted in rerun.
go2_zenoh_htc = autoconnect(
    go2_zenoh_raycaster,
    OdomBodyFrame.blueprint(mount_rotation=_mount_rotation()),
    _mls_planner.remappings([(MLSPlannerNative, "path", "planner_path")]),
    # Fed the leveled odometry, so its start_pose doubles as the body-frame PoseStamped
    # the Dan modules consume — mirroring mls_htc, where planner start and follower odom
    # are the same topic.
    GoalRelay.blueprint().remappings([(GoalRelay, "odometry", "body_odometry")]),
    # Setting resample_spacing_m to > 0.0 will smooth out jagged paths returned by MLSP
    DanLocalPlanner.blueprint(resample_spacing_m=0.1).remappings(
        [(DanLocalPlanner, "odom", "start_pose")]
    ),
    DanHolonomicTC.blueprint(run_profile="walk").remappings(
        [(DanHolonomicTC, "odom", "start_pose")]
    ),
    MovementManager.blueprint(),
).global_config(transport="zenoh", n_workers=9, robot_model="unitree_go2")

# htc with PathCorrector in DanLocalPlanner's seat: it does the 0.1m resampling that
# module did, then pushes waypoints off the costmap and stamps each with the body yaw
# the corrected path wants. The replan gate isn't reproduced — goal cancels still reach
# the TC through MovementManager's stop_movement.
#
#     MLSPlannerNative -> /planner_path
#         -> PathCorrector (resample + repulsion + stop 20cm before collision)
#         -> /path -> DanHolonomicTC
go2_zenoh_avoidance = autoconnect(
    go2_zenoh_raycaster,
    # The corrected path draws as oriented go2-sized cubes instead of a line, so the
    # stamped yaw is visible; the raw plan keeps the plain path renderer.
    vis_module(
        viewer_backend=global_config.viewer,
        rerun_config=_rerun_config(
            {
                "world/pointlio_map": None,
                "world/lidar": None,
                "world/planner_path": _render_path,
                "world/path": _render_corrected_path,
            }
        ),
    ),
    OdomBodyFrame.blueprint(mount_rotation=_mount_rotation()),
    _mls_planner.remappings([(MLSPlannerNative, "path", "planner_path")]),
    GoalRelay.blueprint().remappings([(GoalRelay, "odometry", "body_odometry")]),
    # Feeds the corrector off the raycaster's global map. Its own global_map input is
    # what the planner declines to use, so nothing else competes for it here.
    CostMapper.blueprint(),
    # yaw_offset=π/2 turns this into a strafe test of the TC's yaw tracking.
    PathCorrector.blueprint().remappings([(PathCorrector, "plan", "planner_path")]),
    # track_path_yaw: drive the corrector's stamped orientations, not the tangent. Gains
    # and tolerances are tightened over the defaults so the body matches the corrected
    # poses in position AND yaw — at 0.3 m/s cruise there is clamp headroom for it.
    DanHolonomicTC.blueprint(
        run_profile="walk",
        speed_m_s=0.5,
        track_path_yaw=True,
        k_position_per_s=4.0,
        k_yaw_per_s=2.5,
        goal_tolerance=0.1,
        orientation_tolerance=0.15,
        lookahead_m=0.25,
    ).remappings([(DanHolonomicTC, "odom", "start_pose")]),
    MovementManager.blueprint(),
).global_config(transport="zenoh", n_workers=10, robot_model="unitree_go2")
