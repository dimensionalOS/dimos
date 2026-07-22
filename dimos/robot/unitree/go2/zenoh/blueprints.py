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
"""

import math
from typing import Any

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.global_config import global_config
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
