#!/usr/bin/env python3
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

"""Basic Lynx M20 blueprint: front-camera video + a Rerun viewer.

``MovementManager`` muxes movement sources (``nav_cmd_vel`` / ``tele_cmd_vel`` /
``clicked_point``) into the single ``cmd_vel`` the connection consumes; wire a
teleop or nav source into the manager's inputs to drive.
"""

from typing import Any

from dimos.core.coordination.blueprints import autoconnect
from dimos.mapping.ray_tracing.module import RayTracingVoxelMap
from dimos.navigation.basic_path_follower.module import BasicPathFollower
from dimos.navigation.movement_manager.movement_manager import MovementManager
from dimos.navigation.nav_3d.mls_planner.goal_relay import GoalRelay
from dimos.navigation.nav_3d.mls_planner.mls_planner_native import MLSPlannerNative
from dimos.robot.deeprobotics.m20.connection import M20Connection
from dimos.robot.deeprobotics.m20.tf import M20TF
from dimos.visualization.rerun.bridge import RerunBridgeModule
from dimos.visualization.rerun.websocket_server import RerunWebSocketServer
from dimos.web.websocket_vis.websocket_vis_module import WebsocketVisModule


def _node_edges_on_surface(msg: Any) -> Any:
    # LineSegments3D.to_rerun() defaults to z_offset=1.7 (eye-level lift), which
    # floats the planner graph ~1.7 m above the surface. Render it flat instead.
    return msg.to_rerun(z_offset=0.0)


def m20_rerun_blueprint() -> Any:
    import rerun as rr
    import rerun.blueprint as rrb

    return rrb.Blueprint(
        rrb.Horizontal(
            rrb.Vertical(
                rrb.Spatial2DView(origin="world/color_image", name="M20 Front"),
                rrb.Spatial2DView(origin="world/color_image_rear", name="M20 Rear"),
            ),
            rrb.Spatial3DView(
                origin="world",
                name="3D",
                background=rrb.Background(kind="SolidColor", color=[0, 0, 0]),
                line_grid=rrb.LineGrid3D(
                    plane=rr.components.Plane3D.XY.with_distance(0.5),
                ),
            ),
            column_shares=[1, 2],
        ),
        rrb.TimePanel(state="hidden"),
        rrb.SelectionPanel(state="hidden"),
    )


rerun = autoconnect(
    RerunBridgeModule.blueprint(
        blueprint=m20_rerun_blueprint,
        memory_limit="4GB",
        max_hz={
            "world/color_image": 0,
            "world/color_image_rear": 0,
            "world/global_map": 1.0,
            "world/local_map": 2.0,
        },
        visual_override={
            "world/node_edges": _node_edges_on_surface,
        },
    ),
    RerunWebSocketServer.blueprint(),
    WebsocketVisModule.blueprint(),
)


voxel_size = 0.1

# Andrew's raycasting voxel mapper. The M20's SLAM already emits clouds in the
# global (map) frame on ``slam_aligned_points`` -- they are NOT sensor-frame --
# so ``registered_clouds=True`` leaves them as-is; ``slam_odom`` only supplies
# the ray origins for clearing. Outputs land on ``global_map`` / ``local_map``,
# which the rerun bridge shows under ``world/`` in the 3D view.
ray_tracer = RayTracingVoxelMap.blueprint(
    voxel_size=voxel_size,
    emit_every=2,
    global_emit_every=10,
    registered_clouds=True,
).remappings(
    [
        (RayTracingVoxelMap, "lidar", "dimos/slam_aligned_points"),
        (RayTracingVoxelMap, "odometry", "dimos/slam_odom"),
    ]
)


m20 = autoconnect(
    rerun,
    # M20TF turns the SLAM odometry into the map->base_link TF. The bridge
    # publishes it on ``slam_odom`` (not the default ``odometry``), so remap.
    M20Connection.blueprint(),
    M20TF.blueprint().remappings([(M20TF, "odometry", "slam_odom")]),
).global_config(n_workers=3)

# m20 + the raycasting global/local voxel map built from the SLAM clouds.
m20_nav = autoconnect(
    rerun,
    ray_tracer,
).global_config(n_workers=4)

# m20_nav + the 3D MLS planner stack, mirroring unitree_go2_nav_3d:
#   GoalRelay        slam_odom -> start_pose, clicked goal -> goal_pose
#   MLSPlannerNative local_map + region_bounds + start/goal -> path
#   BasicPathFollower path + slam_odom -> nav_cmd_vel
#   MovementManager  clicked_point -> goal, muxes nav_cmd_vel -> cmd_vel
# The planner runs on the incremental local_map + region_bounds pair, so
# global_map is remapped off. world_frame="map" matches the M20 SLAM frame.
m20_nav_3d = autoconnect(
    m20_nav,
    GoalRelay.blueprint().remappings([(GoalRelay, "odometry", "dimos/slam_odom")]),
    MLSPlannerNative.blueprint(
        world_frame="map",
        voxel_size=voxel_size,
        robot_height=0.6,
        wall_clearance_m=0.2,
        wall_buffer_m=0.75,
        wall_buffer_weight=100.0,
        step_threshold_m=0.25,
        step_penalty_weight=1.0,
        viz_publish_hz=1.0,
    ).remappings([(MLSPlannerNative, "global_map", "global_map_unused")]),
    BasicPathFollower.blueprint(speed=0.5, heading_gain=0.4, max_angular=0.6).remappings(
        [(BasicPathFollower, "odometry", "dimos/slam_odom")]
    ),
    MovementManager.blueprint(),
).global_config(n_workers=10)

m20_api = autoconnect(
    m20_nav,
    M20Connection.blueprint(),
    MovementManager.blueprint(),
).global_config(n_workers=3)
