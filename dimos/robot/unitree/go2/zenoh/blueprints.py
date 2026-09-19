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
``GO2Connection``, no local ``PointLio``. Each layer is a superset of the one above, so a
failure can be bisected by dropping down a level:

- ``go2-zenoh-basic``: streams plus teleop; the bridge, tf and camera, no mapping.
- ``go2-zenoh-raycaster``: adds :class:`RayTracingVoxelMap`.
- ``go2-zenoh-nav``: the full stack: MLS planner and basic path follower.
- ``go2-zenoh-nav-remote``: ``go2-zenoh-nav`` with both natives dropped, for when
  a baked host on the robot publishes their outputs.
- ``go2-zenoh-motion``: ``local_planner`` + ``trajectory_follower`` replanning over the
  raycaster's local map, the follower reading the required precision off the path stamps.
- ``go2-zenoh-motion-pointlio``: ``go2-zenoh-motion`` running its own ``PointLioRust``,
  for when the MID-360 hangs off this box rather than the robot.
- ``go2-viewer``: the rerun half alone, as a zenoh client of the robot's router.
- ``go2-dds-basic``: ``go2-zenoh-basic`` with :class:`GO2DDS` in place of the bridge, for the
  Jetson (or the Go2 itself) talking DDS to the robot directly.
- ``go2-dds-motion-pointlio``: ``go2-zenoh-motion-pointlio`` over DDS, GO2DDS being the
  zenoh router the viewer dials.
"""

import os
from typing import Any

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.global_config import global_config
from dimos.hardware.sensors.lidar.pointlio.module import PointLioRust
from dimos.hardware.sensors.lidar.pointlio.pointlio_blueprints import mid360_for_pointlio
from dimos.mapping.ray_tracing.module import RayTracingVoxelMap, RayTracingVoxelMapConfig
from dimos.navigation.global_planner.mls_planner.mls_planner_native import (
    MLSPlannerNative,
    MLSPlannerNativeConfig,
)
from dimos.navigation.global_planner.mls_planner.viz import planner_visual_override
from dimos.navigation.local_planner.native import LocalPlannerNative
from dimos.navigation.local_planner.viz import motion_visual_override
from dimos.navigation.movement_manager.movement_manager import MovementManager
from dimos.navigation.trajectory_follower.basic.module import BasicPathFollower
from dimos.navigation.trajectory_follower.fancy.native import TrajectoryFollowerNative
from dimos.protocol.service.zenohservice import ZenohConfig
from dimos.robot.unitree.go2.constants import (
    BASE_LINK_HEIGHT,
    ROBOT_HEIGHT,
    ROBOT_LENGTH,
    ROBOT_WIDTH,
)
from dimos.robot.unitree.go2.dds.module import GO2DDS
from dimos.robot.unitree.go2.zenoh.zenohconnection import GO2Zenoh
from dimos.visualization.vis_module import vis_module

voxel_size = 0.08
# Raise above 0 (2.0 works) to draw what the planner searched over: surface, nodes and
# cost-colored edges. Drives both its publishing and the rerun overrides.
planner_viz_hz = 2.0

# Per-side tightening of the measured body (the body table's boxes are the swinging legs,
# not the 0.31 m trunk). Planner and follower must share it: route and room hint agree.
MOTION_BODY_DILATE_M = -0.03


def _static_robot_body(rr: Any) -> list[Any]:
    """Go2-shaped box on the body frame."""
    return [
        rr.Boxes3D(
            half_sizes=[ROBOT_LENGTH / 2, ROBOT_WIDTH / 2, ROBOT_HEIGHT / 2],
            colors=[(0, 255, 127)],
        ),
        rr.Transform3D(parent_frame="tf#/base_link"),
    ]


def _camera_info_to_pinhole(camera_info: Any) -> Any:
    """Log the pinhole onto the video's entity instead of camera_info's own.

    Entities are named after topics, so the two land on sibling paths, and a Pinhole only
    projects its own entity and its children, hence a frustum that draws but stays empty.
    No ``optical_frame``: the video's frame_id already anchors it, a second parent is
    rejected.
    """
    return camera_info.to_rerun(image_topic="world/video")


def _rerun_blueprint() -> Any:
    """Split layout: camera feed + 3D world, as the WebRTC go2 blueprint has.

    The 2D view sits on ``world/video``, not ``world/color_image``: over zenoh the camera
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


def _rerun_config(visual_override: dict[str, Any] | None = None) -> dict[str, Any]:
    """The bridge's own view, plus whatever the layer above it adds."""
    return {
        "blueprint": _rerun_blueprint,
        "tf_axes": 0.5,
        # The robot box hangs off base_link on its own entity: a static transform
        # under world/tf would override the live one.
        "static": {
            "world/robot_body": _static_robot_body,
        },
        "visual_override": {
            "world/camera_info": _camera_info_to_pinhole,
            "world/pointlio_map": _render_map,
            "world/lidar": _render_map,
            "world/local_map": _render_map,
            "world/local_map_fine": _render_map,
            "world/global_map": _render_map,
            # the local plan plus its body poses on world/path/body, coloured by the
            # stamped precision (green room, amber in the ramp, red at the floor)
            **motion_visual_override(body_dilate_m=MOTION_BODY_DILATE_M),
            **planner_visual_override(planner_viz_hz, voxel_size=voxel_size, wall_clearance_m=0.1),
            **(visual_override or {}),
        },
    }


# Streams + teleop only. cmd_vel still reaches the robot through MovementManager, so this
# is the layer to drive from when something upstream is suspect.
go2_zenoh_basic = autoconnect(
    vis_module(viewer_backend=global_config.viewer, rerun_config=_rerun_config()),
    GO2Zenoh.blueprint(),
    MovementManager.blueprint(),
).global_config(transport="zenoh", n_workers=4, robot_model="unitree_go2")

# The same layer over DDS: the native module is the robot side, so this runs on the box
# that has the Go2 on a wire. No pointlio_map: the L1 cloud arrives already in `odom`.
go2_dds_basic = autoconnect(
    vis_module(
        viewer_backend=global_config.viewer,
        rerun_config=_rerun_config({"world/pointlio_map": None}),
    ),
    GO2DDS.blueprint(),
    MovementManager.blueprint(),
).global_config(transport="zenoh", n_workers=4, robot_model="unitree_go2")

# global_map is remapped off so the planner runs purely on the
# incremental local_map + region_bounds pair.
mls_planner_config = MLSPlannerNativeConfig(
    world_frame="odom",
    voxel_size=voxel_size,
    robot_height=ROBOT_HEIGHT,
    start_z_offset_m=BASE_LINK_HEIGHT,
    surface_closing_radius=0.3,
    wall_clearance_m=0.1,
    wall_buffer_m=0.75,
    wall_buffer_weight=100.0,
    step_threshold_m=0.16,
    step_penalty_weight=4.0,
    viz_publish_hz=planner_viz_hz,
)

_mls_planner = MLSPlannerNative.blueprint(
    **mls_planner_config.model_dump(exclude_unset=True)
).remappings([(MLSPlannerNative, "global_map", "global_map_unused")])

# Consumes GO2Zenoh's lidar + odometry directly, stamped as PointLio stamps them locally
# (frames odom / mid360_link, xyz+intensity at point_step 16). Re-declared with the
# pointlio map muted; autoconnect keeps the last duplicate, so this must stay right of basic's.
_raytraced_vis = vis_module(
    viewer_backend=global_config.viewer,
    rerun_config=_rerun_config({"world/pointlio_map": None}),
)

ray_tracing_config = RayTracingVoxelMapConfig(
    voxel_size=voxel_size,
    emit_every=1,
    global_emit_every=50,
    min_health=-1,
    max_health=5,
    support_min=4,
)

# The motion stacks also publish the fine layer at the planner's own field pitch
# (voxel / 2 = FINE in local_planner/search/se2.py); the local planner reads that one.
ray_tracing_fine_config = ray_tracing_config.model_copy(
    update={"fine_divisor": 2, "emit_fine": True}
)
_local_planner_fine = LocalPlannerNative.blueprint(
    body_dilate_m=MOTION_BODY_DILATE_M, pointcloud_resolution=voxel_size / 2
).remappings([(LocalPlannerNative, "local_map", "local_map_fine")])

go2_zenoh_raycaster = autoconnect(
    go2_zenoh_basic,
    _raytraced_vis,
    RayTracingVoxelMap.blueprint(**ray_tracing_config.model_dump(exclude_unset=True)),
).global_config(transport="zenoh", n_workers=6, robot_model="unitree_go2")


go2_zenoh_nav = autoconnect(
    go2_zenoh_raycaster,
    _mls_planner,
    BasicPathFollower.blueprint(speed=0.5, heading_gain=1.5, max_angular=1.5),
    MovementManager.blueprint(),
).global_config(transport="zenoh", n_workers=8, robot_model="unitree_go2")

# What consumes the nav outputs, with nothing that produces them. Both natives run
# elsewhere: a `dimos bake` host on the robot publishes local_map, global_map and
# path onto the same zenoh session.
go2_zenoh_nav_remote = autoconnect(
    go2_zenoh_basic,
    _raytraced_vis,
    BasicPathFollower.blueprint(speed=0.5, heading_gain=1.5, max_angular=1.5),
    MovementManager.blueprint(),
).global_config(transport="zenoh", n_workers=6, robot_model="unitree_go2")

# Permissive global graph: the local planner + follower are the precision layer, so hard
# clearance drops to the 0.05 floor and the soft wall band narrows, pricing corridors.
_mls_planner_motion = MLSPlannerNative.blueprint(
    world_frame="odom",
    voxel_size=voxel_size,
    robot_height=0.4,
    surface_closing_radius=0.4,
    wall_clearance_m=0.05,
    wall_buffer_m=0.2,
    wall_buffer_weight=20.0,
    step_threshold_m=0.16,
    step_penalty_weight=4.0,
    viz_publish_hz=planner_viz_hz,
).remappings([(MLSPlannerNative, "global_map", "global_map_unused")])

# MLS stays global; its path becomes the carrot source (planner_path) for the local
# planner over the raycaster's local map. Pose is read off tf (`odom -> base_link`), not
# odometry: the mount is a lever arm. Private: no follower, so the registry must not offer it.
_go2_zenoh_motion_base = autoconnect(
    go2_zenoh_raycaster,
    # last duplicate wins: the motion stacks' raycaster also emits the fine layer
    RayTracingVoxelMap.blueprint(**ray_tracing_fine_config.model_dump(exclude_unset=True)),
    _mls_planner_motion.remappings([(MLSPlannerNative, "path", "planner_path")]),
    # body_band (default) rides the base's known height above the floor, so the map's z
    # origin is never guessed (local_planner/obstacles.py)
    _local_planner_fine,
    MovementManager.blueprint(),
)

# The follower reads no map: precision arrives in the path stamps (local_planner/profile.py).
# Speed is dialled here, e.g. embodiment=replace(GO2, max_speed=0.4), not in the law.
go2_zenoh_motion = autoconnect(
    _go2_zenoh_motion_base,
    TrajectoryFollowerNative.blueprint(),
).global_config(transport="zenoh", n_workers=9, robot_model="unitree_go2")


# `go2-zenoh-motion` with Point-LIO here: the MID-360 hangs off the Jetson, so the robot's
# onboard LIO is blind. The mount tree stays the bridge's (rooted at mid360_link); rust
# Point-LIO because the C++ SDK is LCM-only. host_ip is explicit: the Jetson has two NICs.
go2_zenoh_motion_pointlio = autoconnect(
    _go2_zenoh_motion_base,
    TrajectoryFollowerNative.blueprint(),
    # last duplicate wins: the three LIO ports go nowhere, leaving PointLio the only producer
    GO2Zenoh.blueprint().remappings(
        [
            (GO2Zenoh, "odometry", "go2_odometry_unused"),
            (GO2Zenoh, "lidar", "go2_lidar_unused"),
            (GO2Zenoh, "pointlio_map", "go2_pointlio_map_unused"),
        ]
    ),
    mid360_for_pointlio(lidar_ip="192.168.123.157", host_ip="192.168.123.5"),
    PointLioRust.blueprint(),
    # the clouds are already drawn as the raytraced map; only this stack has lidar_raw
    vis_module(
        viewer_backend=global_config.viewer,
        rerun_config=_rerun_config(
            {
                "world/pointlio_map": None,
                "world/lidar": None,
                "world/lidar_raw": None,
                "world/region_bounds": None,
            }
        ),
    ),
).global_config(
    transport="zenoh",
    # the Go2's router, on its own eth0 across the Jetson link
    zenoh_connect="tcp/192.168.123.161:7447",
    n_workers=11,
    robot_model="unitree_go2",
)


# `go2-zenoh-motion-pointlio` with GO2DDS as the robot side, no go2web bridge anywhere.
# Its native process is the zenoh router (the Go2 forwards 7447 to the Jetson, so the
# viewer still dials go22); every other process dials it on loopback. The head L1 stays
# off and Point-LIO owns odom, so GO2DDS publishes no lidar, odometry or odom tf edge; its
# body IMU moves aside for the MID-360's, which Point-LIO reads on `imu`.
_go2_dds_pointlio = GO2DDS.blueprint(
    iface="enP8p1s0",
    lidar_on=False,
    tf_root="mid360_link",
    session=ZenohConfig(mode="router", listen=["tcp/0.0.0.0:7447"], connect=[]),
).remappings(
    [
        (GO2DDS, "odometry", "go2_odometry_unused"),
        (GO2DDS, "lidar", "go2_lidar_unused"),
        (GO2DDS, "imu", "body_imu"),
    ]
)

go2_dds_motion_pointlio = autoconnect(
    vis_module(
        viewer_backend=global_config.viewer,
        rerun_config=_rerun_config(
            {
                "world/pointlio_map": None,
                "world/lidar": None,
                "world/lidar_raw": None,
                "world/region_bounds": None,
            }
        ),
    ),
    _go2_dds_pointlio,
    MovementManager.blueprint(),
    RayTracingVoxelMap.blueprint(**ray_tracing_fine_config.model_dump(exclude_unset=True)),
    _mls_planner_motion.remappings([(MLSPlannerNative, "path", "planner_path")]),
    _local_planner_fine,
    TrajectoryFollowerNative.blueprint(),
    mid360_for_pointlio(lidar_ip="192.168.123.157", host_ip="192.168.123.5"),
    PointLioRust.blueprint(),
).global_config(
    transport="zenoh",
    zenoh_connect="tcp/127.0.0.1:7447",
    # the router is a native process; the peers keep dialing until it is up
    zenoh_connect_timeout=15.0,
    n_workers=11,
    robot_model="unitree_go2",
)


# The viewer half alone, for the machine with the screen. Zenoh keeps the newest sample
# per topic, so the drop sits in front of the wifi instead of rerun's lossless stream
# replaying history. `topics` is one subscription per name: unlisted never crosses the link.
# The router is named, not scouted: behind wifi multicast scouting finds nothing
# (docs/usage/transports/zenoh.md). --robot-ip still adds its endpoint alongside.
GO2_ROUTER = os.environ.get("DIMOS_GO2_ROUTER", "tcp/go22:7447")

go2_viewer = autoconnect(
    vis_module(
        viewer_backend=global_config.viewer,
        rerun_config={
            **_rerun_config(),
            "topics": [
                "tf",
                "odometry",
                "local_map_fine",
                "path",
                "planner_path",
                "nodes",
                "node_edges",
                "surface_map",
                "goal",
                "way_point",
                "goal_reached",
                "video",
                "camera_info",
            ],
            # the map's own rate is the lidar's, more than a screen or a bad link needs
            "max_hz": {"world/local_map_fine": 4.0, "world/surface_map": 1.0},
        },
    ),
).global_config(
    transport="zenoh",
    # a client: the router forwards to clients only, never between peers
    zenoh_mode="client",
    zenoh_connect=GO2_ROUTER,
    # the robot's stack owns the bus-wide `Coordinator` name; this one only watches
    serve_coordinator_rpc=False,
    n_workers=3,
    robot_model="unitree_go2",
)
