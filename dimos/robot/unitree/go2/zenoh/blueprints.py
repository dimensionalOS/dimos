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
- ``go2-zenoh-nav-remote`` — ``go2-zenoh-nav`` with both natives dropped, for when
  a baked host on the robot publishes their outputs.
- ``go2-zenoh-nav-baked`` — ``go2-zenoh-nav`` with both natives replaced by the one
  binary ``dimos bake`` links them into.
- ``go2-zenoh-htc`` — ``go2-zenoh-nav`` with the follower swapped for the
  ``DanLocalPlanner`` + ``DanHolonomicTC`` pair from ``unitree-go2-mls-htc``.
- ``go2-zenoh-motion`` — the motion stack (``dimos/navigation/motion``): the evolved
  local planner and the trajectory follower over the raycaster's local map, on the
  ``hinted`` track.
- ``go2-zenoh-motion-blind`` — the same stack on the ``blind`` track: the follower is
  handed no clearance array and reads required precision off the path stamps. Same
  graph, so an A/B against ``go2-zenoh-motion`` isolates the law.
- ``go2-zenoh-motion-local`` — ``go2-zenoh-motion`` with planner, follower and mux
  lifted onto the robot as one ``dimos bake`` host.
"""

from typing import Any

from dimos.core.baked_host import baked_host
from dimos.core.coordination.blueprints import autoconnect
from dimos.core.global_config import global_config
from dimos.mapping.ray_tracing.module import RayTracingVoxelMap, RayTracingVoxelMapConfig
from dimos.navigation.basic_path_follower.module import BasicPathFollower
from dimos.navigation.dannav.holonomic_tc.module import DanHolonomicTC
from dimos.navigation.dannav.local_planner.module import DanLocalPlanner
from dimos.navigation.motion.adapter.follower import TrajectoryFollower
from dimos.navigation.motion.adapter.planner import MotionPlanner
from dimos.navigation.motion.adapter.viz import motion_visual_override
from dimos.navigation.movement_manager.cmd_vel_mux import CmdVelMux
from dimos.navigation.movement_manager.movement_manager import MovementManager
from dimos.navigation.nav_3d.mls_planner.goal_relay import GoalRelay
from dimos.navigation.nav_3d.mls_planner.mls_planner_native import (
    MLSPlannerNative,
    MLSPlannerNativeConfig,
)
from dimos.navigation.nav_3d.mls_planner.viz import planner_visual_override
from dimos.robot.unitree.go2.constants import ROBOT_HEIGHT, ROBOT_LENGTH, ROBOT_WIDTH
from dimos.robot.unitree.go2.zenoh.zenohconnection import GO2Zenoh
from dimos.visualization.vis_module import vis_module

voxel_size = 0.08
# Raise above 0 (2.0 works) to draw what the planner searched over: surface, nodes and
# cost-colored edges. Drives both its publishing and the rerun overrides.
planner_viz_hz = 2.0
# Draw the local plan's expected BODY POSES as oriented boxes, coloured by the
# required precision the planner stamped into the path (green = room, amber =
# inside the governor's ramp, red = at the embodiment's floor). 0.0 = off.
# Drives MotionPlanner's publishing and the rerun override together.
motion_viz_hz = 2.0

# How much tighter than the MEASURED body the local planner is allowed to plan,
# per side. The body table's boxes are the swinging legs, not the trunk (0.31 m
# on a Go2), so straight-line planning asks for a 0.516 m gap at 0.0 and 0.456 m
# here. The planner and the follower must be given the SAME value: one prices
# the route, the other prices the room hint that governs speed along it.
MOTION_BODY_DILATE_M = -0.03

# GO2Zenoh publishes this mount onto tf, where nav reads its odometry corrections.
# Either a raw (roll, pitch, yaw) tuple in degrees or a GO2ZenohConfig.mid360_mount preset.
MID360_MOUNT = "SF"


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
        "tf_axes": 0.5,
        # The robot box hangs off base_link on its own entity: a static transform
        # under world/tf would override the live one.
        "static": {
            "world/robot_body": _static_robot_body,
        },
        "visual_override": {
            "world/camera_info": _camera_info_to_pinhole,
            "world/pointlio_map": _render_map,
            "world/lidar": None,
            "world/local_map": _render_map,
            "world/global_map": _render_map,
            "world/path": _render_path,
            **planner_visual_override(planner_viz_hz, voxel_size=voxel_size, wall_clearance_m=0.1),
            # keyed by entity path, so it is inert on the stacks that have no
            # MotionPlanner to publish world/plan_body -- same as the MLS one
            **motion_visual_override(motion_viz_hz, body_dilate_m=MOTION_BODY_DILATE_M),
            **(visual_override or {}),
        },
    }


# Streams + teleop only. cmd_vel still reaches the robot through MovementManager, so this
# is the layer to drive from when something upstream is suspect.
go2_zenoh_basic = autoconnect(
    vis_module(viewer_backend=global_config.viewer, rerun_config=_rerun_config()),
    GO2Zenoh.blueprint(mid360_mount=MID360_MOUNT),
    MovementManager.blueprint(),
).global_config(transport="zenoh", n_workers=4, robot_model="unitree_go2")

# global_map is remapped off so the planner runs purely on the
# incremental local_map + region_bounds pair.
mls_planner_config = MLSPlannerNativeConfig(
    world_frame="odom",
    voxel_size=voxel_size,
    robot_height=ROBOT_HEIGHT,
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

# Consumes GO2Zenoh's lidar + odometry directly: the bridge stamps them exactly as
# PointLio does locally (frames odom / mid360_link, xyz+intensity at point_step 16).
# Re-declared with the pointlio map muted: the raytraced maps replace it wherever
# ray tracing runs. autoconnect dedupes by instance name and keeps the LAST one
# declared (`_eliminate_duplicates`, core/coordination/blueprints.py), so this vis
# module wins over basic's -- as long as it stays to the right of it in the call.
_raytraced_vis = vis_module(
    viewer_backend=global_config.viewer,
    rerun_config=_rerun_config({"world/pointlio_map": None, "world/lidar": None}),
)

ray_tracing_config = RayTracingVoxelMapConfig(
    voxel_size=voxel_size,
    emit_every=1,
    global_emit_every=50,
    min_health=-1,
    max_health=5,
    support_min=4,
)

go2_zenoh_raycaster = autoconnect(
    go2_zenoh_basic,
    _raytraced_vis,
    RayTracingVoxelMap.blueprint(**ray_tracing_config.model_dump(exclude_unset=True)),
).global_config(transport="zenoh", n_workers=6, robot_model="unitree_go2")


go2_zenoh_nav = autoconnect(
    go2_zenoh_raycaster,
    _mls_planner,
    GoalRelay.blueprint(lidar_height=ROBOT_HEIGHT),
    BasicPathFollower.blueprint(speed=0.5, heading_gain=0.4, max_angular=0.6),
    MovementManager.blueprint(),
).global_config(transport="zenoh", n_workers=8, robot_model="unitree_go2")

# What consumes the nav outputs, with nothing that produces them. Both natives run
# elsewhere: a `dimos bake` host on the robot publishes local_map, global_map and
# path onto the same zenoh session.
go2_zenoh_nav_remote = autoconnect(
    go2_zenoh_basic,
    _raytraced_vis,
    GoalRelay.blueprint(lidar_height=ROBOT_HEIGHT),
    BasicPathFollower.blueprint(speed=0.5, heading_gain=0.4, max_angular=0.6),
    MovementManager.blueprint(),
).global_config(transport="zenoh", n_workers=6, robot_model="unitree_go2")

# `go2-zenoh-nav` with both natives replaced by the single host binary `dimos bake`
# links them into. The internal local_map hop stays inside that process.
GoNav = baked_host(
    "GoNav",
    executable="dist/go2-nav",
    members={"ray_tracing": RayTracingVoxelMap, "mls_planner": MLSPlannerNative},
    remaps={("mls_planner", "global_map"): "global_map_unused"},
)

go2_zenoh_nav_baked = autoconnect(
    go2_zenoh_basic,
    _raytraced_vis,
    GoNav.blueprint(
        ray_tracing_config=ray_tracing_config,
        mls_planner_config=mls_planner_config,
    ),
    GoalRelay.blueprint(lidar_height=ROBOT_HEIGHT),
    BasicPathFollower.blueprint(speed=0.5, heading_gain=0.4, max_angular=0.6),
    MovementManager.blueprint(),
).global_config(transport="zenoh", n_workers=7, robot_model="unitree_go2")

go2_zenoh_htc = autoconnect(
    go2_zenoh_raycaster,
    _mls_planner.remappings([(MLSPlannerNative, "path", "planner_path")]),
    GoalRelay.blueprint(lidar_height=ROBOT_HEIGHT),
    DanLocalPlanner.blueprint(resample_spacing_m=0.1).remappings(
        [(DanLocalPlanner, "odom", "start_pose")]
    ),
    DanHolonomicTC.blueprint(run_profile="walk").remappings(
        [(DanHolonomicTC, "odom", "start_pose")]
    ),
    MovementManager.blueprint(),
).global_config(transport="zenoh", n_workers=9, robot_model="unitree_go2")

# The rig the motion stacks run on. It has to equal `go2_tf`'s
# `mid360_mount_rpy_deg` (robot/unitree/go2/tf/go2_tf.py) on any robot that runs
# the baked host, because the two publish the SAME tf edges: disagree and
# base_link jumps between two mounts at their combined publish rate (test_blueprints.py
# asserts the pair). `MID360_MOUNT` above stays whatever the nav/htc stacks want.
MOTION_MID360_MOUNT = "ATHENS"

# Its own MLS tuning: the local planner + follower are the precision layer (embodiment
# 0.05 floor, clearance-governed speed), so the global graph can be permissive where
# `_mls_planner` has to be the safety margin for BasicPathFollower. Hard clearance drops
# to the precision floor so tight gaps keep their node edges, and the soft wall band
# narrows/cheapens so corridors are priced, not severed.
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

# MLS stays the global planner but its path moves to planner_path and becomes a carrot
# source -- the evolved local planner replans to a point ~5 m of arc along it over the
# raycaster's local map, and the pursuit follower tracks the local plan with the
# clearance-governed speed. Both resolve the sensor odometry into base_link off tf, which
# GO2Zenoh publishes -- the mount is a rotation AND a lever arm, and a stack that skips it
# plans for a body 0.30 m ahead of the robot and 0.16 m above it.
#
# SPEEDS ARE SIM-CALIBRATED. Each law's envelope was measured against the freewalk_mcf
# policy in the matched MuJoCo env, NOT against the gait the robot actually runs:
# `hinted` asks up to 0.95 m/s commanded, and `blind` feeds its twist through a gait-slip
# inverse keyed to that same blob (~23% over-speed on a different gait). Re-probe against
# the deployed gait before trusting either at speed; until then dial the ceiling down
# here rather than in the law -- e.g. embodiment=replace(GO2, max_speed=0.4), or
# replace(GO2, control=GO2.control.model_copy(update={"k_pos": 1.5})) for its gains.
#
# Everything but the follower is shared, so it composes as a sub-blueprint the way
# go2_zenoh_raycaster does and the two tracks differ by one argument. Private (leading
# underscore) so the generated registry does not offer a headless stack as a runnable
# blueprint -- it has no follower and would plan without ever moving.
_go2_zenoh_motion_base = autoconnect(
    go2_zenoh_raycaster,
    # Re-declared on the motion rig: autoconnect keeps the LAST duplicate, so this
    # overrides basic's SF mount. Order-dependent, hence pinned in test_blueprints.py.
    GO2Zenoh.blueprint(mid360_mount=MOTION_MID360_MOUNT),
    GoalRelay.blueprint(lidar_height=ROBOT_HEIGHT),
    _mls_planner_motion.remappings([(MLSPlannerNative, "path", "planner_path")]),
    # The obstacle band rides the BODY (obstacle_model="body_band", the default): the
    # base sits a known height above the surface its feet stand on, so the cloud
    # referenced to that surface says what the planner can hit. Nothing about the map's
    # z origin -- which on a LIO stack is base height -- has to be guessed
    # (motion/obstacles.py).
    MotionPlanner.blueprint(viz_publish_hz=motion_viz_hz, body_dilate_m=MOTION_BODY_DILATE_M),
    MovementManager.blueprint(),
    # Teleop preempts nav on cmd_vel and a watchdog zeros it when the follower dies.
    # MovementManager keeps the click relay; both see tele_cmd_vel.
    CmdVelMux.blueprint(),
)

# hinted: the follower is handed the per-waypoint clearance array recomputed from the
# raycaster's local map, which on this stack is live -- so this is the honest default.
# It reads that map through the planner's obstacle model (the shared default), because
# the room hint has to be measured off the slice the plan was priced in.
go2_zenoh_motion = autoconnect(
    _go2_zenoh_motion_base,
    TrajectoryFollower.blueprint(body_dilate_m=MOTION_BODY_DILATE_M),
).global_config(transport="zenoh", n_workers=9, robot_model="unitree_go2")

# blind: the same graph with the clearance hint withheld. The law recovers the required
# precision from the path's own timestamps instead (control/profile.py), which is the
# regime that survives when the local map is stale, empty, or not the follower's to read.
go2_zenoh_motion_blind = autoconnect(
    _go2_zenoh_motion_base,
    TrajectoryFollower.blueprint(track="blind", body_dilate_m=MOTION_BODY_DILATE_M),
).global_config(transport="zenoh", n_workers=9, robot_model="unitree_go2")

# go2-zenoh-motion with the time-critical half lifted off the laptop. The three modules
# below are ABSENT here because they run on the robot as one baked host
# (`docs/platforms/quadruped/go2/motion-deployment.md`):
#
#     motion_planner -> trajectory_follower -> cmd_vel_mux
#
# So this composes from the module list rather than from go2_zenoh_raycaster, which would
# drag in the CmdVelMux that now belongs on the robot. What stays here is everything that
# is either expensive (the raycaster), global (the MLS graph), or attached to the operator
# (rerun, clicks, teleop).
#
# THIS BLUEPRINT ALONE DOES NOT DRIVE. Without the baked host running there is no planner,
# no follower and no mux, so clicks become goals and MLS plans a global path that nothing
# tracks. Bring the robot host up first:
#
#     dimos bake motion_planner trajectory_follower cmd_vel_mux go2_tf \
#         -o motion-host --builder zigbuild --target aarch64-unknown-linux-gnu.2.31
#
# Topology: go2web runs as the zenoh ROUTER on 7447 (GO2_ZENOH_MODE=router in its unit)
# and everything hangs off it as a client. The host dials it over loopback
# (DIMOS_ZENOH_MODE=client, DIMOS_ZENOH_CONNECT=tcp/127.0.0.1:7447 -- that link is where
# odometry comes from, and it keeps the 30 Hz stream off the wifi); it listens on nothing.
# The laptop dials the same router once: --robot-ip <robot>.
#
# tf is robot-local here: go2_tf is baked into the host and publishes the mount tree
# there, so the base_link <- mid360_link leg no longer depends on the laptop being up.
# Both baked modules hold their pose until that leg arrives on tf -- planning nothing
# rather than planning off-heading -- and say so with one "dropping odometry" line per
# outage.
#
# cmd_vel still crosses back to the laptop, because GO2Zenoh is what talks to the go2web
# bridge. This cut buys jitter immunity on the control loop, not fewer wire crossings.
go2_zenoh_motion_local = autoconnect(
    vis_module(
        viewer_backend=global_config.viewer,
        rerun_config=_rerun_config({"world/pointlio_map": None, "world/lidar": None}),
    ),
    # tf comes from the ROBOT here: go2_tf is baked into the host and publishes
    # the mount tree there. GO2Zenoh would publish the same three edges from the
    # laptop, and two publishers of one static tree is not redundancy -- it is
    # base_link jumping between them, at their combined rate, on whichever mount
    # each was configured with. So its tf goes nowhere and the robot's wins.
    GO2Zenoh.blueprint(mid360_mount=MOTION_MID360_MOUNT).remappings(
        [(GO2Zenoh, "tf", "tf_from_the_laptop_unused")]
    ),
    RayTracingVoxelMap.blueprint(**ray_tracing_config.model_dump(exclude_unset=True)),
    _mls_planner_motion.remappings([(MLSPlannerNative, "path", "planner_path")]),
    GoalRelay.blueprint(lidar_height=ROBOT_HEIGHT),
    MovementManager.blueprint(),
).global_config(transport="zenoh", n_workers=6, robot_model="unitree_go2")
