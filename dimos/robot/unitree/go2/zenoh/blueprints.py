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
- ``go2-zenoh-motion`` — the motion stack: the evolved autoresearch planner and
  the trajectory follower over the raycaster's local map, on the ``hinted`` track.
- ``go2-zenoh-motion-blind`` — the same stack on the ``blind`` track: the follower
  is handed no clearance array and reads required precision off the path stamps.
  Same graph, so an A/B against ``go2-zenoh-motion`` isolates the law.
"""

from typing import Any

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.global_config import global_config
from dimos.mapping.ray_tracing.module import RayTracingVoxelMap
from dimos.navigation.basic_path_follower.module import BasicPathFollower
from dimos.navigation.motion.adapter.follower import TrajectoryFollower
from dimos.navigation.motion.adapter.planner import MotionPlanner
from dimos.navigation.motion.adapter.viz import motion_visual_override
from dimos.navigation.movement_manager.cmd_vel_mux import CmdVelMux
from dimos.navigation.movement_manager.movement_manager import MovementManager
from dimos.navigation.nav_3d.mls_planner.goal_relay import GoalRelay
from dimos.navigation.nav_3d.mls_planner.mls_planner_native import MLSPlannerNative
from dimos.navigation.nav_3d.mls_planner.viz import planner_visual_override
from dimos.robot.unitree.go2.constants import ROBOT_HEIGHT
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

# GO2Zenoh publishes this mount onto tf, where nav reads its odometry corrections.
MID360_MOUNT_RPY_DEG = (-60.0, 0.0, -90.0)


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
            # keyed by entity path, so it is inert on the stacks that have no
            # MotionPlanner to publish world/plan_body -- same as the MLS one
            **motion_visual_override(motion_viz_hz),
            **(visual_override or {}),
        },
    }


# Streams + teleop only. cmd_vel still reaches the robot through CmdVelMux, so this
# is the layer to drive from when something upstream is suspect.
go2_zenoh_basic = autoconnect(
    vis_module(viewer_backend=global_config.viewer, rerun_config=_rerun_config()),
    GO2Zenoh.blueprint(mid360_mount_rpy_deg=MID360_MOUNT_RPY_DEG),
    MovementManager.blueprint(),
    CmdVelMux.blueprint(),
).global_config(transport="zenoh", n_workers=4, robot_model="unitree_go2")

# global_map is remapped off so the planner runs purely on the
# incremental local_map + region_bounds pair.
_mls_planner = MLSPlannerNative.blueprint(
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
        emit_every=10,
        global_emit_every=100,
        min_health=-1,
        max_health=5,
        support_min=4,
    ),
).global_config(transport="zenoh", n_workers=6, robot_model="unitree_go2")


go2_zenoh_nav = autoconnect(
    go2_zenoh_raycaster,
    _mls_planner,
    GoalRelay.blueprint(lidar_height=ROBOT_HEIGHT),
    BasicPathFollower.blueprint(speed=0.5, heading_gain=0.4, max_angular=0.6),
    MovementManager.blueprint(),
    CmdVelMux.blueprint(),
).global_config(transport="zenoh", n_workers=8, robot_model="unitree_go2")

# The motion stack's own raycaster tuning. The percentile-sized emit window BREATHES:
# it follows what the last ten sweeps happened to see, so on 20260805-033007 its radius
# swung 2.53 -> 5.88 m, up to 2.36 m in a single frame, and 68 % of all voxel churn the
# local planner saw was that window moving rather than the world changing. Every
# collapse deletes thousands of voxels the planner was routing around; every expansion
# invents them back, and the plan flips. A fixed radius makes the emitted set stable
# whenever the scene is. 5 m covers the 5 m carrot with room for the search's padding,
# and is inside the map the sweeps actually fill.
_motion_raycaster = dict(
    voxel_size=voxel_size,
    emit_every=10,
    global_emit_every=100,
    min_health=-1,
    max_health=5,
    support_min=4,
    region_radius_m=5.0,
)

# The motion stack's own MLS tuning: the local planner + follower are the precision
# layer (embodiment 0.05 floor, clearance-governed speed), so the global graph can be
# permissive where _mls_planner has to be the safety margin for BasicPathFollower.
# Hard clearance drops to the precision floor so tight gaps keep their node edges, and
# the soft wall band narrows/cheapens so corridors are priced, not severed.
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

# The motion stack (dimos/navigation/motion): MLS stays the global planner but its path
# moves to planner_path and becomes a carrot source — the evolved autoresearch planner
# replans to a point ~5 m of arc along it over the raycaster's local map, and the pursuit
# follower tracks the local plan with the clearance-governed speed. Both resolve the
# sensor odometry into base_link off tf, which GO2Zenoh publishes -- the mount is a
# rotation AND a lever arm, and a stack that skips it plans for a body 0.30 m ahead of
# the robot and 0.16 m above it.
#
# The follower's TRACK picks its law and what it is handed (control/tracks.py). Both are
# wired so the two can be A/B'd on the robot; the graph is otherwise identical, so a
# difference between them is the law and nothing else.
#
# SPEEDS ARE SIM-CALIBRATED. Each law's envelope was measured against the freewalk_mcf
# policy in the matched MuJoCo env, NOT against the gait the robot actually runs:
# `hinted` asks up to 0.95 m/s commanded, and `blind` feeds its twist through a gait-slip
# inverse keyed to that same blob (~23% over-speed on a different gait). Re-measure with
# `python -m dimos.navigation.motion.control.referee.probe_walk_slip` against the deployed gait
# and re-key before trusting either at speed. Until then, dial the ceiling down here
# rather than in the law -- e.g. controller_config=ControllerConfig(max_speed=0.5).
#
# Everything but the follower is shared, so it composes as a sub-blueprint the way
# go2_zenoh_raycaster does above and the two tracks differ by one argument. Private
# (leading underscore) so the generated registry does not offer a headless stack as a
# runnable blueprint -- it has no follower and would plan without ever moving.
_go2_zenoh_motion_base = autoconnect(
    go2_zenoh_raycaster,
    # Re-declared with the emitted window PINNED, and autoconnect keeps the
    # newest duplicate, so this raycaster wins over go2_zenoh_raycaster's.
    RayTracingVoxelMap.blueprint(**_motion_raycaster),
    GoalRelay.blueprint(lidar_height=ROBOT_HEIGHT),
    _mls_planner_motion.remappings([(MLSPlannerNative, "path", "planner_path")]),
    # lidar_height is what lets tf say how far the base sits above the ground,
    # and the local planner needs that: its body z-band is 0.05..0.45 ABOVE THE
    # FLOOR, while the map's z origin is base height. Without it the band reads
    # a slab over the robot's head-room -- blind to the bottom of every
    # obstacle, steering off table tops (adapter/floor.py).
    MotionPlanner.blueprint(viz_publish_hz=motion_viz_hz, lidar_height=ROBOT_HEIGHT),
    MovementManager.blueprint(),
    CmdVelMux.blueprint(),
)

# hinted: the follower is handed the per-waypoint clearance array recomputed from the
# raycaster's local map, which on this stack is live -- so this is the honest default.
# It gets the planner's lidar_height for the same reason the planner does: the room hint
# has to be measured off the slice the plan was priced in, not off the raw band.
go2_zenoh_motion = autoconnect(
    _go2_zenoh_motion_base,
    TrajectoryFollower.blueprint(track="hinted", lidar_height=ROBOT_HEIGHT),
).global_config(transport="zenoh", n_workers=9, robot_model="unitree_go2")

# blind: the same graph with the clearance hint withheld. The law recovers the required
# precision from the path's own timestamps instead (control/profile.py), which is the
# regime that survives when the local map is stale, empty, or not the follower's to read.
go2_zenoh_motion_blind = autoconnect(
    _go2_zenoh_motion_base,
    TrajectoryFollower.blueprint(track="blind", lidar_height=ROBOT_HEIGHT),
).global_config(transport="zenoh", n_workers=9, robot_model="unitree_go2")

# go2-zenoh-motion-local: go2-zenoh-motion with the time-critical half lifted off
# the laptop. The three modules below are ABSENT here because they run on the robot
# as one baked host (`dimos/navigation/motion/deployment_plan.md`):
#
#     motion_planner -> trajectory_follower -> cmd_vel_mux
#
# So this composes from the module list rather than from go2_zenoh_raycaster, which
# would drag in the CmdVelMux that now belongs on the robot. What stays here is
# everything that is either expensive (the raycaster), global (the MLS graph), or
# attached to the operator (rerun, clicks, teleop).
#
# THIS BLUEPRINT ALONE DOES NOT DRIVE. Without the baked host running there is no
# planner, no follower and no mux, so clicks become goals and MLS plans a global
# path that nothing tracks. Bring the robot host up first:
#
#     dimos bake motion_planner trajectory_follower cmd_vel_mux \
#         -o motion-host --target aarch64-unknown-linux-gnu
#
# and run it with DIMOS_ZENOH_LISTEN on a port of its own (the go2web bridge owns
# 7447), then dial both from here: --robot-ips <ip>:7447,<ip>:7448. Dialling both
# is belt-and-braces now that gossip is on at every scope -- one endpoint hands
# back the peers behind it -- but the fixed listen port still earns its keep: it
# is what makes the host dialable at all before any gossip has happened.
#
# THE HOST MUST ALSO DIAL THE BRIDGE ITSELF, over loopback:
#
#     DIMOS_ZENOH_CONNECT=tcp/127.0.0.1:7447
#
# Both robot-side sessions are passive listeners, so without this neither ever
# links to the other and `dimos/odometry` and `dimos/tf` -- which GO2Zenoh and the
# go2web bridge publish, not this laptop -- never reach the baked planner and
# follower. The failure is quiet and asymmetric: local_map and planner_path still
# arrive, because the laptop's native children dial 7448 directly, so the host
# looks half-connected rather than misconfigured. Dialling the bridge locally also
# keeps odometry off the wifi entirely.
#
# tf is the sharp edge now that the mount is not a config knob: both baked modules
# hold their pose down until the base_link <- mid360_link leg arrives on tf, and
# that leg comes from GO2Zenoh on the laptop. A host that never links to it plans
# nothing at all rather than planning off-heading, which is the failure mode we
# wanted -- but it is silent apart from one "dropping odometry" line per outage.
#
# cmd_vel still crosses back to the laptop, because GO2Zenoh is what talks to the
# go2web bridge. This cut buys jitter immunity on the control loop, not fewer wire
# crossings -- see "What this cut does and does not buy" in the deployment plan.
go2_zenoh_motion_local = autoconnect(
    vis_module(
        viewer_backend=global_config.viewer,
        rerun_config=_rerun_config({"world/pointlio_map": None, "world/lidar": None}),
    ),
    GO2Zenoh.blueprint(mid360_mount_rpy_deg=MID360_MOUNT_RPY_DEG),
    RayTracingVoxelMap.blueprint(**_motion_raycaster),
    _mls_planner_motion.remappings([(MLSPlannerNative, "path", "planner_path")]),
    GoalRelay.blueprint(lidar_height=ROBOT_HEIGHT),
    MovementManager.blueprint(),
).global_config(transport="zenoh", n_workers=6, robot_model="unitree_go2")
