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

"""3D navigation on Go2 with a Livox Mid-360 lidar, PointLIO odometry, ray-traced
voxel mapping, MLS planning, and holonomic trajectory control.

This is the Mid-360 counterpart to ``unitree_go2_mls_htc``: it keeps the
DanHolonomicTC follower but swaps in the rust ``RepulsiveFieldNative`` local
planner and replaces the Go2's onboard L1 lidar (over WebRTC) with the
head-mounted Mid-360 driven by PointLIO, matching the sensing front-end of
``unitree_go2_nav_3d``.

PointLIO needs the Mid-360 reachable: set ``DIMOS_POINTLIO_LIDAR_IP`` (and
optionally ``DIMOS_POINTLIO_HOST_IP``). GO2Connection is still used, but only for
motion control — its own lidar/camera are disabled.
"""

from datetime import datetime
import os
from pathlib import Path
from typing import Any

from dimos.constants import RECORDINGS_DIR
from dimos.core.coordination.blueprints import autoconnect
from dimos.core.global_config import global_config
from dimos.core.stream import In
from dimos.hardware.sensors.lidar.pointlio.module import PointLio
from dimos.hardware.sensors.lidar.pointlio.recorder import PointlioRecorder
from dimos.hardware.sensors.lidar.virtual_mid360.recorder import Mid360PcapRecorder
from dimos.mapping.ray_tracing.module import RayTracingVoxelMap
from dimos.memory2.module import pose_setter_for
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.navigation.dannav.holonomic_tc.module import DanHolonomicTC
from dimos.navigation.movement_manager.movement_manager import MovementManager
from dimos.navigation.nav_3d.mls_planner.goal_relay import GoalRelay
from dimos.navigation.nav_3d.mls_planner.mls_planner_native import MLSPlannerNative
from dimos.navigation.nav_3d.mls_planner.odom_body_frame import OdomBodyFrame
from dimos.navigation.nav_3d.repulsive_local_planner.repulsive_field_native import (
    RepulsiveFieldNative,
)
from dimos.robot.unitree.go2.blueprints.basic.unitree_go2_basic import rerun_config
from dimos.robot.unitree.go2.connection import GO2Connection
from dimos.robot.unitree.go2.go2_mid360_static_transforms import (
    base_link_from_mid360,
)
from dimos.visualization.vis_module import vis_module

voxel_size = 0.08

# Robot footprint (m): Go2 body ~0.7 long x ~0.33 wide. One source of truth for
# the whole stack — the local planner's oriented-box collision uses both, and mls's
# global wall clearance is driven off the half-width, so routing (what mls threads)
# and execution (what the local planner fits) agree on the robot's real size.
ROBOT_LENGTH = 0.7
ROBOT_WIDTH = 0.33

# Body-frame axis-triad length (m).
_axis_len = 0.5
# Arrow radius as a fraction of the triad length.
_AXIS_RADIUS_RATIO = 25
_PURPLE = (170, 0, 255)
base_link_from_mid360_tf = base_link_from_mid360()


class Go2Mid360Recorder(PointlioRecorder):
    lidar_l1: In[PointCloud2]
    odom_go2: In[PoseStamped]

    @pose_setter_for("odom_go2")
    async def _odom_go2_pose(self, msg: PoseStamped) -> PoseStamped:
        return msg


# Opt-in recording: set DIMOS_NAV_RECORD=1 to capture pointlio_lidar +
# pointlio_odometry into a timestamped db that plan_rrd replays from.
_RECORD = os.getenv("DIMOS_NAV_RECORD", "").lower() in ("1", "true", "yes", "on")

# Opt-in raw-Livox capture: set RECORD_PCAP=1 to also tcpdump the Mid-360 UDP
# stream into recordings/ (needs DIMOS_MID360_LIDAR_IP).
_RECORD_PCAP = os.getenv("RECORD_PCAP", "").lower() in ("1", "true", "yes", "on")


def _recording_dir() -> Path:
    now = datetime.now().astimezone()
    stamp = (
        now.strftime("%Y-%m-%d") + "_" + now.strftime("%I-%M%p").lower() + "-" + now.strftime("%Z")
    )
    return RECORDINGS_DIR / stamp


_RECORDING_DIR = _recording_dir()


def _render_global_map(msg: Any) -> Any:
    return msg.to_rerun()


def _render_path(msg: Any) -> Any:
    # The planner emits an empty path when it finds no route to the goal.
    # Logging those would blank the line, so drop them and keep the last path.
    if len(msg.poses) == 0:
        return None
    import math

    import rerun as rr

    line = msg.to_rerun(color=_PURPLE)

    # Ghost boxes: the robot's oriented footprint swept along the local path, so
    # the exact box the local planner is committing to (and threading gaps with)
    # is visible travelling the route. Sampled ~a dozen along the path, heading =
    # local travel direction, translucent green so obstacles show through.
    pts = [(p.pose.position.x, p.pose.position.y, p.pose.position.z) for p in msg.poses]
    step = max(1, len(pts) // 12)
    centers: list[list[float]] = []
    half_sizes: list[list[float]] = []
    quaternions: list[Any] = []
    for i in range(0, len(pts), step):
        x, y, z = pts[i]
        j = i + 1 if i + 1 < len(pts) else i - 1
        yaw = math.atan2(pts[j][1] - y, pts[j][0] - x) if 0 <= j < len(pts) and j != i else 0.0
        if j < i:
            yaw = math.atan2(y - pts[j][1], x - pts[j][0])
        centers.append([x, y, z])
        half_sizes.append([ROBOT_LENGTH / 2.0, ROBOT_WIDTH / 2.0, ROBOT_WIDTH / 2.0])
        quaternions.append(rr.Quaternion(xyzw=[0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)]))
    ghosts = rr.Boxes3D(
        centers=centers,
        half_sizes=half_sizes,
        quaternions=quaternions,
        colors=[(0, 255, 127, 45)],
    )
    return [("world/path", line), ("world/ghost_boxes", ghosts)]


def _render_costmap(msg: Any) -> Any:
    # RepulsiveFieldNative publishes its internal costmap's lethal cells (the
    # cells the solver actually repels from) as a flat point slice below the
    # robot. Draw them as red voxel boxes so the obstacle field is legible.
    return msg.to_rerun(colors=[255, 0, 0], mode="boxes", voxel_size=0.1)


def _render_shield_points(msg: Any) -> Any:
    # LidarShield's breach points — the obstacle returns inside the stop bubble
    # that engaged the shield. Draw them as orange spheres so an active shield is
    # obvious in the viewer.
    return msg.to_rerun(colors=[255, 140, 0], mode="spheres", voxel_size=0.1)


# Go2 body box half-extents (m): ~0.7 long, ~0.31 wide, ~0.4 tall.
_BODY_HALF = [0.35, 0.155, 0.2]


def _static_robot_body(rr: Any) -> list[Any]:
    """Go2-shaped box on the gravity-leveled body frame.

    base_footprint (published by OdomBodyFrame) is already horizontal and sits at
    the body center -- exactly where the local planner's footprint is -- so the box
    needs no counter-rotation and honestly shows the footprint, not the head sensor.
    """
    return [
        rr.Boxes3D(half_sizes=_BODY_HALF, colors=[(0, 255, 127)]),
        rr.Transform3D(parent_frame="tf#/base_footprint"),
    ]


def _axis_triad(rr: Any) -> Any:
    """XYZ axis triad, red/green/blue for x/y/z."""
    return rr.Arrows3D(
        origins=[[0.0, 0.0, 0.0]] * 3,
        vectors=[
            [_axis_len, 0.0, 0.0],
            [0.0, _axis_len, 0.0],
            [0.0, 0.0, _axis_len],
        ],
        colors=[[255, 0, 0], [0, 255, 0], [0, 0, 255]],
        radii=_axis_len / _AXIS_RADIUS_RATIO,
    )


def _static_body_axes(rr: Any) -> list[Any]:
    """XYZ triad at the FRONT face of the leveled body box (the robot's front)."""
    return [_axis_triad(rr), rr.Transform3D(translation=[_BODY_HALF[0], 0.0, 0.0])]


def _static_sensor_axes(rr: Any) -> list[Any]:
    """XYZ triad on pointlio's raw sensor frame, tilted by the lidar pitch."""
    return [_axis_triad(rr), rr.Transform3D(parent_frame="tf#/mid360_link")]


_nav_rerun_config = {
    **rerun_config,
    "max_hz": {
        **rerun_config["max_hz"],
        # Rate-limited at the source by global_emit_every, roughly every 5s.
        "world/global_map": 0,
        "world/local_map": 0.5,
        # Costmap cloud is published at costmap_cloud_hz (5); cap the viewer too.
        "world/costmap_cloud": 5,
    },
    # Ring buffer replayed to a connecting viewer. Small so connect catches up fast.
    "memory_limit": "64MB",
    # base_link tf comes from the go2 internal odometry, which is not the map
    # frame. Anchor the robot box to pointlio's mid360_link frame instead and hide
    # the camera frustum that rides base_link. The box lives on its own entity:
    # a static transform on world/tf/mid360_link itself would override the live tf.
    "static": {
        "world/robot_body": _static_robot_body,
        "world/robot_body/axes": _static_body_axes,
        "world/sensor_axes": _static_sensor_axes,
    },
    "visual_override": {
        **rerun_config["visual_override"],
        "world/global_map": _render_global_map,
        "world/path": _render_path,
        "world/costmap_cloud": _render_costmap,
        "world/shield_points": _render_shield_points,
        "world/camera_info": None,
        "world/color_image": None,
        "world/lidar": None,
        "world/surface_map": None,
        "world/nodes": None,
        "world/node_edges": None,
    },
}

unitree_go2_mls_htc_mid360 = autoconnect(
    vis_module(viewer_backend=global_config.viewer, rerun_config=_nav_rerun_config),
    # "mcf" for stair traversal. The Go2's own lidar/camera are off — the Mid-360
    # feeds PointLIO — so GO2Connection is here only to drive motion.
    GO2Connection.blueprint(
        lidar=False, camera=False, motion_mode="mcf", odom_frame_id="go2_odom"
    ).remappings(
        [
            (GO2Connection, "lidar", "lidar_l1"),
            (GO2Connection, "odom", "odom_go2"),
        ]
    ),
    PointLio.blueprint(),
    RayTracingVoxelMap.blueprint(
        voxel_size=voxel_size,
        emit_every=1,
        global_emit_every=50,
        min_health=-1,
        max_health=5,
        support_min=4,
    ),
    # global_map is remapped off so the planner runs purely on the
    # incremental local_map + region_bounds pair.
    MLSPlannerNative.blueprint(
        world_frame="odom",
        voxel_size=voxel_size,
        robot_height=0.3,
        surface_closing_radius=0.3,
        # Global hard clearance = robot half-width, so mls won't route a path
        # through a gap the robot physically can't fit (the local planner then
        # threads it with the oriented box). Was a hard-coded 0.1 m < half-width.
        wall_clearance_m=ROBOT_WIDTH / 2,
        wall_buffer_m=0.75,
        wall_buffer_weight=100.0,
        step_threshold_m=0.16,
        step_penalty_weight=4.0,
        viz_publish_hz=0.0,
    ).remappings(
        [
            (MLSPlannerNative, "global_map", "global_map_unused"),
            (MLSPlannerNative, "path", "planner_path"),
        ]
    ),
    # Re-express odometry at the body center (lidar is on the head): shifts the
    # local planner's footprint back to the body, and gives the follower the body pose.
    OdomBodyFrame.blueprint(
        mount_rotation=list(base_link_from_mid360_tf.rotation.to_tuple()),
        # base_link relative to the sensor, in the leveled body frame. The measured
        # mount put the sensor ~0.29 m ahead of / 0.16 m above the body center, so
        # the body sits that far back/down -- less 0.1 m (the raw offset
        # over-compensated back). One value; make it less negative to move forward.
        body_offset=[
            -base_link_from_mid360_tf.translation.x + 0.1,
            -base_link_from_mid360_tf.translation.y,
            -base_link_from_mid360_tf.translation.z,
        ],
    ),
    GoalRelay.blueprint().remappings([(GoalRelay, "odometry", "body_odometry")]),
    RepulsiveFieldNative.blueprint(
        world_frame="odom",
        output_base_frame=False,
        # Oriented bounding-box footprint (replaces the isotropic vehicle_width
        # circle): robot_width is the narrow dimension it threads gaps by,
        # robot_length is enforced by the oriented-box path validation.
        robot_length=ROBOT_LENGTH,
        robot_width=ROBOT_WIDTH,
        resolution=0.1,  # voxel size
    ).remappings(
        [
            (RepulsiveFieldNative, "terrain_map", "local_map"),
            (RepulsiveFieldNative, "global_path", "planner_path"),
            # route_tail is fed the same stream as global_path (treat them alike),
            # so it lands on the same resolved topic, planner_path.
            (RepulsiveFieldNative, "route_tail", "planner_path"),
            (RepulsiveFieldNative, "local_path", "path"),
            (RepulsiveFieldNative, "odometry", "body_odometry"),
        ]
    ),
    # DanHolonomicTC tracks on a PoseStamped `odom`; GoalRelay's `start_pose` is the
    # robot pose in the same frame the planner plans in, so remap it onto that.
    DanHolonomicTC.blueprint(run_profile="walk").remappings(
        [(DanHolonomicTC, "odom", "start_pose")]
    ),
    # MovementManager's cmd_vel is diverted through the shield: it publishes to
    # cmd_vel_raw, the shield gates it, and the shield drives the robot's cmd_vel.
    MovementManager.blueprint(),
).global_config(n_workers=11, robot_model="unitree_go2", obstacle_avoidance=False)

# The nav blueprint leaves PointLio on its default lidar / odometry topics, so
# remap the recorder's ports onto them. Streams are recorded under the port
# names pointlio_lidar / pointlio_odometry regardless of the topic.
if _RECORD:
    unitree_go2_mls_htc_mid360 = autoconnect(
        unitree_go2_mls_htc_mid360,
        Go2Mid360Recorder.blueprint(db_path=str(_RECORDING_DIR / "mem2.db")).remappings(
            [
                (Go2Mid360Recorder, "pointlio_lidar", "lidar"),
                (Go2Mid360Recorder, "pointlio_odometry", "odometry"),
            ]
        ),
    )

if _RECORD_PCAP:
    unitree_go2_mls_htc_mid360 = autoconnect(
        unitree_go2_mls_htc_mid360,
        Mid360PcapRecorder.blueprint(pcap_path=_RECORDING_DIR / "mid360.pcap"),
    )
