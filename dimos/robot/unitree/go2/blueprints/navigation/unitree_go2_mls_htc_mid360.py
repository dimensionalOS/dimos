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
import math
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
from dimos.navigation.nav_3d.repulsive_local_planner.repulsive_field_native import (
    RepulsiveFieldNative,
)
from dimos.robot.unitree.go2.blueprints.basic.unitree_go2_basic import rerun_config
from dimos.robot.unitree.go2.connection import GO2Connection
from dimos.robot.unitree.go2.go2_mid360_static_transforms import MID360_PITCH_DOWN
from dimos.visualization.vis_module import vis_module

voxel_size = 0.08

# Body-frame axis-triad length (m).
_axis_len = 0.5
# Arrow radius as a fraction of the triad length.
_AXIS_RADIUS_RATIO = 25


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


# Local path drawn purple; Path.to_rerun defaults to green.
_PATH_COLOR = (170, 0, 255)


def _render_path(msg: Any) -> Any:
    # The planner emits an empty path when it finds no route to the goal.
    # Logging those would blank the line, so drop them and keep the last path.
    if len(msg.poses) == 0:
        return None
    return msg.to_rerun(color=_PATH_COLOR)


def _render_costmap(msg: Any) -> Any:
    # RepulsiveFieldNative publishes its internal costmap's lethal cells (the
    # cells the solver actually repels from) as a flat point slice below the
    # robot. Draw them as red voxel boxes so the obstacle field is legible.
    return msg.to_rerun(colors=[255, 0, 0], mode="boxes", voxel_size=0.1)


def _static_robot_body(rr: Any) -> list[Any]:
    """Go2-shaped box on pointlio's sensor frame, counter-rotated for the lidar pitch."""
    return [
        rr.Boxes3D(half_sizes=[0.35, 0.155, 0.2], colors=[(0, 255, 127)]),
        rr.Transform3D(
            parent_frame="tf#/mid360_link",
            rotation=rr.RotationAxisAngle(axis=(0, 1, 0), degrees=-math.degrees(MID360_PITCH_DOWN)),
        ),
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


def _static_body_axes(rr: Any) -> Any:
    """XYZ triad on the leveled robot body (child of the counter-rotated box)."""
    return _axis_triad(rr)


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
        wall_clearance_m=0.1,
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
    GoalRelay.blueprint(),
    RepulsiveFieldNative.blueprint(
        world_frame="odom",
        output_base_frame=False,
        # Keep the internal costmap at the module's validated 0.1 m, NOT the
        # 0.08 m map voxel. A costmap finer than the input gets hole-filled and
        # edge-smoothed (costmap.rs), which ramps sharp obstacle edges into
        # gentle slopes and drops their height-gradient cost below the lethal
        # threshold — low obstacles then stop repelling. Coarser-than-input is
        # the safe direction.
        resolution=0.1,
    ).remappings(
        [
            (RepulsiveFieldNative, "terrain_map", "local_map"),
            (RepulsiveFieldNative, "global_path", "planner_path"),
            # route_tail is fed the same stream as global_path (treat them alike),
            # so it lands on the same resolved topic, planner_path.
            (RepulsiveFieldNative, "route_tail", "planner_path"),
            (RepulsiveFieldNative, "local_path", "path"),
        ]
    ),
    # DanHolonomicTC tracks on a PoseStamped `odom`; GoalRelay's `start_pose` is the
    # robot pose in the same frame the planner plans in, so remap it onto that.
    DanHolonomicTC.blueprint(run_profile="walk").remappings(
        [(DanHolonomicTC, "odom", "start_pose")]
    ),
    MovementManager.blueprint(),
).global_config(n_workers=10, robot_model="unitree_go2", obstacle_avoidance=False)

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
