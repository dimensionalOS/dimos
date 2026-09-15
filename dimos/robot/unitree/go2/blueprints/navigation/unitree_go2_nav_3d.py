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

"""3d navigation on Go2 with ray tracing and MLS planning"""

from datetime import datetime
import os
from pathlib import Path

from dimos.constants import RECORDINGS_DIR
from dimos.core.coordination.blueprints import autoconnect
from dimos.core.global_config import global_config
from dimos.core.stream import In
from dimos.hardware.sensors.lidar.pointlio.module import PointLio
from dimos.hardware.sensors.lidar.pointlio.recorder import PointlioRecorder
from dimos.hardware.sensors.lidar.virtual_mid360.recorder import Mid360PcapRecorder
from dimos.mapping.ray_tracing.module import RayTracingVoxelMap
from dimos.mapping.relocalization.lidar.module import LocalMapRelocalization
from dimos.mapping.relocalization.lidar.relocalize import GO2_NAV
from dimos.memory.module import pose_setter_for
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.navigation.basic_path_follower.module import BasicPathFollower
from dimos.navigation.movement_manager.movement_manager import MovementManager
from dimos.navigation.nav_3d.mls_planner.mls_planner_native import MLSPlannerNative
from dimos.navigation.nav_3d.viz import nav_static, nav_visual_override
from dimos.robot.unitree.go2.blueprints.basic.unitree_go2_basic import rerun_config
from dimos.robot.unitree.go2.connection import GO2Connection
from dimos.robot.unitree.go2.constants import ROBOT_HEIGHT, ROBOT_LENGTH, ROBOT_WIDTH
from dimos.robot.unitree.go2.go2_mid360_static_transforms import Go2Mid360StaticTf
from dimos.robot.unitree.go2.nav_3d_config import (
    mls_planner_config,
    ray_tracing_config,
    voxel_size,
    wall_clearance_m,
)
from dimos.visualization.vis_module import vis_module

# What the planner searched over (surface, nodes, weighted edges). Seeded from a
# premap it is the whole building, several MB a tick, so keep this low.
planner_viz_hz = 0.2


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


nav_rerun_config = {
    **rerun_config,
    "max_hz": {
        **rerun_config["max_hz"],
        # Rate-limited at the source by global_emit_every, roughly every 5s.
        "world/global_map": 0,
        "world/local_map": 0.5,
    },
    # Ring buffer replayed to a connecting viewer. Small so connect catches up fast.
    "memory_limit": "64MB",
    # The robot box hangs off base_link on its own entity: a static transform
    # under world/tf would override the live one.
    "static": nav_static(ROBOT_LENGTH, ROBOT_WIDTH, ROBOT_HEIGHT, wall_clearance_m),
    "visual_override": {
        **rerun_config["visual_override"],
        # The raw premap is millions of points. The seeded voxels are full_map.
        "world/loaded_map": None,
        "world/camera_info": None,
        "world/color_image": None,
        "world/lidar": None,
        **nav_visual_override(planner_viz_hz, voxel_size, wall_clearance_m),
    },
}

unitree_go2_nav_3d = autoconnect(
    vis_module(viewer_backend=global_config.viewer, rerun_config=nav_rerun_config),
    # "mcf" for stair traversal
    GO2Connection.blueprint(
        lidar=False,
        camera=False,
        motion_mode="mcf",
        odom_frame_id="go2_odom",
        publish_tf=False,
    ).remappings(
        [
            (GO2Connection, "lidar", "lidar_l1"),
            (GO2Connection, "odom", "odom_go2"),
        ]
    ),
    PointLio.blueprint(),
    Go2Mid360StaticTf.blueprint(),
    RayTracingVoxelMap.blueprint(**ray_tracing_config.model_dump(exclude_unset=True)),
    MLSPlannerNative.blueprint(
        **mls_planner_config.model_copy(update={"viz_publish_hz": planner_viz_hz}).model_dump(
            exclude_unset=True
        )
    ).remappings([(MLSPlannerNative, "global_map", "global_map_unused")]),
    BasicPathFollower.blueprint(
        speed=0.5,
        heading_gain=1.0,
        max_angular=1.0,
        lookahead_time_s=2.5,
        min_lookahead_m=1.2,
    ),
    MovementManager.blueprint(),
).global_config(n_workers=10, robot_model="unitree_go2", obstacle_avoidance=False)

# PointLio keeps its default topics here, so point the recorder's ports at them.
# Streams are recorded under the port names regardless of the topic.
if _RECORD:
    unitree_go2_nav_3d = autoconnect(
        unitree_go2_nav_3d,
        Go2Mid360Recorder.blueprint(db_path=str(_RECORDING_DIR / "mem2.db")).remappings(
            [
                (Go2Mid360Recorder, "pointlio_lidar", "lidar"),
                (Go2Mid360Recorder, "pointlio_odometry", "odometry"),
            ]
        ),
    )

if _RECORD_PCAP:
    unitree_go2_nav_3d = autoconnect(
        unitree_go2_nav_3d,
        Mid360PcapRecorder.blueprint(pcap_path=_RECORDING_DIR / "mid360.pcap"),
    )

# The republish covers a ray tracer that missed the one-shot loaded_map publish.
unitree_go2_nav_3d_relocalization = autoconnect(
    unitree_go2_nav_3d,
    LocalMapRelocalization.blueprint(
        world_frame="odom",
        republish_loaded_map=30.0,
        relocalize=GO2_NAV,
    ),
).global_config(n_workers=11)
