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

from pathlib import Path

from dimos.core.coordination.blueprints import Blueprint, autoconnect
from dimos.core.global_config import global_config
from dimos.core.stream import In
from dimos.core.transport import LCMTransport
from dimos.mapping.costmapper import CostMapper
from dimos.mapping.relocalization.module import RelocalizationModule
from dimos.mapping.voxels.module import VoxelGridMapper
from dimos.memory2.module import Recorder, RecorderConfig, pose_setter_for
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.msgs.vision_msgs.Detection3DArray import Detection3DArray
from dimos.navigation.frontier_exploration.wavefront_frontier_goal_selector import (
    WavefrontFrontierExplorer,
)
from dimos.navigation.movement_manager.movement_manager import MovementManager
from dimos.navigation.patrolling.module import PatrollingModule
from dimos.navigation.replanning_a_star.module import ReplanningAStarPlanner
from dimos.perception.fiducial.marker_detection_stream_module import MarkerDetectionStreamModule
from dimos.perception.fiducial.marker_tf_module import MarkerTfModule
from dimos.robot.unitree.go2.blueprints.basic.unitree_go2_basic import unitree_go2_basic
from dimos.robot.unitree.go2.blueprints.basic.unitree_mid360_basic import unitree_mid360_basic
from dimos.robot.unitree.go2.connection import GO2Connection

# Shared navigation stack for the smart Go2 / Mid-360 reloc blueprints.
_unitree_go2_navigation = autoconnect(
    CostMapper.blueprint(),
    ReplanningAStarPlanner.blueprint(),
    WavefrontFrontierExplorer.blueprint(),
    PatrollingModule.blueprint(),
    MovementManager.blueprint(),
)

unitree_go2 = autoconnect(
    unitree_go2_basic,
    VoxelGridMapper.blueprint(emit_every=5),
    _unitree_go2_navigation,
).global_config(n_workers=10, robot_model="unitree_go2")


class Go2MemoryConfig(RecorderConfig):
    db_path: str | Path = "recording_go2.db"


class Go2Memory(Recorder):
    color_image: In[Image]
    lidar: In[PointCloud2]
    odom: In[PoseStamped]
    config: Go2MemoryConfig

    _last_odom_pose: Pose | None = None

    @pose_setter_for("odom")
    async def _odom_pose(self, msg: PoseStamped) -> Pose | None:
        self._last_odom_pose = msg
        return self._last_odom_pose

    @pose_setter_for("lidar")
    async def _lidar_pose(self, msg: PointCloud2) -> Pose | None:
        # Yes, it doesn't make sense to register lidar at the odom pose because the
        # go2 lidar is in the world frame, but map.py (for now) needs this.
        # TODO: fix map.py to use a transform frame
        return getattr(self, "_last_odom_pose", None)


unitree_go2_markers = (
    autoconnect(
        unitree_go2,
        MarkerDetectionStreamModule.blueprint(
            marker_length_m=0.1,
            camera_info=GO2Connection.camera_info_static,
        ),
        MarkerTfModule.blueprint(),
    )
    .transports(
        {
            ("detections", MarkerDetectionStreamModule): LCMTransport(
                "/marker_detection/detections",
                Detection3DArray,
            ),
        }
    )
    .global_config(n_workers=11, robot_model="unitree_go2")
)

unitree_go2_relocalization = autoconnect(
    unitree_go2,
    RelocalizationModule.blueprint(),
).global_config(n_workers=11)


def unitree_go2_mid360_relocalization_base(lidar_config: str) -> Blueprint:
    """Pick the reloc base stack from ``lidar_config``.

    ``mid360`` → ``unitree_mid360_basic`` (Point-LIO odom preference + Mid-360
    Rerun / 3 cm voxels) plus the shared nav stack. Anything else → standard
    ``unitree_go2`` (``unitree_go2_basic`` + voxels + nav).

    CLI/env are applied to ``global_config`` before blueprint import, so
    ``--lidar-config mid360`` selects the Mid-360 base at composition time.
    """
    if lidar_config == "mid360":
        # mid360_basic already includes VoxelGridMapper at Mid-360 resolution.
        return autoconnect(unitree_mid360_basic, _unitree_go2_navigation)
    return unitree_go2


# Mid-360 presets (3 cm voxels, reloc FINE_VOXEL=0.06 / RERANK_DIST=0.12).
# Base stack follows ``--lidar-config``: mid360 → unitree_mid360_basic, else
# unitree_go2_basic via unitree_go2. Override with ``--lidar-config default``.
unitree_go2_mid360_relocalization = autoconnect(
    unitree_go2_mid360_relocalization_base(global_config.lidar_config),
    RelocalizationModule.blueprint(),
).global_config(n_workers=11, lidar_config="mid360")

unitree_go2_memory = autoconnect(
    unitree_go2,
    Go2Memory.blueprint(),
).global_config(n_workers=12)
