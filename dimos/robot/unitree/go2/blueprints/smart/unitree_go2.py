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

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.stream import In
from dimos.core.transport import LCMTransport
from dimos.mapping.costmapper import CostMapper
from dimos.mapping.relocalization.module import RelocalizationModule
from dimos.mapping.relocalization.priors import FiducialPriorConfig, RansacPriorConfig
from dimos.mapping.voxels import VoxelGridMapper
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
from dimos.robot.unitree.go2.connection import GO2Connection

unitree_go2 = autoconnect(
    unitree_go2_basic,
    VoxelGridMapper.blueprint(emit_every=5),
    CostMapper.blueprint(),
    ReplanningAStarPlanner.blueprint(),
    WavefrontFrontierExplorer.blueprint(),
    PatrollingModule.blueprint(),
    MovementManager.blueprint(),
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

# lidar-only reloc: RANSAC alone aligns live scans to the premap. No marker
# detector. The base every deployment starts from. Set the map with
#   -o relocalizationmodule.map_file=/abs/path/to/site.pc2.lcm
unitree_go2_relocalization_lidar = autoconnect(
    unitree_go2,
    RelocalizationModule.blueprint(priors=[RansacPriorConfig()]),
).global_config(n_workers=11)

# lidar + fiducial: RANSAC and the fiducial prior BOTH propose into the shared
# judge. WITH MarkerDetectionStreamModule -- its `detections` Out autoconnects
# (by name+type) into RelocalizationModule's `detections` In, where each tag's
# sightings Huber-fuse into ONE world->map candidate that competes on wall
# fitness (never a bypass). The FiducialPriorConfig is the single source of the
# fiducial family: its aruco_dictionary threads into the detector so both decode
# and fuse the same tags. Maps stay unset -- the fiducial prior no-ops until its
# marker survey is provided; a bare name resolves via resolve_named_path (as
# given / DIMOS_PROJECT_ROOT / the shared data dir, LFS if registered).
_lidar_fiducial_prior = FiducialPriorConfig(
    marker_length_m=0.10,
    camera_info=GO2Connection.camera_info_static,
)
unitree_go2_relocalization_lidar_fiducial = autoconnect(
    unitree_go2,
    MarkerDetectionStreamModule.blueprint(
        marker_length_m=_lidar_fiducial_prior.marker_length_m,
        camera_info=GO2Connection.camera_info_static,
        aruco_dictionary=_lidar_fiducial_prior.aruco_dictionary,
    ),
    RelocalizationModule.blueprint(priors=[RansacPriorConfig(), _lidar_fiducial_prior]),
).global_config(n_workers=12, robot_model="unitree_go2")

# fiducial-only reloc: the fiducial prior is the SOLE proposer -- RANSAC disabled
# (omitted from the priors list). WITH MarkerDetectionStreamModule. For deployments
# that trust surveyed markers over the geometric global search.
_fiducial_only_prior = FiducialPriorConfig(
    marker_length_m=0.10,
    camera_info=GO2Connection.camera_info_static,
)
unitree_go2_relocalization_fiducial = autoconnect(
    unitree_go2,
    MarkerDetectionStreamModule.blueprint(
        marker_length_m=_fiducial_only_prior.marker_length_m,
        camera_info=GO2Connection.camera_info_static,
        aruco_dictionary=_fiducial_only_prior.aruco_dictionary,
    ),
    RelocalizationModule.blueprint(priors=[_fiducial_only_prior]),
).global_config(n_workers=12, robot_model="unitree_go2")

unitree_go2_memory = autoconnect(
    unitree_go2,
    Go2Memory.blueprint(),
).global_config(n_workers=12)
