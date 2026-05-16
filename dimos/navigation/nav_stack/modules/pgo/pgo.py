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

"""Native C++ PGO module — faithful reimplementation of the original nav stack PGO.

Uses GTSAM iSAM2 for pose graph optimization and PCL ICP for loop closure.
"""

from __future__ import annotations

from pathlib import Path

from dimos.core.core import rpc
from dimos.core.native_module import NativeModule, NativeModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.nav_msgs.GraphNodes3D import GraphNodes3D
from dimos.msgs.nav_msgs.LineSegments3D import LineSegments3D
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.nav_msgs.Path import Path as NavPath
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class PGOConfig(NativeModuleConfig):
    cwd: str | None = str(Path(__file__).resolve().parent / "cpp")
    executable: str = "result/bin/pgo"
    build_command: str | None = "nix build .#default --no-write-lock-file"

    frame_id: str = "map"
    child_frame_id: str = "start_point"
    parent_frame: str = "world"
    body_frame: str = "current_point"
    tf_channel: str = "/tf#tf2_msgs.TFMessage"

    # The C++ binary's CLI args use the legacy frame names.
    cli_name_override: dict[str, str] = {
        "frame_id": "world_frame",
        "child_frame_id": "local_frame",
    }

    # Keyframe detection
    key_pose_delta_deg: float = 10.0
    key_pose_delta_trans: float = 0.5

    # Loop closure
    loop_search_radius: float = 1.0
    loop_time_thresh: float = 60.0
    loop_score_thresh: float = 0.15
    loop_submap_half_range: int = 5
    submap_resolution: float = 0.1
    min_loop_detect_duration: float = 5.0

    # Input mode: transform world-frame scans to body-frame using odom
    unregister_input: bool = True

    # Global map publishing
    global_map_voxel_size: float = 0.1
    global_map_publish_rate: float = 1.0

    # Scan Context place recognition (used by loop closure search)
    use_scan_context: bool = True
    scan_context_num_rings: int = 20
    scan_context_num_sectors: int = 60
    scan_context_max_range_m: float = 80.0
    scan_context_top_k: int = 10
    scan_context_match_threshold: float = 0.4
    scan_context_lidar_height_m: float = 2.0

    debug: bool = False


class PGO(NativeModule):
    """Pose graph optimization with loop closure using GTSAM iSAM2 + PCL ICP."""

    config: PGOConfig

    registered_scan: In[PointCloud2]
    odometry: In[Odometry]
    corrected_odometry: Out[Odometry]
    global_map: Out[PointCloud2]
    pose_graph_nodes: Out[GraphNodes3D]
    pose_graph_edges: Out[LineSegments3D]
    loop_closure: Out[NavPath]

    @rpc
    def start(self) -> None:
        super().start()
        if self.config.debug:
            logger.info("PGO native module started (C++ iSAM2 + PCL ICP)")

    @rpc
    def stop(self) -> None:
        super().stop()
