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

from reactivex.disposable import Disposable

from dimos.core.core import rpc
from dimos.core.native_module import NativeModule, NativeModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.nav_msgs.Graph3D import Graph3D
from dimos.msgs.nav_msgs.GraphDelta3D import GraphDelta3D
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.navigation.nav_stack.specs import LoopClosure
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class PGOConfig(NativeModuleConfig):
    cwd: str | None = str(Path(__file__).resolve().parent / "cpp")
    executable: str = "result/bin/pgo"
    build_command: str | None = "nix build .#default --no-write-lock-file"

    frame_id: str = "map"
    child_frame_id: str = "odom"
    body_frame: str = "base_link"

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

    # Skip ICP on candidates farther than this (m). 0 disables.
    loop_candidate_max_distance_m: float = 30.0

    # True: drop stale queued scans each tick. False: strict FIFO.
    drain_stale_scans: bool = True

    debug: bool = False


class PGO(NativeModule, LoopClosure):
    """Pose graph optimization with loop closure using GTSAM iSAM2 + PCL ICP."""

    config: PGOConfig

    registered_scan: In[PointCloud2]
    odometry: In[Odometry]
    corrected_odometry: Out[Odometry]
    correction: Out[Transform]
    global_map: Out[PointCloud2]
    pose_graph: Out[Graph3D]
    loop_closure_event: Out[GraphDelta3D]

    @rpc
    def start(self) -> None:
        super().start()
        self.tf.publish(
            Transform(
                frame_id=self.config.frame_id,
                child_frame_id=self.config.child_frame_id,
            )
        )
        self.register_disposable(
            Disposable(
                self.correction.transport.subscribe(self._on_correction_for_tf, self.correction)
            )
        )
        if self.config.debug:
            logger.info("PGO native module started (C++ iSAM2 + PCL ICP)")

    def _on_correction_for_tf(self, msg: Transform) -> None:
        self.tf.publish(msg)

    @rpc
    def stop(self) -> None:
        super().stop()
