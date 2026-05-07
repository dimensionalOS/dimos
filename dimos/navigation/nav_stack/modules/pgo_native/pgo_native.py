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

Uses GTSAM iSAM2 for pose graph optimization and PCL ICP for loop closure
verification.  All parameters match the upstream ``pgo_unity_sim.yaml`` defaults.
"""

from __future__ import annotations

from pathlib import Path
import time

from reactivex.disposable import Disposable

from dimos.core.core import rpc
from dimos.core.native_module import NativeModule, NativeModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class PGONativeConfig(NativeModuleConfig):
    cwd: str | None = str(Path(__file__).resolve().parent / "cpp")
    executable: str = "result/bin/pgo_native"
    build_command: str | None = "nix build .#default --no-write-lock-file"

    # Frame names
    world_frame: str = "map"
    local_frame: str = "odom"

    # Keyframe detection
    key_pose_delta_deg: float = 10.0
    key_pose_delta_trans: float = 0.5

    # Loop closure (defaults match original pgo_unity_sim.yaml)
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


class PGONative(NativeModule):
    """Native C++ pose graph optimization with loop closure.

    Faithful reimplementation of the upstream PGO using GTSAM iSAM2 + PCL ICP.
    Publishes corrected odometry, accumulated global map, and map->odom TF.
    """

    config: PGONativeConfig

    registered_scan: In[PointCloud2]
    odometry: In[Odometry]
    corrected_odometry: Out[Odometry]
    global_map: Out[PointCloud2]
    pgo_tf: Out[Odometry]

    @rpc
    def start(self) -> None:
        super().start()
        # Seed identity TF so consumers can query map->body immediately
        self._publish_tf(
            translation=(0.0, 0.0, 0.0),
            rotation=(0.0, 0.0, 0.0, 1.0),
            ts=time.time(),
        )
        # Subscribe to pgo_tf output to relay TF corrections
        self.register_disposable(
            Disposable(self.pgo_tf.transport.subscribe(self._on_tf_correction, self.pgo_tf))
        )
        logger.info("PGO native module started (C++ iSAM2 + PCL ICP)")

    @rpc
    def stop(self) -> None:
        super().stop()

    def _on_tf_correction(self, msg: Odometry) -> None:
        self._publish_tf(
            translation=(
                msg.pose.position.x,
                msg.pose.position.y,
                msg.pose.position.z,
            ),
            rotation=(
                msg.pose.orientation.x,
                msg.pose.orientation.y,
                msg.pose.orientation.z,
                msg.pose.orientation.w,
            ),
            ts=msg.ts or time.time(),
        )

    def _publish_tf(
        self,
        translation: tuple[float, float, float],
        rotation: tuple[float, float, float, float],
        ts: float,
    ) -> None:
        self.tf.publish(
            Transform(
                frame_id=self.config.world_frame,
                child_frame_id=self.config.local_frame,
                translation=Vector3(*translation),
                rotation=Quaternion(*rotation),
                ts=ts,
            )
        )
