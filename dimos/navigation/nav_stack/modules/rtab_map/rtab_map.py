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

"""Native C++ RtabMap module — alternative SLAM provider to PGO.

Wraps `result/bin/rtab_map`, a C++ binary that links librtabmap and
consumes FastLIO2 odometry as an external odom source. Defaults match the
user spec: 3D enabled, OctoMap enabled, raycasting / point clearing enabled.
The binary subscribes to and publishes on LCM channels matching the streams
declared here. ``rtab_tf`` carries the ``map -> odom`` correction as an
:class:`Odometry` message (parity with PGO's ``pgo_tf``); we re-publish it
through :attr:`self.tf` exactly the way PGO does.
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
from dimos.navigation.nav_stack.frames import FRAME_BODY, FRAME_MAP, FRAME_ODOM
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class RtabMapConfig(NativeModuleConfig):
    """Config for the RtabMap native module."""

    cwd: str | None = str(Path(__file__).resolve().parent / "cpp")
    executable: str = "result/bin/rtab_map"
    build_command: str | None = "nix build .#default --no-write-lock-file"

    # --- Frame names ---
    world_frame: str = FRAME_MAP
    local_frame: str = FRAME_ODOM
    body_frame: str = FRAME_BODY

    # --- OctoMap / Grid defaults (the six required by the locked spec) ---
    grid_3d: bool = True
    grid_ray_tracing: bool = True
    grid_from_depth: bool = False
    grid_cell_size: float = 0.1
    grid_max_ground_angle: float = 45.0
    grid_ground_is_obstacle: bool = False
    grid_flat_obstacle_detected: bool = True

    # --- Publishing cadence ---
    octomap_publish_period: float = 0.5
    global_map_publish_period: float = 1.0
    global_map_voxel_size: float = 0.15

    # --- Input handling ---
    # Input scans arrive in the world (map) frame; the binary undoes the
    # current odom transform so rtabmap sees body-frame scans.
    unregister_input: bool = True


class RtabMap(NativeModule):
    """RtabMap NativeModule — librtabmap behind an LCM wrapper.

    Plays the same role in the nav stack as :class:`PGO`: consumes
    FastLIO2's external odometry + registered lidar scans, runs RTAB-Map
    SLAM internally (lidar-only mode, ICP registration), and publishes
    corrected odometry, the accumulated global cloud, a ``map -> odom`` TF
    correction, plus OctoMap-derived outputs (occupied voxels and a 2D
    projection).
    """

    config: RtabMapConfig

    registered_scan: In[PointCloud2]
    odometry: In[Odometry]

    corrected_odometry: Out[Odometry]
    global_map: Out[PointCloud2]
    rtab_tf: Out[Odometry]
    octomap: Out[PointCloud2]
    projected_2d_grid: Out[PointCloud2]

    @rpc
    def start(self) -> None:
        super().start()
        # Fan map->odom corrections from the C++ binary to the Python TF
        # bridge — same pattern PGO uses for pgo_tf. Subscribing through
        # the transport (not the Out's local subscribers) is required
        # because the publisher lives in the C++ subprocess.
        self.register_disposable(
            Disposable(self.rtab_tf.transport.subscribe(self._on_tf_correction, self.rtab_tf))
        )
        # Seed identity TF so downstream consumers can resolve map->body
        # before the first loop closure shifts the map frame.
        self._publish_tf(
            translation=(0.0, 0.0, 0.0),
            rotation=(0.0, 0.0, 0.0, 1.0),
            ts=time.time(),
        )
        logger.info("RtabMap native module started (C++ librtabmap + LCM)")

    @rpc
    def stop(self) -> None:
        super().stop()

    def _on_tf_correction(self, msg: Odometry) -> None:
        self._publish_tf(
            translation=(msg.pose.position.x, msg.pose.position.y, msg.pose.position.z),
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
