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

from pathlib import Path
from typing import TYPE_CHECKING

from reactivex.disposable import Disposable

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.core import rpc
from dimos.core.stream import In, Out
from dimos.hardware.sensors.lidar.fastlio2.module import FastLio2
from dimos.mapping.ray_tracing.module import RayTracingVoxelMap
from dimos.mapping.voxels import VoxelGridMapper
from dimos.memory2.module import MemoryModule, MemoryModuleConfig, Recorder, RecorderConfig
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.utils.testing.replay import timed_playback
from dimos.visualization.vis_module import vis_module

if TYPE_CHECKING:
    from rerun._baseclasses import Archetype


class FastlioMemoryConfig(RecorderConfig):
    db_path: str | Path = "recording_fastlio.db"
    default_frame_id: str = "base_link"


voxel_size = 0.05


class FastlioMemory(Recorder):
    config: FastlioMemoryConfig
    lidar: In[PointCloud2]
    odometry: In[Odometry]

    @rpc
    def start(self) -> None:
        super().start()

        def _on_odom(msg: Odometry) -> None:
            self.tf.publish(Transform.from_odometry(msg))

        self.register_disposable(Disposable(self.odometry.subscribe(_on_odom)))


class FastlioReplayConfig(MemoryModuleConfig):
    db_path: str | Path = "recording_fastlio.db"
    speed: float = 1.0


class FastlioReplay(MemoryModule):
    """Replays a FastLIO2 recording (lidar + odometry) at real-time speed.

    Drop-in replacement for ``FastLio2`` when feeding rerun off a recorded session.
    Publishes odometry to tf so downstream visualizers see robot pose.
    """

    config: FastlioReplayConfig
    lidar: Out[PointCloud2]
    odometry: Out[Odometry]

    @rpc
    def start(self) -> None:
        super().start()

        lidar_stream = self.store.stream("lidar", PointCloud2)
        odom_stream = self.store.stream("odometry", Odometry)

        def _publish_odom(msg: Odometry) -> None:
            self.tf.publish(Transform.from_odometry(msg))
            self.odometry.publish(msg)

        speed = self.config.speed

        self.register_disposable(
            timed_playback(
                lambda: ((obs.ts, obs.data) for obs in lidar_stream),
                speed=speed,
            ).subscribe(self.lidar.publish)
        )
        self.register_disposable(
            timed_playback(
                lambda: ((obs.ts, obs.data) for obs in odom_stream),
                speed=speed,
            ).subscribe(_publish_odom)
        )


def _convert_global_map(msg: PointCloud2) -> "Archetype":
    return msg.to_rerun(voxel_size=voxel_size)


mid360_fastlio = autoconnect(
    FastLio2.blueprint(voxel_size=voxel_size, map_voxel_size=voxel_size, map_freq=-1),
    vis_module("rerun"),
).global_config(n_workers=2, robot_model="mid360_fastlio2")

mid360_fastlio_memory = autoconnect(
    FastLio2.blueprint(voxel_size=voxel_size, map_voxel_size=voxel_size, map_freq=-1),
    vis_module("rerun"),
    FastlioMemory.blueprint(),
).global_config(n_workers=3, robot_model="mid360_fastlio2_memory")

mid360_fastlio_voxels = autoconnect(
    FastLio2.blueprint(),
    VoxelGridMapper.blueprint(voxel_size=voxel_size, carve_columns=False),
    vis_module(
        "rerun",
        rerun_config={
            "visual_override": {
                "world/lidar": None,
            },
        },
    ),
).global_config(n_workers=3, robot_model="mid360_fastlio2_voxels")

mid360_fastlio_replay = autoconnect(
    FastlioReplay.blueprint(),
    vis_module(
        "rerun",
        rerun_config={
            "visual_override": {
                "world/global_map": _convert_global_map,
            },
        },
    ),
).global_config(n_workers=2, robot_model="mid360_fastlio2_replay")

mid360_fastlio_replay_voxels = autoconnect(
    FastlioReplay.blueprint(),
    VoxelGridMapper.blueprint(voxel_size=voxel_size, carve_columns=True),
    vis_module(
        "rerun",
        rerun_config={
            "visual_override": {
                "world/global_map": _convert_global_map,
            },
        },
    ),
).global_config(n_workers=2, robot_model="mid360_fastlio2_replay")

mid360_fastlio_voxels_native = autoconnect(
    FastLio2.blueprint(voxel_size=voxel_size, map_voxel_size=voxel_size, map_freq=3.0),
    vis_module(
        "rerun",
        rerun_config={
            "visual_override": {
                "world/lidar": None,
            },
        },
    ),
).global_config(n_workers=2, robot_model="mid360_fastlio2")


mid360_fastlio_ray_trace_replay = autoconnect(
    FastlioReplay.blueprint(),
    RayTracingVoxelMap.blueprint(voxel_size=voxel_size),
    vis_module(
        "rerun",
        rerun_config={
            "visual_override": {
                "world/lidar": None,
            },
        },
    ),
).global_config(n_workers=3, robot_model="mid360_fastlio2_ray_trace_replay")


mid360_fastlio_ray_trace = autoconnect(
    FastLio2.blueprint(voxel_size=voxel_size, map_voxel_size=voxel_size, map_freq=-1),
    RayTracingVoxelMap.blueprint(voxel_size=voxel_size),
    vis_module(
        "rerun",
        rerun_config={
            "visual_override": {
                "world/lidar": None,
            },
        },
    ),
).global_config(n_workers=3, robot_model="mid360_fastlio2_ray_trace")
