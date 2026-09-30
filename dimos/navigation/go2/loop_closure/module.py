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

from __future__ import annotations

from reactivex.disposable import Disposable

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.PoseArray import PoseArray
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.nav_msgs.LineSegments3D import LineSegments3D
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.msgs.std_msgs.Header import Header
from dimos.navigation.go2.loop_closure.pgo import PGOConfig, PoseGraph
from dimos.navigation.go2.loop_closure.pgo_map import LIVE_PGO, PGOMap
from dimos.robot.unitree.type.lidar import repair_stale_ts
from dimos.utils.reactive import backpressure


class PGOVoxelMapperConfig(ModuleConfig):
    voxel_size: float = 0.05
    frame_id: str = "world"
    emit_every: int = 1
    # Shortest gap between map rebuilds, in seconds of lidar time.
    rebuild_cooldown_s: float = 10.0
    pgo: PGOConfig = LIVE_PGO


class PGOVoxelMapper(Module):
    """Global voxel map that stays loop-closed, anchored at the robot."""

    config: PGOVoxelMapperConfig
    dedicated_worker = True

    lidar: In[PointCloud2]
    odom: In[PoseStamped]
    global_map: Out[PointCloud2]
    # The pose graph in the map frame: keyframe poses in order, and loop edges.
    pgo_keyframes: Out[PoseArray]
    pgo_loops: Out[LineSegments3D]

    _map: PGOMap | None = None
    _pose: PoseStamped | None = None
    _frames: int = 0
    _graph_ts: float = 0.0
    _graph_keyframes: int = 0

    @rpc
    def start(self) -> None:
        super().start()
        cfg = self.config
        self._map = PGOMap(
            voxel_size=cfg.voxel_size,
            frame_id=cfg.frame_id,
            rebuild_cooldown_s=cfg.rebuild_cooldown_s,
            pgo=cfg.pgo,
        )
        self.register_disposable(Disposable(self.odom.subscribe(self._on_odom)))
        # One frame at a time; frames arriving during an ICP or a rebuild are dropped.
        self.register_disposable(
            backpressure(self.lidar.observable().pipe(repair_stale_ts())).subscribe(self._on_lidar)
        )

    @rpc
    def stop(self) -> None:
        super().stop()
        if self._map is not None:
            self._map.dispose()

    def _on_odom(self, pose: PoseStamped) -> None:
        self._pose = pose

    def _on_lidar(self, cloud: PointCloud2) -> None:
        assert self._map is not None
        rebuilt = self._map.add(cloud, self._pose)
        self._frames += 1
        if rebuilt or self._frames % self.config.emit_every == 0:
            self.global_map.publish(self._map.global_map())

        # A graph snapshot is O(keyframes): on rebuild, else once per cooldown as it grows.
        grown = self._map.n_keyframes != self._graph_keyframes
        if rebuilt or (grown and cloud.ts - self._graph_ts >= self.config.rebuild_cooldown_s):
            graph = self._map.placed if rebuilt else self._map.graph()
            assert graph is not None
            self._graph_ts, self._graph_keyframes = cloud.ts, self._map.n_keyframes
            self._publish_graph(graph, cloud.ts)

    def _publish_graph(self, graph: PoseGraph, ts: float) -> None:
        frame_id = self.config.frame_id
        positions, quats = graph.keyframe_poses()
        self.pgo_keyframes.publish(
            PoseArray(
                Header(ts, frame_id),
                [
                    Pose(*xyz, *quat)
                    for xyz, quat in zip(positions.tolist(), quats.tolist(), strict=True)
                ],
            )
        )
        self.pgo_loops.publish(
            LineSegments3D(
                ts=ts,
                frame_id=frame_id,
                segments=graph.loop_segments(),
                weights=[loop.score for loop in graph.loops],
            )
        )
