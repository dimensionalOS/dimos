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

from copy import deepcopy
from pathlib import Path
import queue
import threading
import time

from pydantic import Field
from reactivex.disposable import Disposable

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos_messages.dimos_msgs.msg import LineSegment3D, LineSegments3D
from dimos_messages.geometry_msgs.msg import (
    Point,
    Pose,
    PoseArray,
    PoseStamped,
    Quaternion,
    TransformStamped,
)
from dimos_messages.sensor_msgs.msg import PointCloud2
from dimos_messages.std_msgs.msg import Float32, Header
from dimos_messages.tf2_msgs.msg import TFMessage
from dimos.navigation.go2.loop_closure.pgo import PGOConfig
from dimos.navigation.go2.loop_closure.pgo_map import LIVE_PGO, PGOMap
from dimos.navigation.go2.loop_closure.premap import MAP_SUFFIX, Premap
from dimos.navigation.go2.loop_closure.relocalizer import PremapMatcher, attempt_fix
from dimos.robot.unitree.type.lidar import repair_stale_ts
from dimos.utils.data import backup_file, resolve_named_path
from dimos.utils.logging_config import setup_logger
from dimos.utils.reactive import backpressure

logger = setup_logger()


class PGOVoxelMapperConfig(ModuleConfig):
    voxel_size: float = 0.05
    frame_id: str = "world"
    emit_every: int = 1
    # Shortest gap between map rebuilds, in seconds of lidar time.
    rebuild_cooldown_s: float = 10.0
    pgo: PGOConfig = LIVE_PGO
    # Publish timings in ms: mapper_frame_ms per frame, pgo_loop_ms and pgo_rebuild_ms per event.
    telemetry: bool = True
    # Premap graph stem or path (`.map.db` appended if absent), relocalized against
    map_file: str | None = None
    # Keep map_file saved with this run joined to it; a missing one starts a new map
    save_map: bool = False
    # The map from before each run is kept as <name>.map.<time>.db, this many of them
    backup_keep_last: int = Field(default=10, ge=0)
    # Seconds of raw lidar matched against the premap, and the fewest points worth a match
    reloc_window_s: float = 30.0
    reloc_min_points: int = 20_000
    # Wall-only fine inlier ratio a match needs to count as a fix
    reloc_fitness_threshold: float = 0.45
    # Relocalize back to back until settled, then this often
    reloc_recheck_s: float = 30.0


def _pgo_config(cfg: PGOVoxelMapperConfig) -> PGOConfig:
    """The PGO settings, loop closure left on unless switched off or in simulation.

    Simulators publish ground-truth odometry, so there is no drift for a loop to correct.
    """
    if cfg.pgo.loop_closure is not None:
        return cfg.pgo
    return cfg.pgo.model_copy(update={"loop_closure": not cfg.g.simulation})


class PGOVoxelMapper(Module):
    """Global voxel map that stays loop-closed, anchored at the robot, optionally on a premap."""

    config: PGOVoxelMapperConfig
    dedicated_worker = True

    lidar: In[PointCloud2]
    odom: In[PoseStamped]
    global_map: Out[PointCloud2]
    # On every new keyframe: graph poses (premap included once attached) and non-odometry edges
    pgo_keyframes: Out[PoseArray]
    pgo_loops: Out[LineSegments3D]
    # world -> map once a premap is attached
    tf: Out[TFMessage]
    # Only published with config.telemetry.
    mapper_frame_ms: Out[Float32]
    pgo_loop_ms: Out[Float32]
    pgo_rebuild_ms: Out[Float32]

    _map: PGOMap | None = None
    _pose: PoseStamped | None = None
    _frames: int = 0
    _graph_keyframes: int = 0
    _matcher: PremapMatcher | None = None
    _reloc_thread: threading.Thread | None = None
    _map_path: Path | None = None

    @rpc
    def start(self) -> None:
        super().start()
        cfg = self.config
        premap = None
        if cfg.map_file:
            self._map_path = resolve_named_path(
                cfg.map_file, MAP_SUFFIX, must_exist=not cfg.save_map
            )
            if self._map_path.exists():
                premap = Premap.load(self._map_path)
            if premap is None or not len(premap):
                premap = None
                logger.info(
                    f"PGO premap: no keyframes in {self._map_path.name} yet, starting a new map"
                )
        elif cfg.save_map:
            raise ValueError("PGOVoxelMapper: save_map needs a map_file to save to")
        self._backed_up = False
        self._writer: threading.Thread | None = None
        self._map = PGOMap(
            voxel_size=cfg.voxel_size,
            frame_id=cfg.frame_id,
            rebuild_cooldown_s=cfg.rebuild_cooldown_s,
            pgo=_pgo_config(cfg),
            premap=premap,
        )
        # fix, its fitness, and the current attachment's fitness on the same window
        self._fixes: queue.Queue[tuple[TransformStamped, float, float | None]] = queue.Queue()
        self._stopping = threading.Event()
        # held while a frame is processed, so stop never disposes the map under one
        self._frame_lock = threading.Lock()
        if premap is not None:
            self._matcher = PremapMatcher(
                premap.cloud(cfg.voxel_size),
                min_points=cfg.reloc_min_points,
                fitness_threshold=cfg.reloc_fitness_threshold,
            )
            # open3d releases the GIL, so matching here does not stall the frame thread
            self._reloc_thread = threading.Thread(
                target=self._relocalize_loop, name="pgo-relocalize", daemon=True
            )
            self._reloc_thread.start()
            logger.info(f"PGO premap loaded: map_file={cfg.map_file!r} keyframes={len(premap)}")
        self.register_disposable(Disposable(self.odom.subscribe(self._on_odom)))
        # One frame at a time; frames arriving during an ICP or a rebuild are dropped.
        self.register_disposable(
            backpressure(self.lidar.observable().pipe(repair_stale_ts())).subscribe(self._on_lidar)
        )

    @rpc
    def stop(self) -> None:
        self._stopping.set()
        if self._reloc_thread is not None:
            self._reloc_thread.join(timeout=5.0)
        super().stop()
        if self._map is None:
            return
        with self._frame_lock:
            if self._writer is not None:
                self._writer.join()
            if self.config.save_map and self._map_path is not None:
                if not self._save_map(background=False):
                    logger.warning(
                        f"PGO premap: never settled, left {self._map_path.name} as it was"
                    )
            self._map.dispose()

    def _save_map(self, background: bool) -> bool:
        """Snapshot the joint map here, build and write it on a thread when ``background``.

        False if there is nothing to save.
        """
        assert self._map is not None and self._map_path is not None
        if background and self._writer is not None and self._writer.is_alive():
            return True  # the next rebuild saves
        build = self._map.save_snapshot()
        if build is None:
            return False
        path = self._map_path

        def write() -> None:
            joint = build()
            # without backups the old map stays until the save replaces it
            if not self._backed_up and self.config.backup_keep_last:
                backup = backup_file(path, keep_last=self.config.backup_keep_last)
                if backup is not None:
                    logger.info(f"PGO premap: backed up {path.name} -> {backup.name}")
                self._backed_up = True
            joint.save(path)
            logger.info(f"PGO premap: saved {path} keyframes={len(joint)} edges={len(joint.edges)}")

        if background:
            self._writer = threading.Thread(target=write, name="pgo-save-map", daemon=True)
            self._writer.start()
        else:
            write()
        return True

    def _relocalize_loop(self) -> None:
        assert self._matcher is not None and self._map is not None
        while not self._stopping.wait(
            self.config.reloc_recheck_s if self._map.pgo.settled else 2.0
        ):
            # the frame thread writes the newest keyframe, so read the window under its lock
            with self._frame_lock:
                if self._stopping.is_set():
                    break
                window = self._map.recent_window(self.config.reloc_window_s)
                current = self._map.pgo.world_to_map()
            found = attempt_fix(self._matcher, window, current) if window is not None else None
            if found is not None:
                self._fixes.put(found)

    def _on_odom(self, pose: PoseStamped) -> None:
        self._pose = pose

    def _on_lidar(self, cloud: PointCloud2) -> None:
        with self._frame_lock:
            if not self._stopping.is_set():
                self._process(cloud)

    def _process(self, cloud: PointCloud2) -> None:
        assert self._map is not None
        t0, loops = time.perf_counter(), self._map.pgo.n_loops
        attached = self._map.pgo.attached
        while not self._fixes.empty():
            fix, fitness, current = self._fixes.get()
            self._map.pgo.relocalize(fix, fitness, current)
        rebuilt = self._map.add(cloud, None if self._pose is None else self._pose.pose)
        if rebuilt and self.config.save_map and self._map_path is not None:
            self._save_map(background=True)
        self._frames += 1
        emit = (
            rebuilt
            or self._map.pgo.attached != attached
            or self._frames % self.config.emit_every == 0
        )
        global_map = self._map.global_map() if emit else None
        frame_ms = (time.perf_counter() - t0) * 1e3

        if global_map is not None:
            self.global_map.publish(global_map)
            placement = self._map.pgo.world_to_map()
            if placement is not None:
                self.tf.publish(TFMessage(transforms=[placement]))
        if self._map.pgo.n_keyframes != self._graph_keyframes:
            self._graph_keyframes = self._map.pgo.n_keyframes
            self._publish_graph(cloud.header)
        if self.config.telemetry:
            self.mapper_frame_ms.publish(Float32(data=frame_ms))
            if self._map.pgo.n_loops != loops:
                self.pgo_loop_ms.publish(Float32(data=self._map.loop_ms))
            if rebuilt:
                self.pgo_rebuild_ms.publish(Float32(data=self._map.rebuild_ms))

    def _publish_graph(self, source: Header) -> None:
        assert self._map is not None
        header = deepcopy(source)
        header.frame_id = self.config.frame_id
        positions, quats = self._map.pgo.keyframe_poses()
        self.pgo_keyframes.publish(
            PoseArray(
                header=deepcopy(header),
                poses=[
                    Pose(position=Point(*xyz), orientation=Quaternion(*quat))
                    for xyz, quat in zip(positions.tolist(), quats.tolist(), strict=True)
                ],
            )
        )
        segments, scores = self._map.pgo.loop_segments()
        self.pgo_loops.publish(
            LineSegments3D(
                header=header,
                segments=[
                    LineSegment3D(start=Point(*ends[0]), end=Point(*ends[1]), weight=float(score))
                    for ends, score in zip(segments.tolist(), scores.tolist(), strict=True)
                ],
            )
        )
