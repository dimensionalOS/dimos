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

"""Live voxel map that stays loop-closed.

Every lidar frame goes into a stamped voxel grid and into incremental PGO.
When PGO closes a loop the old poses move, so the grid re-places each voxel
by the correction for the frame that wrote it (`PoseGraph.place`). The map is
anchored at the robot: current data stays where raw odometry puts it.

Check against the offline method on a recording::

    uv run python -m dimos.navigation.go2.loop_closure.pgo_map go2_hongkong_office
"""

from __future__ import annotations

from collections.abc import Iterator
import math
import sys
import time

import numpy as np

from dimos.mapping.voxels.grid import VoxelGrid
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.navigation.go2.loop_closure.pgo import PGOConfig, PoseGraph, _PGOState, _pose_to_pose3
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

# Go2 odometry yaw swings past 10 deg about twice a second while walking, so the
# offline default keyframes on time rather than distance. 45 deg leaves turns only.
LIVE_PGO = PGOConfig(key_pose_delta_deg=45.0)


class PGOMap:
    """Voxel map plus incremental PGO; rebuilds itself after loop closures."""

    def __init__(
        self,
        *,
        voxel_size: float = 0.05,
        frame_id: str = "world",
        rebuild_cooldown_s: float = 10.0,
        pgo: PGOConfig | None = None,
    ) -> None:
        self._pgo = _PGOState(pgo or LIVE_PGO)
        self._grid = VoxelGrid(
            voxel_size=voxel_size, frame_id=frame_id, stamped=True, show_startup_log=False
        )
        self._cooldown = rebuild_cooldown_s
        self._placed_loops = 0
        self._last_rebuild_ts = -math.inf
        # How long the latest loop-closing PGO step and the latest rebuild took.
        self.loop_ms = 0.0
        self.rebuild_ms = 0.0

    @property
    def n_keyframes(self) -> int:
        return self._pgo.n_keyframes

    @property
    def n_loops(self) -> int:
        return self._pgo.n_loops

    def add(self, cloud: PointCloud2, pose: Pose | None) -> bool:
        """Insert a world-frame lidar frame taken at odom `pose`. True if the map was rebuilt."""
        self._grid.add_frame(cloud)
        if pose is not None and not (pose.position.is_zero() or pose.orientation.is_zero()):
            t0, loops = time.perf_counter(), self.n_loops
            self._pgo.process(_pose_to_pose3(pose), cloud.ts, cloud)
            if self.n_loops != loops:
                self.loop_ms = (time.perf_counter() - t0) * 1e3

        # frame time, not wall time, so replay and tests pace the same as the robot
        if self.n_loops == self._placed_loops or cloud.ts - self._last_rebuild_ts < self._cooldown:
            return False
        t0 = time.perf_counter()
        self._grid.reproject(self.graph().place)
        self._placed_loops, self._last_rebuild_ts = self.n_loops, cloud.ts
        self.rebuild_ms = (time.perf_counter() - t0) * 1e3
        logger.info(
            "PGO map rebuilt",
            loops=self.n_loops,
            voxels=len(self._grid),
            loop_ms=round(self.loop_ms),
            rebuild_ms=round(self.rebuild_ms),
        )
        return True

    def graph(self) -> PoseGraph:
        return self._pgo.snapshot()

    def keyframe_poses(self) -> tuple[np.ndarray, np.ndarray]:
        """Keyframe poses in the map frame: positions (K, 3), xyzw quats (K, 4)."""
        return self._pgo.keyframe_poses()

    def loop_segments(self) -> tuple[np.ndarray, np.ndarray]:
        """Loop edges in the map frame: endpoints (L, 2, 3) and ICP scores (L,)."""
        return self._pgo.loop_segments()

    def global_map(self) -> PointCloud2:
        return self._grid.get_global_pointcloud2()

    def dispose(self) -> None:
        self._grid.dispose()


def _within_one_voxel(a: np.ndarray, b: np.ndarray, voxel: float) -> float:
    """Fraction of voxel centers in `a` with a voxel of `b` at most one cell away."""
    from dimos.mapping.voxels.keys import KEY_OFFSET, X_SHIFT, Y_SHIFT, pack_indices

    ka, kb = (
        np.unique(pack_indices(np.floor(p / voxel).astype(np.int64) + KEY_OFFSET)) for p in (a, b)
    )
    hit = np.zeros(len(ka), dtype=bool)
    for dx in (-1, 0, 1):
        for dy in (-1, 0, 1):
            for dz in (-1, 0, 1):
                q = ka + dx * (1 << X_SHIFT) + dy * (1 << Y_SHIFT) + dz
                hit |= kb[np.minimum(np.searchsorted(kb, q), len(kb) - 1)] == q
    return float(hit.mean())


def main(dataset: str) -> None:
    """Run a recording through PGOMap at full speed and compare with the offline rebuild."""
    from dimos.memory.cli.dataset import open_dataset

    lidar = open_dataset(dataset).stream("lidar", PointCloud2)

    def frames() -> Iterator[tuple[PointCloud2, Pose | None]]:
        for obs in lidar:
            cloud = obs.data
            cloud.ts = obs.ts  # old recordings carry Unitree's stale lidar stamps
            yield cloud, obs.pose

    live = PGOMap()
    step_ms, rebuilds = [], 0
    cpu0, wall0 = time.process_time(), time.perf_counter()
    for cloud, pose in frames():
        t0 = time.perf_counter()
        rebuilds += live.add(cloud, pose)
        step_ms.append((time.perf_counter() - t0) * 1e3)
    cpu, wall = time.process_time() - cpu0, time.perf_counter() - wall0

    # offline method: every frame re-inserted at its final corrected pose
    graph = live.graph()
    live._grid.reproject(graph.place)
    gold = VoxelGrid(device="CPU:0", show_startup_log=False)
    for cloud, _ in frames():
        gold.add_frame(graph.place(cloud))

    got, want = live.global_map().points_f32(), gold.get_global_pointcloud2().points_f32()
    missing = 1 - _within_one_voxel(want, got, 0.05)
    extra = 1 - _within_one_voxel(got, want, 0.05)
    steps = np.array(step_ms)
    print(
        f"frames={len(steps)} keyframes={live.n_keyframes} loops={live.n_loops} rebuilds={rebuilds}\n"
        f"step ms: mean={steps.mean():.1f} p95={np.percentile(steps, 95):.1f} max={steps.max():.0f}\n"
        f"cpu/wall={cpu / wall:.2f} ({cpu:.0f}s cpu over {wall:.0f}s)\n"
        f"voxels: live={len(got)} offline={len(want)}\n"
        f"vs offline, within one voxel: missing={missing:.2%} extra={extra:.2%}"
    )
    assert missing < 0.03 and extra < 0.03, "live map drifted from the offline rebuild"


if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else "go2_hongkong_office")
