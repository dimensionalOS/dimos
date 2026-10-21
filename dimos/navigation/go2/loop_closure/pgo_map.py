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

"""A map made of keyframes, kept loop-closed by incremental PGO (see `PGOMap`).

The map is anchored at the robot: current data stays where raw odometry puts it.

Check against the offline method on a recording::

    uv run python -m dimos.navigation.go2.loop_closure.pgo_map go2_hongkong_office
"""

from __future__ import annotations

from collections.abc import Callable, Iterator
from dataclasses import dataclass
import math
import sys
import time

import numpy as np

from dimos.mapping.voxels.grid import VoxelGrid
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.navigation.go2.loop_closure.pgo import PGOConfig, PGOState, _pose_to_pose3
from dimos.navigation.go2.loop_closure.premap import (
    ROS_GTSAM_SWAP,
    Premap,
    apply,
)
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

# Go2 odom yaw swings past 10 deg while walking; 45 deg keyframes on real turns only.
LIVE_PGO = PGOConfig(key_pose_delta_deg=45.0)


def _columns(pts: np.ndarray, voxel: float) -> np.ndarray:
    ij = np.floor(pts[:, :2] / voxel).astype(np.int64)
    return (ij[:, 0] << 32) ^ (ij[:, 1] & 0xFFFFFFFF)


def _youngest(owner: np.ndarray, cols: np.ndarray) -> np.ndarray:
    """Indices of the points of each column's youngest (highest) owner, in column order."""
    order = np.lexsort((-owner, cols))  # by column, youngest first
    c, o = cols[order], owner[order]
    first = np.r_[True, c[1:] != c[:-1]]
    youngest = np.repeat(o[first], np.diff(np.r_[np.flatnonzero(first), len(o)]))
    return np.asarray(order[o == youngest])


@dataclass(frozen=True)
class _Placed:
    """Points in the graph's frame sorted by column, with each one's owner."""

    pts: np.ndarray
    owner: np.ndarray
    cols: np.ndarray

    @staticmethod
    def youngest(pts: np.ndarray, owner: np.ndarray, voxel: float) -> _Placed:
        """Only the youngest owner of each column."""
        cols = _columns(pts, voxel)
        keep = _youngest(owner, cols)
        return _Placed(pts[keep], owner[keep], cols[keep])

    def outside(self, cols: np.ndarray) -> np.ndarray:
        """Which points stand in none of ``cols``."""
        u = np.unique(cols)
        lo = np.searchsorted(self.cols, u, side="left")
        hi = np.searchsorted(self.cols, u, side="right")
        inside = np.zeros(len(self.cols) + 1, dtype=np.int32)
        np.add.at(inside, lo, 1)
        np.add.at(inside, hi, -1)
        return np.asarray(np.cumsum(inside[:-1]) == 0)

    def under(self, newer: _Placed) -> _Placed:
        """This carved by a younger layer: its columns are the younger one's."""
        keep = self.outside(newer.cols)
        cols = self.cols[keep]
        at = np.searchsorted(cols, newer.cols)
        return _Placed(
            np.insert(self.pts[keep], at, newer.pts, axis=0),
            np.insert(self.owner[keep], at, newer.owner),
            np.insert(cols, at, newer.cols),
        )


_NOTHING = _Placed(np.empty((0, 3)), np.empty(0, dtype=np.int64), np.empty(0, dtype=np.int64))


class PGOMap:
    """A map made of keyframes, kept loop-closed by incremental PGO.

    Each keyframe owns the frames from it to the next, in its own body frame,
    later frames winning each column. The map is every keyframe placed where
    the graph has it now, the youngest keyframe winning each column, so old
    observations give way to new ones. A premap's keyframes are older than any
    of this run's, and join the map once relocalization attaches it.
    """

    def __init__(
        self,
        *,
        voxel_size: float = 0.05,
        frame_id: str = "world",
        rebuild_cooldown_s: float = 10.0,
        pgo: PGOConfig | None = None,
        premap: Premap | None = None,
    ) -> None:
        self.pgo = PGOState(pgo or LIVE_PGO, premap)
        self._voxel_size = voxel_size
        self._frame_id = frame_id
        self._premap = premap
        self._buckets: list[VoxelGrid] = []
        self._cooldown = rebuild_cooldown_s
        # the poses the cached map was placed at, oldest keyframe first, and how far each one's points reach
        self._placed_poses = np.empty((0, 4, 4))
        self._placed_reach = np.empty(0)
        self._placed_attaches = 0
        self._last_rebuild_ts = -math.inf
        self._latest_ts = 0.0
        # every closed keyframe placed and carved, in the graph's frame
        self._base = _NOTHING
        self._base_keyframes = 0
        # How long the latest loop-closing PGO step and the latest rebuild took.
        self.loop_ms = 0.0
        self.rebuild_ms = 0.0

    def add(self, cloud: PointCloud2, pose: Pose | None) -> bool:
        """Insert a world-frame lidar frame taken at odom `pose`. True if the map was re-placed."""
        if pose is not None and not (pose.position.is_zero() or pose.orientation.is_zero()):
            t0, loops = time.perf_counter(), self.pgo.n_loops
            self.pgo.process(_pose_to_pose3(pose), cloud.ts, cloud)
            if self.pgo.n_loops != loops:
                self.loop_ms = (time.perf_counter() - t0) * 1e3
        if not self.pgo.n_keyframes or not len(cloud):
            return False
        self._latest_ts = cloud.ts
        local, _ = self.pgo.keyframe_matrices()
        while len(self._buckets) < self.pgo.n_keyframes:
            self._buckets.append(self._new_bucket())
        body = apply(np.linalg.inv(local[-1]), cloud.points_f32()).astype(np.float32)
        self._buckets[-1].add_frame(PointCloud2.from_numpy(body, timestamp=cloud.ts))

        # frame time, not wall time, so replay and tests pace the same as the robot
        if self.pgo.n_attaches != self._placed_attaches or (
            cloud.ts - self._last_rebuild_ts >= self._cooldown and self._moved()
        ):
            t0 = time.perf_counter()
            self._place_all()
            self._last_rebuild_ts = cloud.ts
            self._placed_attaches = self.pgo.n_attaches
            self.rebuild_ms = (time.perf_counter() - t0) * 1e3
            logger.info(
                "PGO map rebuilt",
                loops=self.pgo.n_loops,
                points=len(self._base.pts),
                loop_ms=round(self.loop_ms),
                rebuild_ms=round(self.rebuild_ms),
            )
            return True
        # keyframes that closed since: each younger than the base, so it carves the base
        while self._base_keyframes < self.pgo.n_keyframes - 1:
            owner = self._owner(self._base_keyframes)
            self._base = self._base.under(self._render([owner]))
            self._placed_poses = np.concatenate([self._placed_poses, [owner[2]]])
            self._placed_reach = np.append(self._placed_reach, _reach(owner[0]))
            self._base_keyframes += 1
        return False

    def _new_bucket(self) -> VoxelGrid:
        return VoxelGrid(voxel_size=self._voxel_size, device="CPU:0", show_startup_log=False)

    def _owners(self) -> list[tuple[np.ndarray, int, np.ndarray]]:
        """Every keyframe in the map, oldest first: points in its own frame, owner, pose.

        Owner numbers match `joint_graph`: premap keyframe j is j, live keyframe k
        is n_premap + k. The premap is in the map only once attached.
        """
        n_premap = len(self._premap) if self._premap is not None else 0
        out: list[tuple[np.ndarray, int, np.ndarray]] = []
        if self._premap is not None and self._has_premap():
            out = list(
                zip(self._premap.scans, range(n_premap), self.pgo.premap_poses(), strict=True)
            )
        return out + [self._owner(k) for k in range(len(self._buckets))]

    def _owner(self, k: int) -> tuple[np.ndarray, int, np.ndarray]:
        """Live keyframe ``k``: its points in its own frame, owner number, pose."""
        _, optimized = self.pgo.keyframe_matrices()
        n_premap = len(self._premap) if self._premap is not None else 0
        return self._buckets[k].get_global_pointcloud2().points_f32(), n_premap + k, optimized[k]

    def _render(self, owners: list[tuple[np.ndarray, int, np.ndarray]]) -> _Placed:
        """These keyframes placed, the youngest winning each column."""
        owners = [(scan, o, m) for scan, o, m in owners if len(scan)]
        if not owners:
            return _NOTHING
        return _Placed.youngest(
            np.concatenate([apply(m, scan) for scan, _, m in owners]),
            np.concatenate([np.full(len(scan), o) for scan, o, _ in owners]),
            self._voxel_size,
        )

    def _placed(self, k: int) -> _Placed:
        """Live keyframe ``k`` alone, placed."""
        return self._render([self._owner(k)])

    def _place_all(self) -> None:
        """Re-place every closed keyframe at the graph's current poses."""
        closed = self._owners()[:-1] if self.pgo.n_keyframes else []
        self._base = self._render(closed)
        self._placed_poses = np.array([m for _, _, m in closed]).reshape(-1, 4, 4)
        self._placed_reach = np.array([_reach(scan) for scan, _, _ in closed])
        self._base_keyframes = max(self.pgo.n_keyframes - 1, 0)

    def _moved(self) -> bool:
        """Whether a closed keyframe's points have moved by more than half a voxel since it was placed."""
        if not len(self._placed_poses):
            return False
        _, live = self.pgo.keyframe_matrices()
        now = np.concatenate([self.pgo.premap_poses(), live]) if self._has_premap() else live
        d = now[: len(self._placed_poses)] - self._placed_poses
        # a point at reach r moves at most |dt| + |dR| r
        shift = (
            np.linalg.norm(d[:, :3, 3], axis=1)
            + np.linalg.norm(d[:, :3, :3], ord=2, axis=(1, 2)) * self._placed_reach
        )
        return bool(shift.max() > self._voxel_size / 2)

    def _has_premap(self) -> bool:
        """Whether the premap is in the map: once attached."""
        return self._premap is not None and self.pgo.attached

    def joint_premap(self) -> Premap | None:
        """This run joined to its premap, as a premap to save; None when there is nothing to save."""
        save = self.save_snapshot()
        return save() if save is not None else None

    def save_snapshot(self) -> Callable[[], Premap] | None:
        """What `joint_premap` needs, taken now; the returned call builds it, on any thread.

        Each keyframe keeps the points it still owns, younger keyframes having
        taken the columns they saw, and keyframes left owning none are pruned.
        None when there is nothing to save.
        """
        # settled, not just attached, so a run outside the premap never writes into it
        if not self.pgo.n_keyframes or (self._premap is not None and not self.pgo.settled):
            return None
        stamps, poses, edges, measured, cov = self.pgo.joint_graph()
        owners = self._owners()

        def build() -> Premap:
            placed = self._render(owners)
            order = np.argsort(placed.owner, kind="stable")
            bounds = np.searchsorted(placed.owner[order], np.arange(len(owners) + 1))
            scans = tuple(
                apply(np.linalg.inv(m), placed.pts[order[bounds[o] : bounds[o + 1]]]).astype(
                    np.float32
                )
                for _, o, m in owners
            )
            return Premap(
                stamps=stamps,
                poses=poses,
                scans=scans,
                edges=edges,
                edge_poses=measured,
                edge_covariances=cov[:, ROS_GTSAM_SWAP][:, :, ROS_GTSAM_SWAP],
            ).pruned()

        return build

    def recent_window(self, seconds: float) -> PointCloud2 | None:
        """The keyframes of the last ``seconds`` in the raw odom frame, stamped at their middle."""
        if not self.pgo.n_keyframes:
            return None
        local, _ = self.pgo.keyframe_matrices()
        stamps = self.pgo.keyframe_stamps()
        recent = np.flatnonzero(stamps >= self._latest_ts - seconds)
        if not len(recent):
            recent = np.array([self.pgo.n_keyframes - 1])
        pts = [
            apply(local[k], self._buckets[k].get_global_pointcloud2().points_f32()) for k in recent
        ]
        window = PointCloud2.from_numpy(np.concatenate(pts).astype(np.float32))
        window.ts = (stamps[recent[0]] + self._latest_ts) / 2
        return window

    def live_map(self) -> PointCloud2:
        """This run's keyframes alone, in the robot-anchored world frame."""
        n_premap = len(self._premap) if self._premap is not None else 0
        pts = self._render([o for o in self._owners() if o[1] >= n_premap]).pts
        world = apply(np.linalg.inv(self.pgo.graph_from_world()), pts) if len(pts) else pts
        return PointCloud2.from_numpy(np.asarray(world, dtype=np.float32).reshape(-1, 3))

    def global_map(self) -> PointCloud2:
        """The map in the robot-anchored world frame."""
        pts = self._base.pts
        if self.pgo.n_keyframes:
            newest = self._placed(self.pgo.n_keyframes - 1)
            pts = np.concatenate([self._base.pts[self._base.outside(newest.cols)], newest.pts])
        world = apply(np.linalg.inv(self.pgo.graph_from_world()), pts) if len(pts) else pts
        return PointCloud2.from_numpy(
            np.asarray(world, dtype=np.float32).reshape(-1, 3),
            frame_id=self._frame_id,
            timestamp=self._latest_ts,
        )

    def dispose(self) -> None:
        for bucket in self._buckets:
            bucket.dispose()


def _reach(scan: np.ndarray) -> float:
    return float(np.linalg.norm(scan, axis=1).max()) if len(scan) else 0.0


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
    graph = live.pgo.snapshot()
    live._place_all()
    gold = VoxelGrid(device="CPU:0", show_startup_log=False)
    for cloud, _ in frames():
        gold.add_frame(graph.place(cloud))

    got, want = live.global_map().points_f32(), gold.get_global_pointcloud2().points_f32()
    missing = 1 - _within_one_voxel(want, got, 0.05)
    extra = 1 - _within_one_voxel(got, want, 0.05)
    steps = np.array(step_ms)
    print(
        f"frames={len(steps)} keyframes={live.pgo.n_keyframes} loops={live.pgo.n_loops} rebuilds={rebuilds}\n"
        f"step ms: mean={steps.mean():.1f} p95={np.percentile(steps, 95):.1f} max={steps.max():.0f}\n"
        f"cpu/wall={cpu / wall:.2f} ({cpu:.0f}s cpu over {wall:.0f}s)\n"
        f"voxels: live={len(got)} offline={len(want)}\n"
        f"vs offline, within one voxel: missing={missing:.2%} extra={extra:.2%}"
    )
    assert missing < 0.03 and extra < 0.03, "live map drifted from the offline rebuild"


if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else "go2_hongkong_office")
