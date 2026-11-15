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

"""A PGO graph saved as standard messages in a memory store, and loaded back as a premap.

``keyframes`` is a PointCloud2 stream: each keyframe's scan in its own body
frame, the observation pose being the keyframe's pose in ``map``. ``edges`` is
an Odometry stream: ``keyframe/<a>`` to ``keyframe/<b>``, the pose of b in a
with its covariance.
"""

from __future__ import annotations

from collections import defaultdict
from dataclasses import dataclass
import itertools
import os
from pathlib import Path
from typing import TYPE_CHECKING

import numpy as np
from scipy.spatial.transform import Rotation

from dimos.memory.store.sqlite import SqliteStore
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.PoseWithCovariance import PoseWithCovariance
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.utils.data import resolve_named_path

if TYPE_CHECKING:
    from numpy.typing import NDArray


MAP_SUFFIX = ".map.db"
KEYFRAMES = "keyframes"
EDGES = "edges"
MAP_FRAME = "map"

# ROS covariance is over [x, y, z, rx, ry, rz], gtsam's Pose3 tangent over [rx, ry, rz, x, y, z];
# the same index swap goes either way
ROS_GTSAM_SWAP = np.array([3, 4, 5, 0, 1, 2])


def keyframe_frame(i: int) -> str:
    return f"keyframe/{i}"


def _keyframe_index(frame_id: str) -> int:
    return int(frame_id.rsplit("/", 1)[1])


def pose_matrix(pose: Pose) -> NDArray[np.float64]:
    m = np.eye(4)
    q = pose.orientation
    m[:3, :3] = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
    m[:3, 3] = [pose.position.x, pose.position.y, pose.position.z]
    return m


def matrix_pose(m: NDArray[np.float64]) -> Pose:
    q = Rotation.from_matrix(m[:3, :3]).as_quat()
    return Pose(*m[:3, 3], *q)


def apply(m: NDArray[np.float64], pts: NDArray[np.floating]) -> NDArray[np.float64]:
    return np.asarray(pts, dtype=np.float64) @ m[:3, :3].T + m[:3, 3]


# A keyframe this close to a younger one taken at least this much later is redundant.
PRUNE_RADIUS_M = 0.5
PRUNE_MIN_GAP_S = 20.0

Edge = tuple[int, int, np.ndarray, np.ndarray]


def _adjoint(m: NDArray[np.float64]) -> NDArray[np.float64]:
    """The pose's adjoint for ROS-order [x, y, z, rx, ry, rz] tangent vectors."""
    r, p = m[:3, :3], m[:3, 3]
    skew = np.array([[0.0, -p[2], p[1]], [p[2], 0.0, -p[0]], [-p[1], p[0], 0.0]])
    ad = np.zeros((6, 6))
    ad[:3, :3], ad[:3, 3:], ad[3:, 3:] = r, skew @ r, r
    return ad


def _inverse(m: NDArray[np.float64], c: NDArray[np.float64]) -> tuple[np.ndarray, np.ndarray]:
    ad = _adjoint(m)
    return np.linalg.inv(m), ad @ c @ ad.T


def _compose(
    ab: tuple[np.ndarray, np.ndarray], bc: tuple[np.ndarray, np.ndarray]
) -> tuple[np.ndarray, np.ndarray]:
    """a -> c from a -> b and b -> c, the first covariance carried into c's frame."""
    (m1, c1), (m2, c2) = ab, bc
    ad = _adjoint(np.linalg.inv(m2))
    return m1 @ m2, ad @ c1 @ ad.T + c2


def splice(edges: list[Edge], drop: list[int]) -> tuple[list[Edge], set[int]]:
    """``edges`` without those ``drop`` keyframes that come out exactly, and which those were.

    A keyframe with two edges is replaced by their composition, one with a single
    edge by nothing; both keep the graph's optimum. A keyframe with more edges
    stays, since pairwise edges cannot carry what it ties together.
    """
    live = dict(enumerate(edges))
    at: dict[int, set[int]] = defaultdict(set)  # keyframe -> its edges
    for e, (a, b, _, _) in live.items():
        at[a].add(e)
        at[b].add(e)
    new_key = itertools.count(len(edges))
    dropped: set[int] = set()
    for r in drop:
        if len(at[r]) > 2:
            continue
        # each edge at r, as (neighbour, pose of r in the neighbour, covariance)
        around = []
        for e in at.pop(r):
            a, b, m, c = live.pop(e)
            other = b if a == r else a
            at[other].discard(e)
            around.append((a, m, c) if b == r else (b, *_inverse(m, c)))
        dropped.add(r)
        if len(around) == 2 and around[0][0] != around[1][0]:
            (n1, r_in_n1, c1), (n2, r_in_n2, c2) = around
            m, c = _compose((r_in_n1, c1), _inverse(r_in_n2, c2))
            e = next(new_key)
            live[e] = (n1, n2, m, c)
            at[n1].add(e)
            at[n2].add(e)
    return list(live.values()), dropped


@dataclass(frozen=True)
class Premap:
    """Keyframe poses in ``map``, each keyframe's body-frame scan, and the graph's edges."""

    stamps: NDArray[np.float64]  # (K,)
    poses: NDArray[np.float64]  # (K, 4, 4) keyframe in map
    scans: tuple[NDArray[np.float32], ...]  # (N_i, 3) each, body frame
    edges: NDArray[np.int64]  # (E, 2) keyframe a, keyframe b
    edge_poses: NDArray[np.float64]  # (E, 4, 4) b in a
    edge_covariances: NDArray[np.float64]  # (E, 6, 6) ROS order

    def __len__(self) -> int:
        return len(self.stamps)

    def cloud(self, voxel_size: float | None = None) -> PointCloud2:
        """Every scan placed at its keyframe, in ``map``."""
        pts = [apply(m, s) for m, s in zip(self.poses, self.scans, strict=True) if len(s)]
        merged = np.concatenate(pts) if pts else np.empty((0, 3))
        cloud = PointCloud2.from_numpy(merged.astype(np.float32), frame_id=MAP_FRAME)
        return cloud if voxel_size is None else cloud.voxel_downsample(voxel_size)

    def pruned(
        self, radius_m: float = PRUNE_RADIUS_M, min_gap_s: float = PRUNE_MIN_GAP_S
    ) -> Premap:
        """Without the keyframes that are no longer needed; keyframe 0 stays as the pin.

        A keyframe hands its points to a younger one taken ``min_gap_s`` later
        within ``radius_m`` of it. Keyframes left owning no points are spliced out
        where that is exact (`splice`); the rest stay as nodes without points.
        """
        poses, scans = self.poses, list(self.scans)
        # index order is the order taken; a session stamped earlier than the one before it still came later
        steps = np.diff(self.stamps)
        age = np.concatenate([[0.0], np.cumsum(np.where(steps < 0, min_gap_s, steps))])
        for k in range(1, len(self)):
            younger = np.arange(k + 1, len(self))
            younger = younger[age[younger] - age[k] >= min_gap_s]
            near = younger[
                np.linalg.norm(poses[younger, :3, 3] - poses[k, :3, 3], axis=1) <= radius_m
            ]
            if len(near) and len(scans[k]):
                heir = int(near[-1])
                moved = apply(np.linalg.inv(poses[heir]) @ poses[k], scans[k]).astype(np.float32)
                scans[heir] = np.concatenate([scans[heir], moved])
                scans[k] = scans[k][:0]
        empty = [k for k in range(1, len(self)) if not len(scans[k])]
        edges, drop = splice(
            [
                (int(a), int(b), m, c)
                for (a, b), m, c in zip(
                    self.edges, self.edge_poses, self.edge_covariances, strict=True
                )
            ],
            empty,
        )
        keep = np.array([k for k in range(len(self)) if k not in drop], dtype=int)
        index = {int(k): i for i, k in enumerate(keep)}
        kept = [(index[a], index[b], m, c) for a, b, m, c in edges]
        return Premap(
            stamps=self.stamps[keep],
            poses=self.poses[keep],
            scans=tuple(scans[k] for k in keep),
            edges=np.array([(a, b) for a, b, _, _ in kept], dtype=np.int64).reshape(-1, 2),
            edge_poses=np.array([m for _, _, m, _ in kept]).reshape(-1, 4, 4),
            edge_covariances=np.array([c for _, _, _, c in kept]).reshape(-1, 6, 6),
        )

    def save(self, path: str | Path) -> None:
        """Write to a temporary file and swap it in, so a crash mid-write leaves the old one whole."""
        path = Path(path)
        tmp = path.with_name(path.name + ".tmp")
        tmp.unlink(missing_ok=True)
        with SqliteStore(path=str(tmp)) as store:
            keyframes = store.stream(KEYFRAMES, PointCloud2)
            for i, (ts, m, scan) in enumerate(
                zip(self.stamps, self.poses, self.scans, strict=True)
            ):
                cloud = PointCloud2.from_numpy(scan, frame_id=keyframe_frame(i), timestamp=ts)
                keyframes.append(cloud, ts=float(ts), pose=matrix_pose(m))
            edges = store.stream(EDGES, Odometry)
            for (a, b), m, cov in zip(
                self.edges, self.edge_poses, self.edge_covariances, strict=True
            ):
                ts = float(self.stamps[b])
                edge = Odometry(
                    ts=ts,
                    frame_id=keyframe_frame(int(a)),
                    child_frame_id=keyframe_frame(int(b)),
                    pose=PoseWithCovariance(matrix_pose(m), cov.reshape(-1)),
                )
                edges.append(edge, ts=ts, pose=None)
        os.replace(tmp, path)

    @classmethod
    def load(cls, name: str | Path) -> Premap:
        """A premap by path, stem, or LFS name; ``.map.db`` is appended if absent."""
        with SqliteStore(path=str(resolve_named_path(name, MAP_SUFFIX)), must_exist=True) as store:
            # by keyframe index, not stamp: a joint map's sessions can interleave in time
            loaded = [(o, o.data) for o in store.stream(KEYFRAMES, PointCloud2)]
            loaded.sort(key=lambda pair: _keyframe_index(pair[1].frame_id))
            kfs = [o for o, _ in loaded]
            scans = tuple(cloud.points_f32() for _, cloud in loaded)
            odom = (
                [o.data for o in store.stream(EDGES, Odometry)]
                if EDGES in store.list_streams()
                else []
            )
        poses = [pose_matrix(o.pose) for o in kfs if o.pose is not None]
        assert len(poses) == len(kfs), "every keyframe carries its pose"
        return cls(
            stamps=np.array([o.ts for o in kfs]),
            poses=np.array(poses).reshape(-1, 4, 4),
            scans=scans,
            edges=np.array(
                [(_keyframe_index(e.frame_id), _keyframe_index(e.child_frame_id)) for e in odom],
                dtype=np.int64,
            ).reshape(-1, 2),
            edge_poses=np.array([pose_matrix(e.pose.pose) for e in odom]).reshape(-1, 4, 4),
            edge_covariances=np.array([e.pose.covariance.reshape(6, 6) for e in odom]).reshape(
                -1, 6, 6
            ),
        )
