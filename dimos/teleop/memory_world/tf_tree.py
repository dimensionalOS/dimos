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

"""A tf tree read from a recording, answering "where was frame X at time t".

Every transform in the recording's ``tf`` stream is kept, per parent-child
edge, as a time-sorted series. A lookup walks the tree between two frames and
composes each edge at the requested time: translation interpolated linearly,
rotation by slerp, between the two bracketing samples. `static` is a FLAG, set for edges
that arrived on tf_static -- nothing inspects the samples to decide it, so an edge whose
values never change is not static, and an edge published once is not either: it answers
from its single sample forever forward, but only within `tolerance_s` backwards. `span`
documents that rule and `recording.fold_static_tf` depends on it.

Camera frames, sensor-relative lidar scans and the robot's path are placed by
asking this tree. A scan already in the world frame (a corrected lidar stream)
keeps its own stamped pose, and a frame the tree does not know falls back to
the pose its observation carries.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass, field
from itertools import pairwise
import math
from typing import TYPE_CHECKING, Any

import numpy as np

if TYPE_CHECKING:
    from collections.abc import Iterable


def pose_matrix(
    position: tuple[float, float, float] | np.ndarray,
    orientation: tuple[float, float, float, float] | np.ndarray,
) -> np.ndarray:
    """4x4 homogeneous matrix from a position and an (x, y, z, w) quaternion."""
    x, y, z, w = orientation
    matrix = np.eye(4)
    matrix[:3, :3] = [
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ]
    matrix[:3, 3] = position
    return matrix


def quaternion_from_matrix(rotation: np.ndarray) -> tuple[float, float, float, float]:
    """(x, y, z, w) quaternion of a 3x3 rotation matrix (Shepperd's method)."""
    m = rotation
    trace = float(m[0, 0] + m[1, 1] + m[2, 2])
    if trace > 0:
        s = np.sqrt(trace + 1.0) * 2
        return (
            float((m[2, 1] - m[1, 2]) / s),
            float((m[0, 2] - m[2, 0]) / s),
            float((m[1, 0] - m[0, 1]) / s),
            float(0.25 * s),
        )
    if m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = np.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2
        return (
            float(0.25 * s),
            float((m[0, 1] + m[1, 0]) / s),
            float((m[0, 2] + m[2, 0]) / s),
            float((m[2, 1] - m[1, 2]) / s),
        )
    if m[1, 1] > m[2, 2]:
        s = np.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2
        return (
            float((m[0, 1] + m[1, 0]) / s),
            float(0.25 * s),
            float((m[1, 2] + m[2, 1]) / s),
            float((m[0, 2] - m[2, 0]) / s),
        )
    s = np.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2
    return (
        float((m[0, 2] + m[2, 0]) / s),
        float((m[1, 2] + m[2, 1]) / s),
        float(0.25 * s),
        float((m[1, 0] - m[0, 1]) / s),
    )


def slerp(q0: np.ndarray, q1: np.ndarray, t: float) -> np.ndarray:
    """Spherical interpolation between two (x, y, z, w) quaternions."""
    dot = float(np.dot(q0, q1))
    if dot < 0.0:
        q1, dot = -q1, -dot
    if dot > 0.9995:
        out = q0 + t * (q1 - q0)
        return np.asarray(out / np.linalg.norm(out))
    theta = np.arccos(dot) * t
    q2 = q1 - q0 * dot
    q2 /= np.linalg.norm(q2)
    return np.asarray(q0 * np.cos(theta) + q2 * np.sin(theta))


@dataclass
class _Edge:
    """The time series of one parent->child transform."""

    stamps: list[float] = field(default_factory=list)
    positions: list[tuple[float, float, float]] = field(default_factory=list)
    orientations: list[tuple[float, float, float, float]] = field(default_factory=list)
    _stamp_array: np.ndarray | None = None
    _position_array: np.ndarray | None = None
    _orientation_array: np.ndarray | None = None
    static: bool = False  # tf_static: one sample, valid for all time

    def add(
        self,
        ts: float,
        position: tuple[float, float, float],
        orientation: tuple[float, float, float, float],
    ) -> None:
        self.stamps.append(ts)
        self.positions.append(position)
        self.orientations.append(orientation)
        self._stamp_array = None

    def _arrays(self) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        if self._stamp_array is None:
            order = np.argsort(self.stamps, kind="stable")
            # Positions and orientations first, the stamps last: another thread's
            # lookup takes the stamps as the sign that all three are there.
            self._position_array = np.asarray(self.positions, dtype=np.float64)[order]
            self._orientation_array = np.asarray(self.orientations, dtype=np.float64)[order]
            self._stamp_array = np.asarray(self.stamps, dtype=np.float64)[order]
        assert self._position_array is not None and self._orientation_array is not None
        return self._stamp_array, self._position_array, self._orientation_array

    @property
    def span(self) -> tuple[float, float]:
        if self.static:
            return -math.inf, math.inf
        stamps, _, _ = self._arrays()
        if len(stamps) == 1:  # published once: held from then on, as at() does
            return float(stamps[0]), math.inf
        return float(stamps[0]), float(stamps[-1])

    def at(self, ts: float, tolerance_s: float) -> np.ndarray | None:
        """parent_T_child at *ts*, interpolated between the bracketing samples.

        Outside the series by more than *tolerance_s* the answer is None; within
        it, the nearest end is held.
        """
        stamps, positions, orientations = self._arrays()
        if self.static:
            return pose_matrix(positions[0], orientations[0])
        if len(stamps) == 1 or ts <= stamps[0]:
            if ts < stamps[0] - tolerance_s:
                return None
            return pose_matrix(positions[0], orientations[0])
        if ts >= stamps[-1]:
            if ts > stamps[-1] + tolerance_s:
                return None
            return pose_matrix(positions[-1], orientations[-1])
        hi = int(np.searchsorted(stamps, ts, side="right"))
        lo = hi - 1
        gap = stamps[hi] - stamps[lo]
        fraction = float((ts - stamps[lo]) / gap) if gap > 0 else 0.0
        position = positions[lo] * (1 - fraction) + positions[hi] * fraction
        orientation = slerp(orientations[lo], orientations[hi], fraction)
        return pose_matrix(position, orientation)


class TfTree:
    """Every transform of a recording, queryable at any time."""

    def __init__(self) -> None:
        self._edges: dict[tuple[str, str], _Edge] = {}
        self._neighbours: dict[str, set[str]] = {}

    @classmethod
    def from_stream(cls, stream: Iterable[Any]) -> TfTree:
        """Load a ``tf`` stream of ``TFMessage`` observations."""
        tree = cls()
        for obs in stream:
            for transform in obs.data.transforms:
                t, r = transform.translation, transform.rotation
                tree.add(
                    str(transform.frame_id),
                    str(transform.child_frame_id),
                    float(getattr(transform, "ts", 0.0) or obs.ts),  # its own stamp when it has one
                    (float(t.x), float(t.y), float(t.z)),
                    (float(r.x), float(r.y), float(r.z), float(r.w)),
                )
        return tree

    def add(
        self,
        parent: str,
        child: str,
        ts: float,
        position: tuple[float, float, float],
        orientation: tuple[float, float, float, float],
        static: bool = False,
    ) -> None:
        edge = self._edges.setdefault((parent, child), _Edge())
        if static:
            if edge.static:  # a latched tf_static republished: the one sample holds
                return
            if edge.stamps:
                # tf also carried this edge, over whatever window it happened to be
                # published. tf_static says it holds for all time, and that is the
                # stronger claim, so it replaces rather than joins.
                edge.stamps.clear()
                edge.positions.clear()
                edge.orientations.clear()
                edge._stamp_array = None
            edge.static = True
        edge.add(ts, position, orientation)
        self._neighbours.setdefault(parent, set()).add(child)
        self._neighbours.setdefault(child, set()).add(parent)

    @property
    def frames(self) -> set[str]:
        return set(self._neighbours)

    def __len__(self) -> int:
        return sum(len(edge.stamps) for edge in self._edges.values())

    def span(self, target: str, source: str) -> tuple[float, float] | None:
        """The time range over which every edge between the two frames has data."""
        path = self._path(target, source)
        if path is None:
            return None
        if not path:  # the same frame: `lookup` returns identity for this, so do not raise
            return (-math.inf, math.inf)
        starts, ends = [], []
        for parent, child, _ in path:
            start, end = self._edges[(parent, child)].span
            starts.append(start)
            ends.append(end)
        return max(starts), min(ends)

    def lookup(
        self, target: str, source: str, ts: float, tolerance_s: float = 0.1
    ) -> np.ndarray | None:
        """target_T_source at *ts*: the matrix taking *source*-frame points into *target*.

        None when the frames are not connected, or when any edge on the way
        has no sample within *tolerance_s* of *ts*.
        """
        if target == source:
            return np.eye(4)
        path = self._path(target, source)
        if path is None:
            return None
        matrix = np.eye(4)
        for parent, child, forward in path:
            step = self._edges[(parent, child)].at(ts, tolerance_s)
            if step is None:
                return None
            matrix = matrix @ (step if forward else np.linalg.inv(step))
        return matrix

    def _path(self, target: str, source: str) -> list[tuple[str, str, bool]] | None:
        """Edges from *target* to *source*; ``forward`` says the edge runs parent->child that way."""
        if target not in self._neighbours or source not in self._neighbours:
            return None
        previous: dict[str, str | None] = {target: None}
        queue = deque([target])
        while queue:
            frame = queue.popleft()
            if frame == source:
                break
            # sorted, not the set's own order: two routes of equal length between the same
            # pair of frames are both shortest, and a set iterates in an order that depends
            # on string hashing, which is randomised PER PROCESS. The same recording then
            # placed its map one way on one run and the other way on the next, with nothing
            # in the recording or the code having changed.
            for neighbour in sorted(self._neighbours[frame]):
                if neighbour not in previous:
                    previous[neighbour] = frame
                    queue.append(neighbour)
        if source not in previous:
            return None
        hops: list[str] = []
        cursor: str | None = source
        while cursor is not None:
            hops.append(cursor)
            cursor = previous[cursor]
        hops.reverse()
        path = []
        for a, b in pairwise(hops):
            forward = (a, b) in self._edges
            path.append((a, b, True) if forward else (b, a, False))
        return path
