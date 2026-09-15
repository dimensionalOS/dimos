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

"""A transform buffer that answers a whole query's worth of lookups at once.

`MultiTBuffer` answers one moment at a time, which is the right shape for a robot
asking "where am I now" and the wrong one for a query placing thousands of patches: at
141 us a call, six thousand of them is 0.85 s against a 100 ms budget.

Here each edge is one growing ``(N, 8)`` array -- timestamp, translation, quaternion --
so a batch of moments is answered by one `searchsorted` and one vectorised interpolation
per edge, whatever the batch size. Transforms are written a slice at a time, since a
`TFMessage` carries a dozen at once and a scalar write per transform costs more than the
bulk conversion it saves.

`reform_edge` is why the values live in a mutable array rather than being frozen at
ingest: a loop closure corrects transforms that have already been used, and every answer
computed after it must reflect the correction. It only fixes what is in memory -- the
correction has to reach the recording separately, or a restart reads the old value back.
"""

from __future__ import annotations

import threading
from typing import TYPE_CHECKING

import numpy as np

from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3

if TYPE_CHECKING:
    from collections.abc import Iterable, Sequence

    from numpy.typing import NDArray

    from dimos.msgs.tf2_msgs.TFMessage import TFMessage

# ts, x, y, z, qx, qy, qz, qw
WIDTH = 8
FIRST = 16  # rows in a fresh edge, doubled from there


class Edge:
    """One parent->child transform over time, as a doubling ``(capacity, 8)`` array.

    ``count`` is the boundary between real rows and reserved space: everything below it
    is a transform that arrived, everything above is uninitialised. A boolean mask is
    the same statement and a second thing to keep true, so it is derived rather than
    stored -- see `mask`.
    """

    def __init__(self) -> None:
        self.rows: NDArray[np.float64] = np.empty((FIRST, WIDTH), dtype=np.float64)
        self.count = 0
        # Transforms normally arrive in order; one that does not forces a re-sort.
        self.sorted = True
        self.last = -np.inf

    @property
    def mask(self) -> NDArray[np.bool_]:
        """Which rows hold a transform rather than reserved space."""
        flags = np.zeros(len(self.rows), dtype=bool)
        flags[: self.count] = True
        return flags

    @property
    def used(self) -> NDArray[np.float64]:
        if not self.sorted:
            self.rows[: self.count] = self.rows[: self.count][
                np.argsort(self.rows[: self.count, 0], kind="stable")
            ]
            self.sorted = True
        return self.rows[: self.count]

    def reserve(self, extra: int) -> None:
        needed = self.count + extra
        if needed <= len(self.rows):
            return
        capacity = len(self.rows)
        while capacity < needed:
            capacity *= 2
        grown = np.empty((capacity, WIDTH), dtype=np.float64)
        grown[: self.count] = self.rows[: self.count]
        self.rows = grown

    def extend(self, block: Sequence[list[float]]) -> None:
        """Write a slice of transforms straight into the array.

        A row is 64 bytes and the write costs a third of a microsecond, so there is
        nothing to defer. What made a first attempt at this five times slower than the
        buffer it replaces was the ORDER CHECK: `np.diff(...)` on a one-row block costs
        3.4 us against 0.35 us for the write, and it ran once per edge per message --
        257,000 times over grocery's 16,048 messages, which is 1.1 s of the 1.24 s.
        Comparing two floats in Python does the same job for 0.04 us.
        """
        self.reserve(len(block))
        for row in block:
            stamp = row[0]
            if stamp < self.last:
                self.sorted = False
            self.last = stamp
            self.rows[self.count] = row
            self.count += 1

    def rewrite(self, stamp: float, row: NDArray[np.float64], tolerance: float) -> bool:
        """Overwrite the transform at ``stamp``. True when one was there to overwrite."""
        used = self.used
        if not len(used):
            return False
        where = int(np.abs(used[:, 0] - stamp).argmin())
        if abs(float(used[where, 0]) - stamp) > tolerance:
            return False
        self.rows[where] = row
        return True


def _row(transform: Transform) -> list[float]:
    t, r = transform.translation, transform.rotation
    return [float(transform.ts), t.x, t.y, t.z, r.x, r.y, r.z, r.w]


def _slerp(
    a: NDArray[np.float64], b: NDArray[np.float64], t: NDArray[np.float64]
) -> NDArray[np.float64]:
    """Shortest-arc quaternion interpolation, row-wise."""
    b = np.where((np.sum(a * b, axis=1) < 0)[:, None], -b, b)
    dot = np.clip(np.sum(a * b, axis=1), -1.0, 1.0)
    theta = np.arccos(dot)
    close = theta < 1e-6
    sin_theta = np.where(close, 1.0, np.sin(theta))
    wa = np.where(close, 1.0 - t, np.sin((1.0 - t) * theta) / sin_theta)
    wb = np.where(close, t, np.sin(t * theta) / sin_theta)
    out = wa[:, None] * a + wb[:, None] * b
    return out / np.linalg.norm(out, axis=1, keepdims=True)


def _matrices(
    translations: NDArray[np.float64], quaternions: NDArray[np.float64]
) -> NDArray[np.float64]:
    """``(N, 4, 4)`` from ``(N, 3)`` translations and ``(N, 4)`` xyzw quaternions."""
    x, y, z, w = quaternions.T
    out = np.zeros((len(quaternions), 4, 4), dtype=np.float64)
    out[:, 0, 0] = 1 - 2 * (y * y + z * z)
    out[:, 0, 1] = 2 * (x * y - z * w)
    out[:, 0, 2] = 2 * (x * z + y * w)
    out[:, 1, 0] = 2 * (x * y + z * w)
    out[:, 1, 1] = 1 - 2 * (x * x + z * z)
    out[:, 1, 2] = 2 * (y * z - x * w)
    out[:, 2, 0] = 2 * (x * z - y * w)
    out[:, 2, 1] = 2 * (y * z + x * w)
    out[:, 2, 2] = 1 - 2 * (x * x + y * y)
    out[:, :3, 3] = translations
    out[:, 3, 3] = 1.0
    return out


class FlexTf:
    """`MultiTBuffer`'s interface, plus `batch_get` and `reform_edge`."""

    def __init__(self, buffer_size: float = float("inf")) -> None:
        self.edges: dict[tuple[str, str], Edge] = {}
        self.buffer_size = buffer_size
        self._lock = threading.RLock()

    # --- MultiTBuffer's interface ---------------------------------------------------

    def receive_transform(self, *transforms: Transform) -> None:
        if not transforms:
            return
        with self._lock:
            grouped: dict[tuple[str, str], list[list[float]]] = {}
            for transform in transforms:
                key = (transform.frame_id, transform.child_frame_id)
                grouped.setdefault(key, []).append(_row(transform))
            for key, block in grouped.items():
                self.edges.setdefault(key, Edge()).extend(block)

    def receive_tfmessage(self, msg: TFMessage) -> None:
        self.receive_transform(*msg.transforms)

    def get_frames(self) -> set[str]:
        with self._lock:
            return {frame for edge in self.edges for frame in edge}

    def get_connections(self, frame_id: str) -> set[str]:
        with self._lock:
            found = set()
            for parent, child in self.edges:
                if parent == frame_id:
                    found.add(child)
                if child == frame_id:
                    found.add(parent)
            return found

    def get(
        self,
        parent_frame: str,
        child_frame: str,
        time_point: float | None = None,
        time_tolerance: float | None = None,
        *,
        forward_tolerance: float = 0.0,
        warn: bool = True,
    ) -> Transform | None:
        """One moment, for callers that have one. `batch_get` is the reason this exists."""
        if time_point is None:
            return None
        poses, valid = self.batch_get(parent_frame, child_frame, np.asarray([time_point]))
        if not valid[0]:
            return None
        matrix = poses[0]
        trace = matrix[0, 0] + matrix[1, 1] + matrix[2, 2]
        w = np.sqrt(max(0.0, 1.0 + trace)) / 2
        if w > 1e-6:
            x = (matrix[2, 1] - matrix[1, 2]) / (4 * w)
            y = (matrix[0, 2] - matrix[2, 0]) / (4 * w)
            z = (matrix[1, 0] - matrix[0, 1]) / (4 * w)
        else:  # 180 degrees: the largest diagonal term is the stable axis
            axis = int(np.argmax([matrix[0, 0], matrix[1, 1], matrix[2, 2]]))
            values = [0.0, 0.0, 0.0]
            values[axis] = np.sqrt(max(0.0, 1.0 + 2 * matrix[axis, axis] - trace)) / 2
            x, y, z = values
        return Transform(
            translation=Vector3(*matrix[:3, 3]),
            rotation=Quaternion(float(x), float(y), float(z), float(w)),
            frame_id=parent_frame,
            child_frame_id=child_frame,
            ts=time_point,
        )

    # --- the two additions ------------------------------------------------------------

    def chain(
        self, parent_frame: str, child_frame: str
    ) -> list[tuple[tuple[str, str], bool]] | None:
        """Edges from ``parent_frame`` down to ``child_frame``, each with a flip flag."""
        if parent_frame == child_frame:
            return []
        with self._lock:
            neighbours: dict[str, list[tuple[str, tuple[str, str], bool]]] = {}
            for key in self.edges:
                parent, child = key
                neighbours.setdefault(parent, []).append((child, key, False))
                neighbours.setdefault(child, []).append((parent, key, True))
            seen = {parent_frame}
            queue: list[tuple[str, list[tuple[tuple[str, str], bool]]]] = [(parent_frame, [])]
            while queue:
                frame, path = queue.pop(0)
                for nxt, key, flipped in neighbours.get(frame, ()):
                    if nxt in seen:
                        continue
                    step = [*path, (key, flipped)]
                    if nxt == child_frame:
                        return step
                    seen.add(nxt)
                    queue.append((nxt, step))
        return None

    def batch_get(
        self,
        parent_frame: str,
        child_frame: str | Sequence[str],
        stamps: Iterable[float],
    ) -> tuple[NDArray[np.float64], NDArray[np.bool_]]:
        """Poses for many moments at once: ``(N, 4, 4)`` and a per-entry valid mask.

        Per entry, because a stamp outside an edge's range has no answer while its
        neighbours do -- returning nothing at all for the batch would throw away every
        patch for one bad timestamp.
        """
        times = np.asarray(list(stamps), dtype=np.float64)
        sources = [child_frame] * len(times) if isinstance(child_frame, str) else list(child_frame)
        poses = np.broadcast_to(np.eye(4), (len(times), 4, 4)).copy()
        valid = np.zeros(len(times), dtype=bool)
        if not len(times):
            return poses, valid

        for source in dict.fromkeys(sources):
            picked = np.asarray([s == source for s in sources])
            chain = self.chain(parent_frame, source)
            if chain is None:
                continue
            answer = np.broadcast_to(np.eye(4), (int(picked.sum()), 4, 4)).copy()
            ok = np.ones(int(picked.sum()), dtype=bool)
            for key, flipped in chain:
                step, step_ok = self._edge_poses(key, times[picked])
                if flipped:
                    step = np.linalg.inv(step)
                answer = answer @ step
                ok &= step_ok
            poses[picked] = answer
            valid[picked] = ok
        return poses, valid

    def _edge_poses(
        self, key: tuple[str, str], times: NDArray[np.float64]
    ) -> tuple[NDArray[np.float64], NDArray[np.bool_]]:
        with self._lock:
            rows = self.edges[key].used
        stamps = rows[:, 0]
        after = np.searchsorted(stamps, times)
        before = np.clip(after - 1, 0, len(stamps) - 1)
        after = np.clip(after, 0, len(stamps) - 1)
        span = stamps[after] - stamps[before]
        fraction = np.where(span > 0, (times - stamps[before]) / np.where(span > 0, span, 1.0), 0.0)
        fraction = np.clip(fraction, 0.0, 1.0)
        translations = rows[before, 1:4] + fraction[:, None] * (
            rows[after, 1:4] - rows[before, 1:4]
        )
        quaternions = _slerp(rows[before, 4:8], rows[after, 4:8], fraction)
        reach = self.buffer_size
        gap = np.minimum(np.abs(times - stamps[before]), np.abs(stamps[after] - times))
        return _matrices(translations, quaternions), gap <= reach

    def reform_edge(
        self, parent_frame: str, child_frame: str, *transforms: Transform, tolerance: float = 1e-6
    ) -> int:
        """Correct transforms already recorded on one edge. Returns how many landed.

        A loop closure amends the past rather than adding to it, so this overwrites the
        entry at each transform's own timestamp instead of appending a correction. Only
        memory: writing the correction back to the recording is a separate job, and
        without it a restart reads the old value.
        """
        with self._lock:
            edge = self.edges.get((parent_frame, child_frame))
            if edge is None:
                return 0
            return sum(
                edge.rewrite(float(t.ts), np.asarray(_row(t), dtype=np.float64), tolerance)
                for t in transforms
            )
