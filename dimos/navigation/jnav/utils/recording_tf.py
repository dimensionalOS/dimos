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

"""``StreamTF`` that caches a whole recording's tf in one pass, with edge override."""

from __future__ import annotations

from collections.abc import Callable
import math
from typing import Any

import numpy as np

from dimos.memory2.tf import StreamTF
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.tf2_msgs.TFMessage import TFMessage


class RecordingTF(StreamTF):
    """``StreamTF`` that caches the whole recording's tf in one pass.

    ``StreamTF`` is built for live/windowed use: each lookup keeps only a window
    around the query time and evicts everything else. Recordings often publish
    near-static frames (sensor mounts, ``world->map``) exactly once at the start
    of the ``tf`` stream rather than into ``tf_static``, so a windowed lookup at
    any later time drops those edges and the transform chain breaks. Loading the
    full stream once keeps them buffered for the entire run; the only time-varying
    edge (``odom->base_link``) is densely sampled, so a nearest lookup
    (``time_tolerance=None``) reproduces the pose at each query time.

    An optional edge override replaces one edge with a fed trajectory so tf
    reflects the poses being scored. It re-parents the child (drops every existing
    edge into that child) so the fed edge is the only route to it, matching a
    shortest-path lookup. The override is applied lazily on first lookup, so
    recordings whose scans are already world-framed never pay for it. Provide it
    at construction (``odom_tf`` + ``odom_stream``) or later via ``override_edge``.
    """

    _override_edge: tuple[str, str] | None = None
    _override_transforms: Callable[[], list[Transform]] | None = None

    @classmethod
    def from_store(
        cls,
        store: Any,
        stream: str = "tf",
        *,
        odom_tf: str | None = None,
        odom_stream: str | None = None,
    ) -> RecordingTF | None:
        if stream not in store.list_streams():
            return None
        recording_tf = cls(store.stream(stream, TFMessage))
        if odom_tf and odom_stream and odom_stream in store.list_streams():
            parent, _, child = odom_tf.partition(":")
            odom_observations = store.stream(odom_stream, Odometry)
            recording_tf._override_edge = (parent, child)
            recording_tf._override_transforms = lambda: [
                Transform(
                    translation=observation.data.position,
                    rotation=observation.data.orientation,
                    frame_id=parent,
                    child_frame_id=child,
                    ts=float(observation.ts),
                )
                for observation in odom_observations
            ]
        return recording_tf

    def override_edge(self, parent: str, child: str, times: np.ndarray, poses: np.ndarray) -> None:
        """Replace the ``parent->child`` edge with a sampled trajectory.

        ``times`` is ``(N,)`` and ``poses`` is ``(N, 7)`` as ``[x, y, z, qx, qy, qz, qw]``.
        Re-parents ``child`` so the fed edge is its only parent. Applied immediately if
        the stream is already loaded, otherwise on the next lookup.
        """
        times = np.asarray(times, float)
        poses = np.asarray(poses, float)
        transforms = [
            Transform(
                translation=Vector3(float(row[0]), float(row[1]), float(row[2])),
                rotation=Quaternion(float(row[3]), float(row[4]), float(row[5]), float(row[6])),
                frame_id=parent,
                child_frame_id=child,
                ts=float(sample_ts),
            )
            for sample_ts, row in zip(times, poses, strict=True)
        ]
        self._override_edge = (parent, child)
        self._override_transforms = lambda: transforms
        if self._covered is not None:
            with self._cv:
                self._apply_override()

    @property
    def frames(self) -> set[str]:
        """Every frame name in the (loaded) tf tree."""
        self._ensure(0.0, 0.0)
        names: set[str] = set()
        for parent, child in self.buffers:
            names.add(parent)
            names.add(child)
        return names

    def _ensure(self, lo: float, hi: float) -> None:
        # Ignores the [lo, hi] window on purpose: load the whole recording once so
        # near-static edges published only at t=0 survive lookups at any later time.
        with self._cv:
            if self._covered is not None:
                return
            for observation in self.stream:
                self.receive_transform(*observation.data.transforms)
            if self._override_edge is not None:
                self._apply_override()
            self._covered = (-math.inf, math.inf)

    def _apply_override(self) -> None:
        _parent, child = self._override_edge  # type: ignore[misc]
        for key in [edge for edge in self.buffers if edge[1] == child]:
            self.buffers.pop(key, None)
        assert self._override_transforms is not None
        self.receive_transform(*self._override_transforms())
