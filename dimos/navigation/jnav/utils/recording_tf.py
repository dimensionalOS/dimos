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

"""Batch-friendly ``StreamTF`` for offline post-processing of a recording."""

from __future__ import annotations

import math

from dimos.memory2.tf import StreamTF


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
    """

    def _ensure(self, lo: float, hi: float) -> None:
        with self._cv:
            if self._covered is not None:
                return
            for observation in self.stream:
                self.receive_transform(*observation.data.transforms)
            self._covered = (-math.inf, math.inf)
