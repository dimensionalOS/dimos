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

"""Object identity over detection streams: many sightings in, one per object out.

``Identity`` is the aggregation stage of a search pipeline::

    detections_2d.transform(ProjectTo3D(cloud, ...)).transform(Identity())

As a stream transformer it is a batch search processor: it consumes the full
upstream of 3D detections, groups the sightings of one physical object, and
then emits one merged :class:`Detection3DPC` per object - the union cloud of
every viewpoint that saw it. What counts as "the same object" is a pluggable
``is_same(a, b)`` strategy; v0 is spatial only ("is it roughly at the same
spot"), so there is no permanence: an object that moved registers as a new
object at its new rest position.

The grouping core (``add`` plus ``groups``) is usable directly for callers
that need the members of each identity rather than the merged stream output;
``localize`` forms its support candidates with it.
"""

from __future__ import annotations

import operator
from typing import TYPE_CHECKING, Any

from dimos.memory.transform import Transformer
from dimos.perception.detection.type.detection3d.pointcloud import Detection3DPC
from dimos.perception.detection.type.imageDetections import ImageDetections

if TYPE_CHECKING:
    from collections.abc import Callable, Iterator

    from dimos.memory.type.observation import Observation


def spatial(radius: float = 0.1) -> Callable[[Detection3DPC, Detection3DPC], bool]:
    """Same object when the cloud centers sit within *radius* meters."""

    def is_same(a: Detection3DPC, b: Detection3DPC) -> bool:
        return float((a.center - b.center).magnitude()) <= radius

    return is_same


class Identity(Transformer[Any, Detection3DPC]):
    """One detection3D per object, aggregated from every sighting.

    Each incoming detection is matched against the running merged
    representative of every known object with ``is_same``; a match joins
    that object and folds into its representative with ``merge``, otherwise
    it founds a new object. Upstream observations may carry a single
    :class:`Detection3DPC` or a per-frame :class:`ImageDetections` batch.
    """

    def __init__(
        self,
        is_same: Callable[[Detection3DPC, Detection3DPC], bool] | None = None,
        merge: Callable[[Detection3DPC, Detection3DPC], Detection3DPC] | None = None,
    ) -> None:
        self.is_same = is_same or spatial()
        self.merge = merge or operator.add
        self.groups: list[list[Detection3DPC]] = []
        self.merged: list[Detection3DPC] = []

    def add(self, detection: Detection3DPC) -> int:
        """Assign one sighting to its object; returns the object's index."""
        for index, representative in enumerate(self.merged):
            if self.is_same(representative, detection):
                self.groups[index].append(detection)
                self.merged[index] = self.merge(representative, detection)
                return index
        self.groups.append([detection])
        self.merged.append(detection)
        return len(self.groups) - 1

    def __call__(
        self, upstream: Iterator[Observation[Any]]
    ) -> Iterator[Observation[Detection3DPC]]:
        template: Observation[Any] | None = None
        for obs in upstream:
            template = obs
            data = obs.data
            for detection in data if isinstance(data, ImageDetections) else [data]:
                self.add(detection)
        if template is None:
            return
        for merged in self.merged:
            yield template.derive(data=merged, ts=merged.ts, pose=merged.pose)
