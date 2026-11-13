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

"""Localize for manipulation: keeps what localize found, so a pick can name it."""

from __future__ import annotations

from typing import Any

import numpy as np

from dimos.core.core import rpc
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.perception.localize.module import PerceptionLocalize, PerceptionLocalizeConfig
from dimos.perception.localize.types import Localization


class ManipulationLocalizeConfig(PerceptionLocalizeConfig):
    # scene_cloud: cell size of the merged cloud, and the farthest depth it keeps, metres
    scene_voxel_m: float = 0.01
    scene_depth_trunc_m: float = 2.0


class ManipulationLocalize(PerceptionLocalize):
    """PerceptionLocalize that keeps the objects its answers found, by id and label.

    A ``localize`` or ``scan`` answer replaces the objects of the labels it was
    asked for and keeps the others. Pick and place reads them by id, or by a
    label only one object has.
    """

    config: ManipulationLocalizeConfig

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._objects: dict[str, tuple[str, Localization]] = {}

    @rpc
    def scan(self, names: list[str], window_s: float = 10.0) -> list[dict[str, Any]]:
        """Find every instance of each name in the last ``window_s`` seconds of memory.

        Each entry has ``id``, ``label``, ``position`` and ``size`` (world frame,
        metres), ``score``, ``views`` and ``seen_s_ago``. Raises RuntimeError
        until ``state`` reads ready.
        """
        if not self._ready.is_set():
            raise RuntimeError(f"localize cannot answer yet: {self._stage}")
        results, (_, _, _, head, _) = self._localize(names, -window_s, window_s, None)
        found = []
        for label, hits in zip(names, results, strict=True):
            for hit in hits:
                points = hit.point_cloud.points_f32()
                size = (points.max(axis=0) - points.min(axis=0)) if len(points) else np.zeros(3)
                found.append(
                    {
                        "id": hit.instance_id,
                        "label": label,
                        "position": [float(v) for v in hit.position_world_xyz],
                        "size": [float(v) for v in size],
                        "score": float(hit.semantic_score),
                        "views": hit.n_views,
                        "seen_s_ago": max(0.0, head - hit.last_seen_timestamp),
                    }
                )
        return found

    @rpc
    def get_pointcloud(self, target: str) -> PointCloud2:
        """The world-frame cloud of one localized object: an id, or a label only one has."""
        return self._resolve(target).point_cloud

    @rpc
    def get_position(self, target: str) -> Vector3:
        """The world-frame position of one localized object, found as in ``get_pointcloud``."""
        return Vector3(*self._resolve(target).position_world_xyz)

    @rpc
    def get_id(self, target: str) -> str:
        """The id of one localized object, found as in ``get_pointcloud``."""
        return self._resolve(target).instance_id

    @rpc
    def forget(self, object_id: str) -> None:
        """Drop the object with this id: it is no longer where it was seen."""
        self._objects.pop(object_id, None)

    @rpc
    def scene_cloud(self, start: float = -5.0, duration: float = 5.0) -> PointCloud2 | None:
        """Every embedded depth view in a window of memory, merged into one world-frame cloud.

        ``start`` and ``duration`` name the window as in ``localize``. The
        cloud is ``scene_voxel_m`` cells, cut at ``scene_depth_trunc_m``.
        None when nothing in the window has depth.
        """
        if not self._ready.is_set() or self.config.geometry != "depth":
            return None
        first, head = self.index.get_time_range()
        lo = max(first, head + start if start < 0 else first + start)
        views = []
        for obs in self.index.time_range(lo, lo + duration):
            for frame in self.rig.cameras:
                cloud = self.rig.backdrop(obs.ts, frame, self.config.scene_depth_trunc_m)
                if cloud is not None and len(cloud):
                    views.append(cloud)
        if not views:
            return None
        return PointCloud2.from_numpy(
            np.concatenate([view.points_f32() for view in views]),
            frame_id=self.rig.world_frame,
            timestamp=max((view.ts or 0.0) for view in views),
        ).voxel_downsample(self.config.scene_voxel_m)

    def _localize(
        self,
        queries: list[str],
        start: float,
        duration: float,
        overrides: dict[str, Any] | None,
    ) -> tuple[list[list[Localization]], tuple[float, float, float, float, int]]:
        results, window = super()._localize(queries, start, duration, overrides)
        # An id names an object in one window only, so a label asked again starts over.
        kept = {i: entry for i, entry in self._objects.items() if entry[0] not in queries}
        self._objects = kept | {
            hit.instance_id: (query, hit)
            for query, hits in zip(queries, results, strict=True)
            for hit in hits
        }
        return results, window

    def _resolve(self, target: str) -> Localization:
        """A localized object by id, or by a label exactly one of them has."""
        if target in self._objects:
            return self._objects[target][1]
        matches = [hit for label, hit in self._objects.values() if label == target]
        if len(matches) == 1:
            return matches[0]
        if matches:
            ids = ", ".join(hit.instance_id for hit in matches)
            raise ValueError(
                f"Localize found {len(matches)} {target!r}: {ids}. Pass one of these ids."
            )
        known = ", ".join(f"{label} {i}" for i, (label, _) in self._objects.items())
        if not known:
            raise LookupError(f"No {target!r}: no object is localized. Localize it first.")
        raise LookupError(f"No {target!r} among the localized objects. They are: {known}.")
