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
from scipy.spatial import cKDTree

from dimos.core.core import rpc
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.perception.localize.module import PerceptionLocalize, PerceptionLocalizeConfig
from dimos.perception.localize.types import Localization


class ManipulationLocalizeConfig(PerceptionLocalizeConfig):
    # scene_cloud: cell size of the merged cloud, and the farthest depth it keeps, metres
    scene_voxel_m: float = 0.01
    scene_depth_trunc_m: float = 2.0
    # scene_cloud(exclude=...) drops scene points within this distance (m) of that object's cloud
    scene_exclude_m: float = 0.01


class ManipulationLocalize(PerceptionLocalize):
    """PerceptionLocalize that keeps the objects its answers found, by id and label.

    A ``localize`` or ``scan`` answer replaces the objects of the labels it was
    asked for and keeps the others. Pick and place reads them by id, or by a
    label only one object has. A forgotten object's older sightings are left out
    of later answers.
    """

    config: ManipulationLocalizeConfig

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._objects: dict[str, tuple[str, Localization]] = {}
        # label, position and the newest frame time of each forgotten object
        self._gone: list[tuple[str, np.ndarray, float]] = []
        # the window of the last answer, and its merged scene cloud once asked for
        self._window: tuple[float, float] | None = None
        self._window_scene: PointCloud2 | None = None

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
        """Drop the object with this id: it is no longer where it was seen.

        Later answers leave out a sighting of its label near that place that is
        not newer than now, so the old place does not come back as a target.
        """
        entry = self._objects.pop(object_id, None)
        if entry is not None:
            label, hit = entry
            _, head = self.index.get_time_range()
            self._gone.append((label, np.asarray(hit.position_world_xyz), head))

    @rpc
    def scene_cloud(
        self, start: float = -5.0, duration: float = 5.0, exclude: str | None = None
    ) -> PointCloud2 | None:
        """Every embedded depth view in a window of memory, merged into one world-frame cloud.

        ``start`` and ``duration`` name the window as in ``localize``. The
        cloud is ``scene_voxel_m`` cells, cut at ``scene_depth_trunc_m``.
        None when nothing in the window has depth.

        ``exclude`` is the id of a localized object: the window is then the one
        of the last answer, and the object's own points are left out, with a
        floor patch under it.
        """
        if exclude is not None:
            return self._scene_without(exclude)
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
        self._window = (window[0], window[1])
        self._window_scene = None
        results = [
            [hit for hit in hits if not self._was_taken(query, hit)]
            for query, hits in zip(queries, results, strict=True)
        ]
        # An id names an object in one window only, so a label asked again starts over.
        kept = {i: entry for i, entry in self._objects.items() if entry[0] not in queries}
        self._objects = kept | {
            hit.instance_id: (query, hit)
            for query, hits in zip(queries, results, strict=True)
            for hit in hits
        }
        return results, window

    def _scene_without(self, object_id: str) -> PointCloud2 | None:
        """The last answer's window merged into one cloud, without this object's own points.

        Points within ``scene_exclude_m`` of the object's cloud are dropped,
        except those within 2 mm of the support plane. A patch of that plane,
        2 mm below it, is added under the object's footprint grown by 4 cm.
        """
        if not self._ready.is_set() or self._window is None:
            return None
        voxel = self.config.scene_voxel_m
        if self._window_scene is None:
            self._window_scene = self._merge_window(*self._window)
        merged = self._window_scene
        entry = self._objects.get(object_id)
        if merged is None or entry is None or self.config.scene_exclude_m <= 0:
            return merged
        own = entry[1].point_cloud.points_f32()
        points = merged.points_f32()
        if not len(own) or not len(points):
            return merged
        distance, _ = cKDTree(own).query(points)
        plane = next(iter(self.rig.plane_cache.values()), None)
        if plane is None:
            points = points[distance > self.config.scene_exclude_m]
        else:
            heights = plane.height_above(points)
            points = points[(distance > self.config.scene_exclude_m) | (heights <= 0.002)]
            low_xy = own[:, :2].min(axis=0) - 0.04
            high_xy = own[:, :2].max(axis=0) + 0.04
            gx, gy = np.meshgrid(
                np.arange(low_xy[0], high_xy[0], voxel),
                np.arange(low_xy[1], high_xy[1], voxel),
            )
            a, b, c, d = plane.coefficients
            gz = -(a * gx + b * gy + d + 0.002) / c
            patch = np.stack([gx.ravel(), gy.ravel(), gz.ravel()], axis=1).astype(np.float32)
            points = np.vstack([points, patch])
        return PointCloud2.from_numpy(points, frame_id=merged.frame_id, timestamp=merged.ts)

    def _merge_window(self, lo: float, hi: float) -> PointCloud2 | None:
        """The depth frames of a window as one world-frame cloud."""
        voxel = self.config.scene_voxel_m
        parts: list[np.ndarray] = []
        newest = 0.0
        single_frame = next(iter(self.rig.cameras)) if len(self.rig.cameras) == 1 else None
        for obs in self._geometry_memory.time_range(lo, hi):
            frame_id = single_frame or str(getattr(obs.data, "frame_id", ""))
            cloud = self.rig.backdrop(obs.ts, frame_id, self.config.scene_depth_trunc_m)
            if cloud is None:
                continue
            pts = cloud.voxel_downsample(voxel).points_f32()
            if len(pts):
                parts.append(pts)
                newest = max(newest, obs.ts)
        if not parts:
            return None
        return PointCloud2.from_numpy(
            np.vstack(parts), frame_id=self.rig.world_frame, timestamp=newest
        ).voxel_downsample(voxel)

    def _was_taken(self, label: str, hit: Localization) -> bool:
        """Whether this sighting is of a forgotten object, at its old place, from before."""
        radius = self.config.params.cluster_radius_m
        position = np.asarray(hit.position_world_xyz)
        return any(
            gone_label == label
            and float(np.linalg.norm(position - place)) < radius
            and hit.last_seen_timestamp <= cutoff
            for gone_label, place, cutoff in self._gone
        )

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
