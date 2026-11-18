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

from dimos.agents.annotation import skill
from dimos.core.core import rpc
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.perception.localize.module import PerceptionLocalize, PerceptionLocalizeConfig
from dimos.perception.localize.rig import TF_TOLERANCE
from dimos.perception.localize.support_plane import find_surfaces, window_points
from dimos.perception.localize.types import Localization
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class ManipulationLocalizeConfig(PerceptionLocalizeConfig):
    # scene_cloud: cell size of the merged cloud, and the farthest depth it keeps, metres
    scene_voxel_m: float = 0.01
    scene_depth_trunc_m: float = 2.0
    # find_surface: keyframes merged per camera, so a sweep over a table counts
    surface_frames: int = 10
    # find_surface: surfaces are parallel to the reporting frame's xy plane, as a
    # table is to the arm's base; False fits their tilt too.
    surface_parallel_to_frame: bool = True


class ManipulationLocalize(PerceptionLocalize):
    """PerceptionLocalize that keeps the objects its answers found, by id and label.

    A ``localize`` or ``scan`` answer replaces the objects of the labels it was
    asked for and keeps the others. Pick and place reads them by id, or by a
    label only one object has. A forgotten object's older sightings are left out
    of later answers. ``find_surface`` measures the table the objects stand on.
    """

    config: ManipulationLocalizeConfig

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._objects: dict[str, tuple[str, Localization]] = {}
        # Forgotten objects: label, id, box low and high corners, newest frame time then.
        self._gone: list[tuple[str, str, np.ndarray, np.ndarray, float]] = []

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

        Later answers leave out a sighting of its label that is not newer than
        now and has its id or is centred inside its box, so the old place does
        not come back as a target.
        """
        entry = self._objects.pop(object_id, None)
        if entry is None:
            return
        label, hit = entry
        points = hit.point_cloud.points_f32()
        if not len(points):
            points = np.asarray([hit.position_world_xyz], dtype=np.float32)
        _, head = self.index.get_time_range()
        # Its box, grown 1 cm, so the same object seen a little off still counts as it.
        self._gone.append(
            (label, object_id, points.min(axis=0) - 0.01, points.max(axis=0) + 0.01, head)
        )

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

    @rpc
    def surfaces(
        self,
        start: float = -10.0,
        duration: float = 10.0,
        frame: str = "",
        max_tilt_deg: float = 25.0,
    ) -> list[dict[str, Any]]:
        """The flat surfaces seen in a window of memory, biggest first, as in ``find_surface``.

        Each is a dict: ``height``, ``center``, ``size``, ``yaw_deg``, ``normal``,
        ``tilt_deg``, ``downhill_deg``, ``coefficients`` (a, b, c, d of ax+by+cz+d=0),
        ``points``, ``rms_m`` and ``points_above`` (what stands on it, 1 to 30 cm over
        it inside its outline), in ``frame``. Empty when localize is not ready, the
        window has no geometry, or ``frame`` has no transform from the world frame.
        """
        if not self._ready.is_set():
            return []
        first, head = self.index.get_time_range()
        lo = max(first, head + start if start < 0 else first + start)
        keyframes = list(self.index.time_range(lo, lo + duration))
        views = (
            window_points(self.rig, keyframes, camera, self.config.surface_frames)
            for camera in self.rig.cameras
        )
        clouds = [cloud for cloud in views if cloud is not None]
        if not clouds:
            return []
        to_frame = None
        if frame and frame != self.rig.world_frame:
            transform = self.rig.tf.get(frame, self.rig.world_frame, head, TF_TOLERANCE)
            if transform is None:
                logger.warning(f"find_surface: no transform from {self.rig.world_frame} to {frame}")
                return []
            to_frame = transform.to_matrix()
        return [
            surface.as_dict()
            for surface in find_surfaces(
                np.concatenate(clouds),
                max_tilt_deg,
                to_frame,
                level=self.config.surface_parallel_to_frame,
            )
        ]

    @skill
    def find_surface(
        self,
        start: float = -10.0,
        duration: float = 10.0,
        frame: str = "",
        max_tilt_deg: float = 25.0,
    ) -> str:
        """Find flat surfaces, such as the table top or the floor, in a window of memory.

        Gives each surface's height under the origin of ``frame``, its centre, its
        size (long side x short side) and the long side's heading, and how far it
        tilts from level and towards which heading it goes down. The biggest
        surface comes first. Only what the camera saw inside the window counts,
        so look over the table before calling; the size runs about 2% short.

        ``start`` and ``duration`` name the window as in ``localize``. ``frame``
        is the frame to report in, e.g. the arm's base link; empty means the
        world frame. By default (``surface_parallel_to_frame``) every surface is
        taken as parallel to that frame's xy plane and reported at the height of
        its centre, tilt 0; this assumes the table is square to the frame within
        about half a degree. With it off, the tilt is fitted, level to the world
        frame. Either way ``max_tilt_deg`` drops surfaces tilted more than that.
        """
        if not self._ready.is_set():
            return (
                f"find_surface cannot answer yet: {self._stage}. Poll state() until it reads ready."
            )
        name = frame or self.rig.world_frame
        found = self.surfaces(start, duration, frame, max_tilt_deg)
        if not found:
            return f"no flat surface in the window, in {name}"
        lines = []
        for i, surface in enumerate(found, 1):
            x, y, z = surface["center"]
            length, width = surface["size"]
            tilt = surface["tilt_deg"]
            slope = (
                f"tilted {tilt:.1f} deg, down towards {round(surface['downhill_deg'])} deg"
                if tilt >= 0.5
                else "level"
            )
            lines.append(
                f"surface {i}: height {surface['height']:.3f} m under the origin of {name}, "
                f"centre ({x:.3f}, {y:.3f}, {z:.3f}), {length:.2f} x {width:.2f} m, "
                f"long side at {round(surface['yaw_deg'])} deg, {slope}, "
                f"{surface['points']} points, rms {surface['rms_m'] * 1000:.1f} mm, "
                f"{surface['points_above']} points standing on it"
            )
        return "\n".join(lines)

    def _localize(
        self,
        queries: list[str],
        start: float,
        duration: float,
        overrides: dict[str, Any] | None,
    ) -> tuple[list[list[Localization]], tuple[float, float, float, float, int]]:
        results, window = super()._localize(queries, start, duration, overrides)
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

    def _was_taken(self, label: str, hit: Localization) -> bool:
        """Whether this sighting is of a forgotten object, at its old place, from before."""
        position = np.asarray(hit.position_world_xyz)
        return any(
            gone_label == label
            and hit.last_seen_timestamp <= cutoff
            and (hit.instance_id == gone_id or bool(np.all((position >= low) & (position <= high))))
            for gone_label, gone_id, low, high, cutoff in self._gone
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
