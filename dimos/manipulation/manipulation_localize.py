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
from scipy.spatial.transform import Rotation

from dimos.agents.annotation import skill
from dimos.core.core import rpc
from dimos.manipulation.manipulation_spec import PlanningObstaclesSpec
from dimos.mapping.pointclouds.box_filter_spec import PointCloudBoxFilterSpec
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.perception.localize.module import PerceptionLocalize, PerceptionLocalizeConfig
from dimos.perception.localize.rig import TF_TOLERANCE
from dimos.perception.localize.support_plane import find_surfaces, window_points
from dimos.perception.localize.types import Localization
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

# The table's planner obstacle and its cut out of the voxel map share this name.
TABLE = "scene/table"
# add_table takes the widest surface with at least this many points standing on it
# (find_surface's points_above): the table the objects are on, not the floor round it.
TABLE_MIN_STANDING = 10
# add_table(height=...) takes the surface within this of the height asked, metres.
TABLE_HEIGHT_TOLERANCE = 0.02


class ManipulationLocalizeConfig(PerceptionLocalizeConfig):
    # scene_cloud: cell size of the merged cloud, and the farthest depth it keeps, metres
    scene_voxel_m: float = 0.01
    scene_depth_trunc_m: float = 2.0
    # find_surface: keyframes merged per camera, so a sweep over a table counts
    surface_frames: int = 10
    # find_surface: surfaces are parallel to the reporting frame's xy plane, as a
    # table is to the arm's base; False fits their tilt too.
    surface_parallel_to_frame: bool = True
    # add_table, metres. The table is taken to reach past everything the arm can:
    # its box is a square table_reach_m either side of the arm's base, its top on
    # the measured plane, table_thickness_m thick. Every voxel map cell under the
    # plane in that square is cut, with those centred up to table_cut_above_m over
    # it: a cell's centre sits up to half a cell over the points in it, so that
    # wants half the map's cell plus the depth noise; 0.01 suits 1 cm cells, 2.5 cm
    # cells want about 0.018.
    table_reach_m: float = 1.0
    table_thickness_m: float = 0.03
    table_cut_above_m: float = 0.01


class ManipulationLocalize(PerceptionLocalize):
    """PerceptionLocalize that keeps the objects its answers found, by id and label.

    A ``localize`` or ``scan`` answer replaces the objects of the labels it was
    asked for and keeps the others. Pick and place reads them by id, or by a
    label only one object has. A forgotten object's older sightings are left out
    of later answers. ``find_surface`` measures the table the objects stand on,
    and ``add_table`` puts it into the motion planner as one flat box.
    """

    config: ManipulationLocalizeConfig

    # Where add_table puts the table; both optional, as in a blueprint with no planner.
    _obstacles: PlanningObstaclesSpec | None = None
    _box_filter: PointCloudBoxFilterSpec | None = None

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

    @skill
    def add_table(
        self, start: float = -10.0, duration: float = 10.0, height: float | None = None
    ) -> str:
        """Put the table the camera sees into the motion planner as one flat box.

        Measures the table's height in the window, as ``find_surface`` does, and
        puts a box whose top is the measured surface under the whole of the
        arm's reach. The table is the widest surface with objects standing on
        it, so it wins over a tray on it and over an empty floor round it;
        ``height`` (metres, in the world frame) takes the ``find_surface``
        surface nearest it instead. The answer lists every surface seen.

        The voxel map's cells under that surface are cut: the coarse cells make
        the table look thicker than it is, and nothing under the table top can
        be reached. Cells just over the table, such as the bottom centimetre of
        an object standing on it, are cut too.

        The box spans the whole reach, past the table's real edges, so the arm
        does not reach below the table top anywhere, e.g. down into a bin beside
        the table; ``clear_table`` before such a reach.

        Calling it again replaces the table; ``clear_table`` removes it.
        """
        if not self._ready.is_set():
            return f"add_table cannot answer yet: {self._stage}. Poll state() until it reads ready."
        if self._obstacles is None or self._box_filter is None:
            return "add_table needs a motion planner and a voxel map box filter in the blueprint"
        surfaces = self.surfaces(start, duration)
        if not surfaces:
            return "no table: no flat surface in the window. Look at the table and try again."
        table = pick_table(surfaces, height)
        heights = ", ".join(
            f"{surface['height']:.3f} m ({surface['size'][0]:.2f} x {surface['size'][1]:.2f})"
            for surface in surfaces
        )
        if table is None:
            return f"no table at {height} m: the surfaces are at {heights}"
        groups = self._obstacles.list_planning_groups()
        bases = sorted({group.base_frame for group in groups})
        origins = [self._origin(base) for base in bases]
        if any(origin is None for origin in origins):
            return f"no table: no transform from {self.rig.world_frame} to the arm's base {bases}"
        (box_pose, box_size), (cut_pose, cut_size) = table_boxes(
            table,
            [origin for origin in origins if origin is not None],
            reach=self.config.table_reach_m,
            thickness=self.config.table_thickness_m,
            cut_above=self.config.table_cut_above_m,
        )
        self.clear_table()
        # The arm's base may stand inside the box, e.g. on a mount below the table top.
        if not self._obstacles.add_obstacle(TABLE, box_pose, "box", box_size, None, bases):
            return "the planner refused the table box"
        self._box_filter.set_box(TABLE, cut_pose, Vector3(*cut_size))
        return (
            f"table added: top at z={table['height']:.3f} m in {self.rig.world_frame}, "
            f"{box_size[0]:.2f} x {box_size[1]:.2f} m round the arm's base; the voxel map "
            f"cells under it are cut. Surfaces seen: {heights}; if the table is another, "
            f"call add_table(height=...)"
        )

    @skill
    def clear_table(self) -> str:
        """Remove the table ``add_table`` put into the planner, and let its voxel map cells back."""
        if self._obstacles is None or self._box_filter is None:
            return "no table: the blueprint has no motion planner or voxel map box filter"
        removed = self._obstacles.remove_obstacle(TABLE)
        self._box_filter.clear_box(TABLE)
        return "table removed" if removed else "no table to remove"

    def _origin(self, frame: str) -> np.ndarray | None:
        """Where a frame's origin is in the world frame now; None while tf lacks it."""
        if frame == self.rig.world_frame:
            return np.zeros(3)
        _, head = self.index.get_time_range()
        transform = self.rig.tf.get(self.rig.world_frame, frame, head, TF_TOLERANCE)
        return None if transform is None else transform.translation.to_numpy()

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


def pick_table(
    surfaces: list[dict[str, Any]], height: float | None = None
) -> dict[str, Any] | None:
    """The table among ``surfaces`` entries: the one nearest ``height``, within
    ``TABLE_HEIGHT_TOLERANCE``, or without a height the widest of those with at least
    ``TABLE_MIN_STANDING`` points standing on them, or the widest when nothing stands
    on any. None when no surface fits."""
    if not surfaces:
        return None
    if height is not None:
        nearest = min(surfaces, key=lambda surface: abs(surface["height"] - height))
        return nearest if abs(nearest["height"] - height) <= TABLE_HEIGHT_TOLERANCE else None
    holding = [s for s in surfaces if s.get("points_above", 0) >= TABLE_MIN_STANDING]
    return max(holding or surfaces, key=lambda surface: surface["size"][0] * surface["size"][1])


def table_boxes(
    surface: dict[str, Any],
    bases: list[np.ndarray],
    *,
    reach: float,
    thickness: float,
    cut_above: float,
) -> tuple[tuple[Pose, list[float]], tuple[Pose, list[float]]]:
    """The planner box and the voxel map cut for one ``surfaces`` entry, each as a pose
    and full edge lengths, z along the surface's normal.

    Both cover the square ``reach`` either side of every base, in the surface's
    plane. The box hangs ``thickness`` under the surface; the cut runs from
    ``cut_above`` over it down ``reach`` under it.
    """
    normal = np.asarray(surface["normal"], dtype=np.float64)
    a, b, c, d = surface["coefficients"]
    # The bases dropped onto the plane, along the normal.
    feet = [base - normal * (float(np.dot([a, b, c], base)) + d) for base in bases]
    x_axis = np.array([1.0, 0.0, 0.0]) - normal * normal[0]
    if np.linalg.norm(x_axis) < 1e-6:
        x_axis = np.array([0.0, 1.0, 0.0]) - normal * normal[1]
    x_axis /= np.linalg.norm(x_axis)
    axes = np.column_stack([x_axis, np.cross(normal, x_axis), normal])
    qx, qy, qz, qw = Rotation.from_matrix(axes).as_quat()
    orientation = Quaternion(float(qx), float(qy), float(qz), float(qw))
    # The square in the plane's own axes, round all the feet.
    in_plane = np.array([axes.T @ foot for foot in feet])
    low, high = in_plane[:, :2].min(axis=0) - reach, in_plane[:, :2].max(axis=0) + reach
    middle = axes @ np.array([*(low + high) / 2, in_plane[0, 2]])
    side = [float(v) for v in high - low]

    def placed(offset: float) -> Pose:
        return Pose(position=Vector3(*(middle + normal * offset)), orientation=orientation)

    box = (placed(-thickness / 2), [*side, thickness])
    cut = (placed((cut_above - reach) / 2), [*side, cut_above + reach])
    return box, cut
