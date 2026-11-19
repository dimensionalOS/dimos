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

from dataclasses import replace
from typing import Any

import numpy as np
from numpy.typing import NDArray
from scipy.spatial.transform import Rotation

from dimos.agents.annotation import skill
from dimos.core.core import rpc
from dimos.manipulation.manipulation_spec import PlanningObstaclesSpec
from dimos.manipulation.planning.spec.models import WORK_SURFACE_OBSTACLE
from dimos.manipulation.surfaces import MAX_TILT_DEG, find_surfaces, in_outline
from dimos.mapping.pointclouds.box_filter_spec import PointCloudBoxFilterSpec, RidingBoxFilterSpec
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.perception.localize.module import PerceptionLocalize, PerceptionLocalizeConfig
from dimos.perception.localize.rig import TF_TOLERANCE
from dimos.perception.localize.support_plane import window_points
from dimos.perception.localize.types import Localization
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

# find_surface reports a surface tilted less than this as level.
LEVEL_BELOW_DEG = 0.5
# add_surface's planner obstacles and voxel map cuts are named SURFACE/<n>.
SURFACE = WORK_SURFACE_OBSTACLE
# The voxel map cuts round the object about to be grasped, and round the held one.
TARGET = "scene/target"
HELD = "scene/held"
# Without a line of sight, add_surface takes the widest surface with this many points on it.
SURFACE_MIN_STANDING = 10
# add_surface(height=...) takes the surface within this, metres.
SURFACE_HEIGHT_TOLERANCE = 0.02


class ManipulationLocalizeConfig(PerceptionLocalizeConfig):
    # scene_cloud: cell size of the merged cloud, and the farthest depth it keeps, metres
    scene_voxel_m: float = 0.01
    scene_depth_trunc_m: float = 2.0
    # find_surface: keyframes merged per camera
    surface_frames: int = 10
    # find_surface: surfaces parallel to the reporting frame's xy plane; False fits tilt
    surface_parallel_to_frame: bool = True
    # add_surface, metres: the box is the surface's measured outline, surface_margin_m
    # wider on every side, surface_thickness_m under the surface. The voxel map is cut from surface_cut_above_m
    # over it (half a cell plus depth noise: 0.01 for 1 cm cells, about 0.018 for 2.5 cm)
    # to surface_cut_below_m under it: the surface's own cells; what lies below stays.
    surface_margin_m: float = 0.05
    surface_thickness_m: float = 0.03
    surface_cut_above_m: float = 0.01
    surface_cut_below_m: float = 0.02
    # ignore_object / hold_object: an object's cut past its points, metres; wider when
    # held, since the grasp can shift it
    object_cut_margin_m: float = 0.02
    object_hold_margin_m: float = 0.03


class ManipulationLocalize(PerceptionLocalize):
    """PerceptionLocalize that keeps the objects its answers found, by id and label.

    A ``localize`` or ``scan`` answer replaces the objects of the labels it was
    asked for and keeps the others. Pick and place reads them by id, or by a
    label only one object has. A forgotten object's older sightings are left out
    of later answers. ``find_surface`` measures the surfaces in view and ``add_surface``
    puts one, such as the table, into the planner as one flat box. ``ignore_object``,
    ``hold_object`` and ``release_object`` keep the object being picked out of the voxel map.
    """

    config: ManipulationLocalizeConfig

    # Where add_surface puts the surface; optional, for blueprints without a planner.
    _obstacles: PlanningObstaclesSpec | None = None
    _box_filter: PointCloudBoxFilterSpec | None = None
    # Where hold_object rides the held object, before mapping.
    _riding: RidingBoxFilterSpec | None = None

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._objects: dict[str, tuple[str, Localization]] = {}
        # Forgotten objects: label, id, box low and high corners, newest frame time then.
        self._gone: list[tuple[str, str, np.ndarray, np.ndarray, float]] = []
        # add_surface's surfaces in the planner: name to height.
        self._surfaces: dict[str, float] = {}
        self._surface_count = 0

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
        lo, _ = self._window_start(start)
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
        max_tilt_deg: float = MAX_TILT_DEG,
    ) -> list[dict[str, Any]]:
        """``find_surface``'s surfaces as dicts, with each plane's ``coefficients`` and ``normal``.

        Empty when localize is not ready, nothing in the window has geometry, or
        ``frame`` has no transform.
        """
        if not self._ready.is_set():
            return []
        lo, head = self._window_start(start)
        keyframes = list(self.index.time_range(lo, lo + duration))
        rig = self.rig
        if rig.lidar is not None:
            # The window's scans only, as in _localize, not the whole feed.
            rig = replace(rig, lidar=rig.lidar.time_range(lo, lo + duration))
        views = (
            window_points(rig, keyframes, camera, self.config.surface_frames)
            for camera in rig.cameras
        )
        clouds = [cloud for cloud in views if cloud is not None]
        if not clouds:
            return []
        to_frame = None
        if frame and frame != self.rig.world_frame:
            transform = self.rig.tf.get(frame, self.rig.world_frame, head, TF_TOLERANCE)
            if transform is None:
                logger.warning(
                    "find_surface: no transform", parent=frame, child=self.rig.world_frame
                )
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
        max_tilt_deg: float = MAX_TILT_DEG,
    ) -> str:
        """Find flat surfaces, such as the table top or the floor, in a window of memory.

        Gives each one's height under the origin of ``frame`` (empty: the world
        frame), its centre, size, heading and tilt, and how many points stand on
        it, the biggest first. ``start`` and ``duration`` name the window as in
        ``localize``; only what the camera saw in it counts. Surfaces are taken as
        parallel to the frame's xy plane unless ``surface_parallel_to_frame`` is off.
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
                if tilt >= LEVEL_BELOW_DEG
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

    def _window_start(self, start: float) -> tuple[float, float]:
        """Where a window named by ``start``, as in ``localize``, begins, and the newest frame."""
        first, head = self.index.get_time_range()
        return max(first, head + start if start < 0 else first + start), head

    @skill
    def add_surface(
        self, start: float = -10.0, duration: float = 10.0, height: float | None = None
    ) -> str:
        """Put the surface the arm works on into the motion planner as one flat box.

        The surface is the one the camera looks at in the newest frame, else the widest
        with objects on it; ``height`` picks one from ``find_surface`` instead. The box
        is the outline the camera saw, a margin wider, and the surface's own voxel map
        cells are cut; what lies below or past its edges stays mapped. Look over the
        whole surface first: a part not yet seen is left out. Several surfaces, e.g. two
        tables at different heights, may be in the planner at once; adding one again at
        the same height replaces it.
        """
        if not self._ready.is_set():
            return (
                f"add_surface cannot answer yet: {self._stage}. Poll state() until it reads ready."
            )
        if self._obstacles is None:
            return "add_surface needs a motion planner in the blueprint"
        surfaces = self.surfaces(start, duration)
        if not surfaces:
            return "no surface: none flat in the window. Look at it and try again."
        platform = pick_surface(surfaces, height, self._line_of_sight())
        heights = ", ".join(
            f"{surface['height']:.3f} m ({surface['size'][0]:.2f} x {surface['size'][1]:.2f})"
            for surface in surfaces
        )
        if platform is None:
            return f"no surface at {height} m: the surfaces are at {heights}"
        bases = sorted({group.base_frame for group in self._obstacles.list_planning_groups()})
        (box_pose, box_size), (cut_pose, cut_size) = surface_boxes(
            platform,
            margin=self.config.surface_margin_m,
            thickness=self.config.surface_thickness_m,
            cut_above=self.config.surface_cut_above_m,
            cut_below=self.config.surface_cut_below_m,
        )
        self._surface_count += 1
        name = f"{SURFACE}/{self._surface_count}"
        # The arm's base may stand inside the box, e.g. on a mount below the surface.
        if not self._obstacles.add_obstacle(name, box_pose, "box", box_size, None, bases):
            return "the planner refused the surface box; the surfaces in it are kept"
        # Only now that the new box is in, the old one at this height goes.
        self._remove_surfaces(self._surfaces_at(platform["height"]))
        # Without a voxel map there are no cells to cut.
        if self._box_filter is not None:
            self._box_filter.set_box(name, cut_pose, Vector3(*cut_size))
        self._surfaces[name] = platform["height"]
        planned = ", ".join(f"{h:.3f} m" for h in sorted(self._surfaces.values()))
        cut = "; its own voxel map cells are cut" if self._box_filter is not None else ""
        return (
            f"surface added: top at z={platform['height']:.3f} m in {self.rig.world_frame}, "
            f"{box_size[0]:.2f} x {box_size[1]:.2f} m, its outline "
            f"{self.config.surface_margin_m:.2f} m wider on each side{cut}. "
            f"In the planner: {planned}. Surfaces seen: {heights}; to add "
            f"another, call add_surface(height=...)"
        )

    @skill
    def clear_surface(self, height: float | None = None) -> str:
        """Remove ``add_surface``'s boxes, or only the one at ``height``, and let their
        voxel map cells back."""
        if self._obstacles is None:
            return "no surface: the blueprint has no motion planner"
        names = list(self._surfaces) if height is None else self._surfaces_at(height)
        self._remove_surfaces(names)
        return f"{len(names)} surface(s) removed" if names else "no surface to remove"

    def _surfaces_at(self, height: float) -> list[str]:
        """The names of the surfaces in the planner at ``height``."""
        return [
            name
            for name, top in self._surfaces.items()
            if abs(top - height) <= SURFACE_HEIGHT_TOLERANCE
        ]

    def _remove_surfaces(self, names: list[str]) -> None:
        assert self._obstacles is not None
        for name in names:
            self._obstacles.remove_obstacle(name)
            if self._box_filter is not None:
                self._box_filter.clear_box(name)
            del self._surfaces[name]

    @skill
    def ignore_object(self, target: str) -> str:
        """Cut the object about to be grasped from the planner's voxel map, one at a time.

        ``target`` is an id from a localize answer, or a label only one object has.
        """
        if self._box_filter is None:
            return "ignore_object needs a voxel map box filter in the blueprint"
        hit = self._resolve(target)
        center, size = self._object_box(hit, self.config.object_cut_margin_m)
        self._box_filter.set_box(TARGET, Pose(position=Vector3(*center)), Vector3(*size))
        return (
            f"object {hit.instance_id} left out of the planner's map: a "
            f"{size[0]:.2f} x {size[1]:.2f} x {size[2]:.2f} m box at "
            f"({center[0]:.3f}, {center[1]:.3f}, {center[2]:.3f})"
        )

    @skill
    def hold_object(self, target: str) -> str:
        """Tell the planner the gripper now holds this object, after closing on it.

        The object rides on the tool tip, out of the map, and its old place stays cut
        until ``release_object``. Localize forgets it.
        """
        if self._obstacles is None or self._box_filter is None or self._riding is None:
            return (
                "hold_object needs a motion planner, a voxel map box filter and a riding box "
                "filter in the blueprint"
            )
        hit = self._resolve(target)
        tips = [
            group.tip_frame
            for group in self._obstacles.list_planning_groups()
            if group.has_gripper and group.tip_frame is not None
        ]
        if len(tips) != 1:
            return f"hold_object needs exactly one arm with a gripper, not {len(tips)}"
        tip = tips[0]
        _, head = self.index.get_time_range()
        transform = self.rig.tf.get(self.rig.world_frame, tip, head, TF_TOLERANCE)
        if transform is None:
            return f"hold_object: no transform from {self.rig.world_frame} to {tip}"
        center, size = self._object_box(hit, self.config.object_hold_margin_m)
        # The box in the tip's frame as it is now, axes as they are in the world.
        world_to_tip = np.linalg.inv(transform.to_matrix())
        x, y, z, w = Rotation.from_matrix(world_to_tip[:3, :3]).as_quat()
        pose = Pose(
            position=Vector3(*(world_to_tip[:3, :3] @ center + world_to_tip[:3, 3])),
            orientation=Quaternion(float(x), float(y), float(z), float(w)),
        )
        self._box_filter.set_box(TARGET, Pose(position=Vector3(*center)), Vector3(*size))
        self._riding.ride_box(HELD, pose, Vector3(*size), tip)
        self.forget(hit.instance_id)
        return f"holding object {hit.instance_id}: it rides on {tip}, out of the map"

    @skill
    def release_object(self) -> str:
        """Let the held or ignored object back into the voxel map.

        Call it after opening the gripper and moving up off the object; released with
        the fingers round it, the next move starts in collision.
        """
        if self._box_filter is None:
            return "release_object needs a voxel map box filter in the blueprint"
        held = self._riding.drop_box(HELD) if self._riding is not None else False
        ignored = self._box_filter.clear_box(TARGET)
        return "released" if held or ignored else "nothing held or ignored"

    def _object_box(
        self, hit: Localization, margin: float
    ) -> tuple[NDArray[np.float64], list[float]]:
        """An object's cut, centre and edge lengths: its points' bounds plus ``margin``,
        down to the surface, since from above the points are mostly its top face."""
        points = hit.point_cloud.points_f32()
        if not len(points):
            points = np.asarray([hit.position_world_xyz], dtype=np.float32)
        low = points.min(axis=0).astype(np.float64) - margin
        high = points.max(axis=0).astype(np.float64) + margin
        # The surface it stands on: the highest one under its top.
        under = [top for top in self._surfaces.values() if top <= high[2]]
        if under:
            low[2] = min(low[2], max(under))
        return (low + high) / 2.0, [float(v) for v in high - low]

    def _line_of_sight(self) -> tuple[NDArray[np.float64], NDArray[np.float64]] | None:
        """The newest frame's camera position and viewing direction, in the world frame."""
        _, head = self.index.get_time_range()
        for camera in self.rig.cameras:
            pose = self.rig.camera_pose(head, camera)
            if pose is not None:
                forward = pose.orientation.rotate_vector(Vector3(0.0, 0.0, 1.0))
                return pose.position.to_numpy(), forward.to_numpy()
        return None

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


def pick_surface(
    surfaces: list[dict[str, Any]],
    height: float | None = None,
    sight: tuple[NDArray[np.float64], NDArray[np.float64]] | None = None,
) -> dict[str, Any] | None:
    """The surface to work on: the biggest at ``height``, else the first the line of
    ``sight`` (origin, direction) meets, else the widest with objects standing on it."""
    if not surfaces:
        return None
    if height is not None:
        # Of those at that height, the biggest: a scrap of floor or a box top may be too.
        near = [s for s in surfaces if abs(s["height"] - height) <= SURFACE_HEIGHT_TOLERANCE]
        return max(near, key=lambda s: s["size"][0] * s["size"][1]) if near else None
    if sight is not None:
        hits = [(t, surface) for surface in surfaces if (t := _sight_hit(surface, *sight))]
        if hits:
            return min(hits, key=lambda hit: hit[0])[1]
    holding = [s for s in surfaces if s.get("points_above", 0) >= SURFACE_MIN_STANDING]
    return max(holding or surfaces, key=lambda surface: surface["size"][0] * surface["size"][1])


def _sight_hit(
    surface: dict[str, Any], origin: NDArray[np.float64], direction: NDArray[np.float64]
) -> float | None:
    """How far along the line of sight it meets the surface's outline, or None."""
    a, b, c, d = surface["coefficients"]
    normal = np.array([a, b, c], dtype=np.float64)
    facing = float(normal @ direction)
    if abs(facing) < 1e-6:
        return None
    t = -(float(normal @ origin) + d) / facing
    hit = origin + t * direction
    center = np.asarray(surface["center"], dtype=np.float64)
    inside = in_outline(center, normal, surface["yaw_deg"], tuple(surface["size"]), hit)
    return t if t > 0.0 and bool(inside[0]) else None


def surface_boxes(
    surface: dict[str, Any],
    *,
    margin: float,
    thickness: float,
    cut_above: float,
    cut_below: float,
) -> tuple[tuple[Pose, list[float]], tuple[Pose, list[float]]]:
    """The planner box and the voxel map cut, as (pose, edge lengths): the surface's
    outline ``margin`` wider on every side, the box ``thickness`` under the surface, the
    cut from ``cut_above`` over it to ``cut_below`` under it."""
    normal = np.asarray(surface["normal"], dtype=np.float64)
    yaw = np.radians(surface["yaw_deg"])
    # The long side: in the plane, heading yaw seen from above.
    heading = np.array([np.cos(yaw), np.sin(yaw), 0.0])
    if abs(normal[2]) > 1e-6:
        heading[2] = -float(normal[:2] @ heading[:2]) / normal[2]
    long_side = heading / np.linalg.norm(heading)
    axes = np.column_stack([long_side, np.cross(normal, long_side), normal])
    qx, qy, qz, qw = Rotation.from_matrix(axes).as_quat()
    orientation = Quaternion(float(qx), float(qy), float(qz), float(qw))
    middle = np.asarray(surface["center"], dtype=np.float64)
    side = [float(length) + 2 * margin for length in surface["size"]]

    def placed(offset: float) -> Pose:
        return Pose(position=Vector3(*(middle + normal * offset)), orientation=orientation)

    box = (placed(-thickness / 2), [*side, thickness])
    cut = (placed((cut_above - cut_below) / 2), [*side, cut_above + cut_below])
    return box, cut
