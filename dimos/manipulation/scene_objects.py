# Copyright 2025-2026 Dimensional Inc.
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

"""Resolve a *query* (an object id, or a world point a user clicked) into the
object's current world pose, blind to where that pose comes from.

This is the one seam that lets click-to-grasp run identically in sim and on the
real robot: the reach (``move_to_pose`` / ``pick``) is already source-blind — it
consumes a *pose*. We add a single resolver in front of it with swappable
sources:

    query ─► ObjectPoseSource.resolve(query) ─► ResolvedObject(object, world pose)
              ├─ PrivilegedObjectSource : the EntityStateBatch stream (sim ground truth)
              └─ PerceivedObjectSource  : a perception detection DB  (Phase 2; sim RGBD or real RealSense)

``ResolvedObject`` is the *un-streamed* form of one ``EntityStateBatch`` entry —
a ``SceneObject`` (§7-A identity + geometry, pose-less by design) paired with the
``PoseStamped`` it currently sits at. We deliberately do **not** fold the pose
into ``SceneObject`` itself: §7-A keeps that noun pose-less because pose lives in
the world / the stream, and that boundary is joint-agreed. This module is the
resolver only — it does not touch ``Obstacle`` / ``sync_entity_poses`` / the
WorldSpec backends.
"""

from __future__ import annotations

from dataclasses import dataclass
import threading
from typing import TYPE_CHECKING, Protocol, runtime_checkable

from dimos.experimental.pimsim.spec.models import SceneObject
from dimos.msgs.geometry_msgs.Point import Point
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped

if TYPE_CHECKING:
    from dimos.experimental.pimsim.entity import EntityDescriptor, EntityKind, EntityStateBatch
    from dimos.msgs.geometry_msgs.Pose import Pose

# What a caller can hand the resolver: a stable object id, or a world-frame point
# (``PointStamped`` is a ``Point`` subclass, so both satisfy ``Point``).
ObjectQuery = str | Point


@dataclass(frozen=True)
class ResolvedObject:
    """An object resolved to where it is *right now*.

    The un-streamed form of one ``EntityStateBatch`` entry: the ``SceneObject``
    (identity + geometry) plus the world-frame ``PoseStamped`` it currently
    occupies. ``object.object_id`` is the identity; ``pose`` is what the reach
    consumes. Geometry on ``object`` (``shape_hint`` / ``extents`` / ``mesh_ref``)
    is carried for downstream grasp generation but is unused by a plain reach.
    """

    object: SceneObject
    pose: PoseStamped

    @property
    def object_id(self) -> str:
        return self.object.object_id

    @property
    def xyz(self) -> tuple[float, float, float]:
        p = self.pose.position
        return (float(p.x), float(p.y), float(p.z))


@runtime_checkable
class ObjectPoseSource(Protocol):
    """Resolve a query to an object's current world pose, blind to the source.

    Implementations differ only in *where the pose comes from*:
    ``PrivilegedObjectSource`` reads the sim's ground-truth entity stream;
    ``PerceivedObjectSource`` (Phase 2) reads a perception detection DB fed by
    the same RGBD contract in sim or on the real robot. Consumers (the grasp
    skill) never learn which one is wired.
    """

    def resolve(self, query: ObjectQuery) -> ResolvedObject | None:
        """Return the object matching ``query``, or ``None`` if nothing matches.

        A ``str`` query is an exact object-id lookup. A ``Point`` query selects
        the nearest known object to that world point (the click-to-grasp case).
        """
        ...

    def list_objects(self) -> list[ResolvedObject]:
        """Every object currently known to the source, with its latest pose."""
        ...


def _scene_object_from_descriptor(descriptor: EntityDescriptor) -> SceneObject:
    """``EntityDescriptor`` → §7-A ``SceneObject`` (entity_id → object_id).

    The two are structurally the same noun (that is why §7-A collapses them);
    this is the conversion the obstacle monitor will eventually share.
    """
    return SceneObject(
        object_id=descriptor.entity_id,
        kind=descriptor.kind,
        mesh_ref=descriptor.mesh_ref,
        shape_hint=descriptor.shape_hint,
        extents=descriptor.extents,
        mass=descriptor.mass,
        rgba=descriptor.rgba,
    )


def _query_xyz(query: Point) -> tuple[float, float, float]:
    return (float(query.x), float(query.y), float(query.z))


class PrivilegedObjectSource:
    """Ground-truth source: resolves from the latest ``EntityStateBatch``.

    Caches the most recent snapshot of ``(descriptor, pose)`` entries — the same
    stream the manipulation module already subscribes to for collision — and
    answers id and nearest-point queries off it. "Privileged" because it reads
    the simulator's exact poses rather than perceiving them; the real-robot path
    swaps in ``PerceivedObjectSource`` behind the same ``ObjectPoseSource``
    interface.

    Thread-safe: ``update`` runs on the entity-stream subscriber thread while
    ``resolve`` runs on the click/grasp handler thread.
    """

    def __init__(
        self,
        *,
        frame_id: str = "world",
        max_pick_distance: float | None = None,
        graspable_kinds: tuple[EntityKind, ...] | None = None,
    ) -> None:
        """
        Args:
            frame_id: Frame stamped onto resolved poses. The entity stream is
                world-frame, matching the world coordinates the reach uses.
            max_pick_distance: If set, a nearest-point query returns ``None``
                when the closest object is farther than this (meters) — so a
                click in empty space grabs nothing instead of a far object.
            graspable_kinds: If set, restrict resolution to these entity kinds
                (e.g. ``("dynamic",)`` to ignore static scenery). ``None`` =
                consider every entity.
        """
        self._lock = threading.Lock()
        self._objects: dict[str, ResolvedObject] = {}
        self._frame_id = frame_id
        self._max_pick_distance = max_pick_distance
        self._graspable_kinds = graspable_kinds

    def update(self, batch: EntityStateBatch) -> None:
        """Replace the cache with a fresh entity snapshot. Cheap; call per tick."""
        objects: dict[str, ResolvedObject] = {}
        for descriptor, pose in getattr(batch, "entries", []):
            if self._graspable_kinds is not None and descriptor.kind not in self._graspable_kinds:
                continue
            objects[descriptor.entity_id] = ResolvedObject(
                object=_scene_object_from_descriptor(descriptor),
                pose=self._stamp(pose, batch.ts),
            )
        with self._lock:
            self._objects = objects

    def resolve(self, query: ObjectQuery) -> ResolvedObject | None:
        with self._lock:
            objects = list(self._objects.values())
            by_id = self._objects.get(query) if isinstance(query, str) else None
        if isinstance(query, str):
            return by_id
        if not objects:
            return None
        qx, qy, qz = _query_xyz(query)
        nearest = min(objects, key=lambda o: _dist_sq(o.xyz, qx, qy, qz))
        if self._max_pick_distance is not None:
            if _dist_sq(nearest.xyz, qx, qy, qz) > self._max_pick_distance**2:
                return None
        return nearest

    def list_objects(self) -> list[ResolvedObject]:
        with self._lock:
            return list(self._objects.values())

    def _stamp(self, pose: Pose, ts: float) -> PoseStamped:
        return PoseStamped(
            ts=ts,
            frame_id=self._frame_id,
            position=pose.position,
            orientation=pose.orientation,
        )


def _dist_sq(xyz: tuple[float, float, float], qx: float, qy: float, qz: float) -> float:
    dx, dy, dz = xyz[0] - qx, xyz[1] - qy, xyz[2] - qz
    return dx * dx + dy * dy + dz * dz


__all__ = [
    "ObjectPoseSource",
    "ObjectQuery",
    "PrivilegedObjectSource",
    "ResolvedObject",
]
