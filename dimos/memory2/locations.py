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

"""Durable named locations, anchored to the cross-run ``map`` frame.

``world`` is per-run — its origin is wherever the robot booted — so a pose saved
in one run is meaningless in the next. ``map`` is durable, published as a tf edge
``world -> map`` by :class:`~dimos.mapping.relocalization.module.RelocalizationModule`.
Locations are therefore stored in ``map`` and converted to the caller's frame at
*use* time, never at save time: the ``world -> map`` edge is a live estimate that
is revised as registration improves, and the tf buffer only retains a short
window anyway.

Records live in two streams so that the (small, always present) record is
separate from the (large, optional) sensory evidence:

``locations``
    Structural fields in observation ``tags`` (indexed, pushed down to SQL), the
    pose in ``pose_tuple`` (R*Tree indexed), and free-form user metadata as the
    dict payload.
``location_keyframes``
    The keyframe image, tagged with ``location_id``, optionally carrying an
    embedding for similarity search.

The stream is append-only, so a named location is a *series of versions* and the
latest ``ts`` wins. Deletes append a tombstone; promotion and re-anchoring append
a new version rather than mutating, which keeps a bad relocalization auditable
and reversible.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import re
import time
from typing import TYPE_CHECKING, Any, Protocol, runtime_checkable
import uuid

from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from collections.abc import Callable, Sequence

    from dimos.memory2.store.base import Store
    from dimos.memory2.stream import Stream
    from dimos.memory2.type.observation import Observation, PoseTuple
    from dimos.models.embedding.base import Embedding
    from dimos.msgs.sensor_msgs.Image import Image
    from dimos.protocol.tf.tf import TFLookup

logger = setup_logger()

FRAME_MAP = "map"
FRAME_WORLD = "world"

LOCATIONS_STREAM = "locations"
KEYFRAMES_STREAM = "location_keyframes"

_WHITESPACE = re.compile(r"\s+")


def normalize_name(name: str) -> str:
    """Canonical form used for identity: casefolded, whitespace-collapsed.

    Exact match only — fuzzy lookup is what :meth:`LocationStore.search` is for.
    """
    collapsed = _WHITESPACE.sub(" ", name).strip()
    if not collapsed:
        raise ValueError("Location name must not be empty")
    return collapsed.casefold()


# --------------------------------------------------------------------------- #
# Errors
# --------------------------------------------------------------------------- #


class LocationError(Exception):
    """Base for every reason a location cannot be resolved."""


class UnknownLocationError(LocationError):
    """No location saved under that name."""


class NotRelocalizedError(LocationError):
    """The ``map`` frame is not reachable yet in this run."""


class MapMismatchError(LocationError):
    """Saved against a map that is not in the current map's lineage."""


class RunLocalError(LocationError):
    """Unanchored, and we are no longer in the run that saved it."""


class StaleAnchorError(LocationError):
    """Registration is failing or too old to trust the ``world -> map`` edge."""


# --------------------------------------------------------------------------- #
# Relocalization status
# --------------------------------------------------------------------------- #


@dataclass(frozen=True)
class RelocStatus:
    """Health of the ``world -> map`` estimate.

    This cannot be inferred from tf: ``RelocalizationModule`` republishes its last
    good transform at a fixed interval, restamped to now, so a failing
    registration still looks perfectly fresh in the tf tree. The module has to
    report this out of band. 
    """

    relocalized: bool = False
    last_success_ts: float | None = None
    fitness: float | None = None
    map_id: str | None = None
    consecutive_failures: int = 0


@runtime_checkable
class RelocStatusSource(Protocol):
    def __call__(self) -> RelocStatus: ...


# --------------------------------------------------------------------------- #
# Record
# --------------------------------------------------------------------------- #


@dataclass(frozen=True)
class SavedLocation:
    """One version of a named location.

    ``pose`` is a full ``(x, y, z, qx, qy, qz, qw)`` tuple. Callers that only care
    about heading convert explicitly via :attr:`yaw` — the orientation is never
    stored as euler angles, which is how the legacy ``RobotLocation`` path lost
    most of its heading range.
    """

    name: str
    pose: PoseTuple
    frame_id: str
    run_id: str
    location_id: str
    created_ts: float
    updated_ts: float
    anchored: bool = False
    map_id: str | None = None
    deleted: bool = False
    keyframe_id: int | None = None
    metadata: dict[str, Any] = field(default_factory=dict)

    @property
    def position(self) -> tuple[float, float, float]:
        return (self.pose[0], self.pose[1], self.pose[2])

    @property
    def orientation(self) -> Quaternion:
        return Quaternion(*self.pose[3:])

    @property
    def yaw(self) -> float:
        return float(self.orientation.to_euler().z)

    @property
    def scope(self) -> str:
        """Identity namespace for the name.

        A location is identified by ``(name, scope)``, never by name alone: the
        kitchen in one building is not the kitchen in another, and merging their
        version history — or worse, reusing one's ``location_id`` for the other —
        would silently rewrite a location the user never touched. Anchored records
        are scoped to their map; run-local ones to the session that made them.
        """
        if self.anchored and self.map_id:
            return f"map:{self.map_id}"
        return f"run:{self.run_id}"

    def to_pose_stamped(self, *, ts: float | None = None) -> PoseStamped:
        x, y, z, qx, qy, qz, qw = self.pose
        return PoseStamped(
            ts=ts if ts is not None else self.updated_ts,
            position=(x, y, z),
            orientation=(qx, qy, qz, qw),
            frame_id=self.frame_id,
        )

    def to_tags(self) -> dict[str, Any]:
        """Structural fields, stored as indexed observation tags."""
        return {
            "name": self.name,
            "scope": self.scope,
            "frame_id": self.frame_id,
            "run_id": self.run_id,
            "location_id": self.location_id,
            "created_ts": self.created_ts,
            "anchored": self.anchored,
            "map_id": self.map_id,
            "deleted": self.deleted,
            "keyframe_id": self.keyframe_id,
        }

    @classmethod
    def from_observation(cls, obs: Observation[Any]) -> SavedLocation:
        tags = obs.tags
        pose = obs.pose_tuple
        if pose is None:
            raise ValueError(f"Location observation {obs.id} has no pose")
        try:
            metadata = obs.data
        except LookupError:
            metadata = {}
        return cls(
            name=tags["name"],
            pose=pose,
            frame_id=tags["frame_id"],
            run_id=tags["run_id"],
            location_id=tags["location_id"],
            created_ts=tags["created_ts"],
            updated_ts=obs.ts,
            anchored=bool(tags.get("anchored", False)),
            map_id=tags.get("map_id"),
            deleted=bool(tags.get("deleted", False)),
            keyframe_id=tags.get("keyframe_id"),
            metadata=metadata if isinstance(metadata, dict) else {},
        )


# --------------------------------------------------------------------------- #
# Store
# --------------------------------------------------------------------------- #


class LocationStore:
    """CRUD + frame resolution for named locations.

    Frame conversion is deliberately concentrated here rather than pushed into the
    planner: the A* planner ignores ``goal.frame_id`` entirely and consumes goal
    coordinates raw against the costmap, so nothing downstream is frame-aware.
    Callers resolve into the costmap frame at dispatch and re-issue the goal if
    the ``world -> map`` estimate shifts.
    """

    def __init__(
        self,
        store: Store,
        *,
        run_id: str,
        map_lineage: Sequence[str] = (),
        tf: TFLookup | None = None,
        status: Callable[[], RelocStatus] | None = None,
        stale_after: float = 30.0,
        promote_min_fitness: float = 0.6,
        stream_name: str = LOCATIONS_STREAM,
        keyframe_stream_name: str = KEYFRAMES_STREAM,
    ) -> None:
        """
        Args:
            map_lineage: ``(current_map_id, parent_map_id, ...)``, newest first.
                Empty means no map is loaded. Only the current map resolves;
                ancestors raise ``MapMismatchError`` with a message that says so,
                because nothing yet guarantees a rebuild preserved the map origin.
            stale_after: seconds since the last successful registration beyond
                which the anchor is considered stale.
            promote_min_fitness: fitness floor for *bulk auto-promotion*, deliberately
                above ``RelocalizationModule``'s 0.45 publish gate. A single
                marginal first fix would otherwise stamp every run-local tag from
                the session with one bad transform, permanently and all at once.
                An explicit user ``save()`` only needs the publish gate — it is one
                record, deliberate, and trivially redone.
        """
        from dimos.msgs.sensor_msgs.Image import Image

        self._store = store
        self._locations: Stream[dict[str, Any]] = store.stream(stream_name, dict)
        self._keyframes: Stream[Image] = store.stream(keyframe_stream_name, Image)
        self._run_id = run_id
        self._map_lineage = tuple(map_lineage)
        self._tf = tf
        self._status = status
        self._stale_after = stale_after
        self._promote_min_fitness = promote_min_fitness

    @property
    def map_id(self) -> str | None:
        return self._map_lineage[0] if self._map_lineage else None

    def set_map_lineage(self, lineage: Sequence[str]) -> None:
        """Set the active map once it is known.

        The map's identity comes from relocalization at load time, which is later
        than store construction, so this is not a constructor-only value.
        """
        self._map_lineage = tuple(lineage)

    @property
    def run_id(self) -> str:
        return self._run_id

    # ----------------------------------------------------------------- write

    def save(
        self,
        name: str,
        pose: Any,
        *,
        frame_id: str = FRAME_WORLD,
        ts: float | None = None,
        keyframe: Image | None = None,
        embedding: Embedding | None = None,
        metadata: dict[str, Any] | None = None,
    ) -> SavedLocation:
        """Upsert a location by name.

        The pose is anchored into ``map`` when the tf edge is available. When it
        is not, the record is stored run-local (``anchored=False``) so that
        tagging works on a first-ever visit to a building — callers are expected
        to tell the user it is session-only, and :meth:`promote_pending` upgrades
        it once registration succeeds.
        """
        norm = normalize_name(name)
        now = ts if ts is not None else time.time()
        pose_tuple, resolved_frame, anchored = self._anchor(pose, frame_id, now)

        previous = self._latest(norm)
        location_id = (
            previous.location_id if previous is not None else f"loc_{uuid.uuid4().hex[:8]}"
        )
        created_ts = previous.created_ts if previous is not None else now

        keyframe_id = self._save_keyframe(
            keyframe, location_id=location_id, name=norm, ts=now, embedding=embedding
        )
        if keyframe_id is None and previous is not None:
            keyframe_id = previous.keyframe_id

        loc = SavedLocation(
            name=norm,
            pose=pose_tuple,
            frame_id=resolved_frame,
            run_id=self._run_id,
            location_id=location_id,
            created_ts=created_ts,
            updated_ts=now,
            anchored=anchored,
            map_id=self.map_id if anchored else None,
            keyframe_id=keyframe_id,
            metadata=metadata or {},
        )
        self._append(loc)
        logger.info(
            "Saved location",
            name=norm,
            frame=resolved_frame,
            anchored=anchored,
            map_id=loc.map_id,
        )
        return loc

    def delete(self, name: str) -> bool:
        """Append a tombstone. Returns False if there was nothing to delete."""
        norm = normalize_name(name)
        current = self.get(norm)
        if current is None:
            return False
        from dataclasses import replace

        self._append(replace(current, deleted=True, updated_ts=time.time()))
        logger.info("Deleted location", name=norm)
        return True

    def promote(self, loc: SavedLocation) -> SavedLocation:
        """Re-express a run-local location in ``map``, appending a new version."""
        if loc.anchored:
            return loc
        if loc.run_id != self._run_id:
            raise RunLocalError(
                f"Cannot promote {loc.name!r}: saved in run {loc.run_id}, current run is "
                f"{self._run_id}. Its `world` frame no longer exists."
            )
        problem = self._status_problem(min_fitness=self._promote_min_fitness)
        if problem is not None:
            raise type(problem)(f"Cannot promote {loc.name!r}: {problem}")

        pose_tuple, frame_id, anchored = self._anchor(
            loc.pose, loc.frame_id, time.time(), min_fitness=self._promote_min_fitness
        )
        if not anchored:
            raise NotRelocalizedError(
                f"Cannot promote {loc.name!r}: `{FRAME_MAP}` frame unavailable"
            )

        from dataclasses import replace

        promoted = replace(
            loc,
            pose=pose_tuple,
            frame_id=frame_id,
            anchored=True,
            map_id=self.map_id,
            updated_ts=time.time(),
        )
        self._append(promoted)
        logger.info("Promoted location to map frame", name=loc.name, map_id=self.map_id)
        return promoted

    def promote_pending(self) -> list[SavedLocation]:
        """Promote every run-local location from this run. Called on first good fix."""
        promoted = []
        for loc in self.list_all():
            if loc.anchored or loc.run_id != self._run_id:
                continue
            try:
                promoted.append(self.promote(loc))
            except LocationError as e:
                logger.warning("Could not promote location", name=loc.name, reason=str(e))
        return promoted

    # ------------------------------------------------------------------ read

    def get(self, name: str) -> SavedLocation | None:
        """Latest live version, or None if missing or deleted."""
        loc = self._latest(normalize_name(name))
        return None if loc is None or loc.deleted else loc

    def list_all(self, *, include_deleted: bool = False) -> list[SavedLocation]:
        """Latest version of every distinct name, newest first.

        Not named ``list`` — that would shadow the ``list[...]`` builtin for every
        annotation in this class body.
        """
        latest: dict[str, SavedLocation] = {}
        for scope in self._scopes():
            for obs in self._locations.tags(scope=scope).order_by("ts", desc=True):
                loc = SavedLocation.from_observation(obs)
                latest.setdefault(loc.name, loc)
        out = [loc for loc in latest.values() if include_deleted or not loc.deleted]
        return sorted(out, key=lambda loc: loc.updated_ts, reverse=True)

    def history(self, name: str) -> list[SavedLocation]:
        """Every version of this location, newest first — the audit trail.

        Keyed on ``location_id`` rather than scope: promotion moves a record from
        its run scope to a map scope, and a scope-keyed history would hide the
        pre-promotion version — exactly the one you need to see when a promote
        anchored to a bad fix.
        """
        current = self._latest(normalize_name(name))
        if current is None:
            return []
        return [
            SavedLocation.from_observation(obs)
            for obs in self._locations.tags(location_id=current.location_id).order_by(
                "ts", desc=True
            )
        ]

    def search(self, query: Embedding, k: int = 5) -> list[SavedLocation]:
        """Similarity search over saved keyframes, mapped back to live locations."""
        seen: set[str] = set()
        out: list[SavedLocation] = []
        for obs in self._keyframes.search(query, k=k * 4):
            location_id = obs.tags.get("location_id")
            name = obs.tags.get("name")
            if not name or not location_id or location_id in seen:
                continue
            seen.add(location_id)
            loc = self.get(name)
            if loc is not None:
                out.append(loc)
            if len(out) >= k:
                break
        return out

    def keyframe(self, loc: SavedLocation) -> Image | None:
        """Load the stored keyframe, if any. Lazy — the blob is fetched on access."""
        if loc.keyframe_id is None:
            return None
        for obs in self._keyframes.tags(location_id=loc.location_id).order_by("ts", desc=True):
            return obs.data
        return None

    # -------------------------------------------------------------- resolve

    def resolve(
        self,
        loc: SavedLocation | str,
        target_frame: str = FRAME_WORLD,
        *,
        ts: float | None = None,
    ) -> PoseStamped:
        """Express a location in ``target_frame`` using the *current* tf.

        Raises a specific :class:`LocationError` rather than returning a
        best-effort pose — a wrong goal is worse than no goal.
        """
        if isinstance(loc, str):
            resolved = self.get(loc)
            if resolved is None:
                elsewhere = self._latest_any_scope(normalize_name(loc))
                if elsewhere is not None and not elsewhere.deleted:
                    self._check_usable(elsewhere)
                raise UnknownLocationError(f"No location named {loc!r}")
            loc = resolved

        self._check_usable(loc)

        if loc.frame_id == target_frame:
            return loc.to_pose_stamped(ts=ts)

        self._check_status()

        if self._tf is None:
            raise NotRelocalizedError(
                f"{loc.name!r} is in frame {loc.frame_id!r} but no tf service is available "
                f"to reach {target_frame!r}"
            )

        transform = self._tf.get(target_frame, loc.frame_id, time_point=ts)
        if transform is None:
            raise NotRelocalizedError(
                f"No transform {target_frame!r} <- {loc.frame_id!r}; the robot has not "
                f"recognised where it is yet"
            )

        base = Pose(Vector3(transform.translation), Quaternion(transform.rotation))
        composed = base + Pose(Vector3(*loc.position), loc.orientation)
        return PoseStamped(
            ts=ts if ts is not None else time.time(),
            position=(composed.position.x, composed.position.y, composed.position.z),
            orientation=composed.orientation.to_tuple(),
            frame_id=target_frame,
        )

    # -------------------------------------------------------------- internal

    def _append(self, loc: SavedLocation) -> None:
        self._locations.append(
            loc.metadata,
            ts=loc.updated_ts,
            pose=loc.pose,
            tags=loc.to_tags(),
        )

    def _scopes(self) -> list[str]:
        """Scopes visible right now, most authoritative first.

        The current map's anchored records win over this session's run-local ones,
        so that a location which has been promoted is not shadowed by the
        pre-promotion version it was derived from.
        """
        scopes = []
        if self.map_id:
            scopes.append(f"map:{self.map_id}")
        scopes.append(f"run:{self._run_id}")
        return scopes

    def _latest(self, norm: str) -> SavedLocation | None:
        """Latest version of *norm* within the current scopes."""
        for scope in self._scopes():
            found = self._latest_in_scope(norm, scope)
            if found is not None:
                return found
        return None

    def _latest_in_scope(self, norm: str, scope: str) -> SavedLocation | None:
        query = self._locations.tags(name=norm, scope=scope).order_by("ts", desc=True).limit(1)
        for obs in query:
            return SavedLocation.from_observation(obs)
        return None

    def _latest_any_scope(self, norm: str) -> SavedLocation | None:
        """Used only to tell "never heard of it" apart from "that's another map's"."""
        for obs in self._locations.tags(name=norm).order_by("ts", desc=True).limit(1):
            return SavedLocation.from_observation(obs)
        return None

    def _save_keyframe(
        self,
        keyframe: Image | None,
        *,
        location_id: str,
        name: str,
        ts: float,
        embedding: Embedding | None,
    ) -> int | None:
        """Capture is one-way — a keyframe cannot be recovered from a past run, so
        it is stored even though nothing consumes it until M3."""
        if keyframe is None:
            return None
        obs = self._keyframes.append(
            keyframe,
            ts=ts,
            tags={"location_id": location_id, "name": name},
            **({"embedding": embedding} if embedding is not None else {}),
        )
        return obs.id

    def _anchor(
        self, pose: Any, frame_id: str, ts: float, *, min_fitness: float | None = None
    ) -> tuple[PoseTuple, str, bool]:
        """Convert *pose* into ``map`` if possible. Falls back to run-local.

        Anchoring is gated on relocalization health, not merely on the tf edge
        existing. The module republishes its last good ``world -> map`` at a fixed
        interval restamped to now, so a long-dead registration still presents a
        perfectly fresh-looking edge — and unlike a bad *read*, a bad anchor is
        written permanently.
        """
        from dimos.memory2.type.observation import _to_tuple

        pose_tuple = _to_tuple(pose)
        if pose_tuple is None:
            raise ValueError("A location needs a pose")

        if frame_id == FRAME_MAP:
            return pose_tuple, FRAME_MAP, True
        if self._tf is None or not self._map_lineage:
            return pose_tuple, frame_id, False

        problem = self._status_problem(min_fitness=min_fitness)
        if problem is not None:
            logger.info(
                "Anchor not trustworthy, saving run-local",
                reason=str(problem),
                run_id=self._run_id,
            )
            return pose_tuple, frame_id, False

        transform = self._tf.get(FRAME_MAP, frame_id, time_point=ts)
        if transform is None:
            logger.info(
                "No map anchor available, saving run-local", frame=frame_id, run_id=self._run_id
            )
            return pose_tuple, frame_id, False

        x, y, z, qx, qy, qz, qw = pose_tuple
        base = Pose(Vector3(transform.translation), Quaternion(transform.rotation))
        composed = base + Pose(Vector3(x, y, z), Quaternion(qx, qy, qz, qw))
        return (
            (
                composed.position.x,
                composed.position.y,
                composed.position.z,
                *composed.orientation.to_tuple(),
            ),
            FRAME_MAP,
            True,
        )

    def _check_usable(self, loc: SavedLocation) -> None:
        if not loc.anchored:
            if loc.run_id != self._run_id:
                raise RunLocalError(
                    f"{loc.name!r} was only saved for the session that created it "
                    f"(run {loc.run_id}); the robot has restarted since."
                )
            return
        if loc.map_id is None or not self._map_lineage:
            return
        if loc.map_id == self._map_lineage[0]:
            return
        if loc.map_id in self._map_lineage:
            raise MapMismatchError(
                f"{loc.name!r} was saved against map {loc.map_id!r}, an ancestor of the "
                f"current map {self._map_lineage[0]!r}. Ancestor poses are only valid if "
                f"the rebuild preserved the map origin, which nothing verifies today, so "
                f"re-anchoring is required."
            )
        raise MapMismatchError(
            f"{loc.name!r} was saved against map {loc.map_id!r}, which is not in the "
            f"current map's lineage {self._map_lineage!r}"
        )

    def _status_problem(self, *, min_fitness: float | None = None) -> LocationError | None:
        """Why the ``world -> map`` estimate should not be trusted, or None.

        Returned rather than raised because the read and write paths want different
        reactions to the same condition: a resolve should fail loudly, while a save
        should quietly degrade to run-local.
        """
        if self._status is None:
            return None
        status = self._status()
        if not status.relocalized:
            return NotRelocalizedError("Relocalization has not produced a fix yet in this run")
        if status.last_success_ts is not None:
            age = time.time() - status.last_success_ts
            if age > self._stale_after:
                return StaleAnchorError(
                    f"Last successful registration was {age:.0f}s ago "
                    f"(limit {self._stale_after:.0f}s) — position is no longer trustworthy"
                )
        if min_fitness is not None and (status.fitness is None or status.fitness < min_fitness):
            return StaleAnchorError(
                f"Registration fitness {status.fitness} is below the anchoring floor "
                f"{min_fitness} — not confident enough to write a permanent anchor"
            )
        return None

    def _check_status(self) -> None:
        problem = self._status_problem()
        if problem is not None:
            raise problem
