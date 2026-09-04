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

"""Places the Microduck knows about: rooms, landmark objects, tagged spots.

The four-room arena (``assets/four_room_scene.xml``) is described here as
plain data (:data:`MICRODUCK_ROOMS`, :data:`MICRODUCK_OBJECTS`) and
persisted through :class:`PlacesMemory`, a thin wrapper over a dimos
``SqliteStore`` stream of :class:`RobotLocation` records. The memory is what
the skills query ("go to space A", "where am I?") and what the cockpit's
``places`` channel is rendered from. A unit test cross-checks the constants
against the scene XML; keep the two in sync.

Several blueprints share one database file (``~/.cache/dimos/microduck/
places.db``) but describe different worlds, so every record is tagged with a
*scene id* and a :class:`PlacesMemory` only ever sees its own scene's rooms,
objects and remembered spots. The id is normally derived from the room /
object tables (:func:`scene_id`), so two blueprints with different tables
never leak places into each other.
"""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass
import hashlib
import json
import math
from pathlib import Path
import re
import threading
import time
from typing import Any

from dimos.memory.store.sqlite import SqliteStore
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.types.robot_location import RobotLocation

# Scene shipped with the package; the cockpit blueprint spawns the duck here.
FOUR_ROOM_XML = Path(__file__).resolve().parent / "assets" / "four_room_scene.xml"

# Frame every place is expressed in (world == MuJoCo == odom for the sim).
PLACES_FRAME = "world"

# The open central hub is not part of any room.
HUB_HALF_EXTENT = 0.8

# Inner faces of the four-room arena walls.
ARENA_HALF_EXTENT = 2.0

# Scene id used when a PlacesMemory is opened without one (direct use).
DEFAULT_SCENE = "default"

# The kickable ball. It is deliberately NOT declared in the scene XML: MuJoCo
# numbers joints in body order and the simulation engine takes joint 0 as the
# robot's root free joint (odom, spawn pose, IMU fallback, lidar exclusion),
# so a jointed body declared before the robot is attached would be mistaken
# for it. The sim module adds the ball with :func:`add_ball_body` *after*
# ``MjSpec.attach`` and later teleports it in front of a foot for kicks.
BALL_BODY = "ball"
BALL_FREEJOINT = "ball_freejoint"
BALL_GEOM = "ball_geom"
BALL_START: tuple[float, float, float] = (1.2, -0.6, 0.035)
BALL_RADIUS = 0.035
BALL_MASS = 0.05
BALL_GEOM_GROUP = 1  # invisible to the raycast lidar (group 0), so never in the costmap
_BALL_RGBA = (0.95, 0.95, 0.95, 1.0)
_BALL_FRICTION = (0.5, 0.005, 0.0001)


def add_ball_body(
    spec: Any,
    *,
    name: str = BALL_BODY,
    pos: tuple[float, float, float] = BALL_START,
) -> Any:
    """Add the kickable ball to a ``mujoco.MjSpec`` scene and return its body.

    Call this **after** the robot has been attached to ``spec`` so the ball's
    free joint is numbered after the robot's root joint (see :data:`BALL_BODY`).
    The joint is named ``f"{name}_freejoint"`` and the geom ``f"{name}_geom"``;
    with the defaults that is :data:`BALL_FREEJOINT` / :data:`BALL_GEOM`.
    ``mujoco`` is imported lazily so this module stays import-light for the
    agent process.
    """
    import mujoco

    body = spec.worldbody.add_body(name=name, pos=[float(v) for v in pos])
    body.add_freejoint(name=f"{name}_freejoint")
    body.add_geom(
        name=f"{name}_geom",
        type=mujoco.mjtGeom.mjGEOM_SPHERE,
        size=[BALL_RADIUS, 0.0, 0.0],
        mass=BALL_MASS,
        group=BALL_GEOM_GROUP,
        rgba=list(_BALL_RGBA),
        friction=list(_BALL_FRICTION),
    )
    return body


@dataclass(frozen=True)
class RoomSpec:
    """A rectangular room of the arena.

    Attributes:
        name: Canonical room name (e.g. ``"kitchen"``).
        aliases: Other names people use for it (e.g. ``"space A"``).
        bounds: ``(xmin, xmax, ymin, ymax)`` in metres, world frame.
        target: ``(x, y, yaw)`` the robot navigates to for "go to <room>":
            a point inside the room just past the hub opening, with the yaw
            looking into the room (away from the central hub) so the head
            camera sees the room's landmark after arriving.
    """

    name: str
    aliases: tuple[str, ...]
    bounds: tuple[float, float, float, float]
    target: tuple[float, float, float]

    def contains(self, x: float, y: float) -> bool:
        xmin, xmax, ymin, ymax = self.bounds
        return xmin <= x <= xmax and ymin <= y <= ymax

    @property
    def center(self) -> tuple[float, float]:
        xmin, xmax, ymin, ymax = self.bounds
        return ((xmin + xmax) / 2.0, (ymin + ymax) / 2.0)


@dataclass(frozen=True)
class PlaceRecord:
    """One entry of the places memory, whatever its kind.

    ``x``/``y``/``yaw`` are the navigation target of the place: the object's
    own position for objects and tagged spots, the room's ``target`` for rooms.
    ``bounds`` is only set for rooms.
    """

    kind: str
    name: str
    aliases: tuple[str, ...]
    x: float
    y: float
    yaw: float
    bounds: tuple[float, float, float, float] | None = None

    @property
    def is_room(self) -> bool:
        return self.kind == "room"

    def distance_to(self, x: float, y: float) -> float:
        return math.hypot(self.x - x, self.y - y)


MICRODUCK_ROOMS: dict[str, RoomSpec] = {
    "kitchen": RoomSpec(
        name="kitchen",
        aliases=("space A",),
        bounds=(0.0, 2.0, 0.0, 2.0),
        target=(1.2, 1.0, 0.0),
    ),
    "living": RoomSpec(
        name="living",
        aliases=("space B", "living room", "lounge"),
        bounds=(-2.0, 0.0, 0.0, 2.0),
        target=(-1.2, 1.0, 3.14159),
    ),
    "bedroom": RoomSpec(
        name="bedroom",
        aliases=("space C",),
        bounds=(-2.0, 0.0, -2.0, 0.0),
        target=(-1.2, -1.0, 3.14159),
    ),
    "office": RoomSpec(
        name="office",
        aliases=("space D", "study"),
        bounds=(0.0, 2.0, -2.0, 0.0),
        target=(1.2, -1.0, 0.0),
    ),
}

# name -> (x, y) of the static landmark objects in four_room_scene.xml.
MICRODUCK_OBJECTS: dict[str, tuple[float, float]] = {
    "red_box": (1.5, 1.5),
    "blue_box": (-1.5, 1.5),
    "green_cylinder": (-1.5, -1.5),
    "yellow_pillar": (1.5, -1.5),
    "orange_crate": (0.6, 1.7),
}


def scene_id(
    rooms: Mapping[str, RoomSpec] | None = None,
    objects: Mapping[str, tuple[float, float]] | None = None,
) -> str:
    """Stable identifier of a world described by its room / object tables.

    Two blueprints that seed different tables get different ids and therefore
    never see each other's records in a shared database; the same tables
    always map to the same id, so remembered places survive restarts.
    """
    canonical = {
        "rooms": {
            name: {
                "aliases": list(room.aliases),
                "bounds": [float(b) for b in room.bounds],
                "target": [float(t) for t in room.target],
            }
            for name, room in sorted((rooms or {}).items())
        },
        "objects": {name: [float(x), float(y)] for name, (x, y) in sorted((objects or {}).items())},
    }
    digest = hashlib.sha1(json.dumps(canonical, sort_keys=True).encode()).hexdigest()
    return f"scene-{digest[:12]}"


# Words that carry no meaning when someone names a place: "the kitchen",
# "space A please", "go to room B", "kitchen area".
_FILLER_WORDS = frozenset(
    {"please", "go", "goto", "to", "the", "room", "space", "area", "zone", "spot"}
)
_NON_ALNUM = re.compile(r"[^a-z0-9]+")


def normalize_place_query(query: str) -> tuple[str, ...]:
    """Return candidate normalised forms of ``query``, most specific first.

    Lower-cases, replaces every run of non-alphanumerics (spaces, underscores,
    punctuation) by a single space, then also yields the form with leading and
    trailing filler words ("the", "space", "room", "please", "go to", ...)
    stripped. ``"Space a"`` -> ``("space a", "a")`` and ``"the kitchen
    please"`` -> ``("the kitchen please", "kitchen")``. Empty forms are
    dropped.
    """
    base = _NON_ALNUM.sub(" ", query.lower()).strip()
    forms: list[str] = []
    if base:
        forms.append(base)
    words = base.split()
    while words and words[0] in _FILLER_WORDS:
        words.pop(0)
    while words and words[-1] in _FILLER_WORDS:
        words.pop()
    stripped = " ".join(words)
    if stripped and stripped not in forms:
        forms.append(stripped)
    return tuple(forms)


def place_key(name: str) -> str:
    """The identity of a place name: its normalised form (case, punctuation
    and spacing folded), so ``"Charger"``, ``"charger"`` and ``"charger!"``
    are one place and re-tagging overwrites instead of duplicating.
    """
    forms = normalize_place_query(name)
    return forms[0] if forms else name.strip().lower()


def _contains_words(haystack: str, needle: str) -> bool:
    """Whole-word containment on already normalised strings."""
    return f" {needle} " in f" {haystack} "


def _yaw_quaternion(yaw: float) -> Quaternion:
    return Quaternion(0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


class PlacesMemory:
    """dimos memory-backed place store: SqliteStore stream ``places`` of RobotLocation.

    Every place is one :class:`RobotLocation` observation tagged with
    ``{"kind": kind, "name": name, "scene": scene}`` and carrying its pose so
    the store's spatial index (``near``) works. Appending is the only write
    the store offers, so re-adding a name appends a newer record and readers
    keep the latest per ``(kind, place_key(name))`` - the newest record also
    decides the display name, so re-tagging "charger" as "Charger" renames
    rather than duplicates; :meth:`seed` skips records that already exist
    unchanged, which makes it idempotent across restarts.

    All reads are restricted to ``scene`` (see :func:`scene_id`), so a
    database shared by several blueprints never shows one world's rooms,
    objects or remembered spots to another.
    """

    STREAM = "places"

    def __init__(self, db_path: str | Path, scene: str = DEFAULT_SCENE) -> None:
        self.db_path = Path(db_path).expanduser()
        self.db_path.parent.mkdir(parents=True, exist_ok=True)
        self.scene = str(scene) or DEFAULT_SCENE
        self._lock = threading.RLock()
        self._store = SqliteStore(path=str(self.db_path))
        self._stream = self._store.stream(self.STREAM, RobotLocation)

    def _view(self) -> Any:
        """The stream restricted to this memory's scene (tag filter, pushed to SQL)."""
        return self._stream.tags(scene=self.scene)

    # -- lifecycle -----------------------------------------------------------

    def close(self) -> None:
        with self._lock:
            self._store.stop()

    def __enter__(self) -> PlacesMemory:
        return self

    def __exit__(self, *exc: object) -> None:
        self.close()

    # -- writes --------------------------------------------------------------

    def seed(
        self,
        rooms: Mapping[str, RoomSpec] | None = None,
        objects: Mapping[str, tuple[float, float]] | None = None,
    ) -> int:
        """Insert the static rooms and objects that are not already stored.

        Idempotent by ``(kind, name)`` within this scene: a record is only
        appended when no record with that key exists or the stored one
        differs (moved object, changed alias list). Returns the number of
        records written.
        """
        written = 0
        with self._lock:
            existing = {(r.kind, place_key(r.name)): r for r in self.all()}
            for name, room in (rooms or {}).items():
                x, y, yaw = room.target
                record = PlaceRecord(
                    "room", name, tuple(room.aliases), float(x), float(y), float(yaw), room.bounds
                )
                if existing.get(("room", place_key(name))) != record:
                    self.add(
                        name,
                        x,
                        y,
                        yaw,
                        kind="room",
                        metadata={
                            "aliases": list(room.aliases),
                            "bounds": list(room.bounds),
                            "target": list(room.target),
                        },
                    )
                    written += 1
            for name, (x, y) in (objects or {}).items():
                record = PlaceRecord("object", name, (), float(x), float(y), 0.0, None)
                if existing.get(("object", place_key(name))) != record:
                    self.add(name, x, y, 0.0, kind="object")
                    written += 1
        return written

    def add(
        self,
        name: str,
        x: float,
        y: float,
        yaw: float = 0.0,
        *,
        kind: str = "tagged",
        metadata: dict[str, Any] | None = None,
    ) -> RobotLocation:
        """Persist a place in this scene and return the stored :class:`RobotLocation`."""
        x, y, yaw = float(x), float(y), float(yaw)
        meta: dict[str, Any] = {
            "kind": kind,
            "scene": self.scene,
            "aliases": [],
            "bounds": None,
            "target": None,
        }
        meta.update(metadata or {})
        location = RobotLocation(
            name=name,
            position=(x, y, 0.0),
            rotation=(0.0, 0.0, yaw),
            frame_id=PLACES_FRAME,
            metadata=meta,
        )
        pose = PoseStamped(
            ts=location.timestamp,
            frame_id=PLACES_FRAME,
            position=Vector3(x, y, 0.0),
            orientation=_yaw_quaternion(yaw),
        )
        with self._lock:
            self._stream.append(
                location,
                ts=location.timestamp,
                pose=pose,
                tags={"kind": kind, "name": name, "scene": self.scene},
            )
        return location

    # -- reads ---------------------------------------------------------------

    @staticmethod
    def _to_record(location: RobotLocation) -> PlaceRecord:
        meta = location.metadata or {}
        kind = str(meta.get("kind", "tagged"))
        bounds = meta.get("bounds")
        return PlaceRecord(
            kind=kind,
            name=location.name,
            aliases=tuple(str(a) for a in (meta.get("aliases") or ())),
            x=float(location.position[0]),
            y=float(location.position[1]),
            yaw=float(location.rotation[2]),
            bounds=tuple(float(b) for b in bounds) if bounds else None,  # type: ignore[arg-type]
        )

    @staticmethod
    def _latest_per_key(observations: list[Any]) -> list[PlaceRecord]:
        """Collapse duplicates to the newest record per ``(kind, place_key(name))``.

        Keeps first-insertion order so seeded rooms list in display order;
        the newest record supplies the display name.
        """
        by_key: dict[tuple[str, str], tuple[float, int, PlaceRecord]] = {}
        for obs in sorted(observations, key=lambda o: (o.ts, o.id)):
            record = PlacesMemory._to_record(obs.data)
            key = (record.kind, place_key(record.name))
            if key in by_key:
                by_key[key] = (by_key[key][0], by_key[key][1], record)
            else:
                by_key[key] = (obs.ts, obs.id, record)
        return [rec for _ts, _id, rec in sorted(by_key.values(), key=lambda v: (v[0], v[1]))]

    def all(self) -> list[PlaceRecord]:
        """Every place of this scene: rooms first (insertion order), then objects, then tagged."""
        with self._lock:
            records = self._latest_per_key(self._view().to_list())
        order = {"room": 0, "object": 1}
        return sorted(records, key=lambda r: order.get(r.kind, 2))

    def rooms(self) -> list[PlaceRecord]:
        return [r for r in self.all() if r.kind == "room"]

    def matches(self, query: str, kind: str | None = None) -> list[PlaceRecord]:
        """Every record a free-form place name could mean.

        An exact match on a canonical name or an alias (after
        :func:`normalize_place_query`) yields exactly that record. Otherwise
        whole-word containment either way is tried and *all* distinct hits
        are returned in table order - so ``"box"`` yields ``red_box`` and
        ``blue_box`` and the caller can ask which one was meant. A query made
        of filler words only (``"room"``, ``"space"``) matches nothing by
        containment.
        """
        forms = normalize_place_query(query)
        if not forms:
            return []
        records = [r for r in self.all() if kind is None or r.kind == kind]
        # (normalised key, record, key-is-canonical-name). Aliases contribute
        # their filler-stripped form too, so "room A" / "A" hit "space A".
        keyed: list[tuple[str, PlaceRecord, bool]] = []
        for record in records:
            for key in normalize_place_query(record.name):
                keyed.append((key, record, True))
            for alias in record.aliases:
                for key in normalize_place_query(alias):
                    keyed.append((key, record, False))

        for form in forms:
            for key, record, is_name in keyed:
                if is_name and key == form:
                    return [record]
            for key, record, is_name in keyed:
                if not is_name and key == form:
                    return [record]

        if all(word in _FILLER_WORDS for word in forms[0].split()):
            return []  # "room" / "space" alone would hit every alias

        for form in forms:
            hits: dict[tuple[str, str], PlaceRecord] = {}
            for key, record, _ in keyed:
                if len(key) < 3:
                    continue  # "a" from "space A" would match everything
                if _contains_words(form, key) or _contains_words(key, form):
                    hits.setdefault((record.kind, record.name), record)
            if hits:
                return list(hits.values())
        return []

    def find(self, query: str, kind: str | None = None) -> PlaceRecord | None:
        """Resolve a free-form place name to one record, or ``None``.

        Matching order: exact canonical name, exact alias, then whole-word
        containment either way. ``"space A"``, ``"Space a"``, ``"the
        kitchen"`` and ``"kitchen please"`` all resolve to the kitchen;
        ``"red box"`` resolves to ``red_box``. A query that only matches by
        containment and fits several places (``"box"``) is ambiguous and
        yields ``None``; use :meth:`matches` to list the candidates.
        """
        candidates = self.matches(query, kind)
        return candidates[0] if len(candidates) == 1 else None

    def room_at(self, x: float, y: float) -> PlaceRecord | None:
        """The room containing ``(x, y)``; ``None`` in the open hub or outside."""
        if abs(x) < HUB_HALF_EXTENT and abs(y) < HUB_HALF_EXTENT:
            return None
        for record in self.rooms():
            if record.bounds is None:
                continue
            xmin, xmax, ymin, ymax = record.bounds
            if xmin <= x <= xmax and ymin <= y <= ymax:
                return record
        return None

    def near(self, x: float, y: float, radius: float) -> list[PlaceRecord]:
        """Places of this scene whose pose lies within ``radius`` metres, nearest first."""
        with self._lock:
            observations = self._view().near((float(x), float(y), 0.0), float(radius)).to_list()
            records = self._latest_per_key(observations)
        return sorted(records, key=lambda r: r.distance_to(x, y))

    def to_json(self, t: float | None = None) -> str:
        """The ``places`` channel payload (compact JSON, world frame)."""
        rooms: list[dict[str, Any]] = []
        objects: list[dict[str, Any]] = []
        tagged: list[dict[str, Any]] = []
        for record in self.all():
            if record.kind == "room":
                rooms.append(
                    {
                        "name": record.name,
                        "aliases": list(record.aliases),
                        "bounds": list(record.bounds or ()),
                        "target": [record.x, record.y, record.yaw],
                    }
                )
            elif record.kind == "object":
                objects.append({"name": record.name, "x": record.x, "y": record.y})
            else:
                tagged.append(
                    {"name": record.name, "x": record.x, "y": record.y, "yaw": record.yaw}
                )
        payload = {
            "frame": PLACES_FRAME,
            "rooms": rooms,
            "objects": objects,
            "tagged": tagged,
            "t": time.time() if t is None else t,
        }
        return json.dumps(payload, separators=(",", ":"))
