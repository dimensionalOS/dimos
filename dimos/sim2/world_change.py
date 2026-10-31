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

"""The world as sim2 sends it: a WorldChange per frame, of only what changed (world_msgs, from dimos-lcm).

A backend writes its frames with a WorldWriter, saying itself what changed; every other module keeps a
WorldMirror, which applies each frame it gets. There is no catching up: a missed frame is missed, so the
Simulation steps only once every module has started.
"""

import asyncio
from collections.abc import AsyncIterator, Callable, Sequence
from dataclasses import dataclass, field
import struct
from threading import RLock
from typing import Any

import cbor2
from dimos_lcm.world_msgs import Change, WorldChange
import numpy as np
from numpy.typing import NDArray

WORLD = 0
# Event entities every world has, at reserved ids.
EVENTS = {
    # On a robot: the world holds until it acts (a lockstep control tick).
    "events/act": 1,
    # On the world: advance a paused world, CBOR {"steps": n}.
    "events/step": 2,
    # On the world: back to the initial state, at sim time zero.
    "events/reset": 3,
    # On an entity: do one of its named actions (open a door), CBOR {"action": name}.
    "events/interact": 4,
    # On an action's target: the sim could not carry it out, CBOR {"reason": text}.
    "events/refused": 5,
}
RESERVED = {"world": WORLD, **EVENTS}
_FIRST_ID = 256

# Tags every backend uses.
ROBOT, OBJECT, JOINT, PAUSED = "robot", "object", "joint", "paused"
# The world's CBOR key holding the wall time at sim time zero: what sim-timed messages are stamped from.
EPOCH = "epoch"

_POSE = struct.Struct(">7d")
_VELOCITY = struct.Struct(">6d")
_JOINT_POSITION = np.dtype(">f8")


def _string(text: str) -> bytes:
    encoded = text.encode()
    return struct.pack(">i", len(encoded) + 1) + encoded + b"\0"


def _blob(data: bytes) -> bytes:
    return struct.pack(">i", len(data)) + data


def _change(
    kind: int, entity: int, updates: Sequence[tuple[int, bytes]] = (), head: bytes = b""
) -> Change:
    data = head + b"".join(payload for _, payload in updates)
    return Change(
        type=kind,
        entity_id=entity,
        update_types_length=len(updates),
        update_types=bytes(update for update, _ in updates),
        data_length=len(data),
        data=data,
    )


# Each update type's payload: the LCM encoding, without fingerprint, of its world_msgs struct.
def pose_update(pose: Sequence[float]) -> tuple[int, bytes]:
    return Change.WORLD_POSE_SET, _POSE.pack(*pose)


def velocity_update(velocity: Sequence[float]) -> tuple[int, bytes]:
    return Change.WORLD_VELOCITY_SET, _VELOCITY.pack(*velocity)


def tag_update(tag: str, on: bool = True) -> tuple[int, bytes]:
    return (Change.ADDED_TAG if on else Change.REMOVED_TAG), _string(tag)


def cbor_update(key: str, value: Any) -> tuple[int, bytes]:
    return Change.CBOR_KEY_SET, _string(key) + _blob(cbor2.dumps(value))


def cbor_remove(key: str) -> tuple[int, bytes]:
    return Change.CBOR_KEY_REMOVE, _string(key)


def event_update(event: int, cbor: Any = None) -> tuple[int, bytes]:
    return Change.EVENT, struct.pack(">q", event) + _blob(
        b"" if cbor is None else cbor2.dumps(cbor)
    )


def connection_update(other: int, cbor: Any = None) -> tuple[int, bytes]:
    """A joint relationship to other, or (with cbor) any other relationship."""
    kind = 0 if cbor is None else 1
    return Change.ENTITY_CONNECTION_ADDED, struct.pack(">qb", other, kind) + _blob(
        b"" if cbor is None else cbor2.dumps(cbor)
    )


def disconnection_update(other: int) -> tuple[int, bytes]:
    return Change.ENTITY_CONNECTION_REMOVED, struct.pack(">q", other)


def joint_positions(values: NDArray[np.float64]) -> bytes:
    """One JointPositionSet (position, velocity, effort) per row."""
    return np.ascontiguousarray(values, dtype=_JOINT_POSITION).tobytes()


def joint_targets(values: NDArray[np.float64]) -> bytes:
    """One JointTargetSet (position, velocity, kp, kd, feedforward) per row; NaN is none given."""
    return np.ascontiguousarray(values, dtype=_JOINT_POSITION).tobytes()


def add_entity(entity: int, path: str, updates: Sequence[tuple[int, bytes]] = ()) -> Change:
    return _change(Change.ADD_ENTITY, entity, updates, _string(path))


def delete_entity(entity: int) -> Change:
    return _change(Change.DELETE_ENTITY, entity)


def entity_update(entity: int, *updates: tuple[int, bytes]) -> Change:
    return _change(Change.ENTITY_UPDATE, entity, updates)


def group_set(group: int, members: Sequence[int]) -> Change:
    return _change(
        Change.GROUP_SET, group, (), struct.pack(f">i{len(members)}q", len(members), *members)
    )


def group_update(group: int, update: int, payloads: bytes) -> Change:
    return _change(Change.GROUP_UPDATE, group, ((update, payloads),))


class WorldWriter:
    """What a backend writes each frame with: it names what changed, this assigns ids and encodes it."""

    def __init__(self) -> None:
        self.ids: dict[str, int] = dict(RESERVED)
        self.paths: dict[int, str] = {i: p for p, i in self.ids.items()}
        # A robot's joints group id -> (the robot, its joint names).
        self.joint_groups: dict[int, tuple[str, tuple[str, ...]]] = {}
        self._joint_groups_by_robot: dict[str, int] = {}
        self._next = _FIRST_ID
        self._changes: list[Change] = []
        self.frame = 0

    def _assign(self, path: str) -> int:
        if path not in self.ids:
            self.ids[path], self.paths[self._next] = self._next, path
            self._next += 1
        return self.ids[path]

    def add(self, path: str, *updates: tuple[int, bytes]) -> int:
        entity = self._assign(path)
        self._changes.append(add_entity(entity, path, updates))
        return entity

    def remove(self, path: str) -> None:
        entity = self.ids.pop(path)
        del self.paths[entity]
        self._changes.append(delete_entity(entity))

    def update(self, path: str, *updates: tuple[int, bytes]) -> None:
        self._changes.append(entity_update(self.ids[path], *updates))

    def event(self, path: str, event: str, cbor: Any = None) -> None:
        self.update(path, event_update(self.ids[event], cbor))

    def add_joints(self, robot: str, names: Sequence[str]) -> None:
        """A robot's joints as entities ("<robot>/<joint>"), connected to it, and the group its joint updates go to."""
        members = [
            self.add(f"{robot}/{name}", tag_update(JOINT), connection_update(self.ids[robot]))
            for name in names
        ]
        group = self.add(f"{robot}/joints")
        self.joint_groups[group] = (robot, tuple(names))
        self._joint_groups_by_robot[robot] = group
        self._changes.append(group_set(group, members))

    def joints(self, robot: str, values: NDArray[np.float64]) -> None:
        """Each of the robot's joints' position, velocity and effort, a row each in add_joints' order."""
        group = self._joint_groups_by_robot[robot]
        self._changes.append(
            group_update(group, Change.JOINT_POSITION_SET, joint_positions(values))
        )

    def end_frame(self, sim_time: float) -> WorldChange:
        self.frame += 1
        changes, self._changes = self._changes, []
        return WorldChange(
            ts=round(sim_time * 1e9),
            frame=self.frame,
            changes_length=len(changes),
            changes=changes,
        )


@dataclass
class Entity:
    path: str
    tags: set[str] = field(default_factory=set)
    cbor: dict[str, Any] = field(default_factory=dict)
    # x, y, z, qx, qy, qz, qw in the world frame.
    pose: NDArray[np.float64] | None = None
    # Linear then angular, world axes.
    velocity: NDArray[np.float64] | None = None
    # A joint's position, velocity, effort.
    joint: NDArray[np.float64] | None = None
    # Other entity id -> the relationship's CBOR (None for a joint relationship).
    connections: dict[int, Any] = field(default_factory=dict)
    mesh: int | None = None


class _Reader:
    def __init__(self, data: bytes) -> None:
        self.data, self.at = data, 0

    def take(self, layout: str) -> tuple[Any, ...]:
        values = struct.unpack_from(layout, self.data, self.at)
        self.at += struct.calcsize(layout)
        return values

    def string(self) -> str:
        (length,) = self.take(">i")
        text = self.data[self.at : self.at + length - 1].decode()
        self.at += length
        return text

    def blob(self) -> bytes:
        (length,) = self.take(">i")
        blob = self.data[self.at : self.at + length]
        self.at += length
        return blob


# The fixed-size payloads, by update type.
_SIZES = {
    Change.WORLD_POSE_SET: 56,
    Change.WORLD_VELOCITY_SET: 48,
    Change.MESH_SET: 8,
    Change.JOINT_POSITION_SET: 24,
    Change.JOINT_TARGET_SET: 40,
    Change.ENTITY_CONNECTION_REMOVED: 8,
}


def read_update(update: int, reader: _Reader) -> Any:
    """One update's payload, decoded."""
    if update in (Change.ADDED_TAG, Change.REMOVED_TAG, Change.CBOR_KEY_REMOVE):
        return reader.string()
    if update == Change.CBOR_KEY_SET:
        key = reader.string()
        return key, cbor2.loads(reader.blob())
    if update == Change.EVENT:
        (event,) = reader.take(">q")
        blob = reader.blob()
        return event, cbor2.loads(blob) if blob else None
    if update == Change.ENTITY_CONNECTION_ADDED:
        other, kind = reader.take(">qb")
        blob = reader.blob()
        return other, cbor2.loads(blob) if kind == 1 else None
    if update == Change.CUSTOM_UPDATE:
        return reader.blob()
    if update == Change.ENTITY_CONNECTION_REMOVED:
        return reader.take(">q")[0]
    if update == Change.MESH_SET:
        return reader.take(">q")[0]
    if update == Change.WRENCH:
        (frame,) = reader.take(">q")
        return frame, np.array(reader.take(">10d"))
    size = _SIZES[update]
    values = np.frombuffer(reader.data, _JOINT_POSITION, size // 8, reader.at).astype(np.float64)
    reader.at += size
    return values


def read_change(change: Change) -> tuple[str | None, list[tuple[int, Any]]]:
    """An entity change's path (for add_entity) and its updates, decoded."""
    reader = _Reader(bytes(change.data))
    path = reader.string() if change.type == Change.ADD_ENTITY else None
    return path, [(update, read_update(update, reader)) for update in change.update_types]


def read_group(change: Change) -> list[int]:
    (count,) = struct.unpack_from(">i", change.data)
    return list(struct.unpack_from(f">{count}q", change.data, 4))


def read_group_payloads(change: Change) -> tuple[int, NDArray[np.float64]]:
    """A group_update of fixed-size payloads: its update type and a row per member."""
    update = change.update_types[0]
    width = _SIZES[update] // 8
    return update, np.frombuffer(bytes(change.data), _JOINT_POSITION).astype(np.float64).reshape(
        -1, width
    )


class WorldMirror:
    """The world as the WorldChanges this module got say it is.

    subscribe applies every frame on the thread that receives it, never dropping one for being busy;
    read under lock. Work paced by sim time awaits reached().
    """

    def __init__(self) -> None:
        self.lock = RLock()
        self.frame = 0
        self.time = 0.0
        self.entities: dict[int, Entity] = {}
        self.ids: dict[str, int] = {}
        self.groups: dict[int, list[int]] = {}
        # This frame's events: (entity id, event id, CBOR).
        self.events: list[tuple[int, int, Any]] = []
        self._waiters: list[tuple[float, asyncio.AbstractEventLoop, asyncio.Future[None]]] = []
        self._listeners: list[Callable[[WorldMirror], None]] = []

    def subscribe(self, stream: Any) -> None:
        stream.subscribe(self.apply)

    def on_frame(self, listener: Callable[["WorldMirror"], None]) -> None:
        """Called on the receiving thread after each frame is applied."""
        self._listeners.append(listener)

    def apply(self, message: WorldChange) -> None:
        with self.lock:
            self.events = []
            for change in message.changes:
                self._apply(change)
            rewound = message.ts / 1e9 < self.time
            self.frame, self.time = message.frame, message.ts / 1e9
            # A reset rewinds the world: every waiter wakes and looks again.
            due = [w for w in self._waiters if rewound or w[0] <= self.time]
            self._waiters = [w for w in self._waiters if not rewound and w[0] > self.time]
        for _, loop, future in due:
            loop.call_soon_threadsafe(lambda f=future: f.done() or f.set_result(None))
        for listener in self._listeners:
            listener(self)

    def _apply(self, change: Change) -> None:
        kind = change.type
        if kind == Change.DELETE_ENTITY:
            entity = self.entities.pop(change.entity_id, None)
            if entity is not None:
                self.ids.pop(entity.path, None)
            return
        if kind == Change.GROUP_SET:
            self.groups[change.entity_id] = read_group(change)
            return
        if kind == Change.GROUP_UPDATE:
            update, rows = read_group_payloads(change)
            for member, row in zip(self.groups.get(change.entity_id, ()), rows, strict=False):
                if member in self.entities:
                    self._set(self.entities[member], member, update, row)
            return
        if kind not in (Change.ADD_ENTITY, Change.ENTITY_UPDATE):
            return
        path, updates = read_change(change)
        if path is not None:
            self.entities[change.entity_id] = Entity(path)
            self.ids[path] = change.entity_id
        entity = self.entities.get(change.entity_id)
        if entity is None:
            return
        for update, value in updates:
            self._set(entity, change.entity_id, update, value)

    def _set(self, entity: Entity, entity_id: int, update: int, value: Any) -> None:
        if update == Change.WORLD_POSE_SET:
            entity.pose = value
        elif update == Change.WORLD_VELOCITY_SET:
            entity.velocity = value
        elif update == Change.JOINT_POSITION_SET:
            entity.joint = value
        elif update == Change.ADDED_TAG:
            entity.tags.add(value)
        elif update == Change.REMOVED_TAG:
            entity.tags.discard(value)
        elif update == Change.CBOR_KEY_SET:
            entity.cbor[value[0]] = value[1]
        elif update == Change.CBOR_KEY_REMOVE:
            entity.cbor.pop(value, None)
        elif update == Change.EVENT:
            self.events.append((entity_id, *value))
        elif update == Change.ENTITY_CONNECTION_ADDED:
            entity.connections[value[0]] = value[1]
        elif update == Change.ENTITY_CONNECTION_REMOVED:
            if value == 0:
                entity.connections.clear()
            else:
                entity.connections.pop(value, None)
        elif update == Change.MESH_SET:
            entity.mesh = value

    def entity(self, path: str) -> Entity | None:
        entity_id = self.ids.get(path)
        return None if entity_id is None else self.entities.get(entity_id)

    def tagged(self, tag: str) -> list[Entity]:
        return [entity for entity in self.entities.values() if tag in entity.tags]

    @property
    def paused(self) -> bool:
        world = self.entities.get(WORLD)
        return world is not None and PAUSED in world.tags

    def stamp(self) -> float:
        """The wall-clock-like timestamp of the current frame: the world's epoch plus its sim time."""
        world = self.entities.get(WORLD)
        return (world.cbor.get(EPOCH, 0.0) if world is not None else 0.0) + self.time

    def joint_group(self, robot: str) -> list[int]:
        return self.groups.get(self.ids.get(f"{robot}/joints", -1), [])

    def joints(self, robot: str) -> tuple[list[str], NDArray[np.float64]]:
        """The robot's joint names (without its "<robot>/") and a row of position, velocity, effort each."""
        members = [self.entities[m] for m in self.joint_group(robot) if m in self.entities]
        names = [m.path.removeprefix(f"{robot}/") for m in members]
        rows = [m.joint if m.joint is not None else np.zeros(3) for m in members]
        return names, np.array(rows).reshape(-1, 3)

    def has_event(self, path: str, event: str) -> bool:
        entity_id, event_id = self.ids.get(path), EVENTS[event]
        return any(e == entity_id and kind == event_id for e, kind, _ in self.events)

    async def reached(self, sim_time: float) -> None:
        """Returns once a frame has come and the world's sim time is at least sim_time (or the world reset)."""
        loop = asyncio.get_running_loop()
        future: asyncio.Future[None] = loop.create_future()
        with self.lock:
            if self.frame and self.time >= sim_time:
                return
            self._waiters.append((sim_time, loop, future))
        await future

    async def ticks(self, rate_hz: float) -> AsyncIterator[float]:
        """Every 1 / rate_hz of sim time, however fast the world runs; as often as the caller keeps up, if it can't."""
        period, due = 1.0 / rate_hz, 0.0
        while True:
            await self.reached(due)
            yield self.time
            now = self.time
            due += period
            # A whole period behind, it goes again at once rather than in a burst; after a reset, it starts over.
            if now >= due + period or now < due - 2 * period:
                due = now
