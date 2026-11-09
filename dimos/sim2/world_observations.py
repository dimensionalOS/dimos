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


"""The world as sim2 sends it: a WorldObservations per frame, of only what changed (world_msgs,
from dimos-lcm).

Three kinds of observation: a body's Position (its pose), a joint's JointState, and an entity's
CborKeyValues (its "path" when it first appears, "act" when the world waits for a robot). A
body's velocity is a difference of its poses over time.

A backend writes its frames with a WorldWriter; every other module keeps a WorldMirror. There
is no catching up: a missed frame is missed, so a world steps only once every module started.
"""

import asyncio
from collections.abc import AsyncIterator, Callable, Sequence
from dataclasses import dataclass, field
from threading import RLock
import time
from typing import Any

import cbor2
from dimos_lcm.world_msgs import CborKeyValue, JointState, Position, WorldObservations
import numpy as np
from numpy.typing import NDArray

WORLD = 0
# An entity's path ("go2", "go2/FL_calf_joint"), sent when it first appears; a "<robot>/<joint>" is a joint.
PATH = "path"
# On a robot: the world holds until it acts (a lockstep control tick).
ACT = "act"
# On an action's target: the sim could not carry it out, {"reason": text}.
REFUSED = "refused"


def cbor_key_value(entity: int, key: str, value: Any) -> CborKeyValue:
    encoded = cbor2.dumps(value)
    return CborKeyValue(entity_id=entity, key=key, value_length=len(encoded), value=encoded)


def position(entity: int, pose: Sequence[float]) -> Position:
    return Position(entity_id=entity, pose=list(map(float, pose)))


def joint_state(entity: int, position: float, velocity: float, effort: float) -> JointState:
    return JointState(
        entity_id=entity, position=float(position), velocity=float(velocity), effort=float(effort)
    )


class WorldWriter:
    """What a backend writes each frame with: it names what changed, this assigns ids and encodes it."""

    def __init__(self) -> None:
        self.ids: dict[str, int] = {"world": WORLD}
        self.paths: dict[int, str] = {WORLD: "world"}
        self._next = WORLD + 1
        self._cbor: list[CborKeyValue] = []
        self._positions: list[Position] = []
        self._joint_states: list[JointState] = []
        self.frame = 0
        # Wall time at sim time zero, set by the first frame: every ts is it plus sim time.
        self.start_ns: int | None = None

    def add(self, path: str, **cbor: Any) -> int:
        if path not in self.ids:
            self.ids[path], self.paths[self._next] = self._next, path
            self._next += 1
        entity = self.ids[path]
        self._cbor.append(cbor_key_value(entity, PATH, path))
        for key, value in cbor.items():
            self._cbor.append(cbor_key_value(entity, key, value))
        return entity

    def set(self, path: str, key: str, value: Any) -> None:
        self._cbor.append(cbor_key_value(self.ids[path], key, value))

    def position(self, path: str, pose: Sequence[float]) -> None:
        self._positions.append(position(self.ids[path], pose))

    def joint(self, path: str, position: float, velocity: float, effort: float) -> None:
        self._joint_states.append(joint_state(self.ids[path], position, velocity, effort))

    def end_frame(self, sim_time: float) -> WorldObservations:
        sim_ns = round(sim_time * 1e9)
        if self.start_ns is None:
            self.start_ns = time.time_ns() - sim_ns
        self.frame += 1
        cbor, self._cbor = self._cbor, []
        positions, self._positions = self._positions, []
        joint_states, self._joint_states = self._joint_states, []
        return WorldObservations(
            ts=self.start_ns + sim_ns,
            frame=self.frame,
            cbor_key_values_length=len(cbor),
            cbor_key_values=cbor,
            positions_length=len(positions),
            positions=positions,
            joint_states_length=len(joint_states),
            joint_states=joint_states,
        )


@dataclass
class Entity:
    path: str
    cbor: dict[str, Any] = field(default_factory=dict)
    # A body's x, y, z, qx, qy, qz, qw in the world frame.
    pose: NDArray[np.float64] | None = None
    # Its pose's change over the last frame it moved in, zero once it stops: linear then angular, world axes.
    velocity: NDArray[np.float64] | None = None
    # The ts of pose, nanoseconds.
    ts: int = 0
    # A joint's position, velocity, effort.
    joint: NDArray[np.float64] | None = None

    def _move(self, pose: NDArray[np.float64], ts: int) -> None:
        if self.pose is not None and ts > self.ts:
            self.velocity = _difference(self.pose, pose, (ts - self.ts) / 1e9)
        elif self.velocity is None:
            self.velocity = np.zeros(6)
        self.pose, self.ts = pose, ts


def _difference(
    before: NDArray[np.float64], after: NDArray[np.float64], dt: float
) -> NDArray[np.float64]:
    """Linear velocity, then the world-axes rotation from before to after as an angular velocity."""
    (x0, y0, z0, w0), (x1, y1, z1, w1) = before[3:], after[3:]
    # after * conjugate(before), as (x, y, z, w).
    w = w1 * w0 + x1 * x0 + y1 * y0 + z1 * z0
    v = np.array(
        [
            -w1 * x0 + x1 * w0 - y1 * z0 + z1 * y0,
            -w1 * y0 + y1 * w0 - z1 * x0 + x1 * z0,
            -w1 * z0 + z1 * w0 - x1 * y0 + y1 * x0,
        ]
    )
    if w < 0:
        w, v = -w, -v
    norm = float(np.linalg.norm(v))
    angle = 2.0 * np.arctan2(norm, w)
    angular = v * (angle / norm) if norm > 1e-12 else 2.0 * v
    return np.concatenate([(after[:3] - before[:3]) / dt, angular / dt])


class WorldMirror:
    """The world as the WorldObservations this module got say it is.

    subscribe applies every frame on the thread that receives it, never dropping one for being busy;
    read under lock. Work paced by sim time awaits reached().
    """

    def __init__(self) -> None:
        self.lock = RLock()
        self.frame = 0
        # The latest frame's ts, in seconds: sim time plus the wall time the sim started at.
        self.time = 0.0
        self.ts = 0
        self.entities: dict[int, Entity] = {}
        self.ids: dict[str, int] = {}
        # This frame's CBOR: (entity id, key, value).
        self.received: list[tuple[int, str, Any]] = []
        self._waiters: list[tuple[float, asyncio.AbstractEventLoop, asyncio.Future[None]]] = []
        self._listeners: list[Callable[[WorldMirror], None]] = []

    def subscribe(self, stream: Any) -> None:
        stream.subscribe(self.apply)

    def on_frame(self, listener: Callable[["WorldMirror"], None]) -> None:
        """Called on the receiving thread after each frame is applied."""
        self._listeners.append(listener)

    def apply(self, message: WorldObservations) -> None:
        with self.lock:
            self.received = []
            for item in message.cbor_key_values:
                value = cbor2.loads(bytes(item.value))
                if item.key == PATH:
                    self.entities.setdefault(item.entity_id, Entity(value))
                    self.ids[value] = item.entity_id
                entity = self.entities.get(item.entity_id)
                if entity is not None:
                    entity.cbor[item.key] = value
                self.received.append((item.entity_id, item.key, value))
            moved = set()
            for item in message.positions:
                entity = self.entities.get(item.entity_id)
                if entity is not None:
                    entity._move(np.array(item.pose, dtype=np.float64), message.ts)
                    moved.add(item.entity_id)
            for item in message.joint_states:
                entity = self.entities.get(item.entity_id)
                if entity is not None:
                    entity.joint = np.array([item.position, item.velocity, item.effort])
            # Only what moved is sent, so what was not, while sim time passed, is still.
            for entity_id, entity in self.entities.items():
                if (
                    entity_id not in moved
                    and entity.velocity is not None
                    and message.ts > entity.ts
                ):
                    entity.velocity = np.zeros_like(entity.velocity)
            self.frame, self.ts, self.time = message.frame, message.ts, message.ts / 1e9
            due = [w for w in self._waiters if w[0] <= self.time]
            self._waiters = [w for w in self._waiters if w[0] > self.time]
        for _, loop, future in due:
            loop.call_soon_threadsafe(lambda f=future: f.done() or f.set_result(None))
        for listener in self._listeners:
            listener(self)

    def entity(self, path: str) -> Entity | None:
        entity_id = self.ids.get(path)
        return None if entity_id is None else self.entities.get(entity_id)

    def stamp(self) -> float:
        """The current frame's timestamp, what sim-timed messages are stamped with."""
        return self.time

    def joint_ids(self, robot: str) -> list[int]:
        """The robot's joints' ids, in the order the world added them."""
        prefix = f"{robot}/"
        return sorted(i for path, i in self.ids.items() if path.startswith(prefix))

    def joints(self, robot: str) -> tuple[list[str], NDArray[np.float64]]:
        """The robot's joint names (without its "<robot>/") and a row of position, velocity, effort each."""
        members = [self.entities[i] for i in self.joint_ids(robot) if i in self.entities]
        names = [m.path.removeprefix(f"{robot}/") for m in members]
        rows = [m.joint if m.joint is not None else np.zeros(3) for m in members]
        return names, np.array(rows, dtype=np.float64).reshape(-1, 3)

    def got(self, path: str, key: str) -> bool:
        """Whether this frame set key on the entity at path."""
        entity_id = self.ids.get(path)
        return any(e == entity_id and k == key for e, k, _ in self.received)

    async def reached(self, ts: float) -> None:
        """Returns once a frame has come with a ts of at least ts seconds."""
        loop = asyncio.get_running_loop()
        future: asyncio.Future[None] = loop.create_future()
        with self.lock:
            if self.frame and self.time >= ts:
                return
            self._waiters.append((ts, loop, future))
        await future

    async def ticks(self, rate_hz: float) -> AsyncIterator[float]:
        """Every 1 / rate_hz of sim time, however fast the world runs; as often as the caller keeps up, if it can't."""
        period, due = 1.0 / rate_hz, 0.0
        while True:
            await self.reached(due)
            yield self.time
            now = self.time
            due += period
            # A whole period behind, it goes again at once rather than in a burst.
            if now >= due + period:
                due = now
