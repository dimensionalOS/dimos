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


import asyncio
from collections.abc import AsyncIterator, Callable, Sequence
from dataclasses import dataclass, field
from threading import RLock
import time
from typing import Any

import cbor2
from dimos_lcm.world_msgs import BodyState, CborKeyValue, JointState, WorldObservations
import numpy as np
from numpy.typing import NDArray

WORLD = 0
PATH = "path"
REFUSED = "refused"
_BODY = "_body"
_JOINT = "_joint"


def cbor_key_value(entity: int, key: str, value: Any) -> CborKeyValue:
    encoded = cbor2.dumps(value)
    return CborKeyValue(entity_id=entity, key=key, value_length=len(encoded), value=encoded)


def body_state(
    entity: int, pose: Sequence[float], velocity: Sequence[float], acceleration: Sequence[float]
) -> BodyState:
    return BodyState(
        entity_id=entity,
        pose=list(map(float, pose)),
        velocity=list(map(float, velocity)),
        acceleration=list(map(float, acceleration)),
    )


def joint_state(entity: int, position: float, velocity: float, effort: float) -> JointState:
    return JointState(
        entity_id=entity, position=float(position), velocity=float(velocity), effort=float(effort)
    )


def merged(earlier: WorldObservations, later: WorldObservations) -> WorldObservations:
    """One frame saying what both did: every cbor value in order, the latest body state and joint of each entity."""
    cbor = [*earlier.cbor_key_values, *later.cbor_key_values]
    body_states = {b.entity_id: b for b in [*earlier.body_states, *later.body_states]}
    joint_states = {j.entity_id: j for j in [*earlier.joint_states, *later.joint_states]}
    return WorldObservations(
        ts=later.ts,
        frame=later.frame,
        cbor_key_values_length=len(cbor),
        cbor_key_values=cbor,
        body_states_length=len(body_states),
        body_states=list(body_states.values()),
        joint_states_length=len(joint_states),
        joint_states=list(joint_states.values()),
    )


def _resolve(future: "asyncio.Future[None]") -> None:
    if not future.done():
        future.set_result(None)


class WorldWriter:
    def __init__(self) -> None:
        self.ids: dict[str, int] = {"world": WORLD}
        self.paths: dict[int, str] = {WORLD: "world"}
        self._next = WORLD + 1
        self._cbor: list[CborKeyValue] = []
        self._body_states: list[BodyState] = []
        self._joint_states: list[JointState] = []
        self.frame = 0
        self.start_ns: int | None = None
        # Everything said about each entity so far, for a listener that missed it: see snapshot().
        self._state: dict[int, dict[str, Any]] = {}
        self._last_ts = 0

    def add(self, path: str, **cbor: Any) -> int:
        if path not in self.ids:
            self.ids[path], self.paths[self._next] = self._next, path
            self._next += 1
        entity = self.ids[path]
        self._cbor.append(cbor_key_value(entity, PATH, path))
        self._state.setdefault(entity, {})[PATH] = path
        for key, value in cbor.items():
            self.set(path, key, value)
        return entity

    def set(self, path: str, key: str, value: Any) -> None:
        self._cbor.append(cbor_key_value(self.ids[path], key, value))
        self._state.setdefault(self.ids[path], {})[key] = value

    def body_state(
        self,
        path: str,
        pose: Sequence[float],
        velocity: Sequence[float],
        acceleration: Sequence[float],
    ) -> None:
        """Pose x, y, z, qx, qy, qz, qw; velocity and acceleration linear then angular, world axes, at the body's origin; no gravity."""
        state = body_state(self.ids[path], pose, velocity, acceleration)
        self._body_states.append(state)
        self._state.setdefault(self.ids[path], {})[_BODY] = [
            state.pose,
            state.velocity,
            state.acceleration,
        ]

    def joint(self, path: str, position: float, velocity: float, effort: float) -> None:
        self._joint_states.append(joint_state(self.ids[path], position, velocity, effort))
        self._state.setdefault(self.ids[path], {})[_JOINT] = [position, velocity, effort]

    def snapshot(self) -> dict[str, Any]:
        """The last frame's stamp and number, and each entity id to everything said about it so far."""
        return {
            "ts": self._last_ts,
            "frame": self.frame,
            "entities": {entity: dict(state) for entity, state in self._state.items()},
        }

    def rebase(self, sim_time: float) -> None:
        """Stamp ``sim_time`` as now, so the frames after a world paused or fell behind are on wall time again."""
        self.start_ns = time.time_ns() - round(sim_time * 1e9)

    def end_frame(self, sim_time: float) -> WorldObservations:
        sim_ns = round(sim_time * 1e9)
        if self.start_ns is None:
            self.start_ns = time.time_ns() - sim_ns
        self.frame += 1
        self._last_ts = self.start_ns + sim_ns
        cbor, self._cbor = self._cbor, []
        body_states, self._body_states = self._body_states, []
        joint_states, self._joint_states = self._joint_states, []
        return WorldObservations(
            ts=self.start_ns + sim_ns,
            frame=self.frame,
            cbor_key_values_length=len(cbor),
            cbor_key_values=cbor,
            body_states_length=len(body_states),
            body_states=body_states,
            joint_states_length=len(joint_states),
            joint_states=joint_states,
        )


@dataclass
class Entity:
    path: str
    cbor: dict[str, Any] = field(default_factory=dict)
    # A body's x, y, z, qx, qy, qz, qw in the world frame.
    pose: NDArray[np.float64] | None = None
    # Linear then angular, world axes, at the body's origin; acceleration without gravity.
    velocity: NDArray[np.float64] | None = None
    acceleration: NDArray[np.float64] | None = None
    # The ts of pose, nanoseconds.
    ts: int = 0
    # A joint's position, velocity, effort.
    joint: NDArray[np.float64] | None = None

    def _move(self, state: Sequence[Sequence[float]], ts: int) -> None:
        pose, velocity, acceleration = (np.array(part, dtype=np.float64) for part in state)
        self.pose, self.velocity, self.acceleration, self.ts = pose, velocity, acceleration, ts


class WorldMirror:
    def __init__(self) -> None:
        self.lock = RLock()
        self.frame = 0
        self.time = 0.0
        self.ts = 0
        self.entities: dict[int, Entity] = {}
        self.ids: dict[str, int] = {}
        self.received: list[tuple[int, str, Any]] = []
        self._waiters: list[tuple[asyncio.AbstractEventLoop, asyncio.Future[None]]] = []
        self._listeners: list[Callable[[WorldMirror], None]] = []
        self._snapshot: Callable[[], dict[str, Any]] | None = None

    def subscribe(self, stream: Any, snapshot: Callable[[], dict[str, Any]]) -> None:
        """Mirror ``stream``; ``snapshot`` is asked for every entity when a frame names one this has not seen."""
        self._snapshot = snapshot
        stream.subscribe(self.apply)

    def _catch_up(self, message: WorldObservations) -> None:
        mentioned = {item.entity_id for item in message.body_states} | {
            item.entity_id for item in message.joint_states
        }
        if self._snapshot is None or (self.entities and mentioned <= self.entities.keys()):
            return
        self._load(self._snapshot()["entities"], message.ts)

    def _load(self, entities: dict[int, dict[str, Any]], ts: int) -> None:
        for entity_id, state in entities.items():
            entity = self.entities.setdefault(entity_id, Entity(state[PATH]))
            self.ids[state[PATH]] = entity_id
            entity.cbor.update({k: v for k, v in state.items() if k not in (_BODY, _JOINT)})
            if _BODY in state:
                entity._move(state[_BODY], ts)
            if _JOINT in state:
                entity.joint = np.array(state[_JOINT])

    def _pull(self) -> None:
        """Take the world as it is now, for a mirror whose first frame went out before it was listening."""
        assert self._snapshot is not None
        snapshot = self._snapshot()
        with self.lock:
            if self.frame or not snapshot["frame"]:
                return
            self._load(snapshot["entities"], snapshot["ts"])
            self.frame, self.ts, self.time = snapshot["frame"], snapshot["ts"], snapshot["ts"] / 1e9
            waiters, self._waiters = self._waiters, []
        for loop, future in waiters:
            loop.call_soon_threadsafe(_resolve, future)

    def on_frame(self, listener: Callable[["WorldMirror"], None]) -> None:
        self._listeners.append(listener)

    def apply(self, message: WorldObservations) -> None:
        with self.lock:
            self._catch_up(message)
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
            for item in message.body_states:
                entity = self.entities.get(item.entity_id)
                if entity is not None:
                    entity._move((item.pose, item.velocity, item.acceleration), message.ts)
            for item in message.joint_states:
                entity = self.entities.get(item.entity_id)
                if entity is not None:
                    entity.joint = np.array([item.position, item.velocity, item.effort])
            self.frame, self.ts, self.time = message.frame, message.ts, message.ts / 1e9
            waiters, self._waiters = self._waiters, []
        for loop, future in waiters:
            loop.call_soon_threadsafe(_resolve, future)
        for listener in self._listeners:
            listener(self)

    def entity(self, path: str) -> Entity | None:
        with self.lock:
            entity_id = self.ids.get(path)
            return None if entity_id is None else self.entities.get(entity_id)

    def joint_ids(self, robot: str) -> list[int]:
        prefix = f"{robot}/"
        with self.lock:
            return sorted(i for path, i in self.ids.items() if path.startswith(prefix))

    def joints(self, robot: str) -> tuple[list[str], NDArray[np.float64]]:
        with self.lock:
            members = [self.entities[i] for i in self.joint_ids(robot) if i in self.entities]
            names = [m.path.removeprefix(f"{robot}/") for m in members]
            rows = [m.joint if m.joint is not None else np.zeros(3) for m in members]
        return names, np.array(rows, dtype=np.float64).reshape(-1, 3)

    def got(self, path: str, key: str) -> bool:
        with self.lock:
            entity_id = self.ids.get(path)
            return any(e == entity_id and k == key for e, k, _ in self.received)

    async def ready(self) -> None:
        """Until it has a frame of the world, taking the world as it is if its first frame went out before it listened."""
        if not self.frame and self._snapshot is not None:
            await asyncio.to_thread(self._pull)
        loop = asyncio.get_running_loop()
        future: asyncio.Future[None] = loop.create_future()
        with self.lock:
            if self.frame:
                return
            self._waiters.append((loop, future))
        await future

    async def ticks(self, rate_hz: float) -> AsyncIterator[float]:
        """Wall times ``rate_hz`` apart, from its first frame of the world on."""
        period = 1.0 / rate_hz
        await self.ready()
        due = time.time()
        while True:
            await asyncio.sleep(due - time.time())
            ticked = time.time()
            yield ticked
            due += period
            if time.time() >= due:
                # Busy past its next tick: the missed ones are skipped, and none comes within a period of this one.
                due = max(time.time(), ticked + period)
