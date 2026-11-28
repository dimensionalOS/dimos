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

"""The simulation backend protocol: the world's observations and actions on the wire, the robots and sensors in it, and the SimWorld module a physics backend runs as."""

import asyncio
from collections.abc import AsyncIterator, Callable, Iterable, Mapping, Sequence
from dataclasses import dataclass, field
import math
from pathlib import Path
from threading import Event, RLock
import time
from typing import Any, Protocol, TypeVar, cast

import cbor2
from dimos_lcm.world_msgs import (
    BodyState,
    CborKeyValue,
    CborKeyValueCommand as CborKeyValueCommandMessage,
    JointCommand as JointCommandMessage,
    JointState,
    WorldAction,
    WorldObservations,
)
import numpy as np
from numpy.typing import NDArray

from dimos.core.coordination.blueprints import (
    Blueprint,
    BlueprintProtocol,
    ChoiceAtom,
    MetaBlueprint,
)
from dimos.core.core import rpc
from dimos.core.hardware_module import HardwareModule, HardwareModuleConfig, HardwareSpec, Mount
from dimos.core.module import Module
from dimos.core.stream import In, Out
from dimos.robot.assets.model import UsdModel
from dimos.robot.assets.usd_utils.lfs import pull_model_assets
from dimos.spec.utils import Spec, get_protocol_method_signatures
from dimos.utils.logging_config import setup_logger
from dimos.utils.safe_thread_map import safe_thread_map

# The world on the wire: observations a backend writes and every reader mirrors.

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
            await _sleep_until(due)
            ticked = time.time()
            yield ticked
            # A tick that came late (busy, or a sleep that overran) skips what it missed, and none comes within a period of it.
            due = max(due + period, ticked + period, time.time())


async def _sleep_until(due: float) -> None:
    """Until wall time ``due``, in short sleeps (macOS lets a background process's sleep overrun by several times its length), yielding to the loop at least once so a loop that is behind never starves the loop's other tasks."""
    await asyncio.sleep(0)
    while (remaining := due - time.time()) > 0:
        await asyncio.sleep(min(max(remaining / 8, 0.0005), 0.5))


# The robots in a world.

# The key a world announces a robot's ``odom_frame`` under, on the robot's entity.
ODOM_FRAME = "odom_frame"
# The key a world marks each robot's entity with.
ROBOT = "robot"


@dataclass(frozen=True)
class SimulatedRobot:
    """A robot's entity in the world: its id (the robot's namespace) and its own USD."""

    robot_id: str
    model: UsdModel
    # The world holds still until this robot is commanded: a robot held up by a policy falls before it is driven.
    held_until_commanded: bool = False
    # The tf frame the robot reports its odometry in, which sensors riding it map in.
    odom_frame: str | None = None
    # Free to move on its own (a floating base the world gives it); a robot driven on its own joints off its mount is not.
    free: bool = True

    @property
    def root_body(self) -> str:
        return self.model.root_link


def validate(simulated: Iterable[SimulatedRobot]) -> None:
    robot_ids = [d.robot_id for d in simulated]
    if len(set(robot_ids)) != len(robot_ids):
        raise ValueError(f"duplicate simulated robot ids: {sorted(robot_ids)}")
    unnamed = [d for d in simulated if not d.robot_id]
    if unnamed:
        raise ValueError(
            "a simulated robot is named by its namespace: put it in one, e.g. .namespace('g1')"
        )


# Actions on the wire: joint commands and CBOR key-value commands.


@dataclass(frozen=True)
class JointCommand:
    entity: str
    names: tuple[str, ...]
    position: tuple[float, ...] | None = None
    velocity: tuple[float, ...] | None = None
    effort: tuple[float, ...] | None = None
    kp: tuple[float, ...] | None = None
    kd: tuple[float, ...] | None = None


@dataclass(frozen=True)
class CborKeyValueCommand:
    entity: str
    key: str
    value: Any


Action = JointCommand | CborKeyValueCommand

_FIELDS = ("position", "velocity", "effort", "kp", "kd")


def world_action(mirror: WorldMirror, *actions: Action) -> WorldAction:
    with mirror.lock:
        joints = [
            JointCommandMessage(
                entity_id=mirror.ids[f"{action.entity}/{name}"],
                **{
                    f: math.nan if getattr(action, f) is None else float(getattr(action, f)[i])
                    for f in _FIELDS
                },
            )
            for action in actions
            if isinstance(action, JointCommand)
            for i, name in enumerate(action.names)
        ]
        cbor = [
            _cbor_command(mirror.ids[action.entity], action.key, action.value)
            for action in actions
            if isinstance(action, CborKeyValueCommand)
        ]
        frame, ts = mirror.frame, mirror.ts
    return WorldAction(
        ts=ts,
        frame=frame,
        joint_commands_length=len(joints),
        joint_commands=joints,
        cbor_key_value_commands_length=len(cbor),
        cbor_key_value_commands=cbor,
    )


def _cbor_command(entity: int, key: str, value: Any) -> CborKeyValueCommandMessage:
    encoded = cbor2.dumps(value)
    return CborKeyValueCommandMessage(
        entity_id=entity, key=key, value_length=len(encoded), value=encoded
    )


def read_actions(message: WorldAction, world: WorldWriter) -> list[Action]:
    actions: list[Action] = []
    for command in message.joint_commands:
        # A joint's path is its robot's (a namespace, any number of segments) and its own name.
        entity, name = world.paths[command.entity_id].rsplit("/", 1)
        values = {f: getattr(command, f) for f in _FIELDS}
        given = {f: None if math.isnan(v) else (v,) for f, v in values.items()}
        previous = actions[-1] if actions else None
        if (
            isinstance(previous, JointCommand)
            and previous.entity == entity
            and all((getattr(previous, f) is None) == (given[f] is None) for f in _FIELDS)
        ):
            actions[-1] = JointCommand(
                entity,
                (*previous.names, name),
                **{
                    f: None if given[f] is None else getattr(previous, f) + given[f]
                    for f in _FIELDS
                },
            )
        else:
            actions.append(JointCommand(entity, (name,), **given))
    for item in message.cbor_key_value_commands:
        actions.append(
            CborKeyValueCommand(
                world.paths[item.entity_id], item.key, cbor2.loads(bytes(item.value))
            )
        )
    return actions


# A simulated world is a physics backend (a ``SimWorld``, picked by the ``sim`` choice) that overlays every HardwareModule's USD onto the scene, serves that USD, steps it, publishes ``world_observations`` and applies ``world_actions``.

logger = setup_logger()


class SimulatedWorldSpec(Spec, Protocol):
    """What a simulated sensor or robot asks the world for: the one composed USD everything is in."""

    def get_world_usd(self) -> str: ...
    def snapshot(self) -> dict[str, Any]: ...


class SimulatedRobotSpec(HardwareSpec, Protocol):
    """Hardware the world moves: its body is an entity of the world, its joints the world's to drive."""

    def simulated_robot(self) -> SimulatedRobot: ...


class Backend(Protocol):
    """A physics engine running the world a SimWorld publishes."""

    world: WorldWriter

    def frame(self) -> WorldObservations: ...
    def snapshot(self) -> dict[str, Any]: ...
    def apply(self, actions: Sequence[Action]) -> None: ...
    def close(self) -> None: ...


class SimWorld(Module):
    """A physics backend's module: builds its backend from the running hardware, steps it, publishes it and applies the actions sent to it."""

    world_actions: In[WorldAction]
    world_observations: Out[WorldObservations]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._backend: Backend | None = None
        self._built = Event()
        self._composed = Event()
        self._world_usd = ""

    async def main(self) -> AsyncIterator[None]:
        self.world_actions.subscribe(self._on_actions)
        running = asyncio.create_task(self._run())
        yield
        running.cancel()

    @rpc
    def on_system_modules(self, modules: list[Any]) -> None:
        simulated = [robot.simulated_robot() for robot in meeting(SimulatedRobotSpec, modules)]
        validate(simulated)
        world = compose_world(meeting(HardwareSpec, modules))
        # Sensors build their views from the composed world alone, so they start on it while the backend builds.
        self._world_usd = world.usd
        self._composed.set()
        self._backend = self._build(world, simulated)
        # The first frame exists before anyone can ask for a snapshot, so a snapshot always has one.
        self._first_frame = self._backend.frame()
        self._built.set()

    @rpc
    def get_world_usd(self) -> str:
        """The one USD of the whole world: the scene with every hardware's USD at its mount."""
        self._composed.wait()
        return self._world_usd

    @rpc
    def snapshot(self) -> dict[str, Any]:
        self._built.wait()
        assert self._backend is not None
        return self._backend.snapshot()

    def _build(self, world: UsdModel, simulated: list[SimulatedRobot]) -> Backend:
        raise NotImplementedError

    async def _run(self) -> None:
        """Step and publish the world, once it is built."""
        raise NotImplementedError

    def _on_actions(self, message: WorldAction) -> None:
        backend = self._backend
        if backend is None:
            return
        try:
            actions = read_actions(message, backend.world)
        except (KeyError, ValueError) as error:
            logger.warning(f"sim2: an action this world cannot read: {error}")
            return
        backend.apply(actions)
        self._applied(actions)

    def _applied(self, actions: list[Action]) -> None:
        """Called with the actions just applied."""

    @rpc
    def stop(self) -> None:
        super().stop()
        # Releases a caller blocked on a world that is never going to be built.
        self._built.set()
        self._composed.set()
        if self._backend is not None:
            self._backend.close()
            self._backend = None


def sim(backends: Mapping[str, Blueprint | MetaBlueprint], default: str) -> ChoiceAtom:
    """The ``sim`` choice between physics backends: ``dimos run <blueprint> --choice sim=dimsim``."""
    return ChoiceAtom(name="sim", choices=dict(backends), default=default)


def meeting(spec: type[object], modules: Iterable[Any]) -> list[Any]:
    """The running modules (as ``on_system_modules`` hands them over) that answer every rpc of ``spec``."""
    rpcs = set(get_protocol_method_signatures(spec))
    return [module for module in modules if rpcs <= set(module.rpcs)]


BlueprintT = TypeVar("BlueprintT", bound=BlueprintProtocol)

# Every namespaced robot and sensor reaches the one simulated world on these.
SIM_STREAMS = frozenset({"world_observations", "world_actions", "tf_static"})


def on_mount(blueprint: BlueprintT, mount: Mount, shared: Iterable[str] = ()) -> BlueprintT:
    """``blueprint`` (a robot attached to ``mount`` and the hardware on it) under the mount's name, as attach() puts one module, with whichever SIM_STREAMS it has shared."""
    names = _stream_names(cast("Blueprint | MetaBlueprint | ChoiceAtom", blueprint))
    shared = SIM_STREAMS.intersection(names).union(shared)
    return blueprint.namespace(mount.name, shared=shared)


def _stream_names(blueprint: Blueprint | MetaBlueprint | ChoiceAtom) -> set[str]:
    """Every stream name a blueprint's modules have, in any of its choices."""
    if isinstance(blueprint, ChoiceAtom):
        return set().union(*map(_stream_names, blueprint.choices.values()))
    if isinstance(blueprint, MetaBlueprint):
        return set().union(*map(_stream_names, blueprint.blueprints))
    return {
        name
        for atom in blueprint.blueprints
        for stream in atom.streams
        if isinstance(
            name := blueprint.remapping_map.get((atom.name, stream.name), stream.name), str
        )
    }


class RunningHardware(HardwareSpec, Protocol):
    """A HardwareModule as ``on_system_modules`` hands it over: its rpcs, and its instance name, whose front is its namespace."""

    instance_name: str


def compose_world(hardware: Sequence[RunningHardware]) -> UsdModel:
    """Overlay every hardware's USD (each already at its tf frames) into one world over the one unmounted USD."""
    # Asked all at once: each part serves its own files, from this machine or another.
    described = safe_thread_map(
        list(hardware),
        lambda part: (fetched_usd(part.instance_name.rpartition("/")[0]), part.attached_to()),
    )
    return compose_usds(described)


def fetched_usd(namespace: str) -> str:
    """The USD of the HardwareModule in ``namespace``, fetched over its hardware_file streams, its files named where this machine keeps them."""
    from dimos.core.hardware_file_client import fetch_hardware_files
    from dimos.robot.assets.usd_utils.assets import resolve_usd_assets

    local = fetch_hardware_files(namespace)
    return resolve_usd_assets(local.read_text(), str(local.parent))


def compose_usds(described: Sequence[tuple[str, str | None]]) -> UsdModel:
    """Overlay each (namespaced USD, its mount's tf frame) onto the one USD that is mounted on nothing, each moved under its mount's prim, refusing one too big for the spawn point it is on."""
    from dimos.robot.assets.usd_utils.frames import usd_world_frame_prims
    from dimos.robot.assets.usd_utils.layers import usd_moved_under
    from dimos.robot.assets.usd_utils.lfs import pull_usd_assets
    from dimos.sim2.scene import check_spawn_fits, overlay_usds

    roots = [usd for usd, attached_to in described if attached_to is None]
    if len(roots) != 1:
        raise ValueError(
            f"a simulated world needs exactly one unmounted HardwareModule (the scene "
            f"everything else mounts onto), and this blueprint has {len(roots)}"
        )
    # Before any stage is opened on them: a stage opens every layer they reference, such as a scene's hulls.
    pull_usd_assets(usd for usd, _ in described)
    # A part's namespace is its mount's tf frame, which some part already placed has, so parents go first.
    prims = usd_world_frame_prims(roots[0])
    waiting = [(usd, frame) for usd, frame in described if frame is not None]
    mounted: list[tuple[str, str]] = []
    while waiting:
        ready = [(usd, frame) for usd, frame in waiting if frame in prims]
        if not ready:
            unplaced = sorted(frame for _, frame in waiting)
            raise ValueError(
                f"no hardware in the world has the mount frames {unplaced}; is each namespaced "
                f"under the hardware it mounts on (.attach(Robot.mounts.<name>))?"
            )
        for usd, frame in ready:
            moved = usd_moved_under(usd, frame, prims[frame])
            prims.update(usd_world_frame_prims(moved))
            mounted.append((moved, prims[frame]))
            waiting.remove((usd, frame))
    check_spawn_fits(roots[0], mounted)
    return UsdModel.from_layer(overlay_usds(roots[0], mounted))


# A simulated sensor: hardware with its own USD, reading the world the physics backend publishes.


class SimSensorConfig(HardwareModuleConfig):
    # Seconds a reading's stamp (the world time it read) may be off wall time and still go out; None is one sample period.
    stamp_tolerance: float | None = None


class SimSensor(HardwareModule):
    """Stamps each reading with the world time it read, which is wall time when the world keeps up; a reading too far off it is not sent."""

    world: SimulatedWorldSpec
    world_observations: In[WorldObservations]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        # The streams whose last reading was too far off wall time, warned about once until one is not.
        self._late: set[str] = set()

    @classmethod
    def attach(cls, mount: Mount, **kwargs: Any) -> "Blueprint":
        """This sensor on ``mount``, as HardwareModule.attach puts it, still reaching the one simulated world."""
        return on_mount(cls.blueprint(**kwargs, attached_to=mount.name), mount)

    @rpc
    def start(self) -> None:
        # Before anything opens its stage, which opens every mesh layer it references.
        pull_model_assets(self.config.usd)
        super().start()

    @property
    def frame_id(self) -> str:
        """The sensor's frame: its USD's root link, under its namespace."""
        return self.tf_frames[self.config.usd.root_link]

    def publish_moving_tf(self, ts: float) -> None:
        """Publish its USD's moving edges (at zero: a sensor drives no joint) stamped ``ts``, so a sensor USD with a joint keeps it on tf."""
        transforms = self.moving_transforms(ts=ts)
        if transforms:
            self.tfbuffer.publish(*transforms)

    def on_time(self, stream: str, stamp: float, period: float, tolerance: float | None) -> bool:
        """Whether a ``stream`` reading stamped ``stamp`` is within ``tolerance`` (None: ``period``) of wall time: one that is not, the real sensor would have missed."""
        off = time.time() - stamp
        if abs(off) <= (period if tolerance is None else tolerance):
            self._late.discard(stream)
            return True
        if stream not in self._late:
            self._late.add(stream)
            logger.warning(
                f"{self.frame_id} {stream}: the world is {off:.3f} s off wall time, so its readings are dropped until it is back"
            )
        return False

    def mirror_world(self) -> WorldMirror:
        world = WorldMirror()
        world.subscribe(self.world_observations, self.world.snapshot)
        return world

    def world_layer(self) -> Path:
        """The composed world, as a layer file a view can open."""
        from dimos.sim2.scene import layer_file

        return layer_file(self.world.get_world_usd(), "world")
