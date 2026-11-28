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


"""A simulated world is a physics backend (a ``SimWorld``, picked by the ``sim`` choice) that overlays every HardwareModule's USD onto the scene, serves that USD, steps it, publishes ``world_observations`` and applies ``world_actions``."""

import asyncio
from collections.abc import AsyncIterator, Iterable, Mapping, Sequence
from threading import Event
from typing import Any, Protocol, TypeVar, cast

from dimos_lcm.world_msgs import WorldAction, WorldObservations

from dimos.core.coordination.blueprints import (
    Blueprint,
    BlueprintProtocol,
    ChoiceAtom,
    MetaBlueprint,
)
from dimos.core.core import rpc
from dimos.core.hardware_module import HardwareSpec, Mount
from dimos.core.module import Module
from dimos.core.stream import In, Out
from dimos.robot.assets.model import UsdModel
from dimos.sim2.actions import Action, read_actions
from dimos.sim2.spec import SimulatedRobot, validate
from dimos.sim2.world_observations import WorldWriter
from dimos.spec.utils import Spec, get_protocol_method_signatures
from dimos.utils.logging_config import setup_logger
from dimos.utils.safe_thread_map import safe_thread_map

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
    from dimos.sim2.overlay import overlay_usds
    from dimos.sim2.spawn import check_spawn_fits

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
