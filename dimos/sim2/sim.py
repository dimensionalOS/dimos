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


"""A simulated world is a physics backend, picked by the ``sim`` choice.

There is no simulation module: a backend (MuJoCo, DimSim) is a module that meets ``PhysicsBackend``.
It asks every running HardwareModule (from ``on_system_modules``) for its USD and what it is
mounted on, overlays them into one world, steps it, and publishes it as ``world_observations``.
Robots and sensors are HardwareModules that read those observations and send ``world_actions`` back.
"""

import asyncio
from collections.abc import Coroutine, Iterable, Mapping, Sequence
from typing import Any, Protocol, TypeVar

from dimos_lcm.world_msgs import WorldAction, WorldObservations

from dimos.core.coordination.blueprints import (
    Blueprint,
    BlueprintProtocol,
    ChoiceAtom,
    MetaBlueprint,
)
from dimos.core.hardware_module import HardwareSpec, Mount
from dimos.core.stream import In, Out
from dimos.robot.assets.model import UsdModel
from dimos.sim2.spec import SimulatedRobot
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


class PhysicsBackend(SimulatedWorldSpec, Protocol):
    """What a simulator module implements; the ``sim`` choice picks one."""

    world_actions: In[WorldAction]
    world_observations: Out[WorldObservations]

    def describe(self) -> dict[str, Any]: ...


def sim(backends: Mapping[str, Blueprint | MetaBlueprint], default: str) -> ChoiceAtom:
    """The ``sim`` choice between physics backends: ``dimos run <blueprint> --choice sim=dimsim``."""
    return ChoiceAtom(name="sim", choices=dict(backends), default=default)


def meeting(spec: type[object], modules: Iterable[Any]) -> list[Any]:
    """The running modules (as ``on_system_modules`` hands them over) that answer every rpc of ``spec``."""
    rpcs = set(get_protocol_method_signatures(spec))
    return [module for module in modules if rpcs <= set(module.rpcs)]


BlueprintT = TypeVar("BlueprintT", bound=BlueprintProtocol)


def on_mount(blueprint: BlueprintT, mount: Mount, shared: Iterable[str] = ()) -> BlueprintT:
    """``blueprint`` (a robot attached to ``mount`` and the hardware on it) under the namespace of the mount's frame, as attach() puts one module."""
    # namespace() puts its prefix in front, so the last segment goes on first.
    for segment in reversed(mount.frame.split("/")):
        blueprint = blueprint.namespace(segment, shared=shared)
    return blueprint


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
    """Overlay each (namespaced USD, mount frame) onto the one USD that is mounted on nothing."""
    from dimos.robot.assets.usd_utils.frames import usd_frame_names
    from dimos.robot.assets.usd_utils.layers import overlay_usds
    from dimos.robot.assets.usd_utils.lfs import pull_usd_assets

    roots = [usd for usd, attached_to in described if attached_to is None]
    if len(roots) != 1:
        raise ValueError(
            f"a simulated world needs exactly one unmounted HardwareModule (the scene "
            f"everything else mounts onto), and this blueprint has {len(roots)}"
        )
    # Before any stage is opened on them: a stage opens every layer they reference, such as a scene's hulls.
    pull_usd_assets(usd for usd, _ in described)
    mounted = [(usd, attached_to) for usd, attached_to in described if attached_to is not None]
    # A mounted USD holds its mount's path itself, so a mount frame is real only if some other part offers it.
    offered = usd_frame_names(roots[0]).union(
        *(usd_frame_names(usd) - _path_to(frame) for usd, frame in mounted)
    )
    unplaced = sorted(frame for _, frame in mounted if frame not in offered)
    if unplaced:
        raise ValueError(f"no hardware in the world has the mount frames {unplaced}")
    return UsdModel.from_layer(overlay_usds(roots[0], mounted))


def _path_to(frame: str) -> set[str]:
    """The frame and every frame above it: the namespace prims a USD mounted on it holds."""
    segments = frame.split("/")
    return {"/".join(segments[:depth]) for depth in range(1, len(segments) + 1)}


async def until_it_stops(what: str, loop: Coroutine[Any, Any, None]) -> None:
    """Run a sensor's loop, and log what it raised if it ever stops (a bare task would keep it silent)."""
    try:
        await loop
    except asyncio.CancelledError:
        raise
    except BaseException as error:
        logger.exception(f"sim2: {what} stopped: {error!r}")
        raise
