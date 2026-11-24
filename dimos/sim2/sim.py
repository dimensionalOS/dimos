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
It asks every HardwareModule in the blueprint for its USD and what it is mounted on, grafts them
into one world, steps it, and publishes it as ``world_observations``. Robots and sensors are
HardwareModules that read those observations and send ``actions`` back.
"""

import asyncio
from collections.abc import Coroutine, Mapping, Sequence
from typing import Any, Protocol

from dimos_lcm.world_msgs import WorldAction, WorldObservations

from dimos.core.coordination.blueprints import Blueprint, ChoiceAtom, MetaBlueprint
from dimos.core.hardware_module import HardwareSpec
from dimos.core.stream import In, Out
from dimos.robot.assets.model import UsdModel
from dimos.sim2.spec import SimulatedRobot
from dimos.spec.utils import Spec
from dimos.utils.logging_config import setup_logger

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

    actions: In[WorldAction]
    world_observations: Out[WorldObservations]
    hardware: list[HardwareSpec]
    robots: list[SimulatedRobotSpec]

    def describe(self) -> dict[str, Any]: ...


def sim(backends: Mapping[str, Blueprint | MetaBlueprint], default: str) -> ChoiceAtom:
    """The ``sim`` choice between physics backends: ``dimos run <blueprint> --choice sim=dimsim``."""
    return ChoiceAtom(name="sim", choices=dict(backends), default=default)


def compose_world(hardware: Sequence[HardwareSpec]) -> UsdModel:
    """Graft every hardware's USD onto the frame it is mounted on, into the one unmounted USD."""
    from dimos.robot.assets.usd_utils.frames import usd_frame_names
    from dimos.robot.assets.usd_utils.layers import graft_usd

    described = [(part.get_usd(), part.mounted_on()) for part in hardware]
    roots = [usd for usd, mounted_on in described if mounted_on is None]
    if len(roots) != 1:
        raise ValueError(
            f"a simulated world needs exactly one unmounted HardwareModule (the scene "
            f"everything else mounts onto), and this blueprint has {len(roots)}"
        )
    mounted = [(usd, mounted_on) for usd, mounted_on in described if mounted_on is not None]

    def grafted(usd: str) -> str:
        frames = usd_frame_names(usd)
        children = {frame: grafted(child) for child, frame in mounted if frame in frames}
        return graft_usd(usd, children) if children else usd

    world = grafted(roots[0])
    placed = usd_frame_names(world)
    unplaced = sorted(frame for _, frame in mounted if frame not in placed)
    if unplaced:
        raise ValueError(f"no hardware in the world has the mount frames {unplaced}")
    return UsdModel.from_layer(world)


async def until_it_stops(what: str, loop: Coroutine[Any, Any, None]) -> None:
    """Run a sensor's loop, and log what it raised if it ever stops (a bare task would keep it silent)."""
    try:
        await loop
    except asyncio.CancelledError:
        raise
    except BaseException as error:
        logger.exception(f"sim2: {what} stopped: {error!r}")
        raise
