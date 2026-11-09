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

"""What sim2 is: a switch between engines, and the little they have in common.

A simulated world is one module. It takes actions on a stream and publishes what changed on
another, and that is the whole contract. How a world is built, stepped or paced is the engine's
business, and nothing in this file knows that anything is watching what it sends -- a sensor
drawing the world is a subscriber like any other, reached through no API of core's.

    from dimos.sim2.sim import sim_module

    sim_module(scene_manager="scene/usdmanager")        # the default engine
    sim_module("mujoco", scene=Path("room.usda"))       # or one by name, configured as it likes

A robot says what it is to `SimulatedWorld`, never to an engine. A sensor says nothing: it reads
the scene the manager composed, builds its own world out of it, and renders from the
observations the engine publishes -- so core never learns that anything is watching::

    class Go2Sim(Module):
        @shared_config(SimulatedWorld)
        def simulated(self) -> dict[str, Any]:
            return {"simulated": SimulatedRobot("go2", self.usd)}

That reaches whichever engine is deployed, because a `shared_config` receiver matches
subclasses and every engine is a subclass of the one below.
"""

from importlib import import_module
from pathlib import Path
from typing import Any, cast

from dimos_lcm.world_msgs import WorldAction, WorldObservations
from pydantic import Field, field_validator

from dimos.core.coordination.blueprints import Blueprint
from dimos.core.module import ModuleConfig
from dimos.core.rpc_client import RPCClient
from dimos.core.shared_config import ConfigSenders, SharedConfigModule
from dimos.core.stream import In, Out
from dimos.robot.assets.model import UsdModel
from dimos.robot.assets.usd import resolve_usd_assets
from dimos.robot.usd_manager import ASSET_ROOT, USD, Mount
from dimos.sim2.spec import Simulated, validate
from dimos.sim2.usd_layer import scene_as_files
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

# The one place that says what a world contains: the manager that composes it. A scene travels
# as text naming its assets, and `get_asset` is how a consumer that has to open files gets them.
_SCENE_RPC = "get_scene"
_ASSET_RPC = "get_asset"


class SceneConfig(ModuleConfig):
    """What anything that needs the world puts in its config: where to get it from.

    Shared by the engine that runs the world and by every sensor that renders from it, because
    they have to be pointed at the same manager and there is one right way to say so.
    """

    # The world, as USD: a layer, or a file holding one. A path is read into a `UsdModel` rather
    # than handed over as a path, because an engine here is only ever given USD -- the one kind
    # of description a robot, a scene and a sensor all agree on.
    scene: Path | UsdModel | None = Field(default_factory=lambda m: m["g"].scene_package)
    # The UsdManager that composes this world, which is the one place that says what is in it:
    # the room, and everything mounted on it. Asked `get_scene` while the world is being built,
    # and what it answers with *is* the world; `scene=` is the fallback for a world nobody
    # manages, which is what the tests are.
    scene_manager: Mount | None = None

    @field_validator("scene_manager", mode="before")
    @classmethod
    def _normalize_scene_manager(cls, value: object) -> object:
        """Accept a class, a (namespaced) Blueprint or an instance name, as `mounts=` does.

        The same normalisation and for the same reason: a blueprint is the useful form, because
        it names the exact instance it deploys, and a Blueprint handed to pydantic as-is is a
        wall of "Unexpected keyword argument: blueprints, transport_map, ...".
        """
        return value if value is None or isinstance(value, Mount) else Mount.of(value)


class SimulatedWorldConfig(SceneConfig):
    # Sim seconds per wall second.
    speed: float = 1.0
    # Wall seconds the world waits on a stalled controller (one that has acted before) before
    # stepping without it.
    controller_timeout: float = 0.5


class SimulatedWorld(SharedConfigModule):
    """A simulated world as a module: actions in, observations out. An engine subclasses this.

    This is the whole of what sim2's core says a simulation is. Core does not own the loop, know
    which engine is running, hold a list of renderers, or make a view for one: it takes the
    descriptions robots and sensors send, finds the world they are in, and leaves the rest to
    whichever subclass is deployed.

    Described to, not configured with: a robot sends what it is with
    `@shared_config(SimulatedWorld)` and reaches the engine that is actually running, because a
    `shared_config` receiver matches subclasses. That is what keeps a robot from naming an engine.
    """

    config: SimulatedWorldConfig
    actions: In[WorldAction]
    world_observations: Out[WorldObservations]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        # Each sender's "simulated" value, keyed by its instance name.
        self.simulated: dict[str, Simulated] = {}

    def config_receive(self, config_senders: ConfigSenders) -> None:
        simulated: dict[str, Simulated] = {}
        for name, config in config_senders:
            description = config.get("simulated")
            if set(config) != {"simulated"} or not isinstance(description, Simulated):
                raise TypeError(
                    f"{name}'s @shared_config(SimulatedWorld) must return"
                    f" {{'simulated': SimulatedRobot}}, not {config!r}"
                )
            simulated[name] = description
        validate(simulated.values())
        self.simulated = simulated

    def world(self) -> UsdModel | None:
        """The world to simulate: what the scene manager composed, or `scene=` without one."""
        return scene_from(self.config.scene_manager, self.rpc, self.config.scene)


def scene_from(
    mount: Mount | None, rpc: Any, fallback: Path | UsdModel | None = None
) -> UsdModel | None:
    """The scene a manager composed, as USD with its assets on disk. Asked once.

    The one way anything here learns what the world is, and used by both kinds of thing that
    need to know: the engine that is going to run it, and every sensor that is going to render
    from it. They ask the same manager the same question and get the same layer, which is what
    makes a scan land on the furniture that is drawn rather than on a generated stand-in.

    One call, and no retry and no deadline on this side. `get_scene` does not answer until the
    manager has every mount's `get_usd` grafted into the scene -- it waits on those itself, with
    no deadline, because half a world is a room with no robots in it -- so there is nothing here
    to poll for and nothing to give up on. Ask it after the modules are built, not while they are
    being built, and it answers once: a manager that is not constructed yet cannot be waited for
    by anybody.

    USD either way, and never a path for the asker to go and open: a scene arrives as text that
    names its assets, which is what lets it reach something sharing no filesystem with whoever
    composed it, and the names are turned back into files here -- once per machine, keyed by
    content.
    """
    if mount is None:
        return UsdModel.from_file(fallback) if isinstance(fallback, Path | str) else fallback
    client = (
        RPCClient(None, None, mount.remote_name, {_SCENE_RPC, _ASSET_RPC}, rpc=rpc)
        if mount.module is None
        else RPCClient.remote(mount.module, mount.remote_name, rpc=rpc)
    )
    answer = getattr(client, _SCENE_RPC)([])
    if not answer:
        raise ValueError(
            f"{mount.remote_name}.{_SCENE_RPC}() answered {answer!r}, which is not a scene. It "
            f"answers [asset_root, usd] once it has composed one, and waits until it can."
        )
    logger.info(f"sim2: the scene came from {mount.remote_name}.{_SCENE_RPC}()")
    layer = scene_as_files(
        resolve_usd_assets(answer[USD], answer[ASSET_ROOT]),
        lambda name: getattr(client, _ASSET_RPC)(name),
    )
    return UsdModel.from_file(layer)


# Which module runs the world, by name: the only thing core knows about any engine, and it knows
# it as a string so core imports none of them.
BACKENDS = {"mujoco": "dimos.sim2.mujoco.sim.MujocoSim"}


def sim_module(backend: str = "mujoco", **config: Any) -> Blueprint:
    """The simulated world, run by whichever engine is named: one module, actions in, observations out.

    A switch and nothing else, the way `vis_module` is for viewers. Every option goes straight
    through to the engine, because what a world needs in order to run is the engine's business
    and core has no opinions left to add.
    """
    if backend not in BACKENDS:
        raise ValueError(f"unknown simulation backend {backend!r}; one of {sorted(BACKENDS)}")
    module, _, name = BACKENDS[backend].rpartition(".")
    return cast("type[SimulatedWorld]", getattr(import_module(module), name)).blueprint(**config)
