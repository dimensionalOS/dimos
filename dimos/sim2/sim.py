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

A simulated world is one module: actions in on a stream, observations out on another. How it is
built, stepped or paced is the engine's business, and nothing here knows that anything is
watching -- a sensor drawing the world is an ordinary subscriber.

    sim_module(scene_manager="scene/usdmanager")        # the default engine
    sim_module("mujoco", scene=Path("room.usda"))       # or one by name
    sim_module(scene_manager="scene/usdmanager", robots=[SimulatedRobot("go2", GO2_USD)])

A backend is any module shaped like `SimulatedWorld` -- a protocol rather than a base class, so
that a native one can be one too. The blueprint says who is in the world.
"""

from importlib import import_module
from pathlib import Path
from typing import Any, Protocol, cast

from dimos_lcm.world_msgs import WorldAction, WorldObservations
from pydantic import Field, field_validator

from dimos.core.coordination.blueprints import Blueprint
from dimos.core.module import ModuleConfig
from dimos.core.rpc_client import RPCClient
from dimos.core.stream import In, Out
from dimos.protocol.rpc.spec import DEFAULT_RPC_TIMEOUTS
from dimos.robot.assets.model import UsdModel
from dimos.robot.assets.usd_assets import resolve_usd_assets
from dimos.robot.usd_manager import ASSET_ROOT, ASSET_RPC, SCENE_RPC, USD, Mount
from dimos.sim2.spec import SimulatedRobot
from dimos.sim2.usd_layer import scene_as_files
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

# The one place that says what a world contains: the manager that composes it. A scene travels
# as text naming its assets, and `get_asset` is how a consumer that has to open files gets them.


# As long as it takes. `get_scene` does not answer until every mount's `get_usd` is grafted in
# and waits on those with no deadline of its own, so the rpc must not impose one either -- and
# the timeout map has no way to spell "never", so this is the same figure the build rpc uses for
# the same reason. Without it the call inherits default_rpc_timeout and an office scene whose
# mounts take two minutes to come up raises TimeoutError in the world and in every sensor.
SCENE_RPC_TIMEOUT_S = 86_400.0


class SceneConfig(ModuleConfig):
    """Where to get the world from. Shared by the engine that runs it and by every sensor that
    renders from it, so they are pointed at the same manager.
    """

    # The world, as USD: a layer, or a file holding one. A path is read into a `UsdModel` rather
    # than handed over as a path, because an engine here is only ever given USD.
    scene: Path | UsdModel | None = None
    # The UsdManager that composes this world, asked `get_scene` while the world is being built;
    # what it answers with *is* the world. `scene=` is the fallback for a world nobody manages.
    scene_manager: Mount | None = None
    # Overridden so that asking for a scene cannot time out; see SCENE_RPC_TIMEOUT_S.
    rpc_timeouts: dict[str, float] = Field(
        default_factory=lambda: {**DEFAULT_RPC_TIMEOUTS, SCENE_RPC: SCENE_RPC_TIMEOUT_S}
    )

    @field_validator("scene_manager", mode="before")
    @classmethod
    def _normalize_scene_manager(cls, value: object) -> object:
        """Accept a class, a (namespaced) Blueprint or an instance name, as `mounts=` does: a
        Blueprint handed to pydantic as-is is a wall of "Unexpected keyword argument: blueprints,
        transport_map, ...".
        """
        return value if value is None or isinstance(value, Mount) else Mount.of(value)


class SimulatedWorldConfig(SceneConfig):
    # The robots in this world: what each one is called, and what the engine has to set up for
    # it (its servos' gains, how often its controller acts, whether it floats).
    robots: tuple[SimulatedRobot, ...] = ()
    # Sim seconds per wall second.
    speed: float = 1.0
    # Wall seconds the world waits on a stalled controller (one that has acted before) before
    # stepping without it.
    controller_timeout: float = 0.5


class SimulatedWorld(Protocol):
    """Actions in on one stream, observations out on another.

    A protocol and not a base class, deliberately: a native backend cannot subclass anything.
    `SimulatedWorldConfig` is here for a Python backend that wants the usual config.
    """

    actions: In[WorldAction]
    world_observations: Out[WorldObservations]


def scene_from(
    mount: Mount | None, rpc: Any, fallback: Path | UsdModel | None = None
) -> UsdModel | None:
    """The scene a manager composed, as USD with its assets on disk. Asked once.

    The one way anything here learns what the world is, for both the engine and every sensor: the
    same question to the same manager gets the same layer, which is what makes a scan land on the
    furniture that is drawn.

    No retry and no deadline on this side: `get_scene` does not answer until every mount's
    `get_usd` is grafted in, waiting on those itself. Ask it after the modules are built.

    USD either way, never a path: a scene arrives as text naming its assets, so it can reach
    something sharing no filesystem with whoever composed it, and the names are turned back into
    files here, once per machine, keyed by content.
    """
    if mount is None:
        return UsdModel.from_file(fallback) if isinstance(fallback, Path | str) else fallback
    client = (
        RPCClient(None, None, mount.remote_name, {SCENE_RPC, ASSET_RPC}, rpc=rpc)
        if mount.module is None
        else RPCClient.remote(mount.module, mount.remote_name, rpc=rpc)
    )
    answer = getattr(client, SCENE_RPC)([])
    if not answer:
        raise ValueError(
            f"{mount.remote_name}.{SCENE_RPC}() answered {answer!r}, which is not a scene. It "
            f"answers [asset_root, usd] once it has composed one, and waits until it can."
        )
    logger.info(f"sim2: the scene came from {mount.remote_name}.{SCENE_RPC}()")
    layer = scene_as_files(
        resolve_usd_assets(answer[USD], answer[ASSET_ROOT]),
        lambda name: getattr(client, ASSET_RPC)(name),
    )
    return UsdModel.from_file(layer)


# Which module runs the world, by name: the only thing core knows about any engine, and it knows
# it as a string so core imports none of them.
BACKENDS = {"mujoco": "dimos.sim2.mujoco.sim.MujocoSim"}


def sim_module(backend: str = "mujoco", **config: Any) -> Blueprint:
    """One module, actions in, observations out, run by whichever engine is named. A switch and
    nothing else, the way `vis_module` is for viewers.
    """
    if backend not in BACKENDS:
        raise ValueError(f"unknown simulation backend {backend!r}; one of {sorted(BACKENDS)}")
    module, _, name = BACKENDS[backend].rpartition(".")
    return cast("Any", getattr(import_module(module), name)).blueprint(**config)
