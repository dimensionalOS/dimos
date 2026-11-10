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


SCENE_RPC_TIMEOUT_S = 86_400.0


class SceneConfig(ModuleConfig):
    scene: Path | UsdModel | None = None
    scene_manager: Mount | None = None
    rpc_timeouts: dict[str, float] = Field(
        default_factory=lambda: {**DEFAULT_RPC_TIMEOUTS, SCENE_RPC: SCENE_RPC_TIMEOUT_S}
    )

    @field_validator("scene_manager", mode="before")
    @classmethod
    def _normalize_scene_manager(cls, value: object) -> object:
        return value if value is None or isinstance(value, Mount) else Mount.of(value)


class SimulatedWorldConfig(SceneConfig):
    robots: tuple[SimulatedRobot, ...] = ()
    speed: float = 1.0
    controller_timeout: float = 0.5
    # How long the world stands still, publishing but not stepping, before it starts.
    #
    # A robot in a world like this is held up by its policy and not by its servos: left at the
    # home pose its stiffness was set to, a G1 folds in about a second and a half (measured --
    # pelvis 0.755 m down to 0.092 m by t=1.5 s, with no controller in the world at all). A
    # policy cannot command one until it has seen its joints, and it sees them in the
    # observations this world publishes, so there is always an opening where the robot is
    # standing on nothing. This is how long the world refuses to spend it.
    #
    # Twenty seconds because that is the shape of this scene's startup: every module that needs
    # a view of the room composes it again and compiles its own copy -- ten compiles of four
    # thousand geoms for three robots -- and the controllers armed 8.5 to 9.8 s after the world
    # itself was built while that was going on. Nothing is stepped during the wait, so it costs
    # no sim time and the robot has nothing to fall over in.
    startup_delay: float = 20.0


class SimulatedWorld(Protocol):
    actions: In[WorldAction]
    world_observations: Out[WorldObservations]


def scene_from(
    mount: Mount | None, rpc: Any, fallback: Path | UsdModel | None = None
) -> UsdModel | None:
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


BACKENDS = {"mujoco": "dimos.sim2.mujoco.sim.MujocoSim"}


def sim_module(backend: str = "mujoco", **config: Any) -> Blueprint:
    if backend not in BACKENDS:
        raise ValueError(f"unknown simulation backend {backend!r}; one of {sorted(BACKENDS)}")
    module, _, name = BACKENDS[backend].rpartition(".")
    engine = getattr(import_module(module), name)
    return cast("Blueprint", engine.blueprint(**config))
