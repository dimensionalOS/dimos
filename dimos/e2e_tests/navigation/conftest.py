# Copyright 2025-2026 Dimensional Inc.
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

from collections.abc import Callable, Iterator
from pathlib import Path

import pytest

from dimos.core.global_config import global_config
from dimos.core.transport_factory import make_transport
from dimos.e2e_tests.dimos_cli_call import DimosCliCall
from dimos.e2e_tests.navigation.probe import StreamProbe
from dimos.e2e_tests.navigation.runtime import (
    NavigationProvider,
    NavigationRun,
    resolve_navigation_provider,
)
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.nav_msgs.OccupancyGrid import OccupancyGrid
from dimos.porcelain.dimos import Dimos
from dimos.protocol.service.zenohservice import (
    ZENOH_LOCAL_ROUTER_ENDPOINT,
    ZENOH_ROUTER_ENDPOINT_ENV,
    ZenohRouter,
)
from dimos.simulation.scene_controls import NavigationSceneControl, load_scene_control

_MODEL_FIXTURE = Path(__file__).parent / "fixtures" / "walk_forward.json"


@pytest.fixture
def navigation_provider() -> NavigationProvider:
    return resolve_navigation_provider()


@pytest.fixture
def navigation_transport_runtime(
    navigation_provider: NavigationProvider,
    monkeypatch: pytest.MonkeyPatch,
) -> Iterator[None]:
    required = navigation_provider.transport
    if global_config.transport != required:
        raise pytest.UsageError(
            f"{navigation_provider.name} navigation tests require DIMOS_TRANSPORT={required}. "
            f"Run with DIMOS_TRANSPORT={required} "
            f"DIMOS_E2E_SIMULATOR={navigation_provider.name}."
        )

    if required == "lcm":
        yield
        return

    monkeypatch.setenv(ZENOH_ROUTER_ENDPOINT_ENV, ZENOH_LOCAL_ROUTER_ENDPOINT)
    router = ZenohRouter(connect=[])
    router.start()
    try:
        yield
    finally:
        router.stop()


@pytest.fixture
def navigation_run(
    navigation_transport_runtime: None,
    navigation_provider: NavigationProvider,
    start_blueprint: Callable[..., DimosCliCall],
    connect_dimos_modules: Callable[[DimosCliCall, tuple[str, ...], float], Dimos],
) -> Iterator[NavigationRun]:
    del navigation_transport_runtime

    agent_idle = StreamProbe[bool]("agent_idle")
    odom = StreamProbe("odom", PoseStamped)
    global_costmap = StreamProbe("global_costmap", OccupancyGrid)
    probes = (agent_idle, odom, global_costmap)
    for probe in probes:
        probe.start()

    human_input = make_transport("human_input")
    scene: NavigationSceneControl | None = None
    try:
        call = start_blueprint(
            "run",
            "--disable",
            "spatial-memory",
            "--disable",
            "security-module",
            "unitree-go2-agentic",
            simulator=navigation_provider.simulator,
            global_args=navigation_provider.global_args,
            extra_env={"MCPCLIENT__MODEL_FIXTURE": str(_MODEL_FIXTURE)},
        )
        connect_dimos_modules(call, ("McpClient",), 300.0)
        agent_idle.wait_for(
            bool,
            timeout=120.0,
            failure_message="The agent did not publish its initial ready state.",
        )

        loaded_scene = load_scene_control(navigation_provider.name)
        if not isinstance(loaded_scene, NavigationSceneControl):
            raise TypeError(f"{navigation_provider.name} does not implement NavigationSceneControl")
        scene = loaded_scene
        scene.start()
        human_input.start()

        yield NavigationRun(
            provider=navigation_provider,
            scene=scene,
            agent_idle=agent_idle,
            odom=odom,
            global_costmap=global_costmap,
            human_input=human_input,
        )
    finally:
        if scene is not None:
            scene.stop()
        human_input.stop()
        global_costmap.stop()
        odom.stop()
        agent_idle.stop()
