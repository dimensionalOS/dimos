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
from contextlib import contextmanager
import json
from pathlib import Path

import pytest

from dimos.core.global_config import global_config
from dimos.core.transport_factory import make_transport
from dimos.e2e_tests.dimos_cli_call import DimosCliCall
from dimos.e2e_tests.navigation.probe import StreamProbe
from dimos.e2e_tests.navigation.runtime import (
    NavigationCapability,
    NavigationProvider,
    NavigationRun,
    resolve_navigation_provider,
)
from dimos.e2e_tests.navigation.scenarios import SemanticNavigationScenario
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.nav_msgs.OccupancyGrid import OccupancyGrid
from dimos.msgs.std_msgs.Bool import Bool
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

    with _running_navigation_stack(
        navigation_provider,
        start_blueprint,
        connect_dimos_modules,
        global_args=navigation_provider.global_args,
        model_fixture=_MODEL_FIXTURE,
        disabled_modules=("spatial-memory", "security-module"),
        required_modules=("McpClient",),
    ) as run:
        yield run


@pytest.fixture
def semantic_navigation_run(
    request: pytest.FixtureRequest,
    navigation_transport_runtime: None,
    navigation_provider: NavigationProvider,
    start_blueprint: Callable[..., DimosCliCall],
    connect_dimos_modules: Callable[[DimosCliCall, tuple[str, ...], float], Dimos],
    tmp_path: Path,
) -> Iterator[tuple[NavigationRun, SemanticNavigationScenario]]:
    del navigation_transport_runtime
    capability = NavigationCapability.SEMANTIC_SPATIAL_MEMORY
    if not navigation_provider.supports(capability):
        pytest.skip(
            f"{navigation_provider.name} does not provide the maintained "
            f"{capability.value} acceptance workflow"
        )
    semantic_global_args = navigation_provider.semantic_global_args
    if semantic_global_args is None:
        raise RuntimeError(
            f"{navigation_provider.name} advertises {capability.value} without scene arguments"
        )
    if not isinstance(request.param, SemanticNavigationScenario):
        raise TypeError("semantic_navigation_run requires a SemanticNavigationScenario")
    scenario = request.param
    model_fixture = _write_semantic_model_fixture(tmp_path, scenario)
    with _running_navigation_stack(
        navigation_provider,
        start_blueprint,
        connect_dimos_modules,
        global_args=semantic_global_args,
        model_fixture=model_fixture,
        disabled_modules=("security-module",),
        required_modules=("McpClient", "SpatialMemory"),
    ) as run:
        yield run, scenario


@contextmanager
def _running_navigation_stack(
    provider: NavigationProvider,
    start_blueprint: Callable[..., DimosCliCall],
    connect_dimos_modules: Callable[[DimosCliCall, tuple[str, ...], float], Dimos],
    *,
    global_args: tuple[str, ...],
    model_fixture: Path,
    disabled_modules: tuple[str, ...],
    required_modules: tuple[str, ...],
) -> Iterator[NavigationRun]:
    agent_idle = StreamProbe[bool]("agent_idle")
    odom = StreamProbe("odom", PoseStamped)
    global_costmap = StreamProbe("global_costmap", OccupancyGrid)
    goal_reached = StreamProbe("goal_reached", Bool)
    probes = (agent_idle, odom, global_costmap, goal_reached)
    for probe in probes:
        probe.start()

    human_input = make_transport("human_input")
    scene: NavigationSceneControl | None = None
    try:
        disable_args = tuple(
            argument for module in disabled_modules for argument in ("--disable", module)
        )
        call = start_blueprint(
            "run",
            *disable_args,
            "unitree-go2-agentic",
            simulator=provider.simulator,
            global_args=global_args,
            extra_env={"MCPCLIENT__MODEL_FIXTURE": str(model_fixture)},
        )
        app = connect_dimos_modules(call, required_modules, 300.0)
        agent_idle.wait_for(
            bool,
            timeout=120.0,
            failure_message="The agent did not publish its initial ready state.",
        )

        loaded_scene = load_scene_control(provider.name)
        if not isinstance(loaded_scene, NavigationSceneControl):
            raise TypeError(f"{provider.name} does not implement NavigationSceneControl")
        scene = loaded_scene
        scene.start()
        human_input.start()

        yield NavigationRun(
            provider=provider,
            app=app,
            scene=scene,
            agent_idle=agent_idle,
            odom=odom,
            global_costmap=global_costmap,
            goal_reached=goal_reached,
            human_input=human_input,
        )
    finally:
        if scene is not None:
            scene.stop()
        human_input.stop()
        goal_reached.stop()
        global_costmap.stop()
        odom.stop()
        agent_idle.stop()


def _write_semantic_model_fixture(
    output_dir: Path,
    scenario: SemanticNavigationScenario,
) -> Path:
    path = output_dir / f"semantic-{scenario.scenario_id}.json"
    payload = {
        "responses": [
            {
                "content": "",
                "tool_calls": [
                    {
                        "name": "navigate_with_text",
                        "args": {"query": scenario.memory_query},
                        "id": f"call_semantic_{scenario.scenario_id}",
                        "type": "tool_call",
                    }
                ],
            },
            {
                "content": f"Navigation started for {scenario.memory_query}.",
                "tool_calls": [],
            },
        ]
    }
    path.write_text(json.dumps(payload, indent=2), encoding="utf-8")
    return path
