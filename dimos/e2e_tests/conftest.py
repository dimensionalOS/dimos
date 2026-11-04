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

from __future__ import annotations

from collections.abc import Callable, Generator, Iterator
import subprocess
import sys
import threading
import time
from typing import TYPE_CHECKING, Any

import pytest

from dimos.core.coordination.coordinator_rpc import CoordinatorRPC
from dimos.core.global_config import global_config
from dimos.core.transport_factory import make_transport
from dimos.e2e_tests.dim_sim_client import DimSimClient
from dimos.e2e_tests.dimos_cli_call import DimosCliCall
from dimos.e2e_tests.lcm_spy import LcmSpy
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.simulation.dimsim.config import DIMSIM_PORT
from dimos.simulation.mujoco.direct_cmd_vel_explorer import DirectCmdVelExplorer

if TYPE_CHECKING:
    from playwright.sync_api import Page

    from dimos.core.coordination.blueprints import Blueprint


@pytest.fixture(autouse=True)
def _pin_dimsim_to_lcm(request: pytest.FixtureRequest, monkeypatch: pytest.MonkeyPatch) -> None:
    """Tests driving DimSim run the whole stack on LCM.

    Its deno bridge publishes odom and subscribes cmd_vel over LCM only, so
    under zenoh nothing reaches the sim and nothing comes back. The env var is
    what reaches the ``dimos`` CLI these tests spawn.
    """
    if "dim_sim" not in request.fixturenames:
        return
    monkeypatch.setenv("DIMOS_TRANSPORT", "lcm")
    monkeypatch.setattr(global_config, "transport", "lcm")


@pytest.fixture(scope="session")
def playwright_browsers() -> None:
    subprocess.run(
        [sys.executable, "-m", "playwright", "install", "chromium", "firefox"], check=True
    )


@pytest.fixture
def chromium_page() -> Iterator[Page]:
    # Playwright ships with the browser-tests group only; the other e2e tests
    # in this package must still collect without it.
    from playwright.sync_api import sync_playwright

    with sync_playwright() as p:
        browser = p.chromium.launch()
        try:
            yield browser.new_page()
        finally:
            browser.close()


@pytest.fixture(scope="module")
def serve_channel() -> Iterator[Callable[..., str]]:
    """Start a cockpit(channels=[...]) bridge in-process on its own local relay
    (an ephemeral port, so it cannot clash with test_sdk_browser's stack on
    :7780 in the same CI job), one stream fed from a bus topic on which
    `message` is republished every 100 ms (frames flow only once the bridge
    lazily subscribes); returns the relay's URL. Everything started stops after
    the module's tests."""
    # The web extra is optional for the other e2e tests in this package.
    from dimos.web.relay_bridge.e2e_support import stop_module

    stops: list[Callable[[], None]] = []

    def start(
        blueprint: Blueprint,
        *,
        stream: str,
        topic: str,
        message: Any,
        robot_id: str,
        serve_dir: str | None = None,
    ) -> str:
        (atom,) = blueprint.blueprints
        module = atom.module(
            local_port=0,
            open_browser=False,
            robot_id=robot_id,
            serve_dir=serve_dir,
            **atom.kwargs,
        )
        bridge_transport = make_transport(topic)
        bridge_transport.start()
        getattr(module, stream).transport = bridge_transport
        publisher = make_transport(topic)
        publisher.start()
        stop = threading.Event()

        def republish() -> None:
            while not stop.is_set():
                publisher.publish(message)
                time.sleep(0.1)

        thread = threading.Thread(target=republish, daemon=True)
        thread.start()

        def stop_all() -> None:
            stop.set()
            thread.join(timeout=2)
            stop_module(module)
            publisher.stop()
            bridge_transport.stop()

        stops.append(stop_all)
        module.start()  # builds web dists if stale, spawns the relay, registers
        relay = module._relay
        assert relay is not None and relay.info is not None
        return relay.info.open_url

    yield start
    for stop_all in reversed(stops):
        stop_all()


@pytest.fixture
def lcm_spy() -> Iterator[LcmSpy]:
    lcm_spy = LcmSpy()
    lcm_spy.start()
    yield lcm_spy
    lcm_spy.stop()


@pytest.fixture(scope="session")
def dimsim_port() -> int:
    """One port for the launched DimSim and the test's DimSimClient."""
    return DIMSIM_PORT


@pytest.fixture
def start_blueprint(mcp_port: int, dimsim_port: int) -> Iterator[Callable[..., DimosCliCall]]:
    dimos_robot_call = DimosCliCall()
    dimos_robot_call.mcp_port = mcp_port
    dimos_robot_call.dimsim_port = dimsim_port

    def set_name_and_start(
        *demo_args: str,
        simulator: str | None = None,
    ) -> DimosCliCall:
        dimos_robot_call.demo_args = list(demo_args)
        if simulator is not None:
            dimos_robot_call.simulator = simulator
        dimos_robot_call.start()
        return dimos_robot_call

    yield set_name_and_start

    dimos_robot_call.stop()


@pytest.fixture
def wait_for_system_ready() -> Callable[..., None]:
    """Block until the blueprint is up.

    The CLI serves Coordinator RPC only after build() started every module and
    delivered on_system_modules.
    """

    def wait(timeout: float = 120.0) -> None:
        CoordinatorRPC.connect(timeout=timeout).stop()

    return wait


@pytest.fixture
def human_input():
    transport = make_transport("/human_input")
    transport.start()

    def send_human_input(message: str) -> None:
        transport.publish(message)

    yield send_human_input

    transport.stop()


@pytest.fixture
def direct_cmd_vel_explorer() -> Generator[DirectCmdVelExplorer, None, None]:
    explorer = DirectCmdVelExplorer()
    explorer.start()
    yield explorer
    explorer.stop()


@pytest.fixture
def dim_sim(dimsim_port: int):
    client = DimSimClient(dimsim_port)
    client.start()
    yield client
    client.stop()


@pytest.fixture
def spawn_wall_on_pose(lcm_spy: LcmSpy, dim_sim: DimSimClient):
    """Spawn a dim_sim wall when the robot's /odom comes within `threshold` metres of `point`."""
    odom_topic = "/odom#geometry_msgs.PoseStamped"
    stop_event = threading.Event()
    workers: list[threading.Thread] = []
    errors: list[BaseException] = []

    def spawn(*, point, threshold, wall) -> None:
        px, py = point
        threshold_sq = threshold * threshold
        triggered = threading.Event()

        def on_odom(data):
            if triggered.is_set():
                return
            pose = PoseStamped.lcm_decode(data)
            dx = pose.x - px
            dy = pose.y - py
            if dx * dx + dy * dy < threshold_sq:
                triggered.set()

        def worker():
            try:
                with lcm_spy.topic_listener(odom_topic, on_odom):
                    while not stop_event.is_set():
                        if triggered.wait(timeout=0.1):
                            dim_sim.add_wall(*wall)
                            return
            except BaseException as e:
                errors.append(e)

        thread = threading.Thread(target=worker, daemon=True)
        thread.start()
        workers.append(thread)

    yield spawn

    stop_event.set()
    for thread in workers:
        thread.join(timeout=5.0)
    if errors:
        raise errors[0]


@pytest.fixture
def explore_house(
    direct_cmd_vel_explorer: DirectCmdVelExplorer,
) -> Callable[[], None]:
    points = [
        (3.881, 4.803),
        (4.160, 1.615),
        (1.596, 1.505),
        (1.649, 0.137),
        (-3.644, -0.064),
        (-3.759, -2.661),
        (-4.186, -4.830),
        (-3.759, -2.661),
        (-1.070, -3.285),
        (-2.504, -2.452),
        (-2.647, 5.243),
        (-3.663, 3.591),
        (-1.178, 1.974),
        (-2.416, 2.629),
        (-2.581, 0.164),
        (1.834, 0.072),
        (3.010, -3.883),
        (1.756, -3.742),
        (6.336, -4.077),
        (8.264, -5.119),
        (6.258, -0.964),
        (6.453, 5.327),
    ]

    direct_cmd_vel_explorer.linear_speed = 0.5

    def explore() -> None:
        direct_cmd_vel_explorer.follow_points(points)

    return explore
