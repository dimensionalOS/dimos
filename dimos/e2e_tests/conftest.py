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

from collections.abc import Callable, Generator, Iterator, Mapping
import os
from pathlib import Path
import threading
import time

import pytest

from dimos.core.transport import pLCMTransport
from dimos.e2e_tests.conf_types import StartPersonTrack
from dimos.e2e_tests.dimos_cli_call import DimosCliCall
from dimos.e2e_tests.episode import EpisodeRun, prepare_episode, reset_episode
from dimos.e2e_tests.lcm_spy import LcmSpy
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import make_vector3
from dimos.msgs.std_msgs.Bool import Bool
from dimos.porcelain.dimos import Dimos
from dimos.simulation.dimsim.control import DimSimSceneControl
from dimos.simulation.episodes import (
    EpisodeProvider,
    EpisodeUnavailableError,
    EvaluationCase,
    load_episode_provider,
)
from dimos.simulation.mujoco.direct_cmd_vel_explorer import DirectCmdVelExplorer
from dimos.simulation.mujoco.person_on_track import PersonTrackPublisher


def _pose(x: float, y: float, theta: float) -> PoseStamped:
    return PoseStamped(
        position=make_vector3(x, y, 0),
        orientation=Quaternion.from_euler(make_vector3(0, 0, theta)),
        frame_id="map",
    )


@pytest.fixture
def lcm_spy() -> Iterator[LcmSpy]:
    lcm_spy = LcmSpy()
    lcm_spy.start()
    yield lcm_spy
    lcm_spy.stop()


@pytest.fixture
def follow_points(lcm_spy: LcmSpy):
    def fun(*, points: list[tuple[float, float, float]], fail_message: str) -> None:
        topic = "/goal_reached#std_msgs.Bool"
        lcm_spy.save_topic(topic)

        for x, y, theta in points:
            lcm_spy.publish("/goal_request#geometry_msgs.PoseStamped", _pose(x, y, theta))
            lcm_spy.wait_for_message_result(
                topic,
                Bool,
                predicate=lambda v: bool(v),
                fail_message=fail_message,
                timeout=60.0,
            )

    yield fun


@pytest.fixture
def start_blueprint(mcp_port: int) -> Iterator[Callable[..., DimosCliCall]]:
    dimos_robot_call = DimosCliCall()
    dimos_robot_call.mcp_port = mcp_port

    def set_name_and_start(
        *demo_args: str,
        simulator: str | None = None,
        global_args: tuple[str, ...] = (),
        extra_env: Mapping[str, str] | None = None,
    ) -> DimosCliCall:
        dimos_robot_call.demo_args = list(demo_args)
        if simulator is not None:
            dimos_robot_call.simulator = simulator
        dimos_robot_call.global_args = list(global_args)
        dimos_robot_call.extra_env = dict(extra_env or {})
        dimos_robot_call.start()
        return dimos_robot_call

    yield set_name_and_start

    dimos_robot_call.stop()


@pytest.fixture
def connect_dimos_modules() -> Iterator[Callable[[DimosCliCall, tuple[str, ...], float], Dimos]]:
    connections: list[Dimos] = []

    def connect(
        call: DimosCliCall,
        required_modules: tuple[str, ...],
        timeout: float = 120.0,
    ) -> Dimos:
        deadline = time.monotonic() + timeout
        last_error: BaseException | None = None
        while time.monotonic() < deadline:
            process = call.process
            if process is not None and process.poll() is not None:
                pytest.fail(f"DimOS exited during startup with code {process.returncode}")
            app: Dimos | None = None
            try:
                app = Dimos.connect(timeout=2.0)
                for module_name in required_modules:
                    app.get_module(module_name)
                connections.append(app)
                return app
            except Exception as error:
                last_error = error
                if app is not None:
                    app.stop()
                time.sleep(0.5)
        pytest.fail(f"DimOS modules were not ready after {timeout:.0f}s: {last_error}")

    yield connect

    for connection in reversed(connections):
        connection.stop()


@pytest.fixture
def episode_provider() -> EpisodeProvider:
    provider_name = os.environ.get("DIMOS_E2E_EPISODE_PROVIDER", "pimsim")
    return load_episode_provider(provider_name)


@pytest.fixture
def evaluation_episode(
    request: pytest.FixtureRequest,
    tmp_path: Path,
    start_blueprint: Callable[..., DimosCliCall],
    connect_dimos_modules: Callable[..., Dimos],
    episode_provider: EpisodeProvider,
) -> Iterator[EpisodeRun]:
    """Prepare and run one case without importing its simulator provider."""

    if not isinstance(request.param, EvaluationCase):
        raise TypeError("evaluation_episode must be parameterized with EvaluationCase")
    case = request.param
    try:
        episode = prepare_episode(episode_provider, case, tmp_path / "episode")
    except EpisodeUnavailableError as error:
        pytest.skip(str(error))

    try:
        call = start_blueprint(
            episode.blueprint_name,
            simulator=episode.simulator,
            global_args=episode.global_args,
            extra_env=episode.extra_env,
        )
        app = connect_dimos_modules(call, episode.required_modules)
        episode_provider.start(episode)
        reset = reset_episode(episode_provider, episode)
        if not reset.initial_conditions_passed:
            pytest.fail(f"episode has invalid initial conditions: {list(reset.failed_conditions)}")
        yield EpisodeRun(
            case=case,
            episode=episode,
            reset=reset,
            app=app,
            provider=episode_provider,
        )
    finally:
        episode_provider.stop()


@pytest.fixture
def human_input():
    transport = pLCMTransport("/human_input")
    transport.lcm.start()

    def send_human_input(message: str) -> None:
        transport.publish(message)

    yield send_human_input

    transport.lcm.stop()


@pytest.fixture
def start_person_track() -> Generator[StartPersonTrack, None, None]:
    publisher: PersonTrackPublisher | None = None
    stop_event = threading.Event()
    thread: threading.Thread | None = None

    def start(track: list[tuple[float, float]]) -> None:
        nonlocal publisher, thread
        publisher = PersonTrackPublisher(track)

        def run_person_track() -> None:
            while not stop_event.is_set():
                publisher.tick()
                time.sleep(1 / 60)

        thread = threading.Thread(target=run_person_track, daemon=True)
        thread.start()

    yield start

    stop_event.set()
    if thread is not None:
        thread.join(timeout=1.0)
    if publisher is not None:
        publisher.stop()


@pytest.fixture
def direct_cmd_vel_explorer() -> Generator[PersonTrackPublisher, None, None]:
    explorer = DirectCmdVelExplorer()
    explorer.start()
    yield explorer
    explorer.stop()


@pytest.fixture
def explore_office(
    direct_cmd_vel_explorer: DirectCmdVelExplorer,
) -> Callable[[], None]:
    points = [
        (0, -7.07),
        (-4.16, -7.07),
        (-4.45, 1.10),
        (-6.72, 2.87),
        (-1.78, 3.01),
        (-1.54, 5.74),
        (3.88, 6.16),
        (2.16, 9.36),
        (4.70, 3.87),
        (4.67, -7.15),
        (4.57, -4.19),
        (-0.84, -2.78),
        (-4.71, 1.17),
        (4.30, 0.87),
    ]

    def explore() -> None:
        direct_cmd_vel_explorer.follow_points(points)

    return explore


@pytest.fixture
def dim_sim():
    client = DimSimSceneControl()
    client.start()
    yield client
    client.stop()


@pytest.fixture
def spawn_wall_on_pose(lcm_spy: LcmSpy, dim_sim: DimSimSceneControl):
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
