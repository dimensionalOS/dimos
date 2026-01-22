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

from collections.abc import Callable, Generator
import os
import threading
import time

import pytest

from dimos.e2e_tests.dimos_cli_call import DimosCliCall
from dimos.e2e_tests.lcm_spy import LcmSpy
from dimos.simulation.mujoco.person_on_track import PersonTrackPublisher

StartPersonTrack = Callable[[list[tuple[float, float]]], None]


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


@pytest.mark.skipif(bool(os.getenv("CI")), reason="LCM spy doesn't work in CI.")
@pytest.mark.skipif(not os.getenv("OPENAI_API_KEY"), reason="OPENAI_API_KEY not set.")
@pytest.mark.mujoco
def test_person_follow(
    lcm_spy: LcmSpy,
    start_blueprint: Callable[[str], DimosCliCall],
    human_input: Callable[[str], None],
    start_person_track: StartPersonTrack,
) -> None:
    start_blueprint("--mujoco-start-pos", "-6.18 0.96", "run", "unitree-go2-agentic")

    lcm_spy.save_topic("/rpc/HumanInput/start/res")
    lcm_spy.wait_for_saved_topic("/rpc/HumanInput/start/res", timeout=120.0)
    lcm_spy.save_topic("/agent")
    lcm_spy.wait_for_saved_topic_content("/agent", b"AIMessage", timeout=120.0)

    time.sleep(5)

    start_person_track(
        [
            (-2.60, 1.28),
            (4.80, 0.21),
            (4.14, -6.0),
            (0.59, -3.79),
            (-3.35, -0.51),
        ]
    )
    human_input("follow the person in beige pants")

    lcm_spy.wait_until_odom_position(4.2, -3, threshold=1.5)
