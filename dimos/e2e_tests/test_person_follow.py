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

from collections.abc import Callable
import time

import pytest

from dimos.e2e_tests.conf_types import StartPersonTrack
from dimos.e2e_tests.dimos_cli_call import DimosCliCall
from dimos.e2e_tests.lcm_spy import LcmSpy


@pytest.mark.skipif_in_ci
@pytest.mark.skipif_no_openai
@pytest.mark.mujoco
def test_person_follow(
    lcm_spy: LcmSpy,
    start_blueprint: Callable[[str], DimosCliCall],
    human_input: Callable[[str], None],
    start_person_track: StartPersonTrack,
) -> None:
    start_blueprint(
        "--mujoco-start-pos",
        "-6.18 0.96",
        "run",
        "--disable",
        "spatial-memory",
        "unitree-go2-agentic",
    )

    lcm_spy.save_topic("/rpc/McpClient/on_system_modules/res")
    lcm_spy.wait_for_saved_topic("/rpc/McpClient/on_system_modules/res", timeout=120.0)

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
