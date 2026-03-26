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

from collections.abc import Callable
import time

from dimos_lcm.std_msgs import String
import pytest

from dimos.e2e_tests.conf_types import StartPersonTrack
from dimos.e2e_tests.dimos_cli_call import DimosCliCall
from dimos.e2e_tests.lcm_spy import LcmSpy


@pytest.mark.skipif_in_ci
@pytest.mark.skipif_no_openai
@pytest.mark.mujoco
def test_security_module(
    lcm_spy: LcmSpy,
    start_blueprint: Callable[[str], DimosCliCall],
    human_input: Callable[[str], None],
    start_person_track: StartPersonTrack,
    explore_office: Callable[[], None],
) -> None:
    start_blueprint(
        "--mujoco-start-pos",
        "-10.75 -6.78",
        "--mujoco-camera-position",
        "-0.797 0.007  0.468 26.825 88.998 -70.321",
        "--nerf-speed",
        "0.8",
        "--dtop",
        "run",
        "--disable",
        "spatial-memory",
        "unitree-go2-security",
    )

    lcm_spy.save_topic("/rpc/McpClient/on_system_modules/res")
    lcm_spy.save_topic("/security_state#std_msgs.String")
    lcm_spy.wait_for_saved_topic("/rpc/McpClient/on_system_modules/res", timeout=120.0)

    time.sleep(2)

    explore_office()

    start_person_track(
        [
            (-10.75, -6.78),
            (0, -7.07),
        ]
    )
    human_input(
        "start the security patrol. Just call start_security_patrol. Do not ask me anything."
    )

    def predicate(s: String) -> bool:
        return s.data == "FOLLOWING"

    lcm_spy.wait_for_message_result(
        "/security_state#std_msgs.String",
        String,
        predicate,
        "Failed to transition to FOLLOWING.",
        360,
    )
