# Copyright 2025 Dimensional Inc.
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
import math
import os
import time

import pytest

from dimos.e2e_tests.dimos_cli_call import DimosCliCall
from dimos.e2e_tests.lcm_spy import LcmSpy


@pytest.mark.skipif(bool(os.getenv("CI")), reason="LCM spy doesn't work in CI.")
@pytest.mark.skipif(not os.getenv("OPENAI_API_KEY"), reason="OPENAI_API_KEY not set.")
@pytest.mark.e2e
def test_spatial_memory_navigation(
    lcm_spy: LcmSpy,
    start_blueprint: Callable[[str], DimosCliCall],
    human_input: Callable[[str], None],
    follow_points: Callable[..., None],
) -> None:
    start_blueprint("unitree-go2-agentic")

    lcm_spy.save_topic("/rpc/HumanInput/start/res")
    lcm_spy.wait_for_saved_topic("/rpc/HumanInput/start/res", timeout=120.0)
    lcm_spy.save_topic("/agent")
    lcm_spy.wait_for_saved_topic_content("/agent", b"AIMessage", timeout=120.0)

    time.sleep(5)

    follow_points(
        points=[
            # Navigate to the bookcase.
            (1, 1, 0),
            (4, 1, 0),
            (4.2, -1.1, -math.pi / 2),
            (4.2, -3, -math.pi / 2),
            (4.2, -5, -math.pi / 2),
            # Move away, until it's not visible.
            (1, 1, math.pi / 2),
        ],
        fail_message="Failed to get to the bookcase.",
    )

    time.sleep(5)

    human_input("go to the bookcase")

    lcm_spy.wait_until_odom_position(4.2, -5, threshold=2.0)
