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

import math
import os
import time

import pytest


@pytest.mark.skipif(bool(os.getenv("CI")), reason="LCM spy doesn't work in CI.")
def test_dimos_skills(lcm_spy, start_blueprint, human_input, follow_points) -> None:
    start_blueprint("unitree-go2-agentic")

    lcm_spy.wait_for_saved_topic("/rpc/HumanInput/start/res", timeout=120.0)
    lcm_spy.wait_for_message_content("/agent", b"AIMessage", timeout=120.0)

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

    lcm_spy.wait_until_odom_position(4.2, -5)
