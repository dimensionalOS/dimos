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

import time

import pytest


@pytest.mark.slow
def test_dim_sim(lcm_spy, start_blueprint, human_input) -> None:
    start_blueprint(
        "run",
        "--disable",
        "spatial-memory",
        "--disable",
        "security-module",
        "unitree-go2-agentic",
        simulator="dimsim",
    )
    lcm_spy.save_topic("/rpc/McpClient/on_system_modules/res")
    lcm_spy.wait_for_saved_topic("/rpc/McpClient/on_system_modules/res", timeout=120.0)

    time.sleep(3)

    # Starts at (3, 2)
    human_input("move forward 1 meter")

    lcm_spy.wait_until_odom_position(4, 2, threshold=0.4)
