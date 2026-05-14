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

import pytest


@pytest.mark.dimsim
def test_go_to_the_bed(lcm_spy, start_blueprint, human_input, dim_sim, explore_house) -> None:
    start_blueprint(
        "run",
        "unitree-go2-agentic",
        simulator="dimsim",
    )
    lcm_spy.save_topic("/rpc/McpClient/on_system_modules/res")
    lcm_spy.wait_for_saved_topic("/rpc/McpClient/on_system_modules/res", timeout=1200.0)

    explore_house()

    human_input("go to the bed")

    lcm_spy.wait_until_odom_position(-3.567, -1.332, threshold=2)
