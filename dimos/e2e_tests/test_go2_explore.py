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

"""Agent-commanded autonomous exploration on the Go2 stack.

First e2e that exercises ``begin_exploration`` end to end: the agent
receives "explore the space", calls the frontier explorer skill, the
explorer publishes goals off the live costmap, and the robot leaves its
spawn area. (The other agentic e2es pre-map with scripted waypoints
instead of exercising the explorer.)
"""

from collections.abc import Callable
import math
import time

import pytest

from dimos.e2e_tests.dimos_cli_call import DimosCliCall
from dimos.e2e_tests.lcm_spy import LcmSpy
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped

# Matches global_config.mujoco_start_pos default ("-1.0, 1.0").
SPAWN = (-1.0, 1.0)


@pytest.mark.skipif_in_ci
@pytest.mark.skipif_no_openai
@pytest.mark.mujoco
def test_go2_begin_exploration(
    lcm_spy: LcmSpy,
    start_blueprint: Callable[[str], DimosCliCall],
    human_input: Callable[[str], None],
) -> None:
    start_blueprint(
        "run",
        "--disable",
        "spatial-memory",
        "--disable",
        "security-module",
        "unitree-go2-agentic",
    )

    lcm_spy.save_topic("/rpc/McpClient/on_system_modules/res")
    lcm_spy.wait_for_saved_topic("/rpc/McpClient/on_system_modules/res", timeout=300.0)

    # Let the first lidar sweeps land so the explorer has a costmap.
    time.sleep(5)

    lcm_spy.save_topic("/goal_request#geometry_msgs.PoseStamped")
    human_input("explore the space")

    # 1) The agent called begin_exploration and the explorer found a frontier.
    lcm_spy.wait_for_saved_topic("/goal_request#geometry_msgs.PoseStamped", timeout=120.0)

    # 2) The robot actually leaves the spawn radius (frontier choice is
    #    nondeterministic, so assert displacement, not a point).
    lcm_spy.wait_for_message_result(
        "/odom#geometry_msgs.PoseStamped",
        PoseStamped,
        predicate=lambda m: math.hypot(m.position.x - SPAWN[0], m.position.y - SPAWN[1]) > 2.0,
        fail_message="Go2 never left the spawn radius while exploring",
        timeout=300.0,
    )
