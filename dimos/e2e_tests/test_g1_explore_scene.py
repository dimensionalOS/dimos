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

"""Flagship sim e2e: G1 explores a cooked scene package on command.

The full simulation standard in one test: G1 spawns in the office scene
package under the GR00T whole-body policy, raycast lidar feeds the voxel
map → costmap → planner, the textured mesh camera streams RGBD, and an
MCP agent receives "explore the space" and drives the frontier explorer.
Asserts the sensor streams are live, the explorer publishes goals, and
the robot walks away from spawn.

Wall time is dominated by scene compose + GR00T load; expect ~6-12 min.
"""

from collections.abc import Callable
import math

import pytest

from dimos.e2e_tests.dimos_cli_call import DimosCliCall
from dimos.e2e_tests.lcm_spy import LcmSpy
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped

SPAWN = (-1.0, 1.0)


@pytest.mark.skipif_in_ci
@pytest.mark.skipif_no_openai
@pytest.mark.mujoco
def test_g1_explore_office(
    lcm_spy: LcmSpy,
    start_blueprint: Callable[[str], DimosCliCall],
    human_input: Callable[[str], None],
) -> None:
    start_blueprint(
        "--mujoco-start-pos",
        f"{SPAWN[0]} {SPAWN[1]}",
        "--scene-package",
        "office",
        "run",
        "--disable",
        "spatial-memory",
        "unitree-g1-groot-agentic",
    )

    # Readiness: the agent fetched its tools ⇒ McpServer up ⇒ all modules
    # started (scene compose + GR00T load can take minutes).
    lcm_spy.save_topic("/rpc/McpClient/on_system_modules/res")
    lcm_spy.wait_for_saved_topic("/rpc/McpClient/on_system_modules/res", timeout=1200.0)

    # Sensor liveness is part of the flagship claim: textured mesh camera
    # + raycast lidar streaming.
    lcm_spy.save_topic("/camera_image#sensor_msgs.Image")
    lcm_spy.save_topic("/lidar#sensor_msgs.PointCloud2")
    lcm_spy.wait_for_saved_topic("/camera_image#sensor_msgs.Image", timeout=120.0)
    lcm_spy.wait_for_saved_topic("/lidar#sensor_msgs.PointCloud2", timeout=120.0)

    lcm_spy.save_topic("/goal_request#geometry_msgs.PoseStamped")
    human_input("explore the space")

    # 1) The agent called begin_exploration and the explorer found a frontier.
    lcm_spy.wait_for_saved_topic("/goal_request#geometry_msgs.PoseStamped", timeout=180.0)

    # 2) The G1 actually walks away from spawn. Displacement radius, not a
    #    point: frontier order is nondeterministic and GR00T walking is slow.
    lcm_spy.wait_for_message_result(
        "/odom#geometry_msgs.PoseStamped",
        PoseStamped,
        predicate=lambda m: math.hypot(m.position.x - SPAWN[0], m.position.y - SPAWN[1]) > 1.5,
        fail_message="G1 never left the spawn radius while exploring",
        timeout=420.0,
    )
