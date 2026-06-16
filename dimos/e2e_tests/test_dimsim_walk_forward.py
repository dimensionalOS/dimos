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

import math

import pytest


@pytest.mark.parametrize(
    "sim_client",
    [
        pytest.param("dimsim", marks=pytest.mark.dimsim),
        pytest.param("pimsim", marks=pytest.mark.pimsim),
    ],
    indirect=True,
)
def test_walk_forward(
    lcm_spy,
    start_blueprint,
    human_input,
    sim_client,
    request,
) -> None:
    simulator = request.node.callspec.params["sim_client"]
    # pimsim has no scene default (dimsim gets one from --dimsim-scene). Without a
    # scene the lidar->global_map->costmap pipeline has no geometry and the nav
    # planner can't plan, so the agent can't execute a relative move.
    scene_args = ["--scene", "dimos-apartment"] if simulator == "pimsim" else []
    start_blueprint(
        *scene_args,
        "run",
        "--disable",
        "spatial-memory",
        "--disable",
        "security-module",
        "unitree-go2-agentic",
        simulator=simulator,
    )
    lcm_spy.save_topic("/rpc/McpClient/on_system_modules/res")
    lcm_spy.wait_for_saved_topic("/rpc/McpClient/on_system_modules/res", timeout=1200.0)

    # Spawn + heading per backend. dimsim uses its own open default scene with
    # +X forward. pimsim uses DimSim's canonical apartment spawn — three.js
    # Y-up (2, 0.5, 3) -> nav (2.0, -3.0) under the package y-up alignment — but
    # +X there looks into a door frame, so face -Y (90 deg clockwise) where the
    # open lane is; "move forward 3 m" then drives -Y to (2.0, -6.0).
    if simulator == "pimsim":
        origin_x, origin_y = 2.0, -3.0
        sim_client.set_agent_position(origin_x, origin_y, yaw=-math.pi / 2)
        goal_x, goal_y = origin_x, origin_y - 3
    else:
        origin_x, origin_y = 1, 2
        sim_client.set_agent_position(origin_x, origin_y)
        goal_x, goal_y = origin_x + 3, origin_y

    human_input("move forward 3 meter")

    lcm_spy.wait_until_odom_position(goal_x, goal_y, threshold=0.4)
