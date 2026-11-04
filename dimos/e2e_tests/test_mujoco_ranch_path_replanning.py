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

import math

import pytest

from dimos.e2e_tests.go2_sim_client import RANCH_ARG


@pytest.mark.self_hosted_large
def test_path_replanning(
    lcm_spy,
    start_blueprint,
    wait_for_system_ready,
    go2_sim,
    direct_cmd_vel_explorer,
    spawn_wall_on_pose,
) -> None:
    start_blueprint(
        "--agentic",
        "run",
        "unitree-go2",
        RANCH_ARG,
        simulator="mujoco",
    )
    wait_for_system_ready(timeout=300.0)

    # From the living room the dining room is 4.7 m away through the kitchen's opening (x = 2.4,
    # y 1.8..3.1), or 11.9 m round through the entry. Map both, ending back at the start.
    go2_sim.teleport(4.2, 3.4, -math.pi / 2)
    lcm_spy.wait_until_odom_position(4.2, 3.4, threshold=0.1, timeout=30)
    direct_cmd_vel_explorer.linear_speed = 0.8
    direct_cmd_vel_explorer.follow_points(
        [(4.03, -0.97), (5.03, -3.17), (2.58, -3.07), (2.08, -3.03), (1.22, -2.12),
         (1.53, 0.33), (1.33, 2.08), (3.78, 3.18), (4.2, 3.4)]
    )  # fmt: skip

    # When the robot comes within 1.5 m of the opening, wall it up so the planner has to bail out
    # and go round through the entry.
    spawn_wall_on_pose(point=(2.4, 2.45), threshold=1.5, wall=(2.4, 1.75, 2.4, 3.15))

    go2_sim.publish_goal(1.5, 0.3)

    lcm_spy.wait_until_odom_position(1.5, 0.3, threshold=1, timeout=180)
