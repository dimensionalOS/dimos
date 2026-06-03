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
def test_path_replanning(
    lcm_spy, start_blueprint, dim_sim, direct_cmd_vel_explorer, spawn_wall_on_pose
) -> None:
    start_blueprint(
        "--dimsim-scene=empty",
        "run",
        "unitree-go2-agentic",
        simulator="dimsim",
    )
    lcm_spy.save_topic("/rpc/McpClient/on_system_modules/res")
    lcm_spy.wait_for_saved_topic("/rpc/McpClient/on_system_modules/res", timeout=1200.0)

    # robot spawns at (3, 2)

    # side wall
    dim_sim.add_wall(2, -2.5, 12, -2.5)
    # other side wall
    dim_sim.add_wall(2, 3.5, 12, 3.5)
    # back wall (behind robot)
    dim_sim.add_wall(2, -2.5, 2, 3.5)
    # forward wall (far end)
    dim_sim.add_wall(12, -2.5, 12, 3.5)
    # dividing wall at x=7 with doors at y=[-1.5,-0.5] and y=[1.5,2.5]
    dim_sim.add_wall(7, -2.5, 7, -1.5)
    dim_sim.add_wall(7, -0.5, 7, 1.5)
    dim_sim.add_wall(7, 2.5, 7, 3.5)

    direct_cmd_vel_explorer.linear_speed = 0.8
    direct_cmd_vel_explorer.follow_points([(10, 2), (2.5, 2), (3, 2)])

    # When the robot comes within 1.5 m of the left door's centre, drop a wall
    # in the opening so the planner has to bail out and route through the
    # right door at y=-1 instead.
    spawn_wall_on_pose(
        point=(7, 2),
        threshold=1.5,
        wall=(7, 1.5, 7, 2.5),
    )

    dim_sim.publish_goal(10.913, 0.588)

    lcm_spy.wait_until_odom_position(10.913, 0.588, threshold=1, timeout=120)
