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

import pytest

from dimos.e2e_tests.go2_sim_client import RANCH_ARG, Go2SimClient, go2_sim  # noqa: F401

# The DimSim test's arena, 8 m east of the ranch house's walls on its open ground (the lidar sees no
# floor out there, so the arena brings its own). The house's own loops all pass through gaps the
# local planner won't take (the dining room's 0.7 m by the wall), so the walls are added as there.
DX = 8.0


@pytest.fixture
def dim_sim(go2_sim: Go2SimClient) -> Go2SimClient:
    """spawn_wall_on_pose's wall goes into the MuJoCo ranch."""
    return go2_sim


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

    go2_sim.add_floor(DX + 1.5, -3.0, DX + 12.5, 4.0)
    # side wall
    go2_sim.add_wall(DX + 2, -2.5, DX + 12, -2.5)
    # other side wall
    go2_sim.add_wall(DX + 2, 3.5, DX + 12, 3.5)
    # back wall (behind robot)
    go2_sim.add_wall(DX + 2, -2.5, DX + 2, 3.5)
    # forward wall (far end)
    go2_sim.add_wall(DX + 12, -2.5, DX + 12, 3.5)
    # dividing wall at x=7 with doors at y=[-1.5,-0.5] and y=[1.5,2.5]
    go2_sim.add_wall(DX + 7, -2.5, DX + 7, -1.5)
    go2_sim.add_wall(DX + 7, -0.5, DX + 7, 1.5)
    go2_sim.add_wall(DX + 7, 2.5, DX + 7, 3.5)

    go2_sim.teleport(DX + 3, 2)
    lcm_spy.wait_until_odom_position(DX + 3, 2, threshold=0.1, timeout=30)

    direct_cmd_vel_explorer.linear_speed = 0.8
    direct_cmd_vel_explorer.follow_points([(DX + 10, 2), (DX + 2.5, 2), (DX + 3, 2)])

    # When the robot comes within 1.5 m of the left door's centre, drop a wall
    # in the opening so the planner has to bail out and route through the
    # right door at y=-1 instead.
    spawn_wall_on_pose(
        point=(DX + 7, 2),
        threshold=1.5,
        wall=(DX + 7, 1.5, DX + 7, 2.5),
    )

    go2_sim.publish_goal(DX + 10.913, 0.588)

    lcm_spy.wait_until_odom_position(DX + 10.913, 0.588, threshold=1, timeout=120)
