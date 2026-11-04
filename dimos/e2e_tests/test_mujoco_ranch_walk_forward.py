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

from dimos.e2e_tests.go2_sim_client import RANCH_ARG, go2_sim  # noqa: F401
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped


@pytest.mark.self_hosted_large
def test_walk_forward(
    lcm_spy, start_blueprint, wait_for_system_ready, human_input, go2_sim
) -> None:
    start_blueprint(
        "--agentic",
        "run",
        "--disable",
        "spatial-memory",
        "unitree-go2",
        RANCH_ARG,
        simulator="mujoco",
    )
    wait_for_system_ready(timeout=300.0)
    lcm_spy.wait_for_message_result(
        "/odom#geometry_msgs.PoseStamped",
        PoseStamped,
        predicate=lambda _: True,
        fail_message="the sim did not publish odometry",
        timeout=120,
    )

    # The living room's west side, facing north up 3.6 m of open floor.
    origin_x, origin_y = 3.75, -0.25
    go2_sim.teleport(origin_x, origin_y, math.pi / 2)
    lcm_spy.wait_until_odom_position(origin_x, origin_y, threshold=0.1, timeout=30)

    human_input("move forward 3 meter")

    lcm_spy.wait_until_odom_position(origin_x, origin_y + 3, threshold=0.4, timeout=120)
