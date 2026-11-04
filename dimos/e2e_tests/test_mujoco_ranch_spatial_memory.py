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

from dimos.e2e_tests.go2_sim_client import (  # noqa: F401
    RANCH_ARG,
    distance_to_nearest,
    explore_ranch,
    go2_sim,
)
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped


@pytest.mark.self_hosted_large
def test_go_to_the_bed(
    lcm_spy, start_blueprint, wait_for_system_ready, human_input, go2_sim, explore_ranch
) -> None:
    start_blueprint(
        "--agentic",
        "run",
        "unitree-go2",
        RANCH_ARG,
        simulator="mujoco",
    )
    wait_for_system_ready(timeout=300.0)

    explore_ranch()

    human_input("go to the bed")

    beds = go2_sim.footprints("bed")  # the ranch has three; any of them
    lcm_spy.wait_for_message_result(
        "/odom#geometry_msgs.PoseStamped",
        PoseStamped,
        predicate=lambda pose: distance_to_nearest(pose.x, pose.y, beds) < 2,
        fail_message=f"the robot never came within 2 m of a bed's footprint {beds}",
        timeout=180,
    )
