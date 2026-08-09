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

from dimos.e2e_tests.navigation.runtime import NavigationRun
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped

_ROOM_WALLS = (
    (-1.0, -1.0, 6.0, -1.0),
    (-1.0, 5.0, 6.0, 5.0),
    (-1.0, -1.0, -1.0, 5.0),
    (6.0, -1.0, 6.0, 5.0),
)


def _near(pose: PoseStamped, x: float, y: float, threshold: float) -> bool:
    return math.hypot(pose.position.x - x, pose.position.y - y) < threshold


@pytest.mark.self_hosted_large
def test_walk_forward(navigation_run: NavigationRun) -> None:
    origin_x, origin_y = 1.0, 2.0
    origin_mark = navigation_run.odom.mark()
    navigation_run.scene.set_agent_position(origin_x, origin_y)
    navigation_run.odom.wait_for(
        lambda pose: _near(pose, origin_x, origin_y, 0.2),
        after=origin_mark,
        timeout=30.0,
        failure_message="The simulator did not publish the requested start pose.",
    )

    map_mark = navigation_run.global_costmap.mark()
    for wall in _ROOM_WALLS:
        navigation_run.scene.add_wall(*wall)
    navigation_run.global_costmap.wait_for(
        lambda _: True,
        after=map_mark,
        timeout=30.0,
        failure_message="The navigation stack did not publish a map after scene setup.",
    )

    completion_mark = navigation_run.agent_idle.mark()
    result_mark = navigation_run.odom.mark()
    navigation_run.send_instruction("Move forward 3 meters.")

    busy = navigation_run.agent_idle.wait_for(
        lambda idle: not idle,
        after=completion_mark,
        timeout=30.0,
        failure_message="The agent did not accept the walk-forward instruction.",
    )
    navigation_run.agent_idle.wait_for(
        bool,
        after=busy.sequence,
        timeout=150.0,
        failure_message="The agent did not report completion of the walk-forward instruction.",
    )
    final_pose = navigation_run.odom.wait_for(
        lambda pose: _near(pose, origin_x + 3.0, origin_y, 0.4),
        after=result_mark,
        timeout=10.0,
        failure_message="The final odometry does not show a three-meter forward movement.",
    ).value

    assert _near(final_pose, origin_x + 3.0, origin_y, 0.4)
