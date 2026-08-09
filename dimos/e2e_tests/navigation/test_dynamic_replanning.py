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
from dimos.e2e_tests.navigation.scenarios import DYNAMIC_CORRIDOR
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.simulation.mujoco.direct_cmd_vel_explorer import DirectCmdVelExplorer


def _near(pose: PoseStamped, point: tuple[float, float], threshold: float) -> bool:
    return math.hypot(pose.position.x - point[0], pose.position.y - point[1]) < threshold


@pytest.mark.self_hosted_large
def test_dynamic_replanning(
    navigation_run: NavigationRun,
    direct_cmd_vel_explorer: DirectCmdVelExplorer,
) -> None:
    scenario = DYNAMIC_CORRIDOR
    start_mark = navigation_run.odom.mark()
    navigation_run.scene.set_agent_position(*scenario.start)
    navigation_run.odom.wait_for(
        lambda pose: _near(pose, scenario.start, 0.2),
        after=start_mark,
        timeout=30.0,
        failure_message="The simulator did not publish the dynamic scenario start pose.",
    )

    map_mark = navigation_run.global_costmap.mark()
    for wall in scenario.static_walls:
        navigation_run.scene.add_wall(*wall)
    navigation_run.global_costmap.wait_for(
        lambda _: True,
        after=map_mark,
        timeout=30.0,
        failure_message="The navigation stack did not observe the corridor walls.",
    )

    direct_cmd_vel_explorer.linear_speed = 0.8
    direct_cmd_vel_explorer.follow_points(list(scenario.exploration_route))

    goal_mark = navigation_run.goal_reached.mark()
    trigger_mark = navigation_run.odom.mark()
    navigation_run.scene.publish_goal(*scenario.goal)
    trigger = navigation_run.odom.wait_for(
        lambda pose: _near(pose, scenario.trigger_point, scenario.trigger_radius_m),
        after=trigger_mark,
        timeout=60.0,
        failure_message="The robot did not approach the initially selected upper doorway.",
    )
    navigation_run.scene.add_wall(*scenario.inserted_wall)

    alternate_route = navigation_run.odom.wait_for(
        lambda pose: scenario.alternate_route_bounds.distance_to(pose.position.x, pose.position.y)
        == 0.0,
        after=trigger.sequence,
        timeout=scenario.timeout_s,
        failure_message="The robot did not take the remaining lower doorway after wall insertion.",
    )
    navigation_run.goal_reached.wait_for(
        lambda result: bool(result.data),
        after=goal_mark,
        timeout=scenario.timeout_s,
        failure_message="The planner did not report completion after replanning.",
    )
    final_pose = navigation_run.odom.wait_for(
        lambda pose: _near(pose, scenario.goal, scenario.goal_tolerance_m),
        after=alternate_route.sequence,
        timeout=10.0,
        failure_message="Final odometry is outside the dynamic scenario goal tolerance.",
    ).value

    assert (
        scenario.alternate_route_bounds.distance_to(
            alternate_route.value.position.x,
            alternate_route.value.position.y,
        )
        == 0.0
    )
    assert _near(final_pose, scenario.goal, scenario.goal_tolerance_m)
