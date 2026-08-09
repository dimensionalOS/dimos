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
import time
from typing import Any, Protocol, cast

import pytest

from dimos.e2e_tests.navigation.runtime import NavigationRun
from dimos.e2e_tests.navigation.scenarios import (
    APARTMENT_EXPLORATION_ORIGIN,
    APARTMENT_EXPLORATION_ROUTE,
    APARTMENT_SEMANTIC_SCENARIOS,
    SemanticNavigationScenario,
)
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.simulation.scene_controls import AgentPoseSceneControl, PlanarBounds


class _SpatialMemoryStats(Protocol):
    def get_stats(self) -> dict[str, Any]: ...


def _near_bounds(
    pose: PoseStamped,
    bounds: PlanarBounds,
    max_distance_m: float,
) -> bool:
    return bounds.distance_to(pose.position.x, pose.position.y) <= max_distance_m


def _wait_for_spatial_frames(run: NavigationRun, minimum: int, timeout: float) -> None:
    spatial_memory = cast("_SpatialMemoryStats", run.app.get_module("SpatialMemory"))
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        stats = spatial_memory.get_stats()
        if stats["stored_frame_count"] >= minimum:
            return
        time.sleep(0.25)
    raise TimeoutError(f"Spatial memory did not store {minimum} exploration frames.")


def _stored_frame_count(run: NavigationRun) -> int:
    spatial_memory = cast("_SpatialMemoryStats", run.app.get_module("SpatialMemory"))
    return int(spatial_memory.get_stats()["stored_frame_count"])


def _normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def _sample_exploration_route(
    run: NavigationRun,
    scene: AgentPoseSceneControl,
) -> None:
    previous = APARTMENT_EXPLORATION_ORIGIN
    for target in APARTMENT_EXPLORATION_ROUTE:
        dx = target[0] - previous[0]
        dy = target[1] - previous[1]
        distance = math.hypot(dx, dy)
        if distance == 0.0:
            continue
        yaw = math.atan2(dy, dx)

        frame_mark = _stored_frame_count(run)
        odom_mark = run.odom.mark()
        scene.set_agent_pose(target[0], target[1], 0.52, yaw)

        def reached_sample(
            pose: PoseStamped,
            x: float = target[0],
            y: float = target[1],
            heading: float = yaw,
        ) -> bool:
            return (
                abs(pose.position.x - x) < 0.25
                and abs(pose.position.y - y) < 0.25
                and abs(_normalize_angle(pose.yaw - heading)) < 0.25
            )

        run.odom.wait_for(
            reached_sample,
            after=odom_mark,
            timeout=30.0,
            failure_message=f"The simulator did not sample exploration waypoint {target!r}.",
        )
        _wait_for_spatial_frames(
            run,
            minimum=frame_mark + 1,
            timeout=10.0,
        )
        previous = target


@pytest.mark.self_hosted_large
@pytest.mark.parametrize(
    "semantic_navigation_run",
    APARTMENT_SEMANTIC_SCENARIOS,
    indirect=True,
    ids=lambda scenario: scenario.scenario_id,
)
def test_semantic_navigation(
    semantic_navigation_run: tuple[NavigationRun, SemanticNavigationScenario],
) -> None:
    run, scenario = semantic_navigation_run
    assert isinstance(run.scene, AgentPoseSceneControl)
    scene = run.scene
    target_bounds = scene.semantic_object_bounds(scenario.evaluator_query)

    _sample_exploration_route(run, scene)
    _wait_for_spatial_frames(run, minimum=10, timeout=30.0)

    task_x, task_y, task_z = scenario.task_start
    reset_mark = run.odom.mark()
    scene.set_agent_pose(task_x, task_y, task_z, 0.0)
    run.odom.wait_for(
        lambda pose: (
            abs(pose.position.x - task_x) < 0.25
            and abs(pose.position.y - task_y) < 0.25
            and abs(_normalize_angle(pose.yaw)) < 0.25
        ),
        after=reset_mark,
        timeout=30.0,
        failure_message="The simulator did not set the semantic task start.",
    )

    idle_mark = run.agent_idle.mark()
    goal_mark = run.goal_reached.mark()
    odom_mark = run.odom.mark()
    run.send_instruction(scenario.command)
    busy = run.agent_idle.wait_for(
        lambda idle: not idle,
        after=idle_mark,
        timeout=30.0,
        failure_message=f"The agent did not accept {scenario.command!r}.",
    )
    run.agent_idle.wait_for(
        bool,
        after=busy.sequence,
        timeout=scenario.agent_response_timeout_s,
        failure_message=f"The agent did not finish handling {scenario.command!r}.",
    )
    run.goal_reached.wait_for(
        lambda result: bool(result.data),
        after=goal_mark,
        timeout=scenario.navigation_timeout_s,
        failure_message=f"Navigation did not report completion for {scenario.command!r}.",
    )
    final_pose = run.odom.wait_for(
        lambda pose: _near_bounds(pose, target_bounds, scenario.max_target_distance_m),
        after=odom_mark,
        timeout=10.0,
        failure_message=f"Final odometry is not near {scenario.evaluator_query!r}.",
    ).value

    assert _near_bounds(final_pose, target_bounds, scenario.max_target_distance_m)
