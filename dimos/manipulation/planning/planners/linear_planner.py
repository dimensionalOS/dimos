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

"""Linear (straight-line) joint-space planner.

Tries a direct joint-space interpolation from start to goal.
If the straight-line path is collision-free, returns it instantly.
Falls back to RRTConnectPlanner if the direct path is in collision.

This is dramatically faster than RRT for uncluttered workspaces, which
covers the vast majority of real-world pick/place scenarios.
"""

from __future__ import annotations

import time
from typing import TYPE_CHECKING

from dimos.manipulation.planning.planners.rrt_planner import RRTConnectPlanner
from dimos.manipulation.planning.spec import (
    JointPath,
    PlanningResult,
    PlanningStatus,
    WorldRobotID,
    WorldSpec,
)
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from dimos.msgs.sensor_msgs import JointState

logger = setup_logger()


def _create_failure_result(
    status: PlanningStatus,
    message: str,
    elapsed: float = 0.0,
    iterations: int = 0,
) -> PlanningResult:
    return PlanningResult(
        path=[],
        status=status,
        planning_time=elapsed,
        iterations=iterations,
        message=message,
    )


def _create_success_result(path: JointPath, elapsed: float, iterations: int) -> PlanningResult:
    return PlanningResult(
        path=path,
        status=PlanningStatus.SUCCESS,
        planning_time=elapsed,
        iterations=iterations,
        message=f"Linear path found in {elapsed:.3f}s",
    )


class LinearPlanner:
    """Straight-line joint interpolation planner with RRT fallback.

    Attempts direct joint-space interpolation first (instant for clear paths).
    Falls back to RRT-Connect if the direct path is in collision.
    """

    def __init__(
        self,
        collision_step_size: float = 0.02,
        rrt_fallback: bool = True,
    ):
        self._collision_step_size = collision_step_size
        self._rrt = (
            RRTConnectPlanner(collision_step_size=collision_step_size) if rrt_fallback else None
        )

    def plan_joint_path(
        self,
        world: WorldSpec,
        robot_id: WorldRobotID,
        start: JointState,
        goal: JointState,
        timeout: float = 60.0,
    ) -> PlanningResult:
        """Plan path: try direct interpolation first, fall back to RRT."""
        start_time = time.time()

        if not world.is_finalized:
            return _create_failure_result(PlanningStatus.NO_SOLUTION, "World must be finalized")

        # Validate start/goal not in collision
        if not world.check_config_collision_free(robot_id, start):
            return _create_failure_result(PlanningStatus.COLLISION_AT_START, "Start in collision")
        if not world.check_config_collision_free(robot_id, goal):
            return _create_failure_result(PlanningStatus.COLLISION_AT_GOAL, "Goal in collision")

        # Try direct linear path
        if world.check_edge_collision_free(robot_id, start, goal, self._collision_step_size):
            elapsed = time.time() - start_time
            logger.info(f"Linear path: direct interpolation succeeded in {elapsed:.3f}s")
            return _create_success_result([start, goal], elapsed, 1)

        # Direct path blocked — fall back to RRT if available
        elapsed_linear = time.time() - start_time
        logger.info(
            f"Linear path: direct path in collision ({elapsed_linear:.3f}s), falling back to RRT"
        )

        if self._rrt is None:
            return _create_failure_result(
                PlanningStatus.NO_SOLUTION,
                "Direct path in collision, RRT fallback disabled",
                elapsed_linear,
            )

        remaining = timeout - elapsed_linear
        if remaining <= 0:
            return _create_failure_result(
                PlanningStatus.TIMEOUT, "Timeout before RRT", elapsed_linear
            )

        return self._rrt.plan_joint_path(world, robot_id, start, goal, timeout=remaining)

    def get_name(self) -> str:
        return "LinearWithRRTFallback"
