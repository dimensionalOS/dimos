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

"""CuRobo V2 configuration-space planning with native trajectory timing."""

from __future__ import annotations

import math
import time
from typing import TYPE_CHECKING, Any

import numpy as np

from dimos.manipulation.planning.spec.enums import PlanningStatus
from dimos.manipulation.planning.spec.models import PlanningResult
from dimos.manipulation.planning.utils.path_utils import compute_path_length
from dimos.msgs.sensor_msgs.JointState import JointState

if TYPE_CHECKING:
    from collections.abc import Mapping, Sequence

    from numpy.typing import NDArray

    from dimos.manipulation.planning.groups.models import PlanningGroupSelection
    from dimos.manipulation.planning.planners.config import CartesianPathConfig, CuRoboPlannerConfig
    from dimos.manipulation.planning.spec.joint_space import JointSpace
    from dimos.manipulation.planning.spec.models import CartesianTarget, PlanningGroupID
    from dimos.manipulation.planning.spec.protocols import WorldSpec

_MAX_ATTEMPTS = 3
_ENDPOINT_ATOL = 1e-5  # Native joint states use float32 by default.


class CuRoboPlanner:
    """Plan full-model joint motions against one CuRobo world's current scene."""

    def __init__(self, world: WorldSpec, config: CuRoboPlannerConfig) -> None:
        from dimos.manipulation.planning.world.curobo_world import CuRoboWorld

        if not isinstance(world, CuRoboWorld):
            raise TypeError("CuRoboPlanner requires a CuRoboWorld")
        self._world = world

    def get_name(self) -> str:
        return "CuRobo"

    def plan_joint_path(
        self,
        world: WorldSpec,
        start: JointState,
        goal: JointState,
        timeout: float = 10.0,
        *,
        speed_scale: float = 1.0,
    ) -> PlanningResult:
        return self._plan(world, None, start, goal, timeout, _MAX_ATTEMPTS, speed_scale)

    def plan_selected_joint_path(
        self,
        world: WorldSpec,
        selection: PlanningGroupSelection,
        start: JointState,
        goal: JointState,
        timeout: float = 10.0,
        max_iterations: int = 5000,
        *,
        speed_scale: float = 1.0,
    ) -> PlanningResult:
        if not selection.groups:
            return PlanningResult(
                status=PlanningStatus.INVALID_GOAL, message="No planning groups selected"
            )
        if len(selection.groups) != 1:
            return PlanningResult(
                status=PlanningStatus.UNSUPPORTED,
                message="CuRobo joint planning requires one full-model planning group",
            )
        return self._plan(
            world,
            selection.joint_names,
            start,
            goal,
            timeout,
            min(max_iterations, _MAX_ATTEMPTS),
            speed_scale,
        )

    def plan_cartesian_path(
        self,
        world: WorldSpec,
        selection: PlanningGroupSelection,
        start: JointState,
        targets: Mapping[PlanningGroupID, CartesianTarget],
        config: CartesianPathConfig,
        *,
        auxiliary_groups: Sequence[PlanningGroupID] = (),
        check_collision: bool = True,
    ) -> PlanningResult:
        return PlanningResult(
            status=PlanningStatus.UNSUPPORTED,
            message="CuRobo does not implement DimOS Cartesian waypoint planning",
        )

    def _plan(
        self,
        world: WorldSpec,
        selection_names: tuple[str, ...] | None,
        start: JointState,
        goal: JointState,
        timeout: float,
        attempts: int,
        speed_scale: float,
    ) -> PlanningResult:
        started = time.monotonic()

        def failure(status: PlanningStatus, message: str) -> PlanningResult:
            return PlanningResult(
                status=status, message=message, planning_time=time.monotonic() - started
            )

        if world is not self._world:
            return failure(
                PlanningStatus.UNSUPPORTED, "CuRobo planner requires its CuRoboWorld instance"
            )
        if not math.isfinite(speed_scale) or not 0.0 < speed_scale <= 1.0:
            return failure(PlanningStatus.INVALID_GOAL, "speed_scale must be finite, > 0, and <= 1")
        if not math.isfinite(timeout) or timeout <= 0.0 or attempts <= 0:
            return failure(
                PlanningStatus.TIMEOUT, "CuRobo planning requires a positive finite budget"
            )
        if not self._world.is_ready():
            return failure(PlanningStatus.INVALID_START, "CuRobo planning scene is not ready")
        joint_space = self._world.get_prepared_model().joint_space
        names = joint_space.names if selection_names is None else selection_names
        if len(names) != len(joint_space.names) or set(names) != set(joint_space.names):
            return failure(
                PlanningStatus.UNSUPPORTED, "CuRobo joint planning requires all model joints"
            )
        if not joint_space.is_interval_only:
            return failure(
                PlanningStatus.UNSUPPORTED, "CuRobo requires interval-only joint coordinates"
            )
        space = joint_space.select(names)
        try:
            q_start = _ordered_positions(start, space)
        except (TypeError, ValueError) as exc:
            return failure(PlanningStatus.INVALID_START, str(exc))
        try:
            q_goal = _ordered_positions(goal, space)
        except (TypeError, ValueError) as exc:
            return failure(PlanningStatus.INVALID_GOAL, str(exc))

        try:
            with self._world.native_context() as native:
                for q, label, status in (
                    (q_start, "start", PlanningStatus.COLLISION_AT_START),
                    (q_goal, "goal", PlanningStatus.COLLISION_AT_GOAL),
                ):
                    state = JointState(name=list(names), position=q.tolist())
                    if not self._world.check_config_collision_free(state):
                        return failure(status, f"CuRobo {label} configuration is in collision")
                if time.monotonic() - started >= timeout:
                    return failure(PlanningStatus.TIMEOUT, "CuRobo planning deadline exceeded")
                native_names = tuple(native.joint_names)
                if len(native_names) != len(names) or set(native_names) != set(names):
                    raise ValueError("CuRobo active joints do not match the canonical model joints")
                indices = [names.index(name) for name in native_names]
                # V2's graph seeding can reject external scenes; keep all
                # attempts in native trajectory optimization instead.
                result = native.plan_cspace(
                    _native_state(native, q_goal[indices], native_names),
                    _native_state(native, q_start[indices], native_names),
                    max_attempts=attempts,
                    enable_graph_attempt=attempts,
                )
                # V2 exposes an attempt budget, not an interruptible wall-clock timeout.
                if time.monotonic() - started >= timeout:
                    return failure(PlanningStatus.TIMEOUT, "CuRobo planning deadline exceeded")
                if result is None or not bool(result.success.any().item()):
                    return failure(
                        PlanningStatus.NO_SOLUTION, "CuRobo found no feasible joint trajectory"
                    )
                path, timestamps = _timed_path(result.get_interpolated_plan(), names, speed_scale)
        except (AttributeError, RuntimeError, TypeError, ValueError) as exc:
            return failure(PlanningStatus.NO_SOLUTION, f"CuRobo joint planning failed: {exc}")

        if not (
            np.allclose(path[0].position, q_start, rtol=0.0, atol=_ENDPOINT_ATOL)
            and np.allclose(path[-1].position, q_goal, rtol=0.0, atol=_ENDPOINT_ATOL)
        ):
            return failure(
                PlanningStatus.NO_SOLUTION, "CuRobo trajectory does not preserve start and goal"
            )
        if time.monotonic() - started >= timeout:
            return failure(PlanningStatus.TIMEOUT, "CuRobo planning deadline exceeded")
        return PlanningResult(
            status=PlanningStatus.SUCCESS,
            path=path,
            timestamps=timestamps,
            planning_time=time.monotonic() - started,
            path_length=compute_path_length(path),
            message="CuRobo joint trajectory found",
        )


def _ordered_positions(state: JointState, space: JointSpace) -> NDArray[np.float64]:
    if state.name and set(state.name) != set(space.names):
        raise ValueError("Joint state names must match the requested model joints")
    return space.from_joint_state(state)


def _native_state(native: Any, q: NDArray[np.float64], names: tuple[str, ...]) -> Any:
    from curobo.types import JointState as NativeJointState

    return NativeJointState.from_position(
        native.device_cfg.to_device(q[None, :]), joint_names=list(names)
    )


def _timed_path(
    trajectory: Any, names: tuple[str, ...], speed_scale: float
) -> tuple[list[JointState], list[float]]:
    native_names = tuple(trajectory.joint_names or ())
    if len(set(native_names)) != len(native_names) or not set(names).issubset(native_names):
        raise ValueError("CuRobo trajectory joint names do not cover the canonical model joints")
    indices = [native_names.index(name) for name in names]

    def rows(tensor: Any) -> NDArray[np.float64]:
        data = np.asarray(tensor.detach().cpu().numpy(), dtype=np.float64)
        # V2 may retain singleton batch/seed axes. Never flatten multiple plans.
        if (
            data.ndim < 2
            or any(size != 1 for size in data.shape[:-2])
            or data.shape[-2] < 2
            or data.shape[-1] != len(native_names)
        ):
            raise ValueError("CuRobo trajectory has an invalid waypoint shape")
        data = data.reshape(data.shape[-2:])[:, indices]
        if not np.isfinite(data).all():
            raise ValueError("CuRobo trajectory contains non-finite values")
        return data

    positions = rows(trajectory.position)
    velocities = rows(trajectory.velocity)
    if positions.shape != velocities.shape:
        raise ValueError("CuRobo trajectory position and velocity lengths differ")
    if trajectory.dt is None:
        raise ValueError("CuRobo trajectory does not include timing")
    # Time dilation preserves the native geometric path. Its first derivative
    # scales by speed_scale (and the implied second derivative by its square).
    dt = float(trajectory.dt.item()) / speed_scale
    if dt <= 0.0 or not math.isfinite((len(positions) - 1) * dt):
        raise ValueError("CuRobo trajectory timestep is not positive and finite")
    return [
        JointState(name=list(names), position=q.tolist(), velocity=dq.tolist())
        for q, dq in zip(positions, velocities * speed_scale, strict=True)
    ], [i * dt for i in range(len(positions))]
