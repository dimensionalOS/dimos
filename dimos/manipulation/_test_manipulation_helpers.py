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

"""Shared lightweight test harnesses for manipulation module tests."""

from unittest.mock import MagicMock

from dimos.manipulation.execution_runtime import ExecutionRuntime, Outcome
from dimos.manipulation.manipulation_module import ManipulationModule
from dimos.manipulation.planning.spec.config import RobotModelConfig

_TEST_RUNTIMES: list[ExecutionRuntime] = []


class FakeCoordinatorGateway:
    """Deterministic coordinator gateway used by module-side tests."""

    def __init__(self, execute_outcome: Outcome = Outcome.ACCEPTED) -> None:
        self.execute_outcome = execute_outcome
        self.execute_calls: list[tuple[str, object]] = []
        self.cancel_calls: list[str] = []
        self.stopped = False

    def execute(self, task_name: str, request: object) -> Outcome:
        self.execute_calls.append((task_name, request))
        return self.execute_outcome

    def cancel(self, task_name: str) -> Outcome:
        self.cancel_calls.append(task_name)
        return Outcome.CANCELLED

    def status(self, task_name: str) -> Outcome:
        del task_name
        return Outcome.INACTIVE

    def reset(self, task_name: str) -> Outcome:
        del task_name
        return Outcome.INACTIVE

    def set_gripper_position(self, hardware_id: str, position: float) -> Outcome:
        del hardware_id, position
        return Outcome.ACCEPTED

    def get_gripper_position(self, hardware_id: str) -> float | None:
        del hardware_id
        return 0.0

    def stop(self) -> None:
        self.stopped = True


class ManipulationModuleHarness(ManipulationModule):
    """Manipulation module initialized only with state needed by unit tests."""

    def __init__(self) -> None:
        self._robots = {}
        self._world_monitor = None
        self._planner = None
        self._kinematics = None
        self._execution_runtime = None
        self._execution_topology = None
        self._init_joints = {}
        self.config = MagicMock(planning_timeout=10.0)


def make_module() -> ManipulationModule:
    """Create a lightweight ManipulationModule harness for behavior tests."""
    return ManipulationModuleHarness()


def install_runtime(
    module: ManipulationModule,
    robots: list[RobotModelConfig],
    gateway: FakeCoordinatorGateway | None = None,
) -> FakeCoordinatorGateway:
    """Install the production runtime with a test-owned fake gateway."""
    gateway = gateway or FakeCoordinatorGateway()
    from dimos.manipulation.execution_runtime import ExecutionTopology

    module._execution_topology = ExecutionTopology.from_robot_configs(robots)
    module._execution_runtime = ExecutionRuntime(
        lambda: gateway,
        topology=module._execution_topology,
        action_timeout=module.config.planning_timeout,
        poll_interval=0.01,
    )
    _TEST_RUNTIMES.append(module._execution_runtime)
    return gateway


def close_test_runtimes() -> None:
    """Close runtimes created by the lightweight test harness."""
    while _TEST_RUNTIMES:
        runtime = _TEST_RUNTIMES.pop()
        operation = runtime.snapshot().operation
        if operation is not None:
            runtime.wait_for_terminal(operation.handle, timeout=10.0)
        runtime.shutdown(timeout=10.0)
