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

"""Generic RPC client for control-coordinator commands."""

from typing import cast

from dimos.control.coordinator import ControlCoordinator
from dimos.control.tasks.trajectory_task.trajectory_task import (
    TrajectoryCancellationResult,
    TrajectoryExecutionResult,
)
from dimos.core.rpc_client import RPCClient
from dimos.msgs.trajectory_msgs.JointTrajectory import JointTrajectory


class ControlCoordinatorClient:
    """Own one coordinator RPC connection and expose typed command helpers."""

    def __init__(self, rpc_client: RPCClient | None = None) -> None:
        self._rpc_client = (
            rpc_client if rpc_client is not None else RPCClient(None, ControlCoordinator)
        )
        self._closed = False

    def execute_trajectory(self, trajectory: JointTrajectory) -> TrajectoryExecutionResult:
        """Execute a trajectory through the coordinator's sole trajectory task."""
        return cast(
            "TrajectoryExecutionResult",
            self._rpc_client.execute_trajectory(trajectory),
        )

    def cancel_trajectory(self) -> TrajectoryCancellationResult:
        """Cancel the coordinator's sole trajectory task."""
        return cast(
            "TrajectoryCancellationResult",
            self._rpc_client.cancel_trajectory(),
        )

    def set_gripper_position(self, hardware_id: str, position: float) -> bool:
        """Set the position of a coordinator-owned gripper."""
        return cast(
            "bool",
            self._rpc_client.set_gripper_position(hardware_id, position),
        )

    def get_gripper_position(self, hardware_id: str) -> float | None:
        """Return the position of a coordinator-owned gripper."""
        return cast(
            "float | None",
            self._rpc_client.get_gripper_position(hardware_id),
        )

    def close(self) -> None:
        """Close the owned RPC connection."""
        if self._closed:
            return
        self._rpc_client.stop_rpc_client()
        self._closed = True
