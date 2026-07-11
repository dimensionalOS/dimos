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

from dataclasses import asdict

import pytest

from dimos.robot_learning.policy_rollout.models import (
    RobotPolicyAction,
    RobotPolicyActionChunk,
    RobotPolicyObservation,
)


def test_robot_policy_observation_is_runtime_independent_dataclass() -> None:
    observation = RobotPolicyObservation(
        observations={"agentview": "image", "robot_state": [0.0] * 8},
        timestamps={"agentview": 1.25},
        metadata={"source": "test", "language": "pick up the object"},
    )

    payload = asdict(observation)

    assert payload["observations"]["robot_state"] == [0.0] * 8
    assert payload["metadata"]["language"] == "pick up the object"
    assert payload["timestamps"] == {"agentview": 1.25}


def test_robot_policy_action_is_runtime_independent_dataclass() -> None:
    action = RobotPolicyAction(
        space_id="libero.ee_delta_6d_gripper.normalized.v1",
        values=(0.0, 0.1, -0.1, 0.2, -0.2, 0.3, 1.0),
        metadata={"backend": "fake"},
    )

    payload = asdict(action)

    assert payload["kind"] == "robot_policy_action"
    assert payload["space_id"] == "libero.ee_delta_6d_gripper.normalized.v1"
    assert payload["values"] == (0.0, 0.1, -0.1, 0.2, -0.2, 0.3, 1.0)


def test_robot_policy_action_chunk_exposes_shape_and_first_action() -> None:
    chunk = RobotPolicyActionChunk(
        space_id="libero.ee_delta_6d_gripper.normalized.v1",
        values=((0.0, 0.1), (0.2, 0.3)),
        sequence=3,
        timestamps=(1.0, 1.1),
        metadata={"backend": "fake"},
    )

    assert chunk.kind == "robot_policy_action_chunk"
    assert chunk.horizon == 2
    assert chunk.action_dim == 2
    assert chunk.shape == (2, 2)
    assert chunk.first_action() == RobotPolicyAction(
        space_id="libero.ee_delta_6d_gripper.normalized.v1",
        values=(0.0, 0.1),
        sequence=3,
        metadata={"backend": "fake"},
    )


def test_robot_policy_action_chunk_rejects_empty_or_ragged_values() -> None:
    with pytest.raises(ValueError, match="at least one"):
        RobotPolicyActionChunk(space_id="space", values=())
    with pytest.raises(ValueError, match="must not be empty"):
        RobotPolicyActionChunk(space_id="space", values=((),))
    with pytest.raises(ValueError, match="consistent dimensions"):
        RobotPolicyActionChunk(space_id="space", values=((0.0,), (0.1, 0.2)))
    with pytest.raises(ValueError, match="timestamps"):
        RobotPolicyActionChunk(space_id="space", values=((0.0,),), timestamps=(1.0, 1.1))
