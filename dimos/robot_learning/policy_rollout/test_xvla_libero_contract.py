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

import numpy as np
import pytest

from dimos.robot_learning.policy_rollout.models import BackendOutputEnvelope, RobotPolicyObservation
from dimos.robot_learning.policy_rollout.xvla_libero_contract import (
    XVLA_LIBERO_ABSOLUTE_ACTION_SPACE_ID,
    XvlaLiberoRobotContract,
)


def test_xvla_contract_converts_sample_to_libero_processor_batch() -> None:
    contract = XvlaLiberoRobotContract()

    batch = contract.to_backend_batch(_sample())

    agentview = batch.payload["observation.images.image"]
    wrist = batch.payload["observation.images.image2"]
    state = batch.payload["observation.state"]
    assert isinstance(agentview, np.ndarray)
    assert isinstance(wrist, np.ndarray)
    assert isinstance(state, np.ndarray)
    assert agentview.shape == (1, 3, 128, 128)
    assert wrist.shape == (1, 3, 128, 128)
    assert state.shape == (20,)
    assert batch.payload["task"] == "pick up the object"
    np.testing.assert_array_equal(batch.payload["domain_id"], [3])
    assert batch.metadata["policy_family"] == "xvla"
    assert batch.metadata["action_mode"] == "absolute"


def test_xvla_contract_accepts_structured_robot_state() -> None:
    contract = XvlaLiberoRobotContract()
    sample = _sample(
        state={
            "eef.pos": np.asarray([[0.1, 0.2, 0.3]], dtype=np.float32),
            "eef.mat": np.eye(3, dtype=np.float32).reshape(1, 3, 3),
        }
    )

    batch = contract.to_backend_batch(sample)

    state = batch.payload["observation.state"]
    assert isinstance(state, np.ndarray)
    np.testing.assert_allclose(state[:3], [0.1, 0.2, 0.3])


def test_xvla_contract_rejects_missing_or_wrong_inputs() -> None:
    contract = XvlaLiberoRobotContract()
    with pytest.raises(ValueError, match="wrist"):
        contract.to_backend_batch(_sample(streams=("agentview", "robot_state")))
    with pytest.raises(ValueError, match="dtype uint8"):
        contract.to_backend_batch(_sample(agentview=np.zeros((128, 128, 3), dtype=np.float32)))
    with pytest.raises(ValueError, match=r"shape \(8,\)"):
        contract.to_backend_batch(_sample(state=[0.0] * 7))
    with pytest.raises(ValueError, match="quaternion"):
        contract.to_backend_batch(_sample(state=[0.0] * 8))
    with pytest.raises(ValueError, match="task language"):
        contract.to_backend_batch(_sample(language=""))


def test_xvla_contract_converts_finite_chunk_output_to_robot_policy_chunk() -> None:
    contract = XvlaLiberoRobotContract()

    chunk = contract.chunk_from_backend_output(
        BackendOutputEnvelope(
            output=(
                (0.0, 0.1, -0.1, 1.2, -2.2, 0.3, 1.0),
                (0.1, 0.2, -0.2, 0.3, -0.3, 2.4, -1.0),
            ),
            metadata={"policy_family": "xvla"},
        )
    )

    assert chunk.space_id == XVLA_LIBERO_ABSOLUTE_ACTION_SPACE_ID
    assert chunk.shape == (2, 7)
    assert chunk.metadata["backend_metadata"] == {"policy_family": "xvla"}
    assert chunk.values[0] == pytest.approx((0.0, 0.1, -0.1, 1.2, -2.2, 0.3, 1.0))


def test_xvla_contract_converts_internal_padded_action_chunk_to_libero_7d() -> None:
    contract = XvlaLiberoRobotContract()

    chunk = contract.chunk_from_backend_output(
        BackendOutputEnvelope(
            output=((0.1, 0.2, 0.3, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.6, *([0.0] * 10)),),
            metadata={"policy_family": "xvla", "output_shape": [1, 20]},
        )
    )

    assert chunk.space_id == XVLA_LIBERO_ABSOLUTE_ACTION_SPACE_ID
    assert chunk.shape == (1, 7)
    assert chunk.values[0] == pytest.approx((0.1, 0.2, 0.3, 0.0, 0.0, 0.0, 1.0))


def test_xvla_contract_rejects_incompatible_output_before_execution() -> None:
    contract = XvlaLiberoRobotContract()
    with pytest.raises(ValueError, match=r"shape \(N, 7\)"):
        contract.chunk_from_backend_output(BackendOutputEnvelope(output=(0.0,) * 7))
    with pytest.raises(ValueError, match="empty"):
        contract.chunk_from_backend_output(BackendOutputEnvelope(output=()))
    with pytest.raises(ValueError, match="non-finite"):
        contract.chunk_from_backend_output(
            BackendOutputEnvelope(output=((0.0,) * 6 + (float("nan"),),))
        )
    with pytest.raises(ValueError, match="shape"):
        contract.from_backend_output(BackendOutputEnvelope(output=(0.0,) * 6))


def _sample(
    *,
    streams: tuple[str, ...] = ("agentview", "eye_in_hand", "robot_state"),
    agentview: np.ndarray | None = None,
    wrist: np.ndarray | None = None,
    state: object | None = None,
    language: str = "pick up the object",
) -> RobotPolicyObservation:
    agentview_payload = (
        agentview if agentview is not None else np.zeros((128, 128, 3), dtype=np.uint8)
    )
    wrist_payload = wrist if wrist is not None else np.ones((128, 128, 3), dtype=np.uint8)
    state_values = state if state is not None else [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0]
    observations: dict[str, object] = {}
    if "agentview" in streams:
        observations["agentview"] = agentview_payload
    if "eye_in_hand" in streams:
        observations["eye_in_hand"] = wrist_payload
    if "robot_state" in streams:
        observations["robot_state"] = state_values
    return RobotPolicyObservation(
        observations=observations,
        metadata={"language": language},
    )
