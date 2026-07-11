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
from numpy.typing import NDArray
import pytest

from dimos.robot_learning.policy_rollout.models import BackendOutputEnvelope, RobotPolicyObservation
from dimos.robot_learning.policy_rollout.vla_jepa_libero_contract import (
    VLA_JEPA_LIBERO_ACTION_SPACE_ID,
    VlaJepaLiberoRobotContract,
)


def test_contract_converts_runtime_sample_to_backend_batch() -> None:
    contract = VlaJepaLiberoRobotContract()
    sample = _sample()

    batch = contract.to_backend_batch(sample)

    agentview = batch.payload["observation.images.image"]
    wrist = batch.payload["observation.images.image2"]
    state = batch.payload["observation.state"]
    assert isinstance(agentview, np.ndarray)
    assert isinstance(wrist, np.ndarray)
    assert isinstance(state, np.ndarray)
    agentview_array: NDArray[np.float32] = agentview
    wrist_array: NDArray[np.float32] = wrist
    assert agentview_array.shape == (3, 128, 128)
    assert agentview_array.dtype == np.float32
    assert np.max(agentview_array) <= 1.0
    assert wrist_array.shape == (3, 128, 128)
    assert state.shape == (8,)
    assert batch.payload["task"] == "pick up the object"
    assert batch.metadata["agentview_stream"] == "agentview"
    assert batch.metadata["wrist_stream"] == "eye_in_hand"


def test_contract_flips_images_height_and_width_before_chw() -> None:
    image = np.array(
        [
            [[10, 0, 0], [20, 0, 0], [30, 0, 0]],
            [[40, 0, 0], [50, 0, 0], [60, 0, 0]],
        ],
        dtype=np.uint8,
    )
    contract = VlaJepaLiberoRobotContract()

    batch = contract.to_backend_batch(_sample(agentview=image))

    agentview = batch.payload["observation.images.image"]

    assert isinstance(agentview, np.ndarray)
    assert agentview.shape == (3, 2, 3)
    assert np.allclose(
        agentview[0],
        np.array(
            [[60 / 255.0, 50 / 255.0, 40 / 255.0], [30 / 255.0, 20 / 255.0, 10 / 255.0]],
            dtype=np.float32,
        ),
    )


def test_contract_rejects_missing_or_wrong_inputs() -> None:
    contract = VlaJepaLiberoRobotContract()
    with pytest.raises(ValueError, match="wrist"):
        contract.to_backend_batch(_sample(streams=("agentview", "robot_state")))
    with pytest.raises(ValueError, match="dtype uint8"):
        contract.to_backend_batch(_sample(agentview=np.zeros((128, 128, 3), dtype=np.float32)))
    with pytest.raises(ValueError, match=r"shape \(8,\)"):
        contract.to_backend_batch(_sample(state=[0.0] * 7))
    with pytest.raises(ValueError, match="task language"):
        contract.to_backend_batch(_sample(language=""))


def test_contract_converts_backend_output_to_runtime_action() -> None:
    contract = VlaJepaLiberoRobotContract()

    action = contract.from_backend_output(
        BackendOutputEnvelope(output=(0.0, 0.1, -0.1, 0.2, -0.2, 0.3, 1.0))
    )

    assert action.space_id == VLA_JEPA_LIBERO_ACTION_SPACE_ID
    assert action.values == pytest.approx((0.0, 0.1, -0.1, 0.2, -0.2, 0.3, 1.0))


def test_contract_rejects_invalid_backend_output() -> None:
    contract = VlaJepaLiberoRobotContract()
    with pytest.raises(ValueError, match="shape"):
        contract.from_backend_output(BackendOutputEnvelope(output=(0.0,) * 6))
    with pytest.raises(ValueError, match="non-finite"):
        contract.from_backend_output(BackendOutputEnvelope(output=(0.0,) * 6 + (float("nan"),)))
    with pytest.raises(ValueError, match="within"):
        contract.from_backend_output(BackendOutputEnvelope(output=(0.0,) * 6 + (2.0,)))


def test_contract_converts_backend_output_to_runtime_action_chunk() -> None:
    contract = VlaJepaLiberoRobotContract()

    chunk = contract.chunk_from_backend_output(
        BackendOutputEnvelope(
            output=(
                (0.0, 0.1, -0.1, 0.2, -0.2, 0.3, 1.0),
                (0.1, 0.2, -0.2, 0.3, -0.3, 0.4, -1.0),
            )
        )
    )

    assert chunk.space_id == VLA_JEPA_LIBERO_ACTION_SPACE_ID
    assert chunk.shape == (2, 7)
    assert chunk.values[0] == pytest.approx((0.0, 0.1, -0.1, 0.2, -0.2, 0.3, 1.0))


def test_contract_rejects_invalid_backend_action_chunk_output() -> None:
    contract = VlaJepaLiberoRobotContract()
    with pytest.raises(ValueError, match=r"shape \(N, 7\)"):
        contract.chunk_from_backend_output(BackendOutputEnvelope(output=(0.0,) * 7))
    with pytest.raises(ValueError, match="empty"):
        contract.chunk_from_backend_output(BackendOutputEnvelope(output=()))
    with pytest.raises(ValueError, match="non-finite"):
        contract.chunk_from_backend_output(
            BackendOutputEnvelope(output=((0.0,) * 6 + (float("nan"),),))
        )
    with pytest.raises(ValueError, match="within"):
        contract.chunk_from_backend_output(BackendOutputEnvelope(output=((0.0,) * 6 + (2.0,),)))


def _sample(
    *,
    streams: tuple[str, ...] = ("agentview", "eye_in_hand", "robot_state"),
    agentview: np.ndarray | None = None,
    wrist: np.ndarray | None = None,
    state: list[float] | None = None,
    language: str = "pick up the object",
) -> RobotPolicyObservation:
    agentview_payload = (
        agentview if agentview is not None else np.zeros((128, 128, 3), dtype=np.uint8)
    )
    wrist_payload = wrist if wrist is not None else np.ones((128, 128, 3), dtype=np.uint8)
    state_values = state if state is not None else [0.0] * 8
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
