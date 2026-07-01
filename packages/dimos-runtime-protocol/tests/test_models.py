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

from dimos_runtime_protocol import (
    CommandMode,
    MotorActionFrame,
    MotorDescription,
    ObservationFrame,
    ObservationKind,
    ProtocolVersion,
    RobotMotorSurface,
    RuntimeActionFrame,
    RuntimeDescription,
    StepRequest,
    check_compatible,
)
from dimos_runtime_protocol.codecs import from_json_bytes, to_json_bytes
from pydantic import ValidationError
import pytest


def test_runtime_description_round_trip() -> None:
    surface = RobotMotorSurface(
        robot_id="panda",
        motors=[MotorDescription(name="panda/joint1", index=0)],
    )
    desc = RuntimeDescription(
        runtime_id="fake",
        backend="fake",
        robot_surfaces=[surface],
        control_step_hz=100,
    )

    decoded = from_json_bytes(to_json_bytes(desc), RuntimeDescription)

    assert decoded == desc
    assert check_compatible(decoded).compatible


def test_invalid_step_request_rejected() -> None:
    with pytest.raises(ValidationError):
        StepRequest.model_validate({"episode_id": "ep", "tick_id": 1})


def test_observation_frame_metadata() -> None:
    frame = ObservationFrame(
        stream="agentview",
        kind=ObservationKind.IMAGE,
        encoding="png",
        shape=[64, 64, 3],
        dtype="uint8",
        data_ref="payloads/agentview_000001.png",
    )

    assert frame.kind == ObservationKind.IMAGE
    assert frame.data_ref is not None


def test_action_extra_fields_rejected() -> None:
    with pytest.raises(ValidationError):
        MotorActionFrame.model_validate(
            {
                "robot_id": "panda",
                "mode": CommandMode.POSITION,
                "names": ["panda/joint1"],
                "q": [0.0],
                "unexpected": True,
            }
        )


def test_step_request_accepts_motor_action_frame_payload() -> None:
    request = StepRequest.model_validate(
        {
            "episode_id": "ep",
            "tick_id": 1,
            "action": {
                "robot_id": "panda",
                "mode": "position",
                "names": ["panda/joint1"],
                "q": [0.0],
                "sequence": 7,
            },
        }
    )

    assert isinstance(request.action, MotorActionFrame)
    assert request.action.mode == CommandMode.POSITION
    assert request.action.sequence == 7


def test_runtime_action_frame_round_trip() -> None:
    frame = RuntimeActionFrame(
        frame_type="runtime_action",
        space_id="libero.ee_delta_6d_gripper.normalized.v1",
        values=[0.0, 0.1, -0.2, 0.3, -0.4, 0.5, 1.0],
        tick_id=2,
    )

    decoded = from_json_bytes(to_json_bytes(frame), RuntimeActionFrame)

    assert decoded == frame
    assert decoded.frame_type == "runtime_action"


def test_step_request_accepts_runtime_action_frame_payload() -> None:
    request = StepRequest.model_validate(
        {
            "episode_id": "ep",
            "tick_id": 2,
            "action": {
                "frame_type": "runtime_action",
                "space_id": "libero.ee_delta_6d_gripper.normalized.v1",
                "values": [0.0, 0.1, -0.2, 0.3, -0.4, 0.5, 1.0],
                "sequence": 10,
            },
        }
    )

    assert isinstance(request.action, RuntimeActionFrame)
    assert request.action.space_id == "libero.ee_delta_6d_gripper.normalized.v1"
    assert request.action.sequence == 10


@pytest.mark.parametrize("value", [float("inf"), float("-inf"), float("nan")])
def test_runtime_action_frame_rejects_non_finite_values(value: float) -> None:
    with pytest.raises(ValidationError):
        RuntimeActionFrame.model_validate(
            {
                "frame_type": "runtime_action",
                "space_id": "libero.ee_delta_6d_gripper.normalized.v1",
                "values": [0.0, value],
                "sequence": 1,
            }
        )


def test_runtime_action_frame_requires_sequence_or_tick_id() -> None:
    with pytest.raises(ValidationError):
        RuntimeActionFrame.model_validate(
            {
                "frame_type": "runtime_action",
                "space_id": "libero.ee_delta_6d_gripper.normalized.v1",
                "values": [0.0],
            }
        )


@pytest.mark.parametrize(
    "action_payload",
    [
        {
            "frame_type": "runtime_action",
            "space_id": "libero.ee_delta_6d_gripper.normalized.v1",
            "values": [0.0],
            "sequence": 1,
            "robot_id": "panda",
        },
        {
            "frame_type": "unsupported",
            "space_id": "libero.ee_delta_6d_gripper.normalized.v1",
            "values": [0.0],
            "sequence": 1,
        },
        {
            "space_id": "libero.ee_delta_6d_gripper.normalized.v1",
            "values": [0.0],
            "sequence": 1,
        },
    ],
)
def test_step_request_rejects_ambiguous_or_unsupported_actions(
    action_payload: dict[str, object],
) -> None:
    with pytest.raises(ValidationError):
        StepRequest.model_validate({"episode_id": "ep", "tick_id": 1, "action": action_payload})


def test_incompatible_protocol_version_rejected() -> None:
    result = check_compatible(ProtocolVersion(version="1.0", min_compatible="1.0"))

    assert not result.compatible
    assert "major mismatch" in result.reason or "older" in result.reason
