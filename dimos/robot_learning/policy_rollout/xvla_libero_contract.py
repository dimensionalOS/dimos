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

from __future__ import annotations

from collections.abc import Mapping, Sequence
from typing import cast

import numpy as np

from dimos.robot_learning.policy_rollout.models import (
    BackendBatch,
    BackendOutputEnvelope,
    RobotPolicyAction,
    RobotPolicyActionChunk,
    RobotPolicyObservation,
)

XVLA_LIBERO_DEFAULT_DOMAIN_ID = 3
XVLA_LIBERO_ABSOLUTE_ACTION_SPACE_ID = "libero.ee_pose_axis_angle_gripper.absolute.v1"


class XvlaLiberoRobotContract:
    """X-VLA LIBERO IO contract for LeRobot's LIBERO processor pipeline."""

    def __init__(
        self,
        *,
        agentview_stream: str = "agentview",
        wrist_stream_candidates: Sequence[str] = ("eye_in_hand", "wrist", "robot0_eye_in_hand"),
        state_stream: str = "robot_state",
        structured_state_stream: str = "xvla_robot_state",
        domain_id: int = XVLA_LIBERO_DEFAULT_DOMAIN_ID,
    ) -> None:
        self._agentview_stream = agentview_stream
        self._wrist_stream_candidates = tuple(wrist_stream_candidates)
        self._state_stream = state_stream
        self._structured_state_stream = structured_state_stream
        self._domain_id = int(domain_id)

    def to_backend_batch(self, sample: RobotPolicyObservation) -> BackendBatch:
        observations = sample.observations
        agentview = self._image_payload(observations, self._agentview_stream, flip=True)
        wrist_stream = self._select_wrist_stream(observations)
        wrist = self._image_payload(observations, wrist_stream, flip=False)
        state = self._state_payload(observations)
        language = self._language(sample)

        return BackendBatch(
            payload={
                "observation.images.image": agentview,
                "observation.images.image2": wrist,
                "observation.state": state,
                "task": language,
                "domain_id": np.asarray([self._domain_id], dtype=np.int64),
            },
            metadata={
                "agentview_stream": self._agentview_stream,
                "wrist_stream": wrist_stream,
                "policy_family": "xvla",
                "domain_id": self._domain_id,
                "action_mode": "absolute",
            },
        )

    def from_backend_output(self, output: BackendOutputEnvelope) -> RobotPolicyAction:
        raw_values = np.asarray(output.output, dtype=np.float32)
        if raw_values.shape != (7,):
            raw_values = _xvla_internal_action_to_libero(raw_values.reshape(1, -1))[0]
        values = _validated_action_values(raw_values, shape=(7,), label="X-VLA LIBERO action")
        return RobotPolicyAction(
            space_id=XVLA_LIBERO_ABSOLUTE_ACTION_SPACE_ID,
            values=tuple(float(value) for value in values),
            metadata={"backend_metadata": dict(output.metadata)},
        )

    def chunk_from_backend_output(self, output: BackendOutputEnvelope) -> RobotPolicyActionChunk:
        if isinstance(output.output, Sequence) and len(output.output) == 0:
            raise ValueError("X-VLA LIBERO action chunk must not be empty")
        values = np.asarray(output.output, dtype=np.float32)
        if values.ndim != 2:
            raise ValueError(
                f"X-VLA LIBERO action chunk must have shape (N, 7), got {values.shape}"
            )
        if values.shape[1] != 7:
            values = _xvla_internal_action_to_libero(values)
        _validate_finite(values, label="X-VLA LIBERO action chunk")
        return RobotPolicyActionChunk(
            space_id=XVLA_LIBERO_ABSOLUTE_ACTION_SPACE_ID,
            values=tuple(tuple(float(item) for item in row) for row in values),
            metadata={"backend_metadata": dict(output.metadata)},
        )

    def _select_wrist_stream(self, observations: Mapping[str, object]) -> str:
        for stream in self._wrist_stream_candidates:
            if stream in observations:
                return stream
        raise ValueError(
            "missing X-VLA LIBERO wrist/eye-in-hand image stream; "
            f"expected one of {self._wrist_stream_candidates}"
        )

    def _image_payload(
        self, observations: Mapping[str, object], stream: str, *, flip: bool
    ) -> np.ndarray:
        payload = observations.get(stream)
        if payload is None:
            raise ValueError(f"missing X-VLA LIBERO image stream {stream!r}")
        image = np.asarray(payload)
        if image.ndim != 3 or image.shape[2] != 3:
            raise ValueError(f"image stream {stream!r} must have HWC RGB shape")
        if image.dtype != np.uint8:
            raise ValueError(f"image stream {stream!r} must have dtype uint8")
        if flip:
            image = np.flip(image, axis=(0, 1)).copy()
        chw = np.transpose(image, (2, 0, 1)).astype(np.float32) / 255.0
        return np.expand_dims(chw, axis=0)

    def _state_payload(self, observations: Mapping[str, object]) -> np.ndarray:
        structured_payload = observations.get(self._structured_state_stream)
        if isinstance(structured_payload, Mapping):
            return _structured_robot_state_to_xvla_state(
                _structured_robot_state(structured_payload)
            )
        state_payload = observations.get(self._state_stream)
        if state_payload is None:
            raise ValueError(f"missing X-VLA LIBERO state stream {self._state_stream!r}")
        if isinstance(state_payload, Mapping):
            robot_state = _structured_robot_state(state_payload)
        else:
            robot_state = _state_vector_to_structured_robot_state(state_payload)
        return _structured_robot_state_to_xvla_state(robot_state)

    def _language(self, sample: RobotPolicyObservation) -> str:
        language = sample.metadata.get("language")
        if isinstance(language, str) and language.strip():
            return language
        observed_language = sample.observations.get("language")
        if isinstance(observed_language, str) and observed_language.strip():
            return observed_language
        raise ValueError("missing X-VLA LIBERO task language")


def _structured_robot_state(payload: Mapping[str, object]) -> dict[str, np.ndarray]:
    pos = _batched_float_array(payload.get("eef.pos"), shape=(1, 3), label="eef.pos")
    mat = _batched_float_array(payload.get("eef.mat"), shape=(1, 3, 3), label="eef.mat")
    return {"eef.pos": pos, "eef.mat": mat}


def _state_vector_to_structured_robot_state(payload: object) -> dict[str, np.ndarray]:
    vector = np.asarray(payload, dtype=np.float32)
    if vector.shape != (8,):
        raise ValueError(f"X-VLA LIBERO robot state must have shape (8,), got {vector.shape}")
    if not np.all(np.isfinite(vector)):
        raise ValueError("X-VLA LIBERO robot state contains non-finite values")
    pos = vector[:3].reshape(1, 3)
    quat = _normalized_quaternion(vector[3:7])
    mat = _quaternion_to_rotation_matrix(quat).reshape(1, 3, 3)
    return {"eef.pos": pos.astype(np.float32), "eef.mat": mat.astype(np.float32)}


def _structured_robot_state_to_xvla_state(robot_state: Mapping[str, np.ndarray]) -> np.ndarray:
    pos = robot_state["eef.pos"]
    mat = robot_state["eef.mat"]
    rot6d = np.concatenate((mat[:, :3, 0], mat[:, :3, 1]), axis=-1)
    extra = np.zeros((pos.shape[0], 1), dtype=np.float32)
    proprio = np.concatenate((pos, rot6d.astype(np.float32), extra), axis=-1)
    state = np.concatenate((proprio, np.zeros_like(proprio)), axis=-1)
    return state[0].astype(np.float32)


def _normalized_quaternion(value: np.ndarray) -> np.ndarray:
    norm = float(np.linalg.norm(value))
    if not np.isfinite(norm) or norm <= 0.0:
        raise ValueError("X-VLA LIBERO robot state quaternion must have non-zero finite norm")
    return value.astype(np.float32) / norm


def _quaternion_to_rotation_matrix(quat: np.ndarray) -> np.ndarray:
    x, y, z, w = (float(item) for item in quat)
    return np.asarray(
        [
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
            [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
            [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
        ],
        dtype=np.float32,
    )


def _batched_float_array(value: object, *, shape: tuple[int, ...], label: str) -> np.ndarray:
    array = np.asarray(value, dtype=np.float32)
    if array.shape != shape:
        raise ValueError(f"X-VLA LIBERO {label} must have shape {shape}, got {array.shape}")
    if not np.all(np.isfinite(array)):
        raise ValueError(f"X-VLA LIBERO {label} contains non-finite values")
    return array


def _validated_action_values(value: object, *, shape: tuple[int, ...], label: str) -> np.ndarray:
    values = np.asarray(value, dtype=np.float32)
    if values.shape != shape:
        raise ValueError(f"{label} must have shape {shape}, got {values.shape}")
    _validate_finite(values, label=label)
    return values


def _xvla_internal_action_to_libero(values: np.ndarray) -> np.ndarray:
    if values.ndim != 2 or values.shape[1] < 10:
        raise ValueError(f"X-VLA LIBERO action chunk must have shape (N, 7), got {values.shape}")
    _validate_finite(values, label="X-VLA LIBERO internal action chunk")
    target_eef = values[:, :3]
    rotation_6d = values[:, 3:9]
    gripper = values[:, 9:10]
    axis_angle = _rotation_6d_to_axis_angle(rotation_6d)
    converted = np.concatenate((target_eef, axis_angle, gripper), axis=-1).astype(np.float32)
    converted[:, -1] = np.where(converted[:, -1] > 0.5, 1.0, -1.0)
    return converted


def _rotation_6d_to_axis_angle(rotation_6d: np.ndarray) -> np.ndarray:
    first = rotation_6d[:, :3]
    second = rotation_6d[:, 3:6]
    basis_1 = _normalize_rows(first)
    second_orthogonal = second - np.sum(basis_1 * second, axis=-1, keepdims=True) * basis_1
    basis_2 = _normalize_rows(second_orthogonal)
    basis_3 = np.cross(basis_1, basis_2)
    matrices = np.stack((basis_1, basis_2, basis_3), axis=-1)
    return _rotation_matrix_to_axis_angle(matrices)


def _normalize_rows(values: np.ndarray) -> np.ndarray:
    norms = np.linalg.norm(values, axis=-1, keepdims=True)
    if np.any(~np.isfinite(norms)) or np.any(norms <= 1.0e-8):
        raise ValueError("X-VLA LIBERO rotation 6D values must have non-zero finite rows")
    return values / norms


def _rotation_matrix_to_axis_angle(matrices: np.ndarray) -> np.ndarray:
    axis_angles: list[np.ndarray] = []
    for matrix in matrices:
        trace = float(np.trace(matrix))
        cos_angle = np.clip((trace - 1.0) / 2.0, -1.0, 1.0)
        angle = float(np.arccos(cos_angle))
        if angle < 1.0e-6:
            axis_angles.append(np.zeros(3, dtype=np.float32))
            continue
        denominator = 2.0 * np.sin(angle)
        axis = (
            np.asarray(
                [
                    matrix[2, 1] - matrix[1, 2],
                    matrix[0, 2] - matrix[2, 0],
                    matrix[1, 0] - matrix[0, 1],
                ],
                dtype=np.float32,
            )
            / denominator
        )
        axis_angles.append((axis * angle).astype(np.float32))
    return np.stack(axis_angles, axis=0)


def _validate_finite(values: np.ndarray, *, label: str) -> None:
    if not np.all(np.isfinite(values)):
        raise ValueError(f"{label} contains non-finite values")


def create_contract(**params: object) -> XvlaLiberoRobotContract:
    wrist_stream_candidates = params.get(
        "wrist_stream_candidates", ("eye_in_hand", "wrist", "robot0_eye_in_hand")
    )
    return XvlaLiberoRobotContract(
        agentview_stream=str(params.get("agentview_stream", "agentview")),
        wrist_stream_candidates=cast("Sequence[str]", wrist_stream_candidates),
        state_stream=str(params.get("state_stream", "robot_state")),
        structured_state_stream=str(params.get("structured_state_stream", "xvla_robot_state")),
        domain_id=int(str(params.get("domain_id", XVLA_LIBERO_DEFAULT_DOMAIN_ID))),
    )
