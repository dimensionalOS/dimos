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
from typing import Any, cast

import numpy as np

from dimos.robot_learning.policy_rollout.models import (
    BackendBatch,
    BackendOutputEnvelope,
    RobotPolicyAction,
    RobotPolicyActionChunk,
    RobotPolicyObservation,
)

VLA_JEPA_LIBERO_ACTION_SPACE_ID = "libero.ee_delta_6d_gripper.normalized.v1"


class VlaJepaLiberoRobotContract:
    """Hardcoded VLA-JEPA LIBERO policy IO contract for the v1 rollout gate."""

    def __init__(
        self,
        *,
        agentview_stream: str = "agentview",
        wrist_stream_candidates: Sequence[str] = ("eye_in_hand", "wrist", "robot0_eye_in_hand"),
        state_stream: str = "robot_state",
    ) -> None:
        self._agentview_stream = agentview_stream
        self._wrist_stream_candidates = tuple(wrist_stream_candidates)
        self._state_stream = state_stream

    def to_backend_batch(self, sample: RobotPolicyObservation) -> BackendBatch:
        observations = sample.observations
        agentview = self._image_payload(observations, self._agentview_stream)
        wrist_stream = self._select_wrist_stream(observations)
        wrist = self._image_payload(observations, wrist_stream)
        state = self._state_vector(observations)
        language = self._language(sample)

        return BackendBatch(
            payload={
                "observation.images.image": agentview,
                "observation.images.image2": wrist,
                "observation.state": state,
                "task": language,
            },
            metadata={
                "agentview_stream": self._agentview_stream,
                "wrist_stream": wrist_stream,
            },
        )

    def from_backend_output(self, output: BackendOutputEnvelope) -> RobotPolicyAction:
        values = np.asarray(output.output, dtype=np.float32)
        if values.shape != (7,):
            raise ValueError(f"VLA-JEPA LIBERO action must have shape (7,), got {values.shape}")
        if not np.all(np.isfinite(values)):
            raise ValueError("VLA-JEPA LIBERO action contains non-finite values")
        if np.any((values < -1.0) | (values > 1.0)):
            raise ValueError("VLA-JEPA LIBERO action must be within [-1, 1]")
        return RobotPolicyAction(
            space_id=VLA_JEPA_LIBERO_ACTION_SPACE_ID,
            values=tuple(float(value) for value in values),
            metadata={"backend_metadata": dict(output.metadata)},
        )

    def chunk_from_backend_output(self, output: BackendOutputEnvelope) -> RobotPolicyActionChunk:
        if isinstance(output.output, Sequence) and len(output.output) == 0:
            raise ValueError("VLA-JEPA LIBERO action chunk must not be empty")
        values = np.asarray(output.output, dtype=np.float32)
        if values.ndim != 2 or values.shape[1] != 7:
            raise ValueError(
                f"VLA-JEPA LIBERO action chunk must have shape (N, 7), got {values.shape}"
            )
        if not np.all(np.isfinite(values)):
            raise ValueError("VLA-JEPA LIBERO action chunk contains non-finite values")
        if np.any((values < -1.0) | (values > 1.0)):
            raise ValueError("VLA-JEPA LIBERO action chunk must be within [-1, 1]")
        return RobotPolicyActionChunk(
            space_id=VLA_JEPA_LIBERO_ACTION_SPACE_ID,
            values=tuple(tuple(float(item) for item in row) for row in values),
            metadata={"backend_metadata": dict(output.metadata)},
        )

    def _select_wrist_stream(self, observations: Mapping[str, object]) -> str:
        for stream in self._wrist_stream_candidates:
            if stream in observations:
                return stream
        raise ValueError(
            "missing VLA-JEPA LIBERO wrist/eye-in-hand image stream; "
            f"expected one of {self._wrist_stream_candidates}"
        )

    def _image_payload(
        self,
        observations: Mapping[str, object],
        stream: str,
    ) -> np.ndarray:
        payload = observations.get(stream)
        if payload is None:
            raise ValueError(f"missing VLA-JEPA LIBERO image stream {stream!r}")
        image = np.asarray(payload)
        if image.ndim != 3 or image.shape[2] != 3:
            raise ValueError(f"image stream {stream!r} must have HWC RGB shape")
        if image.dtype != np.uint8:
            raise ValueError(f"image stream {stream!r} must have dtype uint8")
        flipped = np.flip(image, axis=(0, 1)).copy()
        return np.transpose(flipped, (2, 0, 1)).astype(np.float32) / 255.0

    def _state_vector(self, observations: Mapping[str, object]) -> np.ndarray:
        state = observations.get(self._state_stream)
        if state is None:
            raise ValueError(f"missing VLA-JEPA LIBERO state stream {self._state_stream!r}")
        array = np.asarray(state, dtype=np.float32)
        if array.shape != (8,):
            raise ValueError(f"VLA-JEPA LIBERO robot state must have shape (8,), got {array.shape}")
        if not np.all(np.isfinite(array)):
            raise ValueError("VLA-JEPA LIBERO robot state contains non-finite values")
        return array

    def _language(self, sample: RobotPolicyObservation) -> str:
        language = sample.metadata.get("language")
        if isinstance(language, str) and language.strip():
            return language
        observed_language = sample.observations.get("language")
        if isinstance(observed_language, str) and observed_language.strip():
            return observed_language
        raise ValueError("missing VLA-JEPA LIBERO task language")


def create_contract(**params: object) -> VlaJepaLiberoRobotContract:
    return VlaJepaLiberoRobotContract(**cast("dict[str, Any]", params))
