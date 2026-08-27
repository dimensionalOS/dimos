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

from collections.abc import Iterator
from types import SimpleNamespace
from typing import Any

import numpy as np
import onnxruntime as ort  # type: ignore[import-untyped]
import pytest

from dimos.control.tasks.g1_sonic_wbc_task.sonic_pipeline import (
    ENCODER_OBS_DIM,
    NUM_JOINTS,
    SMPL_JOINTS_OFFSET,
    WRIST_ONNX_INDICES,
    WRISTS_OFFSET,
    SonicPipeline,
)


@pytest.fixture
def pipeline(mocker: Any) -> Iterator[SonicPipeline]:
    encoder = mocker.MagicMock()
    encoder.get_inputs.return_value = [
        SimpleNamespace(name="encoder_input", shape=[1, ENCODER_OBS_DIM])
    ]
    encoder.get_providers.return_value = [
        "CUDAExecutionProvider",
        "CPUExecutionProvider",
    ]
    encoder.run.return_value = [np.zeros((1, 64), dtype=np.float32)]

    decoder = mocker.MagicMock()
    decoder.get_inputs.return_value = [SimpleNamespace(name="decoder_input", shape=[1, 994])]
    decoder.get_providers.return_value = [
        "CUDAExecutionProvider",
        "CPUExecutionProvider",
    ]

    planner = mocker.MagicMock()
    planner.get_providers.return_value = [
        "CUDAExecutionProvider",
        "CPUExecutionProvider",
    ]

    inference_session = mocker.patch.object(
        ort,
        "InferenceSession",
        side_effect=[encoder, decoder, planner],
    )
    preload_dlls = mocker.patch.object(ort, "preload_dlls")
    mocker.patch.object(
        ort,
        "get_available_providers",
        return_value=["CUDAExecutionProvider", "CPUExecutionProvider"],
    )
    instance = SonicPipeline("encoder.onnx", "decoder.onnx", "planner.onnx")
    try:
        preload_dlls.assert_called_once_with()
        assert inference_session.call_count == 3
        assert [call.kwargs["providers"] for call in inference_session.call_args_list] == [
            ["CUDAExecutionProvider"],
            ["CUDAExecutionProvider"],
            ["CUDAExecutionProvider", "CPUExecutionProvider"],
        ]
        yield instance
    finally:
        instance._planner_executor.shutdown(wait=True)


def test_pipeline_fails_before_loading_models_when_cuda_is_unavailable(mocker: Any) -> None:
    mocker.patch.object(
        ort,
        "get_available_providers",
        return_value=["CPUExecutionProvider"],
    )
    preload_dlls = mocker.patch.object(ort, "preload_dlls")
    inference_session = mocker.patch.object(ort, "InferenceSession")

    with pytest.raises(RuntimeError, match="requires CUDAExecutionProvider"):
        SonicPipeline("encoder.onnx", "decoder.onnx", "planner.onnx")

    preload_dlls.assert_not_called()
    inference_session.assert_not_called()


def test_pipeline_fails_if_any_model_does_not_activate_cuda(mocker: Any) -> None:
    cuda_session = mocker.MagicMock()
    cuda_session.get_inputs.return_value = [
        SimpleNamespace(name="input", shape=[1, ENCODER_OBS_DIM])
    ]
    cuda_session.get_providers.return_value = [
        "CUDAExecutionProvider",
        "CPUExecutionProvider",
    ]
    cpu_session = mocker.MagicMock()
    cpu_session.get_providers.return_value = ["CPUExecutionProvider"]
    mocker.patch.object(
        ort,
        "get_available_providers",
        return_value=["CUDAExecutionProvider", "CPUExecutionProvider"],
    )
    mocker.patch.object(ort, "preload_dlls")
    inference_session = mocker.patch.object(
        ort,
        "InferenceSession",
        side_effect=[cuda_session, cuda_session, cpu_session],
    )

    with pytest.raises(RuntimeError, match="planner.*did not activate CUDA"):
        SonicPipeline("encoder.onnx", "decoder.onnx", "planner.onnx")

    assert inference_session.call_count == 3


def test_pipeline_does_not_retry_planner_on_cpu(mocker: Any) -> None:
    cuda_session = mocker.MagicMock()
    cuda_session.get_inputs.return_value = [
        SimpleNamespace(name="input", shape=[1, ENCODER_OBS_DIM])
    ]
    cuda_session.get_providers.return_value = [
        "CUDAExecutionProvider",
        "CPUExecutionProvider",
    ]
    mocker.patch.object(
        ort,
        "get_available_providers",
        return_value=["CUDAExecutionProvider", "CPUExecutionProvider"],
    )
    mocker.patch.object(ort, "preload_dlls")
    inference_session = mocker.patch.object(
        ort,
        "InferenceSession",
        side_effect=[cuda_session, cuda_session, RuntimeError("CUDA load failed")],
    )

    with pytest.raises(RuntimeError, match="CUDA load failed"):
        SonicPipeline("encoder.onnx", "decoder.onnx", "planner.onnx")

    assert inference_session.call_count == 3


def test_smpl_pose_chunk_populates_all_ten_encoder_frames(pipeline: SonicPipeline) -> None:
    smpl_joints = np.zeros((10, 24, 3), dtype=np.float32)
    smpl_joints[:, :, 0] = np.arange(10, dtype=np.float32)[:, np.newaxis]
    joint_pos = np.zeros((10, NUM_JOINTS), dtype=np.float32)
    joint_pos[:, WRIST_ONNX_INDICES] = np.arange(10, dtype=np.float32)[:, np.newaxis]
    identity_quaternions = np.tile(
        np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float32),
        (10, 1),
    )
    fields: dict[str, np.ndarray[Any, Any]] = {
        "frame_index": np.arange(10, dtype=np.int64),
        "joint_pos": joint_pos,
        "joint_vel": np.zeros_like(joint_pos),
        "body_quat_w": identity_quaternions,
        "smpl_joints": smpl_joints,
        "smpl_pose": np.zeros((10, 21, 3), dtype=np.float32),
    }

    summary = pipeline.apply_pose_message(fields)
    observation = pipeline._build_streamed_encoder_obs(identity_quaternions[0])

    assert summary == {"frames": 10, "encode_mode": 2, "catchup": True}
    assert observation.shape == (ENCODER_OBS_DIM,)
    encoded_smpl = observation[SMPL_JOINTS_OFFSET : SMPL_JOINTS_OFFSET + 720].reshape(10, 24, 3)
    encoded_wrists = observation[WRISTS_OFFSET : WRISTS_OFFSET + 60].reshape(10, 6)
    np.testing.assert_array_equal(encoded_smpl, smpl_joints)
    np.testing.assert_array_equal(encoded_wrists, joint_pos[:, WRIST_ONNX_INDICES])


def test_two_frame_pose_chunk_holds_newest_frame_across_encoder_window(
    pipeline: SonicPipeline,
) -> None:
    smpl_joints = np.zeros((2, 24, 3), dtype=np.float32)
    smpl_joints[:, :, 0] = np.arange(2, dtype=np.float32)[:, np.newaxis]
    joint_pos = np.zeros((2, NUM_JOINTS), dtype=np.float32)
    joint_pos[:, WRIST_ONNX_INDICES] = np.arange(2, dtype=np.float32)[:, np.newaxis]
    identity_quaternions = np.tile(
        np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float32),
        (2, 1),
    )
    fields: dict[str, np.ndarray[Any, Any]] = {
        "frame_index": np.arange(2, dtype=np.int64),
        "joint_pos": joint_pos,
        "joint_vel": np.zeros_like(joint_pos),
        "body_quat_w": identity_quaternions,
        "smpl_joints": smpl_joints,
        "smpl_pose": np.zeros((2, 21, 3), dtype=np.float32),
    }

    summary = pipeline.apply_pose_message(fields)
    observation = pipeline._build_streamed_encoder_obs(identity_quaternions[0])

    encoded_smpl = observation[SMPL_JOINTS_OFFSET : SMPL_JOINTS_OFFSET + 720].reshape(10, 24, 3)
    encoded_wrists = observation[WRISTS_OFFSET : WRISTS_OFFSET + 60].reshape(10, 6)
    assert summary == {"frames": 2, "encode_mode": 2, "catchup": True}
    np.testing.assert_array_equal(encoded_smpl[:2], smpl_joints)
    np.testing.assert_array_equal(encoded_smpl[2:], np.repeat(smpl_joints[1:2], 8, axis=0))
    np.testing.assert_array_equal(encoded_wrists[:2], joint_pos[:, WRIST_ONNX_INDICES])
    np.testing.assert_array_equal(
        encoded_wrists[2:],
        np.repeat(joint_pos[1:2, WRIST_ONNX_INDICES], 8, axis=0),
    )
