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
    DDS_TO_ONNX,
    DEFAULT_ANGLES_DDS,
    NUM_JOINTS,
    SMPL_JOINTS_OFFSET,
    SONIC_LOW_LATENCY_PIPELINE,
    SONIC_V1_1_PIPELINE,
    WRIST_ONNX_INDICES,
    SonicPipeline,
    sonic_model_profile,
)

_V1_PROFILE = sonic_model_profile(SONIC_V1_1_PIPELINE)
ENCODER_OBS_DIM = _V1_PROFILE.encoder_obs_dim
WRISTS_OFFSET = _V1_PROFILE.wrists_offset


def _smpl_pose_fields(num_frames: int = 10) -> dict[str, np.ndarray[Any, Any]]:
    joint_pos = np.zeros((num_frames, NUM_JOINTS), dtype=np.float32)
    identity_quaternions = np.tile(
        np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float32),
        (num_frames, 1),
    )
    return {
        "frame_index": np.arange(num_frames, dtype=np.int64),
        "joint_pos": joint_pos,
        "joint_vel": np.zeros_like(joint_pos),
        "body_quat_w": identity_quaternions,
        "smpl_joints": np.zeros((num_frames, 24, 3), dtype=np.float32),
        "smpl_pose": np.zeros((num_frames, 21, 3), dtype=np.float32),
    }


def _policy_step(pipeline: SonicPipeline) -> np.ndarray[Any, Any]:
    return pipeline.step(
        q_dds=DEFAULT_ANGLES_DDS,
        dq_dds=np.zeros(NUM_JOINTS, dtype=np.float32),
        base_quat_wxyz=np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64),
        gyro_body=np.zeros(3, dtype=np.float32),
        gravity_body=np.array([0.0, 0.0, -1.0], dtype=np.float32),
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


def test_live_pose_window_replaces_backlogged_stream(pipeline: SonicPipeline) -> None:
    first = _smpl_pose_fields()
    pipeline.apply_pose_message(first)
    pipeline._streamed_frame = 3

    latest = _smpl_pose_fields()
    latest["frame_index"] += 20
    latest["smpl_joints"][:, :, 0] = np.arange(20, 30, dtype=np.float32)[:, None]

    summary = pipeline.set_pose_window(latest)
    observation = pipeline._build_streamed_encoder_obs(
        np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float32)
    )

    assert summary == {"frames": 10, "encode_mode": 2, "catchup": True}
    assert pipeline.snapshot()["stream_frames"] == 10
    assert pipeline.snapshot()["stream_frame"] == 0
    assert pipeline.snapshot()["stream_backlog_frames"] == 9
    encoded_smpl = observation[SMPL_JOINTS_OFFSET : SMPL_JOINTS_OFFSET + 720].reshape(10, 24, 3)
    np.testing.assert_array_equal(encoded_smpl[:, :, 0], latest["smpl_joints"][:, :, 0])

    for start in range(22, 222, 2):
        rolling = _smpl_pose_fields()
        rolling["frame_index"] += start
        pipeline.set_pose_window(rolling)
        pipeline._streamed_frame += 1
        assert pipeline.snapshot()["stream_frames"] == 10
        assert pipeline.snapshot()["stream_backlog_frames"] == 8


def test_low_latency_profile_is_the_released_four_frame_model_contract() -> None:
    profile = sonic_model_profile(SONIC_LOW_LATENCY_PIPELINE)

    assert profile.model_subdir == "low_latency"
    assert profile.encoder_obs_dim == 1247
    assert profile.smpl_frames == 4
    assert profile.g1_frame_stride == 1
    assert profile.heading_normalized is False
    assert profile.wrists_offset + profile.smpl_frames * 6 == profile.encoder_obs_dim


def test_low_latency_pipeline_accepts_only_its_1247_input_model(mocker: Any) -> None:
    encoder = mocker.MagicMock()
    encoder.get_inputs.return_value = [SimpleNamespace(name="encoder", shape=[1, 1247])]
    encoder.get_providers.return_value = ["CUDAExecutionProvider"]
    encoder.run.return_value = [np.zeros((1, 64), dtype=np.float32)]
    decoder = mocker.MagicMock()
    decoder.get_inputs.return_value = [SimpleNamespace(name="decoder", shape=[1, 994])]
    decoder.get_providers.return_value = ["CUDAExecutionProvider"]
    planner = mocker.MagicMock()
    planner.get_providers.return_value = ["CUDAExecutionProvider", "CPUExecutionProvider"]
    mocker.patch.object(
        ort,
        "get_available_providers",
        return_value=["CUDAExecutionProvider", "CPUExecutionProvider"],
    )
    mocker.patch.object(ort, "preload_dlls")
    mocker.patch.object(ort, "InferenceSession", side_effect=[encoder, decoder, planner])

    instance = SonicPipeline(
        "low_encoder.onnx",
        "low_decoder.onnx",
        "planner.onnx",
        profile=SONIC_LOW_LATENCY_PIPELINE,
    )
    try:
        assert instance.snapshot()["encoder_obs_dim"] == 1247
        assert instance.snapshot()["smpl_reference_frames"] == 4
    finally:
        instance._planner_executor.shutdown(wait=True)


def test_stream_transition_blends_planner_token_to_each_live_pose_token(
    pipeline: SonicPipeline,
    mocker: Any,
) -> None:
    pipeline._needs_replan = False
    pipeline._decoder.run.return_value = [np.zeros((1, NUM_JOINTS), dtype=np.float32)]
    _policy_step(pipeline)
    assert pipeline.apply_pose_message(_smpl_pose_fields()) == {
        "frames": 10,
        "encode_mode": 2,
        "catchup": True,
    }

    encoder_run = mocker.patch.object(
        pipeline._encoder,
        "run",
        side_effect=[
            [np.ones((1, 64), dtype=np.float32)],
            [np.full((1, 64), 2.0, dtype=np.float32)],
        ],
    )
    decoded_tokens: list[np.ndarray[Any, Any]] = []

    def decode(_outputs: Any, feeds: dict[str, np.ndarray[Any, Any]]) -> list[np.ndarray[Any, Any]]:
        decoded_tokens.append(feeds[pipeline._decoder_input][0, :64].copy())
        return [np.zeros((1, NUM_JOINTS), dtype=np.float32)]

    decoder_run = mocker.patch.object(pipeline._decoder, "run", side_effect=decode)

    assert pipeline.begin_stream_transition(0.04) is True
    _policy_step(pipeline)
    assert pipeline.reference_transition_progress == 0.5
    _policy_step(pipeline)

    assert pipeline.reference_transition_active is False
    assert encoder_run.call_count == 2
    assert decoder_run.call_count == 2
    np.testing.assert_allclose(decoded_tokens[0], 0.5)
    np.testing.assert_allclose(decoded_tokens[1], 2.0)


def test_stream_transition_requires_a_previous_planner_token(
    pipeline: SonicPipeline,
) -> None:
    pipeline.apply_pose_message(_smpl_pose_fields())

    assert pipeline.begin_stream_transition(0.5) is False


@pytest.mark.parametrize("duration", [0.0, -0.1, float("inf"), float("nan")])
def test_stream_transition_rejects_invalid_duration(
    pipeline: SonicPipeline,
    duration: float,
) -> None:
    with pytest.raises(ValueError, match="positive and finite"):
        pipeline.begin_stream_transition(duration)


def test_planner_transition_blends_pose_token_to_each_live_planner_token(
    pipeline: SonicPipeline,
    mocker: Any,
) -> None:
    pipeline._needs_replan = False
    pipeline._decoder.run.return_value = [np.zeros((1, NUM_JOINTS), dtype=np.float32)]
    _policy_step(pipeline)
    pipeline.apply_pose_message(_smpl_pose_fields())
    assert pipeline.begin_stream_transition(0.02) is True
    encoder_run = mocker.patch.object(
        pipeline._encoder,
        "run",
        return_value=[np.ones((1, 64), dtype=np.float32)],
    )
    _policy_step(pipeline)
    assert pipeline.reference_transition_active is False
    assert pipeline.prepare_planner_transition() is True
    assert pipeline.snapshot()["stream_active"] is True
    assert pipeline.snapshot()["planner_transition_preparing"] is True
    planner_qpos = np.zeros((2, 36), dtype=np.float32)
    planner_qpos[:, 3] = 1.0
    pipeline._apply_planner_result([planner_qpos, np.array(2, dtype=np.int64)])
    assert pipeline.planner_transition_ready is True

    planner_tokens = iter(
        [
            np.full((1, 64), 2.0, dtype=np.float32),
            np.full((1, 64), 3.0, dtype=np.float32),
        ]
    )
    encoder_run.side_effect = lambda *_args, **_kwargs: [next(planner_tokens)]
    decoded_tokens: list[np.ndarray[Any, Any]] = []

    def decode(_outputs: Any, feeds: dict[str, np.ndarray[Any, Any]]) -> list[np.ndarray[Any, Any]]:
        decoded_tokens.append(feeds[pipeline._decoder_input][0, :64].copy())
        return [np.zeros((1, NUM_JOINTS), dtype=np.float32)]

    mocker.patch.object(pipeline._decoder, "run", side_effect=decode)

    assert pipeline.begin_planner_transition(0.04) is True
    _policy_step(pipeline)
    assert pipeline.reference_transition_progress == 0.5
    _policy_step(pipeline)

    assert pipeline.reference_transition_active is False
    assert pipeline.snapshot()["stream_active"] is False
    assert pipeline.snapshot()["stream_frames"] == 0
    np.testing.assert_allclose(decoded_tokens[0], 1.5)
    np.testing.assert_allclose(decoded_tokens[1], 3.0)


def test_planner_transition_requires_a_previous_stream_token(pipeline: SonicPipeline) -> None:
    assert pipeline.prepare_planner_transition() is False
    assert pipeline.begin_planner_transition(0.5) is False


def test_planner_prepare_uses_measured_joint_context(pipeline: SonicPipeline) -> None:
    pipeline._cur_q_dds = np.arange(NUM_JOINTS, dtype=np.float32)
    pipeline._last_reference_token = np.zeros(64, dtype=np.float32)
    pipeline._last_token_was_stream = True
    pipeline._use_stream = True

    assert pipeline.prepare_planner_transition() is True
    context = pipeline._build_planner_context()

    expected_q = np.broadcast_to(pipeline._cur_q_dds[DDS_TO_ONNX], (context.shape[0], 29))
    np.testing.assert_array_equal(context[:, 7:36], expected_q)


@pytest.mark.parametrize("duration", [0.0, -0.1, float("inf"), float("nan")])
def test_planner_transition_rejects_invalid_duration(
    pipeline: SonicPipeline,
    duration: float,
) -> None:
    with pytest.raises(ValueError, match="positive and finite"):
        pipeline.begin_planner_transition(duration)


def test_stop_clip_cancels_reference_transition(pipeline: SonicPipeline) -> None:
    pipeline._needs_replan = False
    pipeline._decoder.run.return_value = [np.zeros((1, NUM_JOINTS), dtype=np.float32)]
    _policy_step(pipeline)
    pipeline.apply_pose_message(_smpl_pose_fields())
    assert pipeline.begin_stream_transition(0.5) is True

    pipeline.stop_clip()

    assert pipeline.reference_transition_active is False
    assert pipeline.reference_transition_progress == 0.0
    assert pipeline.snapshot()["stream_active"] is False
