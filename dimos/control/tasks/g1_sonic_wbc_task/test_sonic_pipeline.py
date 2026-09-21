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
from concurrent.futures import Future
from types import SimpleNamespace
from typing import Any

import numpy as np
import onnxruntime as ort  # type: ignore[import-untyped]
import pytest

from dimos.control.tasks.g1_sonic_wbc_task.sonic_pipeline import (
    DEFAULT_ANGLES_DDS,
    DEFAULT_HEIGHT,
    NUM_JOINTS,
    SMPL_JOINTS_OFFSET,
    WRIST_ONNX_INDICES,
    WRISTS_OFFSET,
    SonicPipeline,
)
from dimos.control.tasks.g1_sonic_wbc_task.sonic_safety import SonicSafetyError


@pytest.fixture
def velocity_pipeline(mocker: Any) -> Iterator[SonicPipeline]:
    encoder = mocker.MagicMock()
    encoder.get_inputs.return_value = [SimpleNamespace(name="encoder", shape=[1, 1751])]
    encoder.get_providers.return_value = ["CUDAExecutionProvider"]
    encoder.run.return_value = [np.zeros((1, 64), dtype=np.float32)]
    decoder = mocker.MagicMock()
    decoder.get_inputs.return_value = [SimpleNamespace(name="decoder", shape=[1, 994])]
    decoder.get_providers.return_value = ["CUDAExecutionProvider"]
    decoder.run.return_value = [np.zeros((1, NUM_JOINTS), dtype=np.float32)]
    planner = mocker.MagicMock()
    planner.get_providers.return_value = ["CUDAExecutionProvider", "CPUExecutionProvider"]
    qpos = np.tile(
        np.concatenate(([0.0, 0.0, DEFAULT_HEIGHT, 1.0, 0.0, 0.0, 0.0], DEFAULT_ANGLES_DDS)),
        (1, 44, 1),
    ).astype(np.float32)
    planner.run.return_value = [qpos, np.array([44])]
    mocker.patch.object(ort, "InferenceSession", side_effect=[encoder, decoder, planner])
    mocker.patch.object(ort, "preload_dlls")
    mocker.patch.object(
        ort,
        "get_available_providers",
        return_value=["CUDAExecutionProvider", "CPUExecutionProvider"],
    )
    pipeline = SonicPipeline("encoder.onnx", "decoder.onnx", "planner.onnx")
    try:
        yield pipeline
    finally:
        pipeline.close()


@pytest.fixture
def planner_requests(velocity_pipeline: SonicPipeline, mocker: Any) -> Any:
    completed: Future[list[Any]] = Future()
    completed.set_result(velocity_pipeline._planner.run.return_value)
    return mocker.patch.object(
        velocity_pipeline._planner_executor, "submit", return_value=completed
    )


def _step(pipeline: SonicPipeline, *, measured_yaw: float = 0.0) -> None:
    pipeline.step(
        q_dds=DEFAULT_ANGLES_DDS,
        dq_dds=np.zeros(NUM_JOINTS, dtype=np.float32),
        base_quat_wxyz=np.array(
            [np.cos(measured_yaw / 2), 0.0, 0.0, np.sin(measured_yaw / 2)], dtype=np.float64
        ),
        gyro_body=np.zeros(3, dtype=np.float32),
        gravity_body=np.array([0.0, 0.0, -1.0], dtype=np.float32),
    )


@pytest.mark.parametrize("mode", ["IDEL_SQUAT", "IDEL_KNEEL_TWO_LEGS", "IDEL_KNEEL"])
def test_static_floor_modes_request_zero_speed(velocity_pipeline, planner_requests, mode):
    velocity_pipeline.set_mode(mode)
    velocity_pipeline.set_velocity(0.3, 0.2, 0.0)
    for _ in range(120):
        _step(velocity_pipeline)

    inputs = planner_requests.call_args.args[2]
    assert inputs["target_vel"].item() == 0.0
    assert inputs["height"].item() == pytest.approx(0.4)
    np.testing.assert_array_equal(inputs["movement_direction"], [[0.0, 0.0, 0.0]])


@pytest.mark.parametrize(
    "mode, mode_id, height", [("CRAWLING", 8, 0.4), ("ELBOW_CRAWLING", 14, 0.3)]
)
def test_crawl_release_stops_replanning_without_leaving_posture(
    velocity_pipeline, planner_requests, mode, mode_id, height
):
    velocity_pipeline.set_mode(mode)
    velocity_pipeline.set_velocity(0.2, 0.0, 0.0)
    for _ in range(220):
        _step(velocity_pipeline)
    assert planner_requests.call_args.args[2]["target_vel"].item() == pytest.approx(0.7)

    velocity_pipeline.set_velocity(0.0, 0.0, 0.0)
    _step(velocity_pipeline)

    inputs = planner_requests.call_args.args[2]
    assert inputs["mode"].item() == mode_id
    assert inputs["target_vel"].item() == 0.0
    assert inputs["height"].item() == pytest.approx(height)
    np.testing.assert_array_equal(inputs["movement_direction"], [[0.0, 0.0, 0.0]])
    planner_requests.reset_mock()
    for _ in range(150):
        _step(velocity_pipeline)
    planner_requests.assert_not_called()

    velocity_pipeline.set_velocity(0.2, 0.0, 0.0)
    _step(velocity_pipeline)
    assert planner_requests.call_args.args[2]["mode"].item() == mode_id
    assert planner_requests.call_args.args[2]["target_vel"].item() == pytest.approx(0.7)


@pytest.mark.parametrize("mode", ["IDEL_LYING_FACE_DOWN", "idel_lying_face_down", 7])
def test_unavailable_face_down_mode_preserves_current_request(velocity_pipeline, mode):
    velocity_pipeline.set_mode("CRAWLING")

    with pytest.raises(ValueError, match="face.down"):
        velocity_pipeline.set_mode(mode)

    assert velocity_pipeline.target_mode == 8


@pytest.mark.parametrize("vx, vy", [(0.04, 0.04), (0.06, 0.0), (0.0, 0.06)])
def test_crawl_deadzone_crossing_replans_even_for_small_velocity_changes(
    velocity_pipeline, planner_requests, vx, vy
):
    velocity_pipeline.set_mode("CRAWLING")
    velocity_pipeline.set_velocity(vx, vy, 0.0)
    for _ in range(120):
        _step(velocity_pipeline)
    planner_requests.reset_mock()

    velocity_pipeline.set_velocity(vx / 2, vy / 2, 0.0)
    _step(velocity_pipeline)

    assert planner_requests.call_count == 1
    assert planner_requests.call_args.args[2]["target_vel"].item() == 0.0
    planner_requests.reset_mock()
    velocity_pipeline.set_velocity(vx, vy, 0.0)
    _step(velocity_pipeline)
    assert planner_requests.call_count == 1
    assert planner_requests.call_args.args[2]["target_vel"].item() == pytest.approx(0.7)


@pytest.mark.parametrize(
    ("forward", "yaw_rate"),
    [(0.3, 0.0), (0.0, 0.3), (0.0, -0.3), (0.0, 0.06), (0.0, -0.06), (0.3, 0.3)],
)
def test_held_velocity_keeps_refreshing_planner(
    velocity_pipeline: SonicPipeline, planner_requests: Any, forward: float, yaw_rate: float
) -> None:
    for _ in range(250):
        velocity_pipeline.set_velocity(forward, 0.0, yaw_rate)
        _step(velocity_pipeline)

    assert planner_requests.call_count == (50 if yaw_rate else 5)
    assert velocity_pipeline.snapshot()["desired_heading"] == pytest.approx(yaw_rate * 5.0)
    headings = np.unwrap(
        [
            np.arctan2(
                call.args[2]["facing_direction"][0, 1], call.args[2]["facing_direction"][0, 0]
            )
            for call in planner_requests.call_args_list
        ]
    )
    assert headings[-1] == pytest.approx(yaw_rate * 5.0, abs=abs(yaw_rate) * 0.1 + 1e-6)


@pytest.mark.parametrize("yaw_rate", [0.06, -0.06])
def test_small_turn_starts_and_stops_without_waiting_for_periodic_replan(
    velocity_pipeline: SonicPipeline, planner_requests: Any, yaw_rate: float
) -> None:
    _step(velocity_pipeline)
    planner_requests.reset_mock()

    velocity_pipeline.set_velocity(0.0, 0.0, yaw_rate)
    _step(velocity_pipeline)

    assert planner_requests.call_count == 1
    np.testing.assert_allclose(
        planner_requests.call_args.args[2]["facing_direction"],
        [[np.cos(yaw_rate * 0.02), np.sin(yaw_rate * 0.02), 0.0]],
        atol=1e-7,
    )

    velocity_pipeline.set_velocity(0.0, 0.0, 0.0)
    _step(velocity_pipeline)

    assert planner_requests.call_count == 2
    np.testing.assert_allclose(
        planner_requests.call_args.args[2]["facing_direction"],
        [[np.cos(yaw_rate * 0.02), np.sin(yaw_rate * 0.02), 0.0]],
        atol=1e-7,
    )
    for _ in range(100):
        _step(velocity_pipeline)
    assert planner_requests.call_count == 2
    assert velocity_pipeline.snapshot()["desired_heading"] == pytest.approx(yaw_rate * 0.02)


def test_stop_received_during_inference_is_planned_when_worker_is_available(
    velocity_pipeline: SonicPipeline, planner_requests: Any
) -> None:
    pending: Future[list[Any]] = Future()
    completed = planner_requests.return_value
    planner_requests.side_effect = [pending, completed]
    velocity_pipeline.set_velocity(0.0, 0.0, 0.3)
    _step(velocity_pipeline)

    velocity_pipeline.set_velocity(0.0, 0.0, 0.0)
    _step(velocity_pipeline)
    assert planner_requests.call_count == 1

    pending.set_result(completed.result())
    _step(velocity_pipeline)

    assert planner_requests.call_count == 2
    np.testing.assert_allclose(
        planner_requests.call_args.args[2]["facing_direction"],
        [[np.cos(0.006), np.sin(0.006), 0.0]],
        atol=1e-7,
    )


@pytest.mark.parametrize("mode", ["SLOW_WALK", "WALK", "RUN"])
def test_centered_stick_idles_without_forgetting_selected_gait(
    velocity_pipeline: SonicPipeline, planner_requests: Any, mode: str
) -> None:
    selected = velocity_pipeline.set_mode(mode)
    velocity_pipeline.set_velocity(0.3, 0.0, 0.0)
    _step(velocity_pipeline)
    assert planner_requests.call_args.args[2]["mode"].item() == selected

    velocity_pipeline.set_velocity(0.0, 0.0, 0.0)
    _step(velocity_pipeline)
    assert planner_requests.call_args.args[2]["mode"].item() == 0
    np.testing.assert_array_equal(planner_requests.call_args.args[2]["movement_direction"], 0.0)
    assert velocity_pipeline.target_mode == selected
    assert velocity_pipeline.snapshot()["mode"] == 0

    velocity_pipeline.set_velocity(0.0, -0.3, 0.0)
    _step(velocity_pipeline)
    assert planner_requests.call_args.args[2]["mode"].item() == selected
    np.testing.assert_allclose(
        planner_requests.call_args.args[2]["movement_direction"], [[0.0, -1.0, 0.0]], atol=1e-7
    )


@pytest.mark.parametrize(("vx", "vy"), [(0.6, 0.0), (-0.6, 0.0), (0.0, 1.5), (0.0, -1.5)])
def test_default_slow_walk_caps_speed_in_each_direction(
    velocity_pipeline: SonicPipeline, planner_requests: Any, vx: float, vy: float
) -> None:
    velocity_pipeline.set_velocity(vx, vy, 0.0)
    _step(velocity_pipeline)
    inputs = planner_requests.call_args.args[2]
    assert inputs["mode"].item() == 1
    assert inputs["target_vel"].item() == pytest.approx(0.6)
    speed = np.hypot(vx, vy)
    np.testing.assert_allclose(inputs["movement_direction"], [[vx / speed, vy / speed, 0.0]])


@pytest.mark.parametrize(("method", "args"), [("set_mode", (None,)), ("reset", ())])
def test_reset_restores_slow_walk(velocity_pipeline, planner_requests, method, args):
    velocity_pipeline.set_mode("RUN")
    getattr(velocity_pipeline, method)(*args)
    velocity_pipeline.set_velocity(1.5, 0.0, 0.0)
    _step(velocity_pipeline)

    assert planner_requests.call_args.args[2]["mode"].item() == 1


def test_gradual_stick_changes_and_release_replan_before_periodic_timer(
    velocity_pipeline: SonicPipeline, planner_requests: Any
) -> None:
    velocity_pipeline.set_mode("SLOW_WALK")
    for speed in np.arange(0.1, 0.4, 0.01):
        velocity_pipeline.set_velocity(float(speed), 0.0, 0.0)
        _step(velocity_pipeline)
        planned_speed = planner_requests.call_args.args[2]["target_vel"].item()
        assert abs(planned_speed - speed) <= 0.051

    for speed in np.linspace(0.39, 0.0, 25):
        velocity_pipeline.set_velocity(float(speed), 0.0, 0.0)
        _step(velocity_pipeline)
    assert planner_requests.call_args.args[2]["mode"].item() == 0
    np.testing.assert_array_equal(planner_requests.call_args.args[2]["movement_direction"], 0.0)


def test_translation_follows_accumulated_heading_even_when_robot_lags(
    velocity_pipeline: SonicPipeline, planner_requests: Any
) -> None:
    velocity_pipeline.set_velocity(0.3, 0.0, 0.3)
    for _ in range(100):
        _step(velocity_pipeline)
    velocity_pipeline.set_velocity(0.0, 0.3, 0.0)
    _step(velocity_pipeline)

    inputs = planner_requests.call_args.args[2]
    np.testing.assert_allclose(
        inputs["facing_direction"], [[np.cos(0.6), np.sin(0.6), 0.0]], atol=1e-7
    )
    np.testing.assert_allclose(
        inputs["movement_direction"], [[-np.sin(0.6), np.cos(0.6), 0.0]], atol=1e-7
    )


@pytest.mark.parametrize("reset_source", [False, True])
def test_new_reference_reanchors_heading_and_pose_does_not_integrate_planner_yaw(
    velocity_pipeline: SonicPipeline, planner_requests: Any, reset_source: bool
) -> None:
    velocity_pipeline.set_velocity(0.0, 0.0, 0.3)
    for _ in range(50):
        _step(velocity_pipeline)
    if reset_source:
        velocity_pipeline.set_source_stream(True)
        for _ in range(50):
            _step(velocity_pipeline)
        assert velocity_pipeline.snapshot()["desired_heading"] is None
        velocity_pipeline.set_source_stream(False)
        velocity_pipeline.set_velocity(0.0, 0.0, 0.0)
    else:
        velocity_pipeline.reset()
    _step(velocity_pipeline, measured_yaw=1.2)
    assert velocity_pipeline.snapshot()["desired_heading"] == pytest.approx(1.2)


def test_initial_planner_context_keeps_measured_dds_joint_order(
    velocity_pipeline: SonicPipeline,
) -> None:
    velocity_pipeline._cur_q_dds = np.arange(NUM_JOINTS, dtype=np.float32)
    context = velocity_pipeline._build_planner_context()
    np.testing.assert_array_equal(context[:, 7:], np.tile(np.arange(NUM_JOINTS), (4, 1)))


def _constant_velocity_reference(start_time: float = 0.0) -> list[Any]:
    times = start_time + np.arange(44, dtype=np.float32) / 30.0
    qpos = np.zeros((1, 44, 36), dtype=np.float32)
    qpos[0, :, 0] = 0.3 * times
    qpos[0, :, 2] = DEFAULT_HEIGHT
    qpos[0, :, 3] = 1.0
    qpos[0, :, 7:] = DEFAULT_ANGLES_DDS + 0.2 * times[:, None]
    return [qpos, np.array([44])]


def test_planner_context_samples_uniform_thirty_hz_motion(
    velocity_pipeline: SonicPipeline,
) -> None:
    velocity_pipeline._apply_planner_result(_constant_velocity_reference())
    context = velocity_pipeline._build_planner_context()

    # Sampling a constant-speed motion must preserve its speed at 30 Hz.
    np.testing.assert_allclose(np.diff(context[:, 0]) * 30.0, 0.3, atol=1e-6)
    np.testing.assert_allclose(np.diff(context[:, 7:], axis=0) * 30.0, 0.2, atol=2e-6)


@pytest.mark.parametrize("inference_steps", [1, 3, 7])
def test_replanning_preserves_motion_time_and_velocity_during_inference(
    velocity_pipeline: SonicPipeline, planner_requests: Any, inference_steps: int
) -> None:
    velocity_pipeline._apply_planner_result(_constant_velocity_reference())
    velocity_pipeline._needs_replan = False
    for _ in range(10):
        _step(velocity_pipeline)

    pending: Future[list[Any]] = Future()
    planner_requests.return_value = pending
    velocity_pipeline.set_velocity(0.3, 0.0, 0.0)
    for _ in range(inference_steps):
        _step(velocity_pipeline)
    assert planner_requests.call_count == 1
    context = planner_requests.call_args.args[2]["context_mujoco_qpos"]
    generated_time = float(context[0, 0, 0]) / 0.3
    pending.set_result(_constant_velocity_reference(generated_time))
    _step(velocity_pipeline)

    trajectory = velocity_pipeline._trajectory
    assert trajectory is not None
    # Early and late results share the same timeline as uninterrupted playback.
    expected_times = (10 + inference_steps + np.arange(20)) * 0.02
    np.testing.assert_allclose(trajectory.root_pos[:20, 0], 0.3 * expected_times, atol=1e-6)
    np.testing.assert_allclose(trajectory.joint_vel[:20], 0.2, atol=1e-5)


def test_cpu_only_runtime_fails_before_loading_models(mocker):
    mocker.patch.object(ort, "get_available_providers", return_value=["CPUExecutionProvider"])
    load = mocker.patch.object(ort, "InferenceSession")

    with pytest.raises(RuntimeError, match="requires CUDAExecutionProvider"):
        SonicPipeline("encoder.onnx", "decoder.onnx", "planner.onnx")

    load.assert_not_called()


def test_nonfinite_decoder_output_raises_a_control_fault(velocity_pipeline):
    velocity_pipeline._decoder.run.return_value = [np.full((1, NUM_JOINTS), np.nan)]

    with pytest.raises(SonicSafetyError, match="non-finite decoder output"):
        _step(velocity_pipeline)


def test_invalid_robot_orientation_does_not_reuse_previous_targets(velocity_pipeline):
    with pytest.raises(SonicSafetyError, match="invalid robot policy input"):
        velocity_pipeline.step(
            DEFAULT_ANGLES_DDS, np.zeros(29), np.zeros(4), np.zeros(3), np.zeros(3)
        )


@pytest.mark.parametrize(
    "error, elapsed, message",
    [
        (RuntimeError("CUDA failure"), 0.1, "planner inference failed"),
        (None, 1.01, "planner inference timeout"),
    ],
)
def test_failed_or_stuck_planner_raises_a_control_fault(
    velocity_pipeline, mocker, error, elapsed, message
):
    future = Future()
    if error is not None:
        future.set_exception(error)
    velocity_pipeline._planner_future = future
    velocity_pipeline._planner_started_at = 10.0
    mocker.patch(
        "dimos.control.tasks.g1_sonic_wbc_task.sonic_pipeline.time.perf_counter",
        return_value=10.0 + elapsed,
    )

    with pytest.raises(SonicSafetyError, match=message):
        _step(velocity_pipeline)


def test_live_pose_window_encodes_latest_ten_frames_without_backlog(velocity_pipeline):
    for start in (0, 20, 22):
        frames = np.arange(start, start + 10)
        joints = np.tile(frames[:, None], (1, NUM_JOINTS)).astype(np.float32)
        smpl = np.broadcast_to(frames[:, None, None], (10, 24, 3)).astype(np.float32)
        velocity_pipeline.set_pose_window(
            {
                "frame_index": frames,
                "joint_pos": joints,
                "joint_vel": np.zeros_like(joints),
                "body_quat_w": np.tile([1.0, 0.0, 0.0, 0.0], (10, 1)),
                "smpl_joints": smpl,
                "smpl_pose": np.zeros((10, 21, 3), dtype=np.float32),
            }
        )

        observation = velocity_pipeline._build_streamed_encoder_obs(np.array([1.0, 0.0, 0.0, 0.0]))

        assert velocity_pipeline.snapshot()["stream_frames"] == 10
        np.testing.assert_array_equal(
            observation[SMPL_JOINTS_OFFSET : SMPL_JOINTS_OFFSET + 720].reshape(10, 24, 3), smpl
        )
        np.testing.assert_array_equal(
            observation[WRISTS_OFFSET : WRISTS_OFFSET + 60].reshape(10, 6),
            joints[:, WRIST_ONNX_INDICES],
        )
        velocity_pipeline._streamed_frame += 1
