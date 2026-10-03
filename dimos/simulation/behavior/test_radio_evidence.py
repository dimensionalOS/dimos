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

"""Hermetic frame evidence, projection and explicit summary-claim reconciliation."""

import copy
import json

import numpy as np
import pytest

from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.simulation.behavior.radio_evidence import (
    make_sensor_snapshot,
    persist_grounding,
    persist_observation,
    project_base_point,
    reconcile_grounding_claim,
)
from dimos.simulation.behavior.radio_policy import (
    RadioPolicySupervisor,
    camera_observation,
    ground_pixel,
)
from dimos.simulation.behavior.test_radio_policy import (
    raw_camera,  # noqa: F401 - shared hermetic fixture
)


@pytest.fixture
def atomic_capture(raw_camera):
    messages = copy.deepcopy(raw_camera)
    for source, target in (
        ("left_wrist_image", "color_image"),
        ("left_wrist_depth", "depth_image"),
        ("left_wrist_camera_info", "camera_info"),
    ):
        messages[target] = copy.deepcopy(messages[source])
        messages[target].frame_id = "camera_optical"
    messages["tf"].transforms.append(
        Transform(frame_id="base_link", child_frame_id="camera_optical", ts=123)
    )
    return make_sensor_snapshot(messages, "episode-7", 42)


def test_atomic_frame_filters_truth_and_tags_exact_capture(atomic_capture):
    observation = camera_observation(atomic_capture, "left_wrist")
    assert observation["capture"] == {
        "capture_id": atomic_capture["capture_id"],
        "episode": "episode-7",
        "step": 42,
        "captured_at": 123,
    }
    assert "objects" not in atomic_capture["sensors"]
    assert "goal_status" not in atomic_capture["sensors"]
    assert {t.frame_id for t in atomic_capture["sensors"]["tf"].transforms} == {"base_link"}


@pytest.mark.parametrize("field", ["depth", "calibration", "camera_tf"])
def test_atomic_frame_rejects_different_step_timestamp_even_within_old_skew_tolerance(
    atomic_capture, field
):
    if field == "depth":
        atomic_capture["sensors"]["left_wrist_depth"].ts += 0.001
    elif field == "calibration":
        atomic_capture["sensors"]["left_wrist_camera_info"].ts += 0.001
    else:
        atomic_capture["sensors"]["tf"].transforms[0].ts += 0.001
    with pytest.raises(ValueError, match="same native capture"):
        camera_observation(atomic_capture, "left_wrist")


def test_backproject_then_independent_projection_matches_pixel(atomic_capture):
    observation = camera_observation(atomic_capture, "left_wrist")
    point = ground_pixel(observation, 3, 2)
    assert project_base_point(observation, point["position"]) == pytest.approx([3, 2])
    assert point["capture"]["step"] == 42
    assert point["fingerprint"] == observation["fingerprint"]


def test_bundle_retains_exact_allowed_data_and_matching_pixel_result(atomic_capture, tmp_path):
    observation = camera_observation(atomic_capture, "left_wrist")
    bundle = persist_observation(tmp_path / "observations", observation)
    assert persist_observation(tmp_path / "observations", observation) == bundle
    with np.load(bundle, allow_pickle=False) as saved:
        metadata = json.loads(saved["metadata"].tobytes())
        assert np.array_equal(saved["rgb"], observation["rgb"].data)
        assert np.array_equal(saved["depth"], observation["depth"].data)
    assert metadata["capture"]["step"] == 42
    assert metadata["intrinsics"]["K"] == [2, 0, 2, 0, 2, 2, 0, 0, 1]
    assert metadata["camera_to_base"]["translation"] == [1, 2, 3]
    point = ground_pixel(observation, 3, 2)
    record = json.loads(persist_grounding(tmp_path, observation, point).read_text())
    assert record["sensor_bundle"] == bundle.name
    assert record["projection_check_pixel"] == pytest.approx([3, 2])


@pytest.mark.parametrize("field", ["rgb", "depth", "calibration", "transform", "capture"])
def test_modified_snapshot_fails_before_grounding(atomic_capture, field):
    observation = camera_observation(atomic_capture, "left_wrist")
    if field == "rgb":
        observation["rgb"].data[0, 0, 0] = 1
    elif field == "depth":
        observation["depth"].data[0, 0] = 3
    elif field == "calibration":
        observation["calibration"].K[0] = 3
    elif field == "transform":
        observation["camera_to_base"].translation.x = 5
    else:
        observation["capture"]["step"] = 43
    with pytest.raises(ValueError, match="fingerprint"):
        ground_pixel(observation, 3, 2)


def test_changed_capture_id_payload_and_episode_are_rejected(atomic_capture, mocker, tmp_path):
    service = RadioPolicySupervisor(
        mocker.Mock(), mocker.Mock(), lambda: atomic_capture, Transform.identity, tmp_path
    )
    try:
        first = service.observe()
        # Mutating the returned client copy does not corrupt the retained owner frame.
        first["rgb"].data[0, 0, 0] = 99
        assert service.ground(first["id"], 3, 2)["position"] == pytest.approx([1, 3, 5])
        atomic_capture["sensors"]["left_wrist_image"].data[0, 0, 0] = 1
        with pytest.raises(ValueError, match="reused"):
            service.observe()
        atomic_capture["episode"] = "episode-8"
        with pytest.raises(ValueError, match="Episode changed"):
            service.ground(first["id"], 3, 2)
    finally:
        service.close()


def test_stale_atomic_capture_rejected(atomic_capture, monkeypatch):
    monkeypatch.setattr("dimos.simulation.behavior.radio_policy.time.time", lambda: 125)
    with pytest.raises(ValueError, match="stale"):
        camera_observation(atomic_capture, "left_wrist")


def test_mismatched_pixel_result_is_never_persisted_as_matching_snapshot(atomic_capture, tmp_path):
    observation = camera_observation(atomic_capture, "left_wrist")
    point = ground_pixel(observation, 3, 2)
    point["observation_id"] = "other-frame"
    with pytest.raises(ValueError, match="does not match"):
        persist_grounding(tmp_path, observation, point)
    assert list(tmp_path.glob("grounding-*.json")) == []


@pytest.mark.parametrize(
    "claim,error,contradiction",
    [(False, False, True), (True, False, False), (True, True, True), (False, True, False)],
)
def test_explicit_model_claim_reconciles_with_successful_sdk_tool_feedback(
    claim, error, contradiction
):
    feedback = {"observation_id": "frame-7", "frame": "base_link", "position": [1, 2, 3]}
    events = [
        {
            "type": "tool_execution_start",
            "toolName": "bash",
            "toolCallId": "call-7",
            "args": {"command": "timeout 12s env PYTHONPATH=/snapshot /venv/bin/python ground.py"},
        },
        {
            "type": "tool_execution_end",
            "toolName": "bash",
            "toolCallId": "call-7",
            "isError": error,
            "result": {"content": [{"type": "text", "text": json.dumps(feedback)}]},
        },
    ]
    result = reconcile_grounding_claim(events, claim)
    assert result["contradiction"] is contradiction
    assert result["recorded_grounding_completed"] is not error


@pytest.mark.parametrize("field", ["pixel", "depth", "capture", "position"])
def test_grounding_feedback_must_match_pixel_and_retained_depth(atomic_capture, tmp_path, field):
    observation = camera_observation(atomic_capture, "left_wrist")
    point = ground_pixel(observation, 3, 2)
    if field == "pixel":
        point["pixel"] = [2, 2]
    elif field == "depth":
        point["depth"] = 9
    elif field == "capture":
        point["capture"]["step"] = 43
    else:
        point["position"][2] += 0.01
    with pytest.raises(ValueError, match="does not match"):
        persist_grounding(tmp_path, observation, point)
    assert list(tmp_path.glob("grounding-*.json")) == []


def test_corrupted_saved_bundle_is_not_silently_reused(atomic_capture, tmp_path):
    observation = camera_observation(atomic_capture, "left_wrist")
    bundle = persist_observation(tmp_path, observation)
    with np.load(bundle, allow_pickle=False) as saved:
        rgb, depth, metadata = saved["rgb"].copy(), saved["depth"].copy(), saved["metadata"].copy()
    rgb[0, 0, 0] = 99
    np.savez_compressed(bundle, rgb=rgb, depth=depth, metadata=metadata)
    with pytest.raises(ValueError, match="conflicts"):
        persist_observation(tmp_path, observation)


def test_nonzero_distortion_is_not_silently_used_as_pinhole(atomic_capture):
    atomic_capture["sensors"]["left_wrist_camera_info"].D = [0.01, 0, 0, 0, 0]
    with pytest.raises(ValueError, match="undistorted"):
        camera_observation(atomic_capture, "left_wrist")


def test_saved_depth_dtype_cannot_disagree_with_immutable_metadata(atomic_capture, tmp_path):
    observation = camera_observation(atomic_capture, "left_wrist")
    bundle = persist_observation(tmp_path, observation)
    with np.load(bundle, allow_pickle=False) as saved:
        rgb, depth, metadata = (
            saved["rgb"].copy(),
            saved["depth"].astype(np.float64),
            saved["metadata"].copy(),
        )
    np.savez_compressed(bundle, rgb=rgb, depth=depth, metadata=metadata)
    with pytest.raises(ValueError, match="conflicts"):
        persist_observation(tmp_path, observation)
