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


import base64
from concurrent.futures import ThreadPoolExecutor
from contextlib import closing
from threading import Event
from types import SimpleNamespace

import cv2
import numpy as np
import pytest

from dimos.imitation.policy.dreamlabs.backend import DreamlabsBackend
from dimos.imitation.policy.dreamlabs.config import DreamlabsBackendConfig
from dimos.robot.manipulators.dual_openyam.joints import DUAL_OPENYAM_JOINTS

STATE = np.array([0.25, 0.2], dtype=np.float32)


def test_observation_encodes_each_rgb_once_without_request_or_recording(backend, images, mocker):
    instance, client = backend
    encode = mocker.spy(cv2, "imencode")
    instance.observe(images, STATE, task="move", timestamp=1000.0)
    client.get_action.assert_not_called()
    config = client.policy.call_args.args[0]
    assert config["_record"] is False
    assert config["robot"] == "other_embodiment"
    frames, state = client.set_observation.call_args.args
    for role, expected in (
        ("overhead", [0, 0, 210]),
        ("wrist_l", [0, 220, 0]),
        ("wrist_r", [230, 0, 0]),
    ):
        pixels = cv2.imdecode(
            np.frombuffer(base64.b64decode(frames[f"video.{role}"]), dtype=np.uint8),
            cv2.IMREAD_COLOR,
        )
        assert pixels.shape == (176, 320, 3)
        np.testing.assert_allclose(pixels[80, 80], expected, atol=2)
    assert encode.call_count == 3
    np.testing.assert_allclose(state["state.left_arm"], [0.0] * 6)
    np.testing.assert_allclose(state["state.left_eef_xyz"], [0.3, 0.0, 0.0], atol=1e-7)
    np.testing.assert_allclose(state["state.left_eef_rot6"], [1, 0, 0, 0, 1, 0])
    np.testing.assert_allclose(state["state.left_grip"], [0.25])


def test_eef_reply_becomes_model_root_trajectory_with_tool_inverse_and_anchor(backend, images):
    instance, client = backend
    instance.observe(images, STATE, task="move", timestamp=1000.0)
    trajectory = instance.predict(images, STATE, task="move")
    assert len(trajectory.points) == 49
    assert trajectory.points[0].time_from_start == 0.0
    assert trajectory.points[1].time_from_start == 0.05
    assert trajectory.points[-1].time_from_start == 2.4
    anchor = trajectory.points[0].poses["tool"]
    target = trajectory.points[1].poses["tool"]
    assert target.frame_id == "robot_root"
    np.testing.assert_allclose([anchor.x, anchor.y, anchor.z], [1.2, 2, 3], atol=1e-7)
    np.testing.assert_allclose([target.x, target.y, target.z], [1.3, 2.1, 3.2], atol=1e-7)
    assert trajectory.points[0].grippers == {"grip": 0.25}
    assert trajectory.points[1].grippers == {"grip": 0.75}
    assert client.get_action.call_count == 1


@pytest.mark.parametrize(
    "key, value",
    [
        ("action.left_eef_xyz", [[0.0, 0.0, float("nan")]] * 48),
        ("action.left_eef_rot6", [[1, 0, 0, 2, 0, 0]] * 48),
        ("action.left_eef_rot6", [[0.0] * 6] * 48),
        ("action.left_grip", [[float("inf")]] * 48),
        ("action.left_eef_xyz", [[0, 0, 0]] * 47),
    ],
)
def test_invalid_eef_reply_rejected_and_retained_for_diagnostics(backend, images, key, value):
    instance, client = backend
    instance.observe(images, STATE, task="move", timestamp=1000.0)
    client.get_action.return_value["per_key"][key] = value
    with pytest.raises(ValueError):
        instance.predict(images, STATE, task="move")
    instance.reset()
    raw = instance.get_last_action_response()
    assert raw["per_key"][key] == value
    raw["per_key"]["action.left_arm"] = "changed"
    assert instance.get_last_action_response()["per_key"]["action.left_arm"] is None


def test_finite_eef_gripper_clipped_but_raw_response_preserved(backend, images):
    instance, client = backend
    client.get_action.return_value["per_key"]["action.left_grip"] = [[1.02]] * 48
    instance.observe(images, STATE, task="move", timestamp=1000.0)
    trajectory = instance.predict(images, STATE, task="move")
    assert trajectory.points[-1].grippers == {"grip": 1.0}
    assert instance.get_last_action_response()["per_key"]["action.left_grip"][0] == [1.02]


def test_observation_continues_during_inference_and_fresh_feedback_anchors_result(
    backend, images, mocker
):
    instance, client = backend
    instance.observe(images, STATE, task="move", timestamp=1000.0)
    entered, release = Event(), Event()
    response = client.get_action.return_value

    def infer():
        entered.set()
        assert release.wait(5)
        return response

    mocker.patch.object(client, "get_action", side_effect=infer)
    with ThreadPoolExecutor(max_workers=1) as pool:
        result = pool.submit(instance.predict, images, STATE, task="move")
        try:
            assert entered.wait(5)
            instance.observe(
                images, np.array([0.5, 0.3], dtype=np.float32), task="move", timestamp=1000.2
            )
        finally:
            release.set()
        trajectory = result.result(timeout=5)
    assert trajectory.points[0].grippers == {"grip": 0.5}
    assert trajectory.points[0].poses["tool"].x == pytest.approx(1.3)


def test_reset_requires_fresh_observation_and_allows_new_prompt_and_timestamp(backend, images):
    instance, client = backend
    with pytest.raises(RuntimeError, match="Fresh observations"):
        instance.predict(images, STATE, task="move")
    instance.observe(images, STATE, task="move", timestamp=1000.0)
    with pytest.raises(RuntimeError, match="Reset"):
        instance.observe(images, STATE, task="other", timestamp=1000.2)
    with pytest.raises(ValueError, match="newer"):
        instance.observe(images, STATE, task="move", timestamp=1000.0)
    instance.reset()
    client.reset_episode.assert_called_once()
    instance.observe(images, STATE, task="other", timestamp=900.0)
    assert client.set_observation.call_args.args[1]["prompt"] == "other"


@pytest.mark.parametrize("state", [np.array([1.1, 0.2]), np.array([0.5]), np.array([0.5, np.nan])])
def test_invalid_state_never_initializes_sdk(backend, images, state):
    instance, client = backend
    with pytest.raises(ValueError):
        instance.observe(images, state, task="move", timestamp=1000.0)
    client.policy.assert_not_called()


def test_joint_mode_rejects_other_robot_even_with_similar_width(policy_config_fixture):
    policy_config_fixture.dreamlabs = DreamlabsBackendConfig()
    with pytest.raises(ValueError, match="configured arm|dual YAM"):
        DreamlabsBackend(policy_config_fixture)


@pytest.mark.self_hosted
def test_joint_backend_reads_and_returns_hardware_order_independent_of_policy_order(images, mocker):
    left = [0.1, 0.5, 0.3, -0.4, 0.1, 0.0]
    right = [-0.2, 0.7, 0.6, 0.1, 0.2, 0.3]
    left_target = [0.11, 0.51, 0.31, -0.39, 0.11, 0.01]
    right_target = [-0.19, 0.71, 0.61, 0.11, 0.21, 0.31]
    client = mocker.MagicMock()
    client.get_action.return_value = {
        "q_l_chunk": [[*left_target, 0.3]] * 48,
        "q_r_chunk": [[*right_target, 0.7]] * 48,
    }
    mocker.patch("dimos.imitation.policy.dreamlabs.backend.create_client", return_value=client)
    config = SimpleNamespace(
        fps=None,
        execution_steps=None,
        dreamlabs=DreamlabsBackendConfig(),
        policy_path="dreamlabs/dream-1-beta",
        robot_type="dual_openyam",
        joint_names=list(reversed(DUAL_OPENYAM_JOINTS)),
        policy_joint_names=list(DUAL_OPENYAM_JOINTS),
    )
    state = np.array(list(reversed([*left, *right, 0.25, 0.75])), dtype=np.float32)
    with closing(DreamlabsBackend(config)) as instance:
        instance.observe(images, state, task="move", timestamp=1000.0)
        observed = client.set_observation.call_args.args[1]
        np.testing.assert_allclose(observed["state.left_arm"], left)
        np.testing.assert_allclose(observed["state.right_arm"], right)
        np.testing.assert_allclose(observed["state.left_gripper"], [0.25])
        np.testing.assert_allclose(observed["state.right_gripper"], [0.75])
        result = instance.predict(images, state, task="move")
        np.testing.assert_allclose(
            result, np.tile(list(reversed([*left_target, *right_target, 0.3, 0.7])), (48, 1))
        )


@pytest.mark.parametrize(
    "field,value,error", [("fps", 30, "fps=20"), ("execution_steps", 24, "execution_steps=48")]
)
def test_unsupported_chunk_timing_rejected_before_sdk_creation(
    policy_config_fixture, field, value, error
):
    setattr(policy_config_fixture, field, value)
    with pytest.raises(ValueError, match=error):
        DreamlabsBackend(policy_config_fixture)


def test_eef_reply_accepts_sdk_flat_gripper_shape(backend, images):
    instance, client = backend
    client.get_action.return_value["per_key"]["action.left_grip"] = [0.75] * 48
    instance.observe(images, STATE, task="move", timestamp=1000.0)
    trajectory = instance.predict(images, STATE, task="move")
    assert trajectory.points[-1].grippers == {"grip": 0.75}
    assert instance.get_last_action_response()["per_key"]["action.left_grip"] == [0.75] * 48
