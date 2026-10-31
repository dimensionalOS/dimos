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


"""Pinned real-client tests with transport and credential boundaries replaced."""

import base64

import cv2
import dream
from dream import credentials
from dream.transport import Transport
import numpy as np
import pytest

from dimos.imitation.policy.dreamlabs.conversion import encode_image, policy_config
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat


@pytest.fixture
def client(mocker, tmp_path):
    # Explicitly replace the loader before constructing the real client. Neither
    # the credentials file nor credential environment variables are read.
    mocker.patch.object(
        credentials,
        "load",
        return_value=credentials.Credentials(
            gateway_url="https://offline.invalid",
            platform_url="https://offline.invalid",
            data_url="https://offline.invalid",
            api_key="offline-fixture",
            path=tmp_path / "unused",
        ),
    )
    request = mocker.patch.object(Transport, "request")
    instance = dream.DreamClient()
    try:
        yield instance, request
    finally:
        instance.close()


@pytest.mark.parametrize("mode", ["joint", "eef"])
def test_pinned_sdk_preserves_capture_history_and_raw_eef_parts_without_ik(client, mode):
    instance, request = client
    instance.policy(policy_config("dreamlabs/dream-1-beta", 1500, mode))
    raw_parts = {}
    for side in ("left", "right"):
        raw_parts.update(
            {
                f"action.{side}_arm": [[0.1] * 6] * 48 if mode == "joint" else None,
                f"action.{side}_eef_xyz": [[0.3, 0.1, 0.2]] * 48,
                f"action.{side}_eef_rot6": [[1, 0, 0, 0, 1, 0]] * 48,
                f"action.{side}_grip": [[0.75]] * 48,
            }
        )
    actions = {"per_key": raw_parts}
    if mode == "joint":
        actions.update(q_l_chunk=[[0.1] * 6 + [0.75]] * 48, q_r_chunk=[[0.1] * 6 + [0.75]] * 48)
    request.return_value = {"status": {"code": "DA-0000"}, "actions": actions}
    for index in range(9):
        frame = encode_image(
            Image(
                data=np.full((32, 64, 3), [index * 20, 0, 0], dtype=np.uint8),
                format=ImageFormat.RGB,
            )
        )
        frames = {f"video.{role}": frame for role in ("overhead", "wrist_l", "wrist_r")}
        state = {"prompt": "move"}
        for side in ("left", "right"):
            state.update(
                {
                    f"state.{side}_arm": np.zeros(6),
                    f"state.{side}_gripper": [0.25],
                    f"state.{side}_eef_xyz": [0.1, 0.2, 0.3],
                    f"state.{side}_eef_rot6": [1, 0, 0, 0, 1, 0],
                    f"state.{side}_grip": [0.25],
                }
            )
        instance.set_observation(frames, state, timestamp=100.0 + index * 0.2)
    request.assert_not_called()
    response = instance.get_action()
    assert response["per_key"] == raw_parts
    sent = request.call_args.args[0]["observation"]
    assert sent["embodiment"] == ("yam" if mode == "joint" else "other_embodiment")
    assert len(sent["video.overhead"]) == 9
    first = cv2.imdecode(
        np.frombuffer(base64.b64decode(sent["video.overhead"][0]), dtype=np.uint8), cv2.IMREAD_COLOR
    )
    last = cv2.imdecode(
        np.frombuffer(base64.b64decode(sent["video.overhead"][-1]), dtype=np.uint8),
        cv2.IMREAD_COLOR,
    )
    np.testing.assert_allclose(first[80, 80], [0, 0, 0], atol=2)
    np.testing.assert_allclose(last[80, 80], [0, 0, 160], atol=2)
