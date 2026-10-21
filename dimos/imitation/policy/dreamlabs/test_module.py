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
from collections.abc import Iterator
import subprocess
import sys
from typing import Any

import cv2
import dream
from dream.transport import Transport
import numpy as np
import pytest
from pytest_mock import MockerFixture

from dimos.imitation.policy.dreamlabs.module import DreamlabsModule
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.protocol.rpc.pubsubrpc import LCMRPC
from dimos.robot.manipulators.dual_openyam.joints import DUAL_OPENYAM_JOINTS

LEFT = [0.0] * 6
RIGHT = [0.0, 0.5, 0.5, -0.35, 0.0, 0.0]
LEFT_ACTION = [0.01, 0.51, 0.52, -0.34, 0.02, 0.03]
RIGHT_ACTION = [-0.2, 0.8, 0.9, 0.1, 0.3, 0.4]


@pytest.fixture
def module(mocker: MockerFixture) -> Iterator[tuple[DreamlabsModule, Any]]:
    mocker.patch("dimos.core.module.get_loop", return_value=(mocker.MagicMock(), None))
    for method in ("__init__", "start", "stop", "serve_module_rpc"):
        mocker.patch.object(LCMRPC, method, return_value=None)
    request = mocker.patch.object(
        Transport,
        "request",
        return_value={
            "status": {"code": "DA-0000"},
            "actions": {
                "q_l_chunk": [[*LEFT_ACTION, 0.25]] * 48,
                "q_r_chunk": [[*RIGHT_ACTION, 0.75]] * 48,
                "per_key": {
                    "action.left_arm": [LEFT_ACTION] * 48,
                    "action.right_arm": [RIGHT_ACTION] * 48,
                },
            },
        },
    )
    mocker.patch(
        "dimos.imitation.policy.dreamlabs.module.create_client",
        side_effect=lambda: dream.DreamClient(api_key="offline-fixture"),
    )
    instance = DreamlabsModule(rpc_transport=LCMRPC)
    try:
        yield instance, request
    finally:
        instance.stop()


def tick(module: DreamlabsModule) -> None:
    overhead = Image(
        data=np.full((240, 400, 3), [210, 0, 0], dtype=np.uint8), format=ImageFormat.RGB
    )
    left = Image(data=np.full((240, 400, 3), [0, 220, 0], dtype=np.uint8), format=ImageFormat.RGB)
    right = Image(data=np.full((240, 400, 3), [0, 0, 230], dtype=np.uint8), format=ImageFormat.RGB)
    state = JointState(
        name=list(reversed(DUAL_OPENYAM_JOINTS)), position=list(reversed([*LEFT, *RIGHT, 0.2, 0.8]))
    )
    module.set_observation(overhead, left, right, state, 10.0)


def test_real_colored_images_and_state_reach_client_without_execution(
    module: tuple[DreamlabsModule, Any], mocker: MockerFixture
) -> None:
    instance, request = module
    encode = mocker.spy(cv2, "imencode")
    instance.start()
    request.assert_not_called()
    tick(instance)
    request.assert_not_called()
    preview = instance.get_action()
    request.assert_called_once()
    raw = instance.get_last_action_response()
    np.testing.assert_allclose(raw["q_l_chunk"][0], [*LEFT_ACTION, 0.25])
    raw["q_l_chunk"][0][-1] = 99.0
    assert instance.get_last_action_response()["q_l_chunk"][0][-1] == pytest.approx(0.25)
    sent = request.call_args.args[0]["observation"]
    for role, expected in (
        ("overhead", [0, 0, 210]),
        ("wrist_l", [0, 220, 0]),
        ("wrist_r", [230, 0, 0]),
    ):
        encoded = sent[f"video.{role}"][0]
        assert not encoded.startswith("data:")
        pixels = cv2.imdecode(
            np.frombuffer(base64.b64decode(encoded), dtype=np.uint8), cv2.IMREAD_COLOR
        )
        assert pixels.shape == (176, 320, 3)
        np.testing.assert_allclose(pixels[70, 120], expected, atol=2)
    assert encode.call_count == 3
    assert all(call.args[2] == [cv2.IMWRITE_JPEG_QUALITY, 85] for call in encode.call_args_list)
    np.testing.assert_allclose(sent["state.left_arm"], LEFT)
    np.testing.assert_allclose(sent["state.right_arm"], RIGHT)
    np.testing.assert_allclose(sent["state.left_gripper"], [0.2])
    np.testing.assert_allclose(sent["state.right_gripper"], [0.8])
    np.testing.assert_allclose(sent["state.left_eef_xyz"], [0.244997, 0, 0.164002], atol=2e-5)
    np.testing.assert_allclose(sent["state.right_eef_xyz"], [0.262523, 0, 0.206904], atol=2e-5)
    np.testing.assert_allclose(
        sent["state.right_eef_rot6"], [0, 1, 0, 0.342891, 0, 0.939375], atol=2e-5
    )
    np.testing.assert_allclose(preview.points[0].positions, [*LEFT, *RIGHT, 0.2, 0.8])
    assert preview.joint_names == DUAL_OPENYAM_JOINTS
    np.testing.assert_allclose(
        preview.points[1].positions, [*LEFT_ACTION, *RIGHT_ACTION, 0.25, 0.75]
    )
    assert preview.points[1].time_from_start == 0.05
    assert preview.duration == 2.4


def test_explicit_input_and_normal_client_cleanup(
    module: tuple[DreamlabsModule, Any], mocker: MockerFixture
) -> None:
    instance, _ = module
    client = mocker.MagicMock()
    factory = mocker.patch(
        "dimos.imitation.policy.dreamlabs.module.create_client", return_value=client
    )
    instance.start()
    factory.assert_not_called()
    with pytest.raises(RuntimeError, match="set_observation"):
        instance.get_action()
    factory.assert_not_called()
    tick(instance)
    factory.assert_called_once()
    assert client.policy.call_args.args[0]["_record"] is False
    client.get_action.assert_not_called()
    instance.reset_episode()
    client.reset_episode.assert_called_once()
    with pytest.raises(RuntimeError, match="set_observation"):
        instance.get_action()
    instance.stop()
    instance.stop()
    client.close.assert_called_once()


def test_offline_entrypoint_needs_no_sdk_or_executor() -> None:
    code = """
import sys
sys.modules['dream'] = None
sys.modules['dream_ik'] = None
from dimos.imitation.policy.dreamlabs.module import DreamlabsModule
from dimos.robot.manipulators.dual_openyam.blueprints.dreamlabs import dreamlabs_dual_openyam
from dimos.imitation.policy.dreamlabs.probe import main
[policy] = dreamlabs_dual_openyam.blueprints
assert policy.module is DreamlabsModule
sys.argv = ['probe']
main()
"""
    result = subprocess.run(
        [sys.executable, "-c", code], capture_output=True, text=True, timeout=30
    )
    assert result.returncode == 0, result.stderr
    assert "No API or hardware used" in result.stdout


def test_raw_rejected_response_survives_episode_reset(
    module: tuple[DreamlabsModule, Any],
    mocker: MockerFixture,
) -> None:
    instance, _ = module
    client = mocker.MagicMock()
    client.get_action.return_value = {"q_l_chunk": [[0.0] * 7] * 47, "q_r_chunk": [[0.0] * 7] * 48}
    mocker.patch("dimos.imitation.policy.dreamlabs.module.create_client", return_value=client)
    tick(instance)
    with pytest.raises(ValueError, match="48 by 7"):
        instance.get_action()
    instance.reset_episode()
    raw = instance.get_last_action_response()
    assert len(raw["q_l_chunk"]) == 47
    assert len(raw["q_r_chunk"]) == 48
