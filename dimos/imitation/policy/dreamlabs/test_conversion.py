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

import cv2
import numpy as np
import pytest
from pytest_mock import MockerFixture

from dimos.imitation.policy.dreamlabs.conversion import (
    action_trajectory,
    encode_image,
    observation,
    state_vector,
)
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.robot.manipulators.dual_openyam.joints import DUAL_OPENYAM_JOINTS


def test_names_override_message_order_and_preserve_radians_and_gripper_units() -> None:
    state = JointState(
        name=list(reversed(DUAL_OPENYAM_JOINTS)), position=[0.8, 0.2, *range(12, 0, -1)]
    )
    vector = state_vector(state)
    assert vector.tolist() == [*range(1, 13), 0.2, 0.8]
    obs = observation(vector, "fold")
    assert obs["state.right_arm"].tolist() == [7, 8, 9, 10, 11, 12]
    assert obs["state.left_arm"].tolist() == [1, 2, 3, 4, 5, 6]
    np.testing.assert_allclose(obs["state.left_gripper"], [0.2])
    np.testing.assert_allclose(obs["state.right_gripper"], [0.8])


def test_jpeg_contract_and_rgb_channel_order(mocker: MockerFixture) -> None:
    encode = mocker.spy(cv2, "imencode")
    rgb = np.zeros((30, 40, 3), dtype=np.uint8)
    rgb[:, :, 0] = 255
    encoded = encode_image(Image(data=rgb, format=ImageFormat.RGB))
    decoded = cv2.imdecode(
        np.frombuffer(base64.b64decode(encoded), dtype=np.uint8), cv2.IMREAD_COLOR
    )
    assert decoded.shape == (176, 320, 3)
    np.testing.assert_allclose(decoded[80, 100], [0, 0, 254], atol=1)
    assert not encoded.startswith("data:")
    encode.assert_called_once()
    assert encode.call_args.args[2] == [cv2.IMWRITE_JPEG_QUALITY, 85]


def test_action_fixture_order_grippers_timing_and_linear_seam() -> None:
    reply = {"q_l_chunk": [[0.01] * 6 + [0.52]] * 48, "q_r_chunk": [[-0.02] * 6 + [0.48]] * 48}
    start = np.array([0.0] * 12 + [0.5, 0.5])
    trajectory = action_trajectory(reply, start)
    assert trajectory.joint_names == DUAL_OPENYAM_JOINTS
    assert trajectory.points[0].positions == [0.0] * 12 + [0.5, 0.5]
    assert trajectory.points[1].positions == [0.01] * 6 + [-0.02] * 6 + [0.52, 0.48]
    assert trajectory.points[1].time_from_start == 0.05
    assert trajectory.duration == 2.4
    np.testing.assert_allclose(
        trajectory.sample(0.025)[0], [0.005] * 6 + [-0.01] * 6 + [0.51, 0.49]
    )


@pytest.mark.parametrize("kind", ["missing", "shape", "nan", "gripper"])
def test_malformed_or_unbounded_actions_rejected(kind: str) -> None:
    left = np.array([[0.0] * 6 + [0.5]] * 48)
    right = left.copy()
    reply = {"q_l_chunk": left, "q_r_chunk": right}
    if kind == "missing":
        del reply["q_l_chunk"]
    elif kind == "shape":
        reply["q_l_chunk"] = left[:47]
    elif kind == "nan":
        left[10, 2] = np.nan
    elif kind == "gripper":
        right[10, 6] = 1.1
    with pytest.raises(ValueError):
        action_trajectory(
            reply,
            np.array([0.0] * 12 + [0.5, 0.5]),
        )


@pytest.mark.parametrize("positions", [[0.0] * 13, [0.0] * 12 + [-0.1, 0.0], [float("nan")] * 14])
def test_bad_state_rejected(positions: list[float]) -> None:
    with pytest.raises(ValueError):
        state_vector(JointState(name=list(DUAL_OPENYAM_JOINTS), position=positions))
