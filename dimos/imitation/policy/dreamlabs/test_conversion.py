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


import numpy as np
import pytest

from dimos.imitation.policy.dreamlabs.conversion import joint_actions, policy_config
from dimos.robot.manipulators.dual_openyam.joints import DUAL_OPENYAM_JOINTS

VALID = [[0.0] * 6 + [0.5]] * 48
BOUNDS = {name: (-np.pi, np.pi) for name in DUAL_OPENYAM_JOINTS[:12]}


@pytest.mark.parametrize(
    "reply",
    [
        {"q_r_chunk": VALID},
        {"q_l_chunk": VALID[:47], "q_r_chunk": VALID},
        {"q_l_chunk": [[float("nan")] * 6 + [0.5]] * 48, "q_r_chunk": VALID},
        {"q_l_chunk": VALID, "q_r_chunk": [[0.0] * 6 + [float("inf")]] * 48},
    ],
)
def test_invalid_action_contract(reply):
    with pytest.raises(ValueError):
        joint_actions(reply, BOUNDS)


@pytest.mark.parametrize("opening, expected", [(-0.001, 0.0), (1.006, 1.0), (0.0, 0.0), (1.0, 1.0)])
def test_finite_grippers_clipped_without_changing_arms_or_raw_reply(opening, expected):
    left = [0.1, -0.2, 0.3, -0.4, 0.5, -0.6]
    right = [-0.7, 0.8, -0.9, 1.0, -1.1, 1.2]
    reply = {"q_l_chunk": [[*left, opening]] * 48, "q_r_chunk": [[*right, opening]] * 48}
    rows = joint_actions(reply, BOUNDS)
    np.testing.assert_allclose(rows, np.tile([*left, *right, expected, expected], (48, 1)))
    assert reply["q_l_chunk"][0][-1] == opening
    assert reply["q_r_chunk"][0][-1] == opening


@pytest.mark.parametrize("value, expected", [(-0.02, 0.0), (1.02, 1.0), (0.5, 0.5)])
def test_small_joint_overshoot_is_explicitly_clipped_and_raw_reply_untouched(value, expected):
    left = [0.1, value, 0.3, -0.4, 0.5, -0.6, 0.5]
    reply = {"q_l_chunk": [left] * 48, "q_r_chunk": VALID}
    rows = joint_actions(reply, {**BOUNDS, "left_joint2": (0.0, 1.0)})
    np.testing.assert_allclose(
        rows[0], [0.1, expected, 0.3, -0.4, 0.5, -0.6, *([0.0] * 6), 0.5, 0.5]
    )
    assert reply["q_l_chunk"][0][1] == value


@pytest.mark.parametrize("value", [-0.04, 1.04])
def test_joint_overshoot_over_two_degrees_is_rejected(value):
    with pytest.raises(ValueError, match="left_joint2.*more than 2 degrees"):
        joint_actions(
            {"q_l_chunk": [[0.0, value, 0.0, 0.0, 0.0, 0.0, 0.5]] * 48, "q_r_chunk": VALID},
            {**BOUNDS, "left_joint2": (0.0, 1.0)},
        )


def test_modes_disable_recording_and_never_request_sdk_ik():
    joint = policy_config("test/checkpoint", 1000, "joint")
    eef = policy_config("test/checkpoint", 1000, "eef")
    assert joint["robot"] == "yam"
    assert eef["robot"] == "other_embodiment"
    assert joint["_record"] is eef["_record"] is False
    assert "ik" not in joint and "ik" not in eef
    assert eef["embodiment"]["action"]["left_eef_xyz"] == [10, 13]
