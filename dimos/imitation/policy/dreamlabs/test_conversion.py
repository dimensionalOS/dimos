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

from dimos.imitation.policy.dreamlabs.conversion import action_trajectory, state_vector
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.robot.manipulators.dual_openyam.joints import DUAL_OPENYAM_JOINTS

VALID = [[0.0] * 6 + [0.5]] * 48


@pytest.mark.parametrize(
    "reply",
    [
        {"q_r_chunk": VALID},
        {"q_l_chunk": VALID[:47], "q_r_chunk": VALID},
        {"q_l_chunk": [[float("nan")] * 6 + [0.5]] * 48, "q_r_chunk": VALID},
        {"q_l_chunk": VALID, "q_r_chunk": [[0.0] * 6 + [float("inf")]] * 48},
    ],
)
def test_invalid_action_contract(reply: dict) -> None:
    with pytest.raises(ValueError):
        action_trajectory(reply, np.array([0.0] * 12 + [0.5, 0.5]))


@pytest.mark.parametrize("positions", [[0.0] * 13, [0.0] * 12 + [-0.1, 0.0], [float("nan")] * 14])
def test_invalid_state_contract(positions: list[float]) -> None:
    with pytest.raises(ValueError):
        state_vector(JointState(name=list(DUAL_OPENYAM_JOINTS), position=positions))


@pytest.mark.parametrize("opening, expected", [(-0.001, 0.0), (1.006, 1.0), (0.0, 0.0), (1.0, 1.0)])
def test_finite_output_grippers_are_clamped_without_changing_arms_or_raw_reply(
    opening: float,
    expected: float,
) -> None:
    left = [0.1, -0.2, 0.3, -0.4, 0.5, -0.6]
    right = [-0.7, 0.8, -0.9, 1.0, -1.1, 1.2]
    reply = {"q_l_chunk": [[*left, opening]] * 48, "q_r_chunk": [[*right, opening]] * 48}
    trajectory = action_trajectory(reply, np.array([0.0] * 12 + [0.5, 0.5]))
    assert trajectory.points[1].positions == [*left, *right, expected, expected]
    assert trajectory.points[-1].positions == [*left, *right, expected, expected]
    assert reply["q_l_chunk"][0][-1] == opening
    assert reply["q_r_chunk"][0][-1] == opening


@pytest.mark.parametrize("opening", [float("nan"), float("inf"), -float("inf")])
def test_nonfinite_output_gripper_is_rejected(opening: float) -> None:
    with pytest.raises(ValueError, match="non-finite gripper"):
        action_trajectory(
            {"q_l_chunk": VALID, "q_r_chunk": [[0.0] * 6 + [opening]] * 48},
            np.array([0.0] * 12 + [0.5, 0.5]),
        )
