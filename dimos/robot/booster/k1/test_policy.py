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
import torch

from dimos.hardware.whole_body.spec import IMUState
from dimos.robot.booster.k1.control_config import HOME, POLICY_PATH
from dimos.robot.booster.k1.policy import K1Policy

pytestmark = pytest.mark.self_hosted  # The released TorchScript checkpoint is an LFS asset.


@pytest.fixture
def policy():
    threads = torch.get_num_threads()
    torch.set_num_threads(1)
    model = K1Policy(POLICY_PATH)
    model.start()
    yield model
    torch.set_num_threads(threads)


def inputs(t):
    q = (
        np.asarray(HOME, dtype=np.float32)
        + np.sin(np.arange(22, dtype=np.float32) + t * 0.1) * 0.03
    )
    dq = np.cos(np.arange(22, dtype=np.float32) + t * 0.2) * 0.2
    quat = np.array(
        [1, 0.02 * np.sin(t), 0.03 * np.cos(t), 0.05 * np.sin(t * 0.1)], dtype=np.float32
    )
    quat /= np.linalg.norm(quat)
    return (
        q,
        dq,
        IMUState(tuple(quat), (0.01 * t, -0.02, 0.03), (0, 0, 0)),
        np.array([0.2, -0.1, 0.15], dtype=np.float32),
    )


def test_matches_boosters_released_runner_after_history_fills(policy):
    # Golden targets from Booster LocomotionPolicy, revision
    # 7bb1462e2ac742eba5c01659b815e2e99fbe617f, using inputs(0..24).
    expected = [
        0,
        0,
        -0.135307133,
        -1.268580556,
        0.593581855,
        -0.324244082,
        0.043983907,
        1.214447618,
        0.239263207,
        0.444473326,
        -0.172619551,
        0.166149542,
        -0.051890314,
        0.432377100,
        -0.076198526,
        0.002402181,
        -0.257892698,
        -0.060706265,
        0.013797787,
        0.454561442,
        -0.234856084,
        0.004711795,
    ]
    for t in range(25):
        result = policy.step(*inputs(t))
    np.testing.assert_allclose(result, expected, rtol=0, atol=2e-6)


def test_reset_reproduces_the_first_action(policy):
    first = policy.step(*inputs(0))
    for t in range(1, 12):
        policy.step(*inputs(t))
    policy.reset()
    np.testing.assert_array_equal(policy.step(*inputs(0)), first)


@pytest.mark.parametrize("quat", [(0, 0, 0, 0), (0, 1, 0, 0), (float("nan"), 0, 0, 0)])
def test_invalid_or_fallen_orientation_cannot_produce_motor_targets(policy, quat):
    q, dq, _, command = inputs(0)
    with pytest.raises(ValueError):
        policy.step(q, dq, IMUState(quat, (0, 0, 0), (0, 0, 0)), command)
