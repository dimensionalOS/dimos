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

from dimos.robot.unitree.g1.sonic_observation import (
    POLICY_JOINT_NAMES,
    SonicObservation,
    policy_joint_state,
)


def test_named_state_uses_training_order_not_model_tree_order():
    positions = {name: float(i) for i, name in reversed(list(enumerate(POLICY_JOINT_NAMES)))}
    result = policy_joint_state(positions)
    np.testing.assert_array_equal(result, np.arange(28))
    assert POLICY_JOINT_NAMES[24:] == (
        "right_hand_index_0_joint",
        "right_hand_index_1_joint",
        "right_hand_middle_0_joint",
        "right_hand_middle_1_joint",
    )


@pytest.fixture
def observation():
    return SonicObservation(4, 10.0, np.zeros(28), np.zeros((480, 640, 3), np.uint8))


def test_observation_owns_snapshot_and_preserves_source_time():
    state, rgb = np.ones(28), np.ones((480, 640, 3), np.uint8)
    obs = SonicObservation(4, 10.0, state, rgb)
    state[:] = 2
    rgb[:] = 2
    obs.require_fresh(now=10.1, after_sequence=3, after_observed_at=9.9)
    np.testing.assert_array_equal(obs.state, np.ones(28))
    assert np.all(obs.rgb == 1) and obs.observed_at == 10.0
    assert not obs.state.flags.writeable and not obs.rgb.flags.writeable


@pytest.mark.parametrize(
    "kwargs",
    [
        dict(now=10.21),
        dict(now=9.99),
        dict(now=10.1, after_sequence=4),
        dict(now=10.1, after_observed_at=10.0),
        dict(now=float("nan")),
        dict(now=10.1, max_age=0),
        dict(now=10.1, max_age=float("inf")),
    ],
)
def test_stale_future_duplicate_and_invalid_clocks_are_rejected(observation, kwargs):
    with pytest.raises(ValueError):
        observation.require_fresh(**kwargs)


@pytest.mark.parametrize(
    "change",
    [
        dict(sequence=True),
        dict(sequence=-1),
        dict(observed_at=float("nan")),
        dict(state=np.zeros(29)),
        dict(state=np.full(28, np.inf)),
        dict(state=np.ones(28, dtype=bool)),
        dict(rgb=np.zeros((480, 640, 3))),
        dict(rgb=np.zeros((640, 480, 3), np.uint8)),
    ],
)
def test_invalid_observation_schema_rejected(change):
    kwargs = dict(
        sequence=1, observed_at=10.0, state=np.zeros(28), rgb=np.zeros((480, 640, 3), np.uint8)
    )
    with pytest.raises(ValueError):
        SonicObservation(**(kwargs | change))


def test_missing_named_joint_rejected():
    with pytest.raises(ValueError, match="joint"):
        policy_joint_state({})
