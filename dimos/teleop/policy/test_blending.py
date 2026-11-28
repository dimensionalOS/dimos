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

from copy import deepcopy

import numpy as np
import pytest

from dimos.msgs.geometry_msgs.TwistStamped import TwistStamped
from dimos.teleop.policy.blending import blend_twist
from dimos.utils.transform_utils import twist_to_numpy


@pytest.fixture
def commands():
    return (
        TwistStamped(ts=10, frame_id="world", linear=[2, 4, 6], angular=[8, 10, 12]),
        TwistStamped(ts=11, frame_id="world", linear=[12, 10, 8], angular=[6, 4, 2]),
    )


@pytest.mark.parametrize(
    "weight, expected",
    [
        (0, [2, 4, 6, 8, 10, 12]),
        (1, [12, 10, 8, 6, 4, 2]),
        (0.5, [7, 7, 7, 7, 7, 7]),
        ((0, 0.5, 1, 1, 0.5, 0), [2, 7, 8, 6, 7, 12]),
    ],
)
def test_exact_axis_blending_preserves_inputs(commands, weight, expected):
    teleop, policy = commands
    before = deepcopy(commands)
    result = blend_twist(teleop, policy, weight)
    np.testing.assert_array_equal(twist_to_numpy(result), expected)
    assert result.frame_id == "world"
    assert teleop == before[0] and policy == before[1]
    assert result is not teleop and result is not policy


@pytest.mark.parametrize(
    "weight",
    [float("nan"), float("inf"), -0.1, 1.1, (), (0, 1), [[0] * 6], (0, 0, 0, 0, 0, float("nan"))],
)
def test_invalid_weights_are_rejected(commands, weight):
    with pytest.raises(ValueError, match="weights"):
        blend_twist(*commands, weight)


@pytest.mark.parametrize("weight, missing_index", [(0, 1), (1, 0)])
def test_entirely_unused_source_can_be_absent_or_nonfinite(commands, weight, missing_index):
    sources = list(commands)
    expected = commands[1 - missing_index]
    sources[missing_index] = None
    assert blend_twist(*sources, weight) == expected
    sources[missing_index] = TwistStamped(linear=[np.nan, np.inf, 0])
    assert blend_twist(*sources, weight) == expected


@pytest.mark.parametrize("source", [0, 1])
@pytest.mark.parametrize("field", ["linear", "angular"])
def test_nonfinite_active_command_is_rejected(commands, source, field):
    getattr(commands[source], field).z = np.nan
    with pytest.raises(ValueError, match="nonfinite"):
        blend_twist(*commands, 0.5)


def test_required_source_and_frame_mismatch_are_rejected(commands):
    with pytest.raises(ValueError, match="missing"):
        blend_twist(None, commands[1], 0.5)
    commands[1].frame_id = "tool"
    with pytest.raises(ValueError, match="same frame"):
        blend_twist(*commands, 0.5)
