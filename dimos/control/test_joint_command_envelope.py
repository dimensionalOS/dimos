# Copyright 2025-2026 Dimensional Inc.
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

from dimos.control.joint_command_envelope import bound_joint_command


def test_combined_joint_bounds_enforce_velocity_feedback_and_position():
    result = bound_joint_command(
        np.array([2.0, -2.0, 2.0]),
        np.array([0.0, 0.0, 0.95]),
        np.array([0.0, 0.05, 0.95]),
        np.array([-1.0] * 3),
        np.ones(3),
        np.ones(3),
        0.1,
        0.08,
    )
    np.testing.assert_allclose(result, [0.08, -0.03, 1.0])


def test_invalid_candidate_and_empty_envelope_fail():
    with pytest.raises(ValueError, match="Invalid"):
        bound_joint_command(
            np.array([np.nan]),
            np.zeros(1),
            np.zeros(1),
            -np.ones(1),
            np.ones(1),
            np.ones(1),
            0.01,
            0.1,
        )
    with pytest.raises(ValueError, match="Empty"):
        bound_joint_command(
            np.zeros(1), np.zeros(1), np.ones(1), -np.ones(1), np.ones(1), np.ones(1), 0.01, 0.1
        )
