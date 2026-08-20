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

from dimos.manipulation.planning.utils.joint_positions import (
    repair_unconfigured_joint_positions,
)


def test_repair_unconfigured_joint_positions_preserves_configured_positions() -> None:
    positions = np.array([2.0, 0.0, -1.0])
    lower = np.array([-1.0, 0.018, -0.5])
    upper = np.array([1.0, 0.06, 0.5])

    repaired = repair_unconfigured_joint_positions(
        positions,
        lower,
        upper,
        configured_indices=[0],
    )

    np.testing.assert_allclose(repaired, [2.0, 0.018, 0.0])
