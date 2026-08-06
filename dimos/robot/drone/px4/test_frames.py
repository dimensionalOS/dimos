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

import math

import pytest

from dimos.robot.drone.px4.frames import flu_to_frd_body_velocity


def test_flu_to_frd_body_velocity_preserves_forward_and_negates_lateral_vertical_and_yaw() -> None:
    result = flu_to_frd_body_velocity(1.25, 2.5, 3.75, math.pi / 2.0)

    assert result == pytest.approx((1.25, -2.5, -3.75, -90.0))


@pytest.mark.parametrize("value", [math.nan, math.inf, -math.inf])
def test_flu_to_frd_body_velocity_rejects_non_finite_inputs(value: float) -> None:
    with pytest.raises(ValueError, match="finite"):
        _ = flu_to_frd_body_velocity(value, 0.0, 0.0, 0.0)
