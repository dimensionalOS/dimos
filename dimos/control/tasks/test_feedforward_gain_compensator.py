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

import pytest

from dimos.control.tasks.feedforward_gain_compensator import (
    FeedforwardGainCompensator,
    FeedforwardGainConfig,
)


def test_default_is_unity_passthrough():
    out = FeedforwardGainCompensator().compute(0.3, -0.2, 0.5)
    assert out == pytest.approx((0.3, -0.2, 0.5))


def test_inverts_plant_gain():
    comp = FeedforwardGainCompensator(FeedforwardGainConfig(K_vx=0.8, K_vy=0.5, K_wz=2.0))
    vx, vy, wz = comp.compute(0.4, 0.1, 1.0)
    assert vx == pytest.approx(0.5)  # 0.4 / 0.8
    assert vy == pytest.approx(0.2)  # 0.1 / 0.5
    assert wz == pytest.approx(0.5)  # 1.0 / 2.0


@pytest.mark.parametrize("axis", ["K_vx", "K_vy", "K_wz"])
@pytest.mark.parametrize("bad", [0.0, -0.5, float("nan"), float("inf"), -float("inf")])
def test_unusable_gain_rejected_at_construction(axis, bad):
    with pytest.raises(ValueError, match="invalid calibration artifact"):
        FeedforwardGainCompensator(FeedforwardGainConfig(**{axis: bad}))
