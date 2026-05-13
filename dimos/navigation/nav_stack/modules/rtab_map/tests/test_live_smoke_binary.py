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

"""Binary smoke test: the rtab_map binary launches and emits corrected
odometry within a small window after receiving an odom + scan pair."""

from __future__ import annotations

import numpy as np
import pytest

from dimos.navigation.nav_stack.modules.rtab_map.tests.conftest import (
    RtabHarness,
    identity_quat,
    square_room_scan,
)

pytestmark = [pytest.mark.self_hosted]


def test_binary_emits_corrected_odometry(rtab_harness: RtabHarness) -> None:
    scan = square_room_scan()
    for i in range(6):
        ts = float(i) * 0.5
        rtab_harness.publish_odom(np.array([0.6 * i, 0.0, 0.0]), identity_quat(), ts)
        rtab_harness.publish_scan(scan, ts)
        # Small inter-frame pause so rtabmap doesn't drop the second message
        # in the queue before LCM delivers it.
        rtab_harness.drain(seconds=0.15)

    rtab_harness.drain(seconds=2.0)

    assert len(rtab_harness.corrected.messages) >= 1, (
        f"expected at least one corrected_odometry message, got "
        f"{len(rtab_harness.corrected.messages)}"
    )
    assert len(rtab_harness.rtab_tf.messages) >= 1, (
        "rtab_tf should be published alongside corrected_odometry"
    )
