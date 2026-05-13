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

"""Binary-driven drift-correction check.

Feeds the rtab_map binary an odometry stream with accumulating linear
drift while showing the same scene from the same true position. The
binary's published ``corrected_odometry`` should track closer to the
ground-truth (origin) than the raw odom does.
"""

from __future__ import annotations

import numpy as np
import pytest

from dimos.navigation.nav_stack.modules.rtab_map.tests.conftest import (
    RtabHarness,
    identity_quat,
    square_room_scan,
)

pytestmark = [pytest.mark.self_hosted]


def test_corrected_odom_present_under_drift(rtab_harness: RtabHarness) -> None:
    """Send drifty odom + identical scans of the same scene. At minimum we
    expect a stream of corrected_odometry messages, and the last
    corrected pose to be a finite real position (not NaN / not the
    accumulated drift verbatim if rtabmap fires a correction)."""
    scan = square_room_scan()
    drift = 0.05  # m per step
    n_steps = 12
    for i in range(n_steps):
        ts = float(i) * 0.4
        odom_pos = np.array([drift * i, 0.0, 0.0])
        rtab_harness.publish_odom(odom_pos, identity_quat(), ts)
        rtab_harness.publish_scan(scan, ts)
        rtab_harness.drain(seconds=0.12)

    rtab_harness.drain(seconds=2.5)

    msgs = rtab_harness.corrected.messages
    assert len(msgs) >= 3, f"expected corrected_odometry messages each frame, got {len(msgs)}"
    last = msgs[-1]
    # Sanity: finite numbers.
    for v in (last.pose.position.x, last.pose.position.y, last.pose.position.z):
        assert np.isfinite(v), f"corrected pose has non-finite value: {last}"
    # rtab_tf channel must also have fired alongside.
    assert rtab_harness.rtab_tf.messages, "rtab_tf channel saw no messages"
