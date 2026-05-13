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

"""Contract parity: the binary publishes every channel on the wire that the
Python wrapper declares (and that PGO's wrapper declares for the shared
streams). This is the swap-compatibility check at the binary level."""

from __future__ import annotations

import numpy as np
import pytest

from dimos.navigation.nav_stack.modules.rtab_map.tests.conftest import (
    RtabHarness,
    identity_quat,
    square_room_scan,
)

pytestmark = [pytest.mark.self_hosted]


def test_binary_publishes_all_five_outputs(rtab_harness: RtabHarness) -> None:
    """All five output channels (corrected_odometry, global_map, rtab_tf,
    octomap, projected_2d_grid) must receive at least one message within a
    short bring-up window."""
    scan = square_room_scan()
    for i in range(10):
        ts = float(i) * 0.4
        rtab_harness.publish_odom(np.array([0.5 * i, 0.0, 0.0]), identity_quat(), ts)
        rtab_harness.publish_scan(scan, ts)
        rtab_harness.drain(seconds=0.15)

    rtab_harness.drain(seconds=2.0)

    assert rtab_harness.corrected.messages, "no corrected_odometry messages"
    assert rtab_harness.rtab_tf.messages, "no rtab_tf messages"
    assert rtab_harness.global_map.messages, "no global_map messages"
    assert rtab_harness.octomap.messages, "no octomap messages"
    # The projected 2d grid can legitimately be empty for the first few frames
    # (rtabmap needs time to build up the occupancy grid). Don't require it
    # to be non-empty, only that the topic exists and at least one message
    # has been emitted — which the collector confirms.
    assert rtab_harness.proj2d.messages, "no projected_2d_grid messages"
