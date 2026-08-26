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

"""Numbers the embodiment duplicates on purpose, pinned to their originals.

``embodiment/base.py`` is shared with the standalone referee export, so it may not
import a dimos robot package -- the go2's own constants are therefore copied
rather than referenced, and a copy that can drift silently is a bug waiting.
"""

from dimos.navigation.motion.embodiment.go2 import GO2
from dimos.robot.unitree.go2.constants import ROBOT_HEIGHT


def test_go2_height_matches_the_robot_constant() -> None:
    assert GO2.height == ROBOT_HEIGHT


def test_the_base_rides_under_the_body_top() -> None:
    # base_height is where the base FRAME sits, so it is inside the envelope;
    # a base above the body's own height would be a typo, not a mount.
    assert 0.0 < GO2.base_height < GO2.height


def test_dilate_moves_every_box_by_the_same_amount_per_side():
    """The knob has to reach the union AND the rows, or the shape it plans with
    depends on which edge the search happens to test.

    `adapter/rust/src/emb.rs::dilated` is the same formula; the deployed planner
    and this one must ask for the same gap.
    """
    tight = GO2.dilated(by=-0.03)
    assert tight.length == GO2.length - 0.06
    assert tight.width == GO2.width - 0.06
    for row, was in zip(tight.envelope, GO2.envelope, strict=True):
        assert row[0] == was[0]  # the drift angle a row answers for
        assert abs(row[1] - (was[1] - 0.06)) < 1e-12
        assert abs(row[2] - (was[2] - 0.06)) < 1e-12
        assert row[3:] == was[3:]  # offsets are where the box sits, not its size
    assert GO2.dilated(by=0.0) is GO2 or GO2.dilated(by=0.0) == GO2


def test_dilate_leaves_the_clearance_floor_alone():
    """It moves the BODY. The margin the search adds is the follower's tracking
    floor and is not this knob's to spend."""
    assert GO2.dilated(by=-0.05).precision == GO2.precision
