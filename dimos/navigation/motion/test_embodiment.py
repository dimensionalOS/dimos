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

``embodiment.py`` is shared with the standalone referee export, so it may not
import a dimos robot package -- the go2's own constants are therefore copied
rather than referenced, and a copy that can drift silently is a bug waiting.
"""

from dimos.navigation.motion.embodiment import GO2
from dimos.robot.unitree.go2.constants import ROBOT_HEIGHT


def test_go2_height_matches_the_robot_constant() -> None:
    assert GO2.height == ROBOT_HEIGHT


def test_the_base_rides_under_the_body_top() -> None:
    # base_height is where the base FRAME sits, so it is inside the envelope;
    # a base above the body's own height would be a typo, not a mount.
    assert 0.0 < GO2.base_height < GO2.height
