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
from scipy.spatial.transform import Rotation

from dimos.mapping.relocalization.go2.module import Go2Relocalization
from dimos.msgs.geometry_msgs.Transform import Transform


def _fix(x: float = 0.0, yaw_deg: float = 0.0) -> Transform:
    m = np.eye(4)
    m[:3, :3] = Rotation.from_euler("z", yaw_deg, degrees=True).as_matrix()
    m[0, 3] = x
    return Transform.from_matrix(m, frame_id="world", child_frame_id="map")


def test_settles_after_agreeing_fixes_and_a_jump_restarts_the_count() -> None:
    m = Go2Relocalization(settle_after=3)
    try:
        m.submit(_fix())
        m.submit(_fix(x=0.1, yaw_deg=1.0))
        m.submit(_fix(x=2.0))  # disagrees: count restarts here
        m.submit(_fix(x=2.1))
        assert m.keep_relocalizing()
        m.submit(_fix(x=2.0, yaw_deg=-1.0))
        assert m.settled and not m.keep_relocalizing()
    finally:
        m.dispose()


def test_settle_after_zero_never_stops() -> None:
    m = Go2Relocalization(settle_after=0)
    try:
        for _ in range(5):
            m.submit(_fix())
        assert m.keep_relocalizing()
    finally:
        m.dispose()
