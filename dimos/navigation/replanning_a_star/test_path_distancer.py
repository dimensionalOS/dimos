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

from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.nav_msgs.Path import Path
from dimos.navigation.replanning_a_star.path_distancer import PathDistancer


def test_path_progress_reports_arc_length_at_waypoints() -> None:
    path = Path(
        poses=[
            PoseStamped(position=[0.0, 0.0, 0.0]),
            PoseStamped(position=[3.0, 0.0, 0.0]),
            PoseStamped(position=[3.0, 4.0, 0.0]),
        ]
    )
    distancer = PathDistancer(path)

    assert distancer.path_length_m == pytest.approx(7.0)
    assert distancer.progress_at_index_m(0) == pytest.approx(0.0)
    assert distancer.progress_at_index_m(1) == pytest.approx(3.0)
    assert distancer.progress_at_index_m(2) == pytest.approx(7.0)
    assert distancer.progress_at_index_m(100) == pytest.approx(7.0)
