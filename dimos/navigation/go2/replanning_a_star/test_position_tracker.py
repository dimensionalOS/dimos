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
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.navigation.go2.replanning_a_star import position_tracker
from dimos.navigation.go2.replanning_a_star.position_tracker import PositionTracker


def test_old_points_leave_the_window(monkeypatch) -> None:
    now = [1794251786.0]
    monkeypatch.setattr(position_tracker.time, "time", lambda: now[0])
    tracker = PositionTracker(time_window=8, threshold=0.05)

    tracker.add_position(PoseStamped(5.0, 0.0, 0.0))

    now[0] += 20
    for _ in range(10):
        tracker.add_position(PoseStamped(0.0, 0.0, 0.0))

    assert tracker.is_stuck() is True
