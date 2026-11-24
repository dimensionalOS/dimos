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

from pytest_mock import MockerFixture

from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.navigation.go2.replanning_a_star.position_tracker import PositionTracker


def test_positions_older_than_the_time_window_are_ignored(mocker: MockerFixture) -> None:
    # A present-day epoch, which float32 can only hold to the nearest 128 s.
    start = 1_795_000_064.0
    clock = mocker.patch("time.time", return_value=start)
    tracker = PositionTracker(time_window=8.0, threshold=1.0)

    # Driving in from 5 m away, then sitting still for the whole window.
    tracker.add_position(PoseStamped(position=[0.0, 0.0, 0.0]))
    for elapsed in (12.0, 16.0, 20.0):
        clock.return_value = start + elapsed
        tracker.add_position(PoseStamped(position=[5.0, 0.0, 0.0]))

    assert tracker.is_stuck()
