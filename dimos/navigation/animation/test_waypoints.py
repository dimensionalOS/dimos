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

from dimos.navigation.animation.waypoints import apply_click


def test_click_appends_places_or_inserts() -> None:
    points = [[0.0, 0, 0], [1.0, 0, 0]]
    assert apply_click(points, [2.0, 0, 0], None) == 2
    assert apply_click(points, [9.0, 0, 0], {"op": "place", "index": 0}) == 0
    assert apply_click(points, [5.0, 0, 0], {"op": "insert", "index": 1}) == 1
    assert [p[0] for p in points] == [9.0, 5.0, 1.0, 2.0]
