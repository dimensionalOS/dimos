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

from dimos.navigation.basic_path_follower.module import lookahead_distance


def test_lookahead_floor_at_low_speed():
    assert lookahead_distance(0.1, 1.5, 0.4, 1.5) == 0.4


def test_lookahead_scales_in_linear_region():
    assert lookahead_distance(0.5, 1.5, 0.4, 1.5) == 0.75


def test_lookahead_clamped_at_ceiling():
    assert lookahead_distance(2.0, 1.5, 0.4, 1.5) == 1.5


def test_lookahead_monotonic_in_speed():
    assert lookahead_distance(0.8, 1.5, 0.4, 1.5) > lookahead_distance(0.45, 1.5, 0.4, 1.5)
