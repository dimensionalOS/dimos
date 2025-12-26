# Copyright 2025 Dimensional Inc.
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

import math

import pytest

from dimos.utils.trigonometry import angle_diff


def from_rad(x):
    return x / (math.pi / 180)


def to_rad(x):
    return x * (math.pi / 180)


def test_angle_diff():
    a = to_rad(1)
    b = to_rad(359)

    assert from_rad(angle_diff(a, b)) == pytest.approx(2, abs=0.00000000001)

    assert from_rad(angle_diff(b, a)) == pytest.approx(-2, abs=0.00000000001)
