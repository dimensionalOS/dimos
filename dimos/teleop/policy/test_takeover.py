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

from functools import partial
from itertools import pairwise
import pickle

import numpy as np
import pytest

from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.teleop.policy.demo_sources import bottle_cap_commands, bottle_cap_takeover
from dimos.teleop.policy.takeover import (
    distance_to_target,
    linear,
    logarithmic,
    power,
    quadratic,
    reference_path_progress,
)


@pytest.mark.parametrize(
    "curve",
    [linear, quadratic, partial(power, q=0.5), partial(power, q=3), partial(logarithmic, k=9)],
)
def test_curve_endpoints_range_and_monotonicity(curve):
    values = [curve(float(x)) for x in np.linspace(0, 1, 21)]
    assert values[0] == 0 and values[-1] == 1
    assert all(0 <= x <= 1 for x in values)
    assert all(a <= b for a, b in pairwise(values))
    assert pickle.loads(pickle.dumps(curve))(0.5) == curve(0.5)


@pytest.mark.parametrize(
    "curve, expected",
    [
        (linear, 0.5),
        (quadratic, 0.25),
        (partial(power, q=3), 0.125),
        (partial(logarithmic, k=3), np.log(2.5) / np.log(4)),
    ],
)
def test_curve_values(curve, expected):
    assert curve(0.5) == pytest.approx(expected)


@pytest.mark.parametrize("curve", [linear, quadratic, power, logarithmic])
@pytest.mark.parametrize("value", [-0.1, 1.1, np.nan, np.inf])
def test_invalid_progress_raises(curve, value):
    with pytest.raises(ValueError):
        curve(value)


@pytest.mark.parametrize("curve", [power, logarithmic])
@pytest.mark.parametrize("parameter", [0, -1, np.nan, np.inf])
def test_invalid_curve_parameters_raise(curve, parameter):
    with pytest.raises(ValueError):
        curve(0.5, parameter)


@pytest.mark.parametrize("x, expected", [(3, 0), (2, 0), (1.5, 0.5), (1, 1), (0, 1)])
def test_distance_mapping_clamps_and_increases_towards_target(x, expected):
    pose = Transform(translation=Vector3(x, 0, 0))
    assert distance_to_target(pose, (0, 0, 0), 2, 1) == expected


@pytest.mark.parametrize("outer, inner", [(0, 0), (1, 1), (1, -1), (np.inf, 0), (1, np.nan)])
def test_invalid_distance_radii_raise(outer, inner):
    with pytest.raises(ValueError):
        distance_to_target(Transform(), (0, 0, 0), outer, inner)


@pytest.mark.parametrize(
    "position, expected",
    [((-1, 0, 0), 0), ((0.5, 0, 0), 1 / 6), ((1, 1, 0), 2 / 3), ((1, 4, 0), 1)],
)
def test_path_progress_uses_nearest_segment_and_arc_length(position, expected):
    path = [(0, 0, 0), (0, 0, 0), (1, 0, 0), (1, 2, 0)]
    assert reference_path_progress(Transform(translation=Vector3(position)), path) == pytest.approx(
        expected
    )


@pytest.mark.parametrize(
    "path", [[], [(0, 0, 0)], [(0, 0, 0)] * 2, [(0, 0, 0), (np.nan, 0, 0)], [(0, 0), (1, 1)]]
)
def test_invalid_reference_path_raises(path):
    with pytest.raises(ValueError):
        reference_path_progress(Transform(), path)


def test_bottle_cap_uses_human_lifting_and_policy_rotation_and_finishes_at_zero():
    far = bottle_cap_takeover(Transform(translation=Vector3(1, 0, 0.4)))
    near = bottle_cap_takeover(Transform(translation=Vector3(0.4, 0, 0.4)))
    assert far == (0, 0, 0, 0, 0, 0.8)
    assert near == (1, 1, 0.1, 1, 1, 1)
    human, policy = bottle_cap_commands(2, 8, "world")
    assert human.linear.z > policy.linear.z
    assert policy.angular.z > human.angular.z
    assert all(command.is_zero() for command in bottle_cap_commands(8, 8, "world"))
