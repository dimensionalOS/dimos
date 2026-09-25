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

from __future__ import annotations

import numpy as np
import pytest

from dimos.experimental.agent_encode.pointcloud.shapes.box import Box
from dimos.experimental.agent_encode.pointcloud.shapes.cylinder import Cylinder


def test_shapes_contain_and_measure() -> None:
    pts = np.array([[0.0, 0.0, 0.5], [2.0, 0.0, 0.5], [0.0, 0.0, 5.0]], dtype=np.float32)
    box = Box(center=(0.0, 0.0, 0.5), size=(1.0, 1.0, 1.0))
    assert box.contains(pts).tolist() == [True, False, False]
    assert box.distance(pts)[1] == pytest.approx(1.5)
    turned = Box(center=(0.0, 0.0, 0.5), size=(6.0, 0.2, 1.0), yaw_deg=90.0)
    assert turned.contains(np.array([[0.0, 2.5, 0.5]], dtype=np.float32))[0], "rotated onto y"
    cyl = Cylinder(center=(0.0, 0.0), radius=0.5, z=(0.0, 1.0))
    assert cyl.contains(pts).tolist() == [True, False, False]
    assert cyl.distance(pts)[1] == pytest.approx(1.5)
    assert np.isinf(cyl.distance(pts)[2]), "outside the z band"
    assert Cylinder(center=(0.0, 0.0), radius=0.5).contains(pts).tolist() == [True, False, True]
