#!/usr/bin/env python3
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


import cv2
import numpy as np

from dimos.mapping.occupancy.inflation import simple_inflate
from dimos.mapping.occupancy.visualizations import visualize_occupancy_grid
from dimos.utils.data import get_data


def test_inflation(occupancy) -> None:
    expected = cv2.imread(get_data("inflation_simple.png"), cv2.IMREAD_COLOR)

    og = simple_inflate(occupancy, 0.2)

    result = visualize_occupancy_grid(og, "rainbow")
    np.testing.assert_array_equal(result.data, expected)
