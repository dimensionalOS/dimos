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
import pytest

from dimos.mapping.occupancy.visualizations import visualize_occupancy_grid
from dimos.utils.data import get_data


@pytest.mark.parametrize("palette", ["rainbow", "turbo"])
def test_visualize_occupancy_grid(occupancy_gradient, palette) -> None:
    expected = cv2.imread(get_data(f"visualize_occupancy_{palette}.png"), cv2.IMREAD_COLOR)

    result = visualize_occupancy_grid(occupancy_gradient, palette)

    np.testing.assert_array_equal(result.data, expected)
