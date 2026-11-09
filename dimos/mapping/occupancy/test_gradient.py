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


import cv2
import numpy as np
import pytest

from dimos.mapping.occupancy.gradient import gradient, voronoi_gradient
from dimos.mapping.occupancy.visualizations import visualize_occupancy_grid
from dimos.message_utils import image, image as image_utils, time as message_time
from dimos.utils.data import get_data


@pytest.mark.parametrize("method", ["simple", "voronoi"])
def test_gradient(occupancy, method) -> None:
    expected = image.from_array(
        cv2.imread(str(get_data(f"gradient_{method}.png"))),
        encoding="bgr8",
        header=message_time.header_now(),
    )

    match method:
        case "simple":
            og = gradient(occupancy, max_distance=1.5)
        case "voronoi":
            og = voronoi_gradient(occupancy, max_distance=1.5)
        case _:
            raise ValueError(f"Unknown resampling method: {method}")

    actual = visualize_occupancy_grid(og, "rainbow")
    np.testing.assert_array_equal(image_utils.view(actual), image_utils.view(expected))
