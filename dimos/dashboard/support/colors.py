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

import numpy as np


def color_by_height(points: np.ndarray) -> np.ndarray:
    import matplotlib

    # currently need to calculate the color manually
    # see https://github.com/rerun-io/rerun/issues/4409
    cmap = matplotlib.colormaps["turbo_r"]
    heights = points[:, 2]
    norm = matplotlib.colors.Normalize(
        vmin=heights.min(),
        vmax=heights.max(),
    )
    return cmap(norm(heights))


def color_by_distance(points: np.ndarray) -> np.ndarray:
    import matplotlib

    # currently need to calculate the color manually
    # see https://github.com/rerun-io/rerun/issues/4409
    cmap = matplotlib.colormaps["turbo_r"]
    point_distances = np.linalg.norm(points, axis=1)
    norm = matplotlib.colors.Normalize(
        vmin=point_distances.min(),
        vmax=point_distances.max(),
    )
    return cmap(norm(point_distances))
