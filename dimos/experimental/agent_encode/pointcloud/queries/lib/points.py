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

"""A cloud's finite returns, and returns wrapped back into a cloud."""

from __future__ import annotations

from typing import TYPE_CHECKING

import numpy as np
from numpy.typing import NDArray

if TYPE_CHECKING:
    from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2


def finite_points(cloud: PointCloud2) -> NDArray[np.float32]:
    """The cloud's returns with every coordinate finite, as (N, 3) float32."""
    points = np.asarray(cloud.points_f32(), dtype=np.float32).reshape(-1, 3)
    finite: NDArray[np.float32] = points[np.isfinite(points).all(axis=1)]
    return finite


def as_cloud(points: NDArray[np.float32], like: PointCloud2) -> PointCloud2:
    """``points`` as a cloud with ``like``'s frame and timestamp."""
    return type(like).from_numpy(
        np.ascontiguousarray(points, dtype=np.float32), frame_id=like.frame_id, timestamp=like.ts
    )
