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

from typing import Any

import numpy as np
from numpy.typing import NDArray

from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3


def body_twist(
    quat_wxyz: NDArray[Any], qvel: NDArray[Any]
) -> tuple[NDArray[np.float64], NDArray[np.float64]]:
    """Body-frame (linear, angular) velocity of a free joint, whose qvel is world-frame linear, body-frame angular."""
    orientation = Quaternion(quat_wxyz[1], quat_wxyz[2], quat_wxyz[3], quat_wxyz[0])
    linear = orientation.inverse().rotate_vector(Vector3(qvel[0], qvel[1], qvel[2]))
    return linear.to_numpy(), np.asarray(qvel[3:6], dtype=np.float64).copy()
