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

"""Validation and lookup for complete named rigid-body pose samples."""

from dimos_lcm.geometry_msgs import Pose
from dimos_lcm.sim_msgs import BodyPoses
import numpy as np
from numpy.typing import NDArray


def pose_array(pose: Pose) -> NDArray[np.float64]:
    p, q = pose.position, pose.orientation
    return np.asarray([p.x, p.y, p.z, q.x, q.y, q.z, q.w])


def poses_by_id(sample: BodyPoses) -> dict[str, Pose]:
    result = {p.body_id: p.pose_world for p in sample.bodies}
    if len(result) != len(sample.bodies) or sample.count != len(result):
        raise ValueError("complete body poses require unique IDs and matching count")
    for pose in result.values():
        values = pose_array(pose)
        if not np.isfinite(values).all() or not np.isclose(
            np.linalg.norm(values[3:]), 1, atol=1e-6
        ):
            raise ValueError("invalid body pose")
    return result
