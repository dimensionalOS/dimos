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

from types import SimpleNamespace

import numpy as np

from dimos.manipulation.visualization.grasp_layers import _wireframe
from dimos.msgs.geometry_msgs.Pose import Pose


def test_wireframe_converts_tcp_pose_back_to_grasp_frame() -> None:
    pose = Pose(1.0, 2.0, 3.0)
    gripper = SimpleNamespace(
        extents_open=(0.08, 0.03, 0.06),
        offset_open=(0.0, 0.0, 0.09),
        extents_half_open=(0.04, 0.03, 0.04),
        offset_half_open=(0.0, 0.0, 0.02),
    )
    grasp_frame_to_tcp = np.eye(4)
    grasp_frame_to_tcp[0, 3] = 0.2

    vertices, edges = _wireframe(pose, gripper, grasp_frame_to_tcp)

    np.testing.assert_allclose(vertices[1], [0.8, 2.0, 3.0], atol=1e-6)
    np.testing.assert_array_equal(edges, [[0, 1], [2, 3], [4, 5], [6, 7]])
