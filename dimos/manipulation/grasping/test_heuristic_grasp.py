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

from unittest.mock import patch

import numpy as np
import pytest

from dimos.core.module import ModuleBase
from dimos.manipulation.grasping.heuristic_grasp import HeuristicGraspModule
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2


def test_heuristic_grasp_proposes_centered_top_down_pose() -> None:
    with patch.object(ModuleBase, "__init__", lambda self, config_args: None):
        module = HeuristicGraspModule()
    cloud = PointCloud2.from_numpy(
        np.asarray(
            [
                [-0.10, -0.02, 0.10],
                [-0.10, 0.02, 0.10],
                [0.10, -0.02, 0.20],
                [0.10, 0.02, 0.20],
            ],
            dtype=np.float32,
        ),
        frame_id="link_base",
        timestamp=1.0,
    )

    proposals = module.propose_grasps(cloud)

    assert len(proposals.candidates) == 1
    pose = proposals.candidates[0].pose
    assert pose.position.x == pytest.approx(0.0)
    assert pose.position.y == pytest.approx(0.0)
    assert pose.position.z == pytest.approx(0.15)
    approach = pose.orientation.rotate_vector(Vector3(0.0, 0.0, 1.0))
    assert approach.z == pytest.approx(-1.0)
