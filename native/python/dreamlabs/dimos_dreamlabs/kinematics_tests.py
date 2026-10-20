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


"""SDK standard-YAM fixtures, not a calibration of the deployed robot."""

from dream_ik.solver import ChunkIK
import numpy as np
import pytest


@pytest.mark.parametrize(
    ("q", "xyz", "rotation"),
    [
        ([0, 0, 0, 0, 0, 0], [0.244997, 0, 0.164002], [[0, 0, 1], [1, 0, 0], [0, 1, 0]]),
        (
            [0, 0.5, 0.5, -0.35, 0, 0],
            [0.262523, 0, 0.206904],
            [[0, 0.342891, 0.939375], [1, 0, 0], [0, 0.939375, -0.342891]],
        ),
    ],
)
def test_official_standard_yam_tcp_pose(
    q: list[float], xyz: list[float], rotation: list[list[float]]
) -> None:
    solver = ChunkIK("yam", "tcp", 20.0)
    pose = solver.fk(np.asarray(q, dtype=np.float64))
    np.testing.assert_allclose(pose[:3, 3], xyz, atol=2e-5)
    np.testing.assert_allclose(pose[:3, :3], rotation, atol=2e-5)
