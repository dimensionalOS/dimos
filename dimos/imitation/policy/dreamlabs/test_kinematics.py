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


import numpy as np
import pytest

from dimos.imitation.policy.dreamlabs.kinematics import DreamlabsKinematics


# Canonical dual_openyam_abc_box_v2 references with the explicit SDK TCP transform.
# These are offline pose fixtures, not physical calibration or a duplicate model.
@pytest.mark.parametrize(
    ("q", "xyz", "rot6"),
    [
        (
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            [0.24529726268831478, 1.479606077632217e-06, 0.1735023068035244],
            [
                -3.684216608235147e-06,
                0.9999999403953552,
                -6.3371844589710236e-06,
                -7.3197793426516e-06,
                6.3591660364181735e-06,
                0.9999999403953552,
            ],
        ),
        (
            [0.0, 0.5, 0.5, -0.35, 0.0, 0.0],
            [0.2628045210176952, 2.0465567064393546e-06, 0.21630107502670454],
            [
                -5.072791736893123e-06,
                1.0,
                -7.287463631655555e-06,
                0.34289097785949707,
                8.57667146192398e-06,
                0.9393751621246338,
            ],
        ),
        (
            [
                -0.059506389827621575,
                3.0708837082059555,
                0.47549121676525663,
                0.4637176008186161,
                0.3820993768416505,
                0.3238620147410465,
            ],
            [-0.022005077517198553, -0.06230098400405956, -0.24285246094908358],
            [
                0.13360722362995148,
                0.8732455968856812,
                -0.46860575675964355,
                0.8467977643013,
                -0.3462457060813904,
                -0.4037916362285614,
            ],
        ),
    ],
)
def test_canonical_arm_local_tcp(q: list[float], xyz: list[float], rot6: list[float]) -> None:
    fk = DreamlabsKinematics()
    pose = fk.pose(np.array(q))
    np.testing.assert_allclose(
        fk.pose(np.array(q), "right").homogeneous, pose.homogeneous, atol=2e-6, rtol=0
    )
    np.testing.assert_allclose(pose.translation, xyz, atol=2e-6, rtol=0)
    np.testing.assert_allclose(
        np.concatenate((pose.rotation[:, 0], pose.rotation[:, 1])), rot6, atol=1e-5, rtol=0
    )
