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

"""Station geometry and TCP forward kinematics match the trained plant."""

import os

import numpy as np
import pytest

from dimos.robot.manipulators.dual_makermods_metal.policy.contract import (
    HOME,
    JOINT_ORDER,
    StationGeometry,
)
from dimos.robot.manipulators.dual_makermods_metal.policy.station import (
    MetalStationKinematics,
    rpy_to_matrix,
)

pytest.importorskip("pinocchio")


def test_station_defaults_are_the_trained_mounts_not_the_bench_example():
    geometry = StationGeometry()
    assert geometry.left_mount_xyz == (0.0, -0.345, 0.008)
    assert geometry.right_mount_xyz == (0.0, 0.345, 0.008)
    # The repo's example bench layout mirrors the sides; mixing them up would
    # command each arm across the station.
    assert geometry.left_mount_xyz[1] < 0.0 < geometry.right_mount_xyz[1]
    assert np.allclose(geometry.tcp_offset_xyz, (0.0, 0.0, 0.06))


def test_rpy_zero_is_identity_and_yaw_is_right_handed():
    assert np.allclose(rpy_to_matrix((0.0, 0.0, 0.0)), np.eye(3))
    assert np.allclose(rpy_to_matrix((0.0, 0.0, np.pi / 2)) @ [1.0, 0.0, 0.0], [0.0, 1.0, 0.0])


def test_home_pose_is_finite_and_left_right_are_distinct():
    kinematics = MetalStationKinematics()
    pose = kinematics.pose24(np.asarray(HOME, dtype=np.float64))
    assert pose.shape == (24,)
    assert np.isfinite(pose).all()
    assert pose[1] < 0.0 < pose[4]  # left TCP at -Y, right TCP at +Y
    assert not np.allclose(pose[:6], 0.0)


def test_each_arm_moves_its_own_tcp():
    kinematics = MetalStationKinematics()
    home = np.asarray(HOME, dtype=np.float64)
    posed = home.copy()
    posed[JOINT_ORDER.index("left_joint1")] += 0.3
    before = kinematics.pose24(home)
    after = kinematics.pose24(posed)
    assert not np.allclose(before[:3], after[:3])
    assert np.allclose(before[3:6], after[3:6])
    # Rotating the shoulder yaws the whole arm, so the left TCP frame turns too.
    assert not np.allclose(before[6:15], after[6:15])
    assert np.allclose(before[15:24], after[15:24])


def test_fk_rejects_wrong_shape_and_non_finite():
    kinematics = MetalStationKinematics()
    with pytest.raises(ValueError, match="joint angles"):
        kinematics.pose24(np.zeros(11))
    bad = np.asarray(HOME, dtype=np.float64)
    bad[4] = np.nan
    with pytest.raises(ValueError, match="non-finite"):
        kinematics.pose24(bad)


# Optional: recorded station poses from the accepted checkpoint evaluation.
_EVIDENCE_DIR = os.environ.get("DIMOS_METAL_POLICY_EVIDENCE_DIR")


@pytest.mark.skipif(
    not _EVIDENCE_DIR, reason="set DIMOS_METAL_POLICY_EVIDENCE_DIR for recorded-pose parity"
)
def test_fk_matches_every_recorded_station_pose():
    roll = np.load(os.path.join(_EVIDENCE_DIR, "rollouts.npz"))
    q, actual = roll["q"], roll["actual"]
    kinematics = MetalStationKinematics()
    worst = 0.0
    for step in range(q.shape[0]):
        for case in range(q.shape[1]):
            worst = max(
                worst, float(np.abs(kinematics.pose24(q[step, case]) - actual[step, case]).max())
            )
    assert worst < 1e-5, f"station FK diverges from the recorded plant by {worst}"
