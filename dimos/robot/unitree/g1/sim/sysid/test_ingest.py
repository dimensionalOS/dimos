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

"""The G1 reader: the frame conversions it owns, and the recording it was written for."""

from pathlib import Path

import numpy as np
import pytest

from dimos.control.tasks.g1_groot_wbc_task.g1_groot_wbc_task import G1_GROOT_KD, G1_GROOT_KP
from dimos.robot.unitree.g1.sim.sysid.ingest import (
    _interp_quat,
    _quat_mul,
    read_streams,
    segments_from_commands,
    waist_rotation,
    world_T_pelvis,
)
from dimos.simulation.sysid.rotations import mat_to_quat, quat_to_mat

RECORDING = Path("data/g1_groot_characterization_2026-08-27.db")


def test_the_batched_waist_rotation_is_the_tf_publishers():
    """The reader's vectorised chain must be base_to_torso's, or the IMU lands
    on the pelvis through a different waist than the tf tree uses."""
    from dimos.robot.unitree.g1.g1_tf_publisher import base_to_torso

    rng = np.random.default_rng(0)
    for yaw, roll, pitch in rng.uniform(-0.5, 0.5, size=(20, 3)):
        want = np.asarray(base_to_torso(yaw, roll, pitch).to_matrix())[:3, :3]
        got = waist_rotation(np.array([yaw]), np.array([roll]), np.array([pitch]))[0]
        assert np.allclose(got, want, atol=1e-12)


def test_quaternion_helpers_agree_with_the_matrices():
    rng = np.random.default_rng(1)
    q = rng.normal(size=(5, 4))
    q /= np.linalg.norm(q, axis=1, keepdims=True)
    r = quat_to_mat(q)
    prod = _quat_mul(q[:1].repeat(5, 0), q)
    for i in range(5):
        want = r[0] @ r[i]
        got = quat_to_mat(prod[i : i + 1])[0]
        assert np.allclose(got, want, atol=1e-12)
    mid = _interp_quat(np.array([0.5]), np.array([0.0, 1.0]), np.stack([q[0], -q[0]]))
    assert np.allclose(np.abs(mid[0] @ q[0]), 1.0), "a sign flip must not average to zero"
    assert np.allclose(mat_to_quat(quat_to_mat(q[:1])[0]) @ q[0], 1.0) or np.allclose(
        mat_to_quat(quat_to_mat(q[:1])[0]) @ q[0], -1.0
    )


def test_segments_follow_the_policy_switch_and_drop_blips():
    t = np.arange(0.0, 10.0, 0.05)
    cmd = np.zeros((len(t), 3))
    cmd[(t >= 2.0) & (t < 6.0), 0] = 0.3  # a 4 s walk
    cmd[(t >= 7.0) & (t < 7.2), 0] = 0.3  # a 0.2 s blip: too short to be a segment
    seg_t, seg_mode = segments_from_commands(t, cmd, -1.0)
    assert seg_mode == ("stand", "walk", "stand")
    assert np.allclose(seg_t, [-1.0, 2.0, 6.0])


def test_world_T_pelvis_is_rigid():
    class Odom:
        x, y, z = 1.0, 2.0, 3.0
        orientation = type("Q", (), {"to_rotation_matrix": staticmethod(lambda: np.eye(3))})()

    for waist in [(0.0, 0.0, 0.0), (0.1, -0.05, 0.2)]:
        T = world_T_pelvis(Odom(), waist)
        assert np.allclose(T[:3, :3] @ T[:3, :3].T, np.eye(3), atol=1e-12)
        assert np.isclose(np.linalg.det(T[:3, :3]), 1.0)
        assert T[3, 3] == 1.0


@pytest.mark.self_hosted
@pytest.mark.skipif(
    not RECORDING.is_file() or RECORDING.stat().st_size < 1_000_000,
    reason="the LFS recording is not materialised",
)
def test_the_recording_reads_as_declared():
    st = read_streams(RECORDING)
    n = len(st.lt)
    assert st.lq.shape == st.ldq.shape == st.ltau.shape == (n, 29)
    assert st.lquat.shape == (n, 4) and st.lgyro.shape == st.lacc.shape == (n, 3)
    assert 480 < 1.0 / np.median(np.diff(st.lt)) < 510
    assert 45 < 1.0 / np.median(np.diff(st.ct)) < 55, "distinct targets arrive at the policy rate"
    assert np.allclose(st.ckp[0], G1_GROOT_KP) and np.allclose(st.ckd[0], G1_GROOT_KD)
    assert not st.cdq.any() and not st.ctau.any()
    assert st.lt[0] < 0.0 < st.lt[-1], "the epoch is the first velocity command"
    assert "walk" in st.seg_mode and "stand" in st.seg_mode
    assert 9.0 < np.median(np.linalg.norm(st.lacc, axis=1)) < 10.6
    assert st.has_markers and np.isclose(1.0 / np.median(np.diff(st.vt)), 100.0)
    assert np.allclose(np.linalg.norm(st.lquat, axis=1), 1.0)
    from dimos.simulation.sysid.regimes import propose_suspended

    assert not propose_suspended(st)
