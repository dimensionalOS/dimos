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

"""Loop 2's real side: the instrument split, on recordings with known answers."""

from __future__ import annotations

import numpy as np
import pytest

from dimos.robot.unitree.go2.sim.sysid.ingest import mount_matrix
from dimos.robot.unitree.go2.sim.sysid.real import cmd_at, real_summary
from dimos.simulation.sysid.recording import Streams
from dimos.simulation.sysid.rotations import mat_to_quat, quat_to_mat


def _streams(
    wt: np.ndarray,
    wcmd: np.ndarray,
    *,
    lt: np.ndarray | None = None,
    lquat: np.ndarray | None = None,
    vt: np.ndarray | None = None,
    vp: np.ndarray | None = None,
    vq: np.ndarray | None = None,
) -> Streams:
    z = np.zeros(0)
    lt = z if lt is None else lt
    n = len(lt)
    st = Streams(
        lt=lt,
        lq=np.zeros((n, 12)),
        ldq=np.zeros((n, 12)),
        ltau=np.zeros((n, 12)),
        lquat=np.tile([1.0, 0.0, 0.0, 0.0], (n, 1)) if lquat is None else lquat,
        lgyro=np.zeros((n, 3)),
        lacc=np.zeros((n, 3)),
        ct=z,
        cq=np.zeros((0, 12)),
        ckp=np.zeros((0, 12)),
        ckd=np.zeros((0, 12)),
        ctau=np.zeros((0, 12)),
        cdq=np.zeros((0, 12)),
        wt=wt,
        wcmd=wcmd,
    )
    if vt is not None:
        st.vt, st.vp, st.vq = vt, vp if vp is not None else np.zeros((len(vt), 3)), vq
    return st


def test_cmd_at_holds_zero_order():
    st = _streams(np.array([0.0, 1.0, 2.0]), np.array([[0.1, 0, 0], [0.5, 0, 0], [0.0, 0, 0]]))
    got = cmd_at(st, np.array([-0.5, 0.5, 1.0, 1.9, 5.0]))
    assert got[:, 0] == pytest.approx([0.1, 0.1, 0.5, 0.5, 0.0])


def _roll_quat(phi: np.ndarray) -> np.ndarray:
    return np.stack([np.cos(phi / 2), np.sin(phi / 2), np.zeros_like(phi), np.zeros_like(phi)], 1)


def _split_streams(imu_amp: float, tracker_amp: float) -> Streams:
    """Tracker walking forward with a flexing mount; IMU with the true wobble.

    ``vq`` is built so the BASE attitude the mount matrix recovers is exactly
    the intended upright roll wobble — the flex lives in the tracker's story
    of the base, which is what the split must refuse to read.
    """
    lt = np.arange(0, 20, 0.002)  # 500 Hz IMU
    vt = np.arange(0, 20, 0.005)  # 200 Hz tracker
    vp = np.stack([0.5 * vt, np.zeros_like(vt), np.full_like(vt, 1.0)], axis=1)
    base_r = quat_to_mat(_roll_quat(tracker_amp * np.sin(2 * np.pi * 2.0 * vt)))
    return _streams(
        np.array([0.0]),
        np.array([[0.5, 0.0, 0.0]]),
        lt=lt,
        lquat=_roll_quat(imu_amp * np.sin(2 * np.pi * 2.0 * lt)),
        vt=vt,
        vp=vp,
        vq=np.stack([mat_to_quat(r @ mount_matrix()) for r in base_r]),
    )


def test_real_summary_reads_attitude_from_the_imu_not_the_tracker():
    """The 5e instrument split: the tracker's flexing mount invents rotation,
    so oscillation statistics must come from the rigidly-mounted IMU while
    position stays with the tracker."""
    st = _split_streams(imu_amp=0.02, tracker_amp=0.10)
    s = real_summary(st, start=0.0, seconds=20.0)
    assert s.source == "pos:tracker att:imu"
    assert s.speed == pytest.approx(0.5, abs=0.03)  # position: still the tracker
    assert s.roll_std == pytest.approx(0.02 / np.sqrt(2), rel=0.15)
    # the retracted instrument stays available for diagnosis, clearly labelled
    old = real_summary(st, start=0.0, seconds=20.0, attitude="tracker")
    assert old.source == "pos:tracker att:tracker"
    assert old.tilt_p99 > 3 * s.tilt_p99


def test_real_summary_without_a_tracker_scores_attitude_only():
    lt = np.arange(0, 20, 0.002)
    st = _streams(
        np.array([0.0]),
        np.array([[0.5, 0.0, 0.0]]),
        lt=lt,
        lquat=_roll_quat(0.02 * np.sin(2 * np.pi * 2.0 * lt)),
    )
    s = real_summary(st, start=0.0, seconds=20.0)
    assert s.source == "att:imu (no tracker)"
    assert np.isnan(s.speed) and np.isnan(s.gait_hz)
    assert s.roll_std == pytest.approx(0.02 / np.sqrt(2), rel=0.15)
    with pytest.raises(ValueError, match="no tracker"):
        real_summary(st, start=0.0, seconds=20.0, attitude="tracker")
