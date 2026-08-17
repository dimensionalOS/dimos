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

"""Loop 2's REAL side: statistics read from a recording. No engine, ever.

The other half of the grounding — the sim side, the closed-loop rollout and
the chaos floor — lives in :mod:`~dimos.robot.unitree.go2.sim.sysid.ground`
and is engine-coupled. This module is pure recording processing: it must
stay importable, testable and correct with no simulator installed at all,
which is also what keeps it honest — nothing here can leak a candidate
plant's behaviour into the real side of a comparison.
"""

from __future__ import annotations

from collections.abc import Sequence
from dataclasses import replace

import numpy as np

from dimos.robot.unitree.go2.sim.rotations import mat_to_quat
from dimos.robot.unitree.go2.sim.sysid.gait import real_strides
from dimos.robot.unitree.go2.sim.sysid.ingest import TRACKER_Z, mount_matrix
from dimos.robot.unitree.go2.sim.sysid.recording import Streams
from dimos.robot.unitree.go2.sim.sysid.stats import Summary, spread_of, summarize


def cmd_at(st: Streams, t_abs: np.ndarray) -> np.ndarray:
    """The operator command in force at each absolute recording time."""
    if len(st.wt) == 0:
        return np.zeros((len(t_abs), 3))
    idx = np.clip(np.searchsorted(st.wt, t_abs, "right") - 1, 0, len(st.wt) - 1)
    return st.wcmd[idx]


def real_summary(st: Streams, *, start: float, seconds: float, attitude: str = "imu") -> Summary:
    """The recording's statistics: POSITION from the tracker, ATTITUDE from the IMU.

    Split by what each instrument is actually good at (README 6). The
    tracker is precise on position and unusable for attitude: its flexing
    mount invents 2.47x the roll rate at correlation 0.44, growing 34x with
    activity, and no constant calibration can remove a time-varying error.
    The IMU is rigidly mounted, and its quaternion reproduces the raw gyro at
    correlation 0.992-1.000 — so the quaternion (angles directly, no
    integration drift to manage) over integrating the gyro is a readability
    choice, not a numerical one.

    ``attitude="tracker"`` keeps the retracted instrument for diagnosis and
    for reproducing pre-split numbers; it is not for claims. A tracker-less
    recording scores attitude only: position statistics are NaN and drop out
    of the SNR. The ``source`` field on the result says which instruments
    spoke — a claim whose instrument changed is a new claim.
    """
    if attitude not in ("imu", "tracker"):
        raise ValueError(f"attitude must be 'imu' or 'tracker', got {attitude!r}")
    isel = (st.lt >= start) & (st.lt < start + seconds)
    g = real_strides(st, start=start, seconds=seconds)
    if not st.has_markers:
        if attitude == "tracker":
            raise ValueError("recording has no tracker: there is no tracker attitude to read")
        t_att = st.lt[isel] - start
        s = summarize(t_att, None, st.lquat[isel], cmd_at(st, st.lt[isel]))
        return replace(
            s, stride_hz=g.stride_hz, stride_len=g.stride_len, source="att:imu (no tracker)"
        )
    base_p, base_r = st.base_pose_room(mount_matrix(), TRACKER_Z)
    sel = (st.vt >= start) & (st.vt < start + seconds)
    p = base_p[sel].copy()
    p[:, 2] = st.vp[sel][:, 2]  # sensor-space height; see ground.sim_summary
    if attitude == "imu":
        t_att, quat = st.lt[isel] - start, st.lquat[isel]
    else:
        t_att = st.vt[sel] - start
        quat = np.stack([mat_to_quat(r) for r in base_r[sel]])
    s = summarize(st.vt[sel] - start, p, quat, cmd_at(st, st.vt[sel]), t_att=t_att)
    return replace(
        s, stride_hz=g.stride_hz, stride_len=g.stride_len, source=f"pos:tracker att:{attitude}"
    )


def robot_noise(
    recordings: Sequence[Streams],
    *,
    start: float = 6.0,
    seconds: float | None = None,
) -> dict[str, float]:
    """Noise floor measured as the ROBOT against itself: repeat recordings.

    The better yardstick — it carries battery sag and motor temperature, not
    just chaos — and the one the publishable claim is expressed against.
    Needs two or more recordings of the same walk; the spread of their
    statistics IS the floor. Keep the recording being grounded OUT of the
    floor set, so the floor and the verdict are measured on different data.
    """
    if len(recordings) < 2:
        raise ValueError("robot_noise needs at least two recordings of the same walk")
    sums = []
    for st in recordings:
        span = float(st.wt[-1]) - start
        sums.append(real_summary(st, start=start, seconds=span if seconds is None else seconds))
    return spread_of(sums)
