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

"""Recorded Vive tracker poses, expressed as a base_link trajectory.

Two conventions were read off the data rather than assumed (see
``test_vive.py`` for the checks that pin them):

* the quaternion is **wxyz** — under that reading the tracker's z axis holds
  0.997 +/- 0.003 alignment with world z across a run, i.e. the robot stays
  upright; xyzw gives 0.725 +/- 0.279, which is nonsense for a walking robot
* the frame is **z-up** — over a 55 s walk the z range is 0.09 m against
  1.0 and 1.6 m in x and y

The target frame is settled. The official URDF (unitree_ros
``go2_description``) has a single root link named **`base`** — there is no
`base_link` and no separate `trunk` — whose collision box is
``0.3762 x 0.0935 x 0.114`` at origin ``0 0 0``. So the frame sits at the
*geometric centre of the trunk*, level with the hip joints
(``FL_hip_joint`` is at ``z = 0``). menagerie's MJCF inherits that body
one-to-one, so MuJoCo's ``qpos[0:3]`` is exactly the URDF ``base`` origin --
nothing to map between them.

What is *not* known is where the tracker sits on the robot. The trunk is
0.114 m tall, so its top surface is 0.057 m above the frame:

    tracker 15 cm above the trunk *top*     ->  0.057 + 0.15 = 0.207 m
    tracker 15 cm above the trunk *centre*  ->  0.15 m

The **sign is positive** because the tracker is mounted inverted: its z axis
points down, ``R[2,2] = -0.997`` on himloco01 and ``-0.996`` on v11 (the robot
is upright the whole time, so this is the mounting, not the motion). The offset
is expressed in tracker coordinates and rotated by the tracker's orientation,
so "below in world" is "+z in tracker frame".

:data:`DEFAULT_TRACKER_OFFSET` takes the first reading, 0.207 m. Correct it by
eye against the ghost; it is not a measurement, and the in-plane (x, y)
mounting position is not modelled at all.

The vertical offset cannot be recovered from these recordings. The Vive frame's
origin is the room calibration, not the floor (mean tracker z is 0.241 m on one
run and 0.180 m on the other, for the same robot), so it cannot be compared
against the simulated base height either. Measure it, or fit it against a run
with deliberate pitch/roll -- the lever arm is only weakly observable here,
with mean body tilt of 3.7 and 4.4 degrees.
"""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np

# `base` expressed in the tracker's frame. Positive z because the tracker is
# mounted inverted; see the module docstring.
DEFAULT_TRACKER_OFFSET = np.array([0.0, 0.0, 0.207])

# Yaw of the robot's forward axis within the tracker's xy plane, degrees.
# Fitted on himloco01, sim-free, two ways that agree: the circular mean of the
# travel direction under a pure +vx command gives +93.6 deg (concentration
# 0.88, n=2347), and maximizing cos(body velocity, commanded direction) over
# yaw gives +94.0 deg at 0.840 with a clean unimodal curve.
#
# Do NOT fit this against the simulator. The policy rollout diverges from the
# real robot within a second or two, so a sim-vs-ghost displacement score is
# dominated by that divergence: swept over yaw it is nearly flat (0.79-1.09 m
# on 2 s windows, against ~1 m of travel) and its argmin lands at 285 deg,
# roughly 180 deg wrong. The mount is a property of the recording and has to
# be fitted from the recording alone.
DEFAULT_MOUNT_YAW_DEG = 94.0


def mount_rotation(yaw_deg: float = DEFAULT_MOUNT_YAW_DEG, flip: bool = True) -> np.ndarray:
    """Rotation taking base-frame vectors into the tracker frame.

    Columns are the robot's axes expressed in tracker coordinates. ``flip``
    encodes the tracker being mounted upside down (its z points at the floor),
    which is what the recordings show: ``R[2, 2]`` is -0.997 and -0.996.
    """
    th = np.radians(yaw_deg)
    up = -1.0 if flip else 1.0
    bx = np.array([np.cos(th), np.sin(th), 0.0])
    bz = np.array([0.0, 0.0, up])
    by = np.cross(bz, bx)
    return np.column_stack([bx, by, bz])


def quat_to_mat(q: np.ndarray) -> np.ndarray:
    """(..., 4) wxyz -> (..., 3, 3)."""
    q = np.asarray(q, float)
    q = q / np.linalg.norm(q, axis=-1, keepdims=True)
    w, x, y, z = q[..., 0], q[..., 1], q[..., 2], q[..., 3]
    m = np.empty((*q.shape[:-1], 3, 3))
    m[..., 0, 0] = 1 - 2 * (y * y + z * z)
    m[..., 0, 1] = 2 * (x * y - w * z)
    m[..., 0, 2] = 2 * (x * z + w * y)
    m[..., 1, 0] = 2 * (x * y + w * z)
    m[..., 1, 1] = 1 - 2 * (x * x + z * z)
    m[..., 1, 2] = 2 * (y * z - w * x)
    m[..., 2, 0] = 2 * (x * z - w * y)
    m[..., 2, 1] = 2 * (y * z + w * x)
    m[..., 2, 2] = 1 - 2 * (x * x + y * y)
    return m


def mat_to_quat(m: np.ndarray) -> np.ndarray:
    """(3, 3) -> (4,) wxyz."""
    t = np.trace(m)
    if t > 0:
        s = np.sqrt(t + 1.0) * 2
        return np.array(
            [0.25 * s, (m[2, 1] - m[1, 2]) / s, (m[0, 2] - m[2, 0]) / s, (m[1, 0] - m[0, 1]) / s]
        )
    i = int(np.argmax(np.diag(m)))
    j, k = (i + 1) % 3, (i + 2) % 3
    s = np.sqrt(m[i, i] - m[j, j] - m[k, k] + 1.0) * 2
    q = np.empty(4)
    q[0] = (m[k, j] - m[j, k]) / s
    q[i + 1] = 0.25 * s
    q[j + 1] = (m[j, i] + m[i, j]) / s
    q[k + 1] = (m[k, i] + m[i, k]) / s
    return q


def read_vive_pose(dataset: str | Path) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Return ``(t, pos, quat)`` — (n, 3), (n, 4) wxyz, seconds on a shared epoch.

    t=0 is the recording's **first walk command**, the same zero
    :func:`walk.read_control_log` uses. Zeroing each stream at its own first
    message instead put himloco01's vive samples 0.313 s early and v11's 4.4 s
    early — how long the tracker streamed before the operator pressed walk —
    and that bookkeeping offset read as "command delay": a search recovered
    0.317 s for it, within 4 ms of the artifact. Samples before the first
    command carry negative times.

    Sample *spacing* still comes from the payload ``t_host``: monotonic, worst
    gap 15 ms against 92 ms for log_time (the payload's own "ts" goes backwards
    at 91 points in himloco01, so it is unusable raw). t_host and log_time are
    the same clock to 2.5 ms +/- 3.4 ms with no drift over a run, so the mean
    difference rebases t_host onto the log clock the commands are stamped in.
    """
    from mcap.reader import make_reader

    ts: list[float] = []
    deltas: list[float] = []
    pos: list[list[float]] = []
    quat: list[list[float]] = []
    first_walk: float | None = None
    with Path(dataset).open("rb") as f:
        for _schema, channel, msg in make_reader(f).iter_messages(
            topics=["vive/pose", "control_log"]
        ):
            d = json.loads(msg.data)
            if channel.topic == "control_log":
                if d.get("action") == "walk":
                    t_cmd = msg.log_time / 1e9
                    first_walk = t_cmd if first_walk is None else min(first_walk, t_cmd)
                continue
            t_host = d.get("t_host", msg.log_time / 1e9)
            ts.append(t_host)
            deltas.append(msg.log_time / 1e9 - t_host)
            pos.append(d["p"])
            quat.append(d["q"])
    if not ts:
        raise ValueError(f"{dataset}: no vive/pose messages")
    t = np.array(ts) + float(np.mean(deltas))  # onto the log clock
    zero = t[0] if first_walk is None else first_walk
    return t - zero, np.array(pos), np.array(quat)


def base_track(
    dataset: str | Path,
    *,
    tracker_offset: np.ndarray | None = None,
    mount: np.ndarray | None = None,
    anchor_at: float = 0.0,
    anchor_pos: np.ndarray | None = None,
    anchor_quat: np.ndarray | None = None,
    sensor_z: bool = False,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Recorded base_link pose, re-anchored so t=0 sits at the sim's start pose.

    Anchoring is what makes this comparable without knowing the Vive room's
    extrinsics: the unknown constant rotation and origin cancel, leaving the
    motion *relative to the anchor sample*. The tracker offset does not
    cancel — it is a lever arm, so it still shows up as soon as the body
    rotates.

    ``anchor_at`` picks the anchor time. Use it to skip a stand-up at the
    start of a run: the simulator always begins standing, so comparing from
    t=0 lines a standing robot up against one still getting to its feet.

    ``sensor_z`` replaces the z column with the **raw tracker height** in the
    room frame. The lever-arm correction moves the guessed tracker offset into
    the "ground truth": on himloco01 it contributes 11.4 mm of z std against
    5.6 mm from the tracker itself, so height statistics computed from the
    corrected z mostly measure the guess. Score height against a virtual
    tracker synthesized on the sim side instead (see ``evaluate``), so the
    same guess distorts both sides identically and cancels.
    """
    off = DEFAULT_TRACKER_OFFSET if tracker_offset is None else np.asarray(tracker_offset, float)
    mnt = mount_rotation() if mount is None else np.asarray(mount, float)
    t, p, q = read_vive_pose(dataset)
    rot = quat_to_mat(q)

    # tracker -> base, in the Vive frame
    base_p = p + np.einsum("nij,j->ni", rot, off)
    base_r = np.einsum("nij,jk->nik", rot, mnt)

    # re-express relative to the anchor sample
    a = min(int(np.searchsorted(t, anchor_at)), len(t) - 1) if anchor_at else 0
    r0t = base_r[a].T
    rel_p = np.einsum("ij,nj->ni", r0t, base_p - base_p[a])
    rel_r = np.einsum("ij,njk->nik", r0t, base_r)

    if anchor_quat is not None:
        anchor_rot = quat_to_mat(np.asarray(anchor_quat, float))
        rel_p = np.einsum("ij,nj->ni", anchor_rot, rel_p)
        rel_r = np.einsum("ij,njk->nik", anchor_rot, rel_r)
    if anchor_pos is not None:
        rel_p = rel_p + np.asarray(anchor_pos, float)
    if sensor_z:
        rel_p[:, 2] = p[:, 2]

    return t, rel_p, np.array([mat_to_quat(r) for r in rel_r])
