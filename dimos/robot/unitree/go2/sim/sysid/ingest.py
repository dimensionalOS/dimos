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

"""The Go2 DDS reader: MCAP through the Unitree wire types -> :class:`Streams`.

One implementation of the :class:`~dimos.robot.unitree.go2.sim.sysid
.recording.RecordingReader` seam — everything Go2 about a recording lives
HERE: the topics, the DDS message classes, the motor-order permutation, both
generations of the executor's control_log schema, and the tracker rig's
mount constants. The typed streams, the declarations and the cache are the
recording module's; a second robot brings a reader, not a fork of this file.

Decodes through the dimos Go2 wire types
(:mod:`dimos.robot.unitree.go2.dds.msgs`), so a recording and a live robot
present the same surface. The tracker is OPTIONAL: detect it and use it if
present — a bare Go2 walked around a room is a first-class input.
"""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass
import json
from pathlib import Path
from typing import Any

import numpy as np

from dimos.robot.unitree.go2.sim.plant import MUJOCO_ACTUATOR_NAMES, UNITREE_MOTOR_NAMES
from dimos.simulation.sysid.recording import Streams, read_recording

# The tracker mount for the 2026-08-16 rig (R8-SYSID), fitted as the circular
# mean of travel direction in the tracker frame under pure +vx (concentration
# 0.962, consistent to ~3 deg across four policy modes). The previous rig's
# 94.0 deg does NOT apply: this tracker is mounted essentially backwards.
# These belong to robot.json's `tracker` block; the constants are that
# recording session's values, kept as defaults so its numbers reproduce.
MOUNT_YAW_DEG = 272.92
MOUNT_FLIP = True
TRACKER_Z = 0.207  # a constant vertical offset; visually harmless guess


def mount_matrix(yaw_deg: float = MOUNT_YAW_DEG, flip: bool = MOUNT_FLIP) -> np.ndarray:
    """Base-frame vectors -> tracker frame. Columns are the robot's axes."""
    th = np.radians(yaw_deg)
    bx = np.array([np.cos(th), np.sin(th), 0.0])
    bz = np.array([0.0, 0.0, -1.0 if flip else 1.0])
    return np.stack([bx, np.cross(bz, bx), bz], 1)


def command_coverage(cmd_t0: float, cmd_t1: float, n_cmd: int, span_s: float) -> float:
    """Fraction of the measured span a command stream covers.

    The command source is chosen by TIME COVERAGE, not presence: a 60 s sport
    recording can carry an 850-message stub of ``policy/lowcmd`` beside 23520
    on ``rt/lowcmd``, and picking the stub silently replays two seconds of a
    sixty-second file and reports a superb score for it.
    """
    if n_cmd < 2:
        return 0.0
    return (cmd_t1 - cmd_t0) / (span_s or 1.0)


def _velocity_command(d: Mapping[str, Any]) -> tuple[float, float, float] | None:
    """The operator's vx/vy/vyaw from either executor schema, else ``None``.

    The executor renamed this between the 2026-08-16 and 2026-08-17 sessions::

        old  {"action": "walk",           "vx": …, "vy": …, "vyaw": …}
        new  {"type":   "velocity_input", "lx": …, "ly": …, "az": …}

    Both carry stick values, not SI rates. Reading only the old spelling
    produced an EMPTY schedule for every 08-17 recording, and an empty schedule
    is indistinguishable downstream from "the robot was never asked to move":
    :func:`~...ground.cmd_at` returns zeros, the ``moving`` mask is empty, and
    ``speed`` reads 0.0 — a silent wrong answer rather than a failure.
    """
    if d.get("action") == "walk":
        return float(d.get("vx", 0.0)), float(d.get("vy", 0.0)), float(d.get("vyaw", 0.0))
    if d.get("type") == "velocity_input":
        return float(d.get("lx", 0.0)), float(d.get("ly", 0.0)), float(d.get("az", 0.0))
    return None


# The parse version half of the cache tag: bumped whenever the cached field
# set OR the parse changes — a stale cache would otherwise hide this file's
# fixes behind a v2 npz. v3 = velocity_input.
_CACHE_VERSION = 3


@dataclass(frozen=True)
class Go2DdsReader:
    """The Go2 recording reader. Stateless and picklable — see the protocol."""

    @property
    def cache_tag(self) -> str:
        return f"v{_CACHE_VERSION}"

    def read(self, path: Path) -> Streams:
        return _read_streams_uncached(path)


GO2_READER = Go2DdsReader()


def read_streams(path: str | Path, *, cache: bool = True) -> Streams:
    """Read lowstate / lowcmd / tracker / control_log out of one Go2 MCAP.

    The Go2 convenience over :func:`~dimos.robot.unitree.go2.sim.sysid
    .recording.read_recording`.

    COMMAND SOURCE. ``policy/lowcmd`` is our own executor's output and drives
    when it covers >= 50% of the measured span; otherwise ``rt/lowcmd``, the
    DDS channel the motors actually listen to (see :func:`command_coverage`).

    EPOCH. Times are rebased on the first operator velocity command in
    control_log — either executor spelling, see :func:`_velocity_command`. A
    sport-only recording has none, so its first command stands in — readable,
    but the epoch is comparable within the file only.
    """
    return read_recording(path, GO2_READER, cache=cache)


def _read_streams_uncached(path: Path) -> Streams:
    from mcap.reader import make_reader

    from dimos.robot.unitree.go2.dds import cdr
    from dimos.robot.unitree.go2.dds.msgs.LowCmd import LowCmd
    from dimos.robot.unitree.go2.dds.msgs.LowState import LowState

    perm = [UNITREE_MOTOR_NAMES.index(n) for n in MUJOCO_ACTUATOR_NAMES]
    low: list[list[float]] = []
    cmds: dict[str, list[list[float]]] = {"policy/lowcmd": [], "rt/lowcmd": []}
    with path.open("rb") as f:
        for _s, ch, msg in make_reader(f).iter_messages(
            topics=["rt/lowstate", "policy/lowcmd", "rt/lowcmd"]
        ):
            if ch.topic == "rt/lowstate":
                ls, end = cdr.decode(msg.data, LowState)
                assert end == len(msg.data), f"LowState: {end} != {len(msg.data)}"
                ms = ls.motor_state[:12]
                low.append(
                    [
                        msg.log_time / 1e9,
                        float(ls.tick),
                        *[ms[i].q for i in perm],
                        *[ms[i].dq for i in perm],
                        *[ms[i].tau_est for i in perm],
                        *ls.imu_state.quaternion,
                        *ls.imu_state.gyroscope,
                        *ls.imu_state.accelerometer,
                    ]
                )
            else:
                lc, end = cdr.decode(msg.data, LowCmd)
                assert end == len(msg.data), f"LowCmd: {end} != {len(msg.data)}"
                mc = lc.motor_cmd[:12]
                cmds[ch.topic].append(
                    [
                        msg.log_time / 1e9,
                        *[mc[i].q for i in perm],
                        *[mc[i].kp for i in perm],
                        *[mc[i].kd for i in perm],
                        *[mc[i].tau for i in perm],
                        *[mc[i].dq for i in perm],
                    ]
                )
    span = (low[-1][0] - low[0][0]) if low else 1.0
    ours = cmds["policy/lowcmd"]
    covers = command_coverage(ours[0][0], ours[-1][0], len(ours), span) if ours else 0.0
    cmd = ours if covers >= 0.5 else cmds["rt/lowcmd"]
    if not cmd:
        cmd = ours

    viv: list[list[float]] = []
    seg: list[tuple[float, str]] = []
    walk: list[list[float]] = []
    gait: list[list[float]] = []
    first_walk: float | None = None
    with path.open("rb") as f:
        for _s, ch, msg in make_reader(f).iter_messages(
            topics=["vive/pose", "control_log", "policy/state"]
        ):
            t = msg.log_time / 1e9
            d = json.loads(msg.data)
            if ch.topic == "vive/pose":
                viv.append(
                    [
                        float(d.get("t_host", t)),
                        t,
                        *[float(x) for x in d["p"]],
                        *[float(x) for x in d["q"]],
                    ]
                )
            elif ch.topic == "policy/state":
                if d.get("mode") and d["mode"] != "climb_engage":
                    seg.append((t, str(d["mode"])))
            elif (vel := _velocity_command(d)) is not None:
                first_walk = t if first_walk is None else min(first_walk, t)
                walk.append([t, *vel])
            elif d.get("action") == "gait_height":
                gait.append([t, float(d["gh"])])
    if not low or not cmd:
        raise ValueError(f"{path}: needs rt/lowstate and one of policy/lowcmd, rt/lowcmd")
    if first_walk is None:
        first_walk = float(cmd[0][0])

    L = np.array(low)
    C = np.array(cmd)
    # rt/lowstate arrives BATCHED — its log_time stamps a whole batch at
    # arrival (median batch 3.5 messages / 4.1 ms). The robot's own `tick` is
    # the true 500 Hz clock; one linear fit puts it on the host clock.
    tk = L[:, 1] - L[0, 1]
    unit = (L[-1, 0] - L[0, 0]) / (tk[-1] - tk[0])
    a, b = np.polyfit(tk * unit, L[:, 0], 1)
    st = Streams(
        lt=a * (tk * unit) + b - first_walk,
        lq=L[:, 2:14],
        ldq=L[:, 14:26],
        ltau=L[:, 26:38],
        lquat=L[:, 38:42],
        lgyro=L[:, 42:45],
        lacc=L[:, 45:48],
        ct=C[:, 0] - first_walk,
        cq=C[:, 1:13],
        ckp=C[:, 13:25],
        ckd=C[:, 25:37],
        ctau=C[:, 37:49],
        cdq=C[:, 49:61],
        seg_t=np.array([s[0] - first_walk for s in seg]) if seg else np.array([0.0]),
        seg_mode=tuple(s[1] for s in seg) or ("unknown",),
        wt=(np.array([w[0] for w in walk]) - first_walk) if walk else np.zeros(0),
        wcmd=np.array([w[1:] for w in walk]) if walk else np.zeros((0, 3)),
        ght=(np.array([g[0] for g in gait]) - first_walk) if gait else np.zeros(0),
        gh=np.array([g[1] for g in gait]) if gait else np.zeros(0),
    )
    if viv:
        V = np.array(viv)
        V = V[np.argsort(V[:, 0])]
        # The payload t_host carries near-duplicate clusters (about half the
        # samples land within 1 ms of their neighbour), so anything that
        # differentiates raw t_host explodes. Resample to a uniform grid.
        t_host = V[:, 0] + float(np.mean(V[:, 1] - V[:, 0])) - first_walk
        grid = np.arange(t_host[0], t_host[-1], 1.0 / 200.0)
        st.vt = grid
        st.vp = np.stack([np.interp(grid, t_host, V[:, 2 + i]) for i in range(3)], 1)
        q = np.stack([np.interp(grid, t_host, V[:, 5 + i]) for i in range(4)], 1)
        st.vq = q / np.linalg.norm(q, axis=1, keepdims=True)
    return st
