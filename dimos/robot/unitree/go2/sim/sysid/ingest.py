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

"""MCAP -> one typed :class:`Streams` object. One reader, cached.

Decodes through the dimos Go2 wire types
(:mod:`dimos.robot.unitree.go2.dds.msgs`), so a recording and a live robot
present the same surface. The tracker is OPTIONAL: detect it and use it if
present — a bare Go2 walked around a room is a first-class input.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import json
from pathlib import Path

import numpy as np

from dimos.robot.unitree.go2.sim.plant import MUJOCO_ACTUATOR_NAMES, UNITREE_MOTOR_NAMES
from dimos.robot.unitree.go2.sim.rotations import quat_to_mat

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


@dataclass
class Streams:
    """Every recorded signal the pipeline needs, on the first-walk-command epoch."""

    lt: np.ndarray  # lowstate time, s (robot `tick` clock, rebased)
    lq: np.ndarray  # (n,12) measured joint angles, MuJoCo actuator order
    ldq: np.ndarray  # (n,12) measured joint speeds
    ltau: np.ndarray  # (n,12) motor-side torque estimate
    lquat: np.ndarray  # (n,4) IMU attitude, wxyz
    lgyro: np.ndarray  # (n,3) body angular rate
    lacc: np.ndarray  # (n,3) body specific force — ~0 in free fall
    ct: np.ndarray  # command time, s
    cq: np.ndarray  # (m,12) COMMANDED joint targets — the replay input
    ckp: np.ndarray
    ckd: np.ndarray
    ctau: np.ndarray  # feed-forward torque (zero from our own executor)
    cdq: np.ndarray  # (m,12) COMMANDED joint speeds; nonzero from Unitree's builtins
    vt: np.ndarray = field(default_factory=lambda: np.zeros(0))  # tracker time, s
    vp: np.ndarray = field(default_factory=lambda: np.zeros((0, 3)))
    vq: np.ndarray = field(default_factory=lambda: np.zeros((0, 4)))
    seg_t: np.ndarray = field(default_factory=lambda: np.zeros(0))
    seg_mode: tuple[str, ...] = ()
    # Operator command schedule (control_log "walk" actions): what drives the
    # policy closed loop in Mode B, and the cmd axis of loop 2's statistics.
    wt: np.ndarray = field(default_factory=lambda: np.zeros(0))  # walk cmd time, s
    wcmd: np.ndarray = field(default_factory=lambda: np.zeros((0, 3)))  # vx, vy, vyaw
    # Gait-height slider ("gait_height" actions); empty when never touched.
    ght: np.ndarray = field(default_factory=lambda: np.zeros(0))
    gh: np.ndarray = field(default_factory=lambda: np.zeros(0))

    @property
    def has_markers(self) -> bool:
        return len(self.vt) > 0

    def segments(self) -> list[tuple[int, str, float, float]]:
        """``(index, policy mode, t_start, t_end)`` for each policy span."""
        end = float(self.ct[-1])
        out = []
        for i, mode in enumerate(self.seg_mode):
            a = float(self.seg_t[i])
            b = float(self.seg_t[i + 1]) if i + 1 < len(self.seg_t) else end
            out.append((i, mode, max(a, float(self.ct[0])), min(b, end)))
        return out

    def base_pose_room(self, mount: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """Measured base position and rotation in the ROOM frame, from the tracker."""
        rot = quat_to_mat(self.vq)  # tracker -> room
        base_r = np.einsum("nij,jk->nik", rot, mount.T)  # base -> room
        base_p = self.vp - base_r @ np.array([0.0, 0.0, TRACKER_Z])
        return base_p, base_r


@dataclass(frozen=True)
class Declarations:
    """The two things no signal can reveal; everything else is measured.

    A hanging robot reads ~1 g with unloaded legs — and so does one lying
    down: suspension is ambiguous in principle. Floor material is in no
    recorded signal at all. So exactly these two are DECLARED, in order of
    authority: an MCAP file-level ``go2sim`` metadata record written at
    capture time, else a ``<recording>.meta.json`` sidecar. A detector may
    cross-check a declaration; it never decides.
    """

    suspended: bool | None = None
    floor: str | None = None


def sidecar_path(recording: str | Path) -> Path:
    return Path(recording).with_suffix(".meta.json")


def read_declarations(path: str | Path) -> Declarations:
    path = Path(path)
    from mcap.reader import make_reader

    with path.open("rb") as f:
        for md in make_reader(f).iter_metadata():
            if md.name != "go2sim":
                continue
            kv = dict(md.metadata)
            return Declarations(
                suspended=json.loads(kv["suspended"]) if "suspended" in kv else None,
                floor=kv.get("floor"),
            )
    side = sidecar_path(path)
    if side.is_file():
        d = json.loads(side.read_text())
        return Declarations(suspended=d.get("suspended"), floor=d.get("floor"))
    return Declarations()


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


# Bumped whenever the cached field set changes.
_CACHE_VERSION = 2


def _cache_path(path: Path) -> Path:
    d = Path.home() / ".cache" / "dimos_go2sim"
    d.mkdir(parents=True, exist_ok=True)
    stat = path.stat()
    return d / f"{path.stem}.v{_CACHE_VERSION}.{int(stat.st_mtime)}.{stat.st_size}.npz"


def read_streams(path: str | Path, *, cache: bool = True) -> Streams:
    """Read lowstate / lowcmd / tracker / control_log out of one MCAP.

    COMMAND SOURCE. ``policy/lowcmd`` is our own executor's output and drives
    when it covers >= 50% of the measured span; otherwise ``rt/lowcmd``, the
    DDS channel the motors actually listen to (see :func:`command_coverage`).

    EPOCH. Times are rebased on the first ``action == "walk"`` control_log
    entry; a sport-only recording has none, so its first command stands in —
    readable, but the epoch is comparable within the file only.

    Decoding a large file takes tens of seconds, so parsed streams are cached
    under ``~/.cache/dimos_go2sim`` keyed by path, mtime and size.
    """
    path = Path(path)
    cp = _cache_path(path) if cache else None
    if cp is not None and cp.is_file():
        z = np.load(cp, allow_pickle=False)
        return Streams(
            lt=z["lt"],
            lq=z["lq"],
            ldq=z["ldq"],
            ltau=z["ltau"],
            lquat=z["lquat"],
            lgyro=z["lgyro"],
            lacc=z["lacc"],
            ct=z["ct"],
            cq=z["cq"],
            ckp=z["ckp"],
            ckd=z["ckd"],
            ctau=z["ctau"],
            cdq=z["cdq"],
            vt=z["vt"],
            vp=z["vp"],
            vq=z["vq"],
            seg_t=z["seg_t"],
            seg_mode=tuple(str(m) for m in z["seg_mode"]),
            wt=z["wt"],
            wcmd=z["wcmd"],
            ght=z["ght"],
            gh=z["gh"],
        )
    st = _read_streams_uncached(path)
    if cp is not None:
        np.savez_compressed(
            cp,
            lt=st.lt,
            lq=st.lq,
            ldq=st.ldq,
            ltau=st.ltau,
            lquat=st.lquat,
            lgyro=st.lgyro,
            lacc=st.lacc,
            ct=st.ct,
            cq=st.cq,
            ckp=st.ckp,
            ckd=st.ckd,
            ctau=st.ctau,
            cdq=st.cdq,
            vt=st.vt,
            vp=st.vp,
            vq=st.vq,
            seg_t=st.seg_t,
            seg_mode=np.array(st.seg_mode, dtype="U32"),
            wt=st.wt,
            wcmd=st.wcmd,
            ght=st.ght,
            gh=st.gh,
        )
    return st


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
            elif d.get("action") == "walk":
                first_walk = t if first_walk is None else min(first_walk, t)
                walk.append([t, d.get("vx", 0.0), d.get("vy", 0.0), d.get("vyaw", 0.0)])
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
