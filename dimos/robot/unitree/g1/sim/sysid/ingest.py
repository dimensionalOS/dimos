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

"""The G1 recording reader: a mem2 ``.db`` into :class:`Streams`.

The only G1-specific place above the seam. Streams: ``motor_states`` (495 Hz
q/dq/tau), ``imu`` (495 Hz), ``motor_command`` (the GR00T targets, 99 Hz on
the wire with distinct rows at 50 Hz), ``pointlio_odometry`` (~30 Hz, the
base-pose reference in place of a tracker) and ``cmd_vel`` (the operator's
velocity commands). Every stream is read on the store clock ``o.ts``.

Three facts of the rig live here as constants. The IMU site is in
``torso_link``, so the recorded attitude is the torso's and is moved to the
pelvis through the measured waist angles before it seeds the free joint.
The Mid-360 is mounted upside down on the torso; Point-LIO odometry is
moved to the pelvis the same way, so ``Streams.mount`` and ``lever`` stay
identity. ``CMD_TIME_OFFSET_S`` is the command-clock question left open.

    python -m dimos.robot.unitree.g1.sim.sysid.ingest data/g1_groot_characterization_2026-08-27.db
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any, Protocol

import numpy as np
from numpy.typing import NDArray

from dimos.robot.unitree.g1.sim.plant import JOINT_NAMES, NUM_JOINTS
from dimos.simulation.sysid.recording import Streams, read_recording
from dimos.simulation.sysid.rotations import mat_to_quat

_CACHE_VERSION = 1

# The payload's own `motor_command.timestamp` runs ~50 ms behind the store
# clock; commands and states are compared on the clock they were logged on.
# Replay with 0 and -0.05 and compare the joint score before believing either.
CMD_TIME_OFFSET_S = 0.0

# GR00T's own balance/walk switch (CMD_NORM_THRESHOLD): a segment is `walk`
# where |cmd| exceeds it, so these ARE the policy-mode segments.
WALK_THRESHOLD = 0.05
MIN_SEGMENT_S = 1.0

# The base-pose grid: Point-LIO arrives at ~30 Hz, the prediction's pose rate
# is one fifth of the physics rate; 100 Hz is finer than either.
POSE_HZ = 100.0

# rt/lowstate motor indices of the waist, actuator order.
_WAIST = slice(12, 15)

# The Mid-360 is mounted rolled 180 deg relative to the URDF convention.
_MID360_MOUNT_UNROLL = np.diag([1.0, -1.0, -1.0])


class Mid360Odometry(Protocol):
    x: float
    y: float
    z: float

    @property
    def orientation(self) -> Any: ...


def waist_rotation(yaw: NDArray[Any], roll: NDArray[Any], pitch: NDArray[Any]) -> NDArray[Any]:
    """pelvis_R_torso for the g1.urdf waist chain (yaw, then roll, then pitch), batched.

    The same rotation :func:`~dimos.robot.unitree.g1.g1_tf_publisher.base_to_torso`
    builds one Transform at a time; held equal by test_ingest.
    """
    cy, sy = np.cos(yaw), np.sin(yaw)
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    z = np.zeros_like(cy)
    o = np.ones_like(cy)
    rz = np.stack([cy, -sy, z, sy, cy, z, z, z, o], -1).reshape(-1, 3, 3)
    rx = np.stack([o, z, z, z, cr, -sr, z, sr, cr], -1).reshape(-1, 3, 3)
    ry = np.stack([cp, z, sp, z, o, z, -sp, z, cp], -1).reshape(-1, 3, 3)
    out: NDArray[Any] = rz @ rx @ ry
    return out


def world_T_pelvis(
    odom: Mid360Odometry, waist: tuple[float, float, float] = (0.0, 0.0, 0.0)
) -> NDArray[np.float64]:
    """Point-LIO mid360 odometry -> world_T_pelvis at the measured waist pose."""
    from dimos.robot.unitree.g1.g1_tf_publisher import base_to_torso, torso_to_mid360

    pelvis_T_physical = (base_to_torso(*waist) + torso_to_mid360()).to_matrix()
    pelvis_T_mid360 = np.asarray(pelvis_T_physical, dtype=np.float64).copy()
    pelvis_T_mid360[:3, :3] = pelvis_T_physical[:3, :3] @ _MID360_MOUNT_UNROLL

    world_T_mid360 = np.eye(4, dtype=np.float64)
    world_T_mid360[:3, :3] = odom.orientation.to_rotation_matrix() @ _MID360_MOUNT_UNROLL
    world_T_mid360[:3, 3] = (odom.x, odom.y, odom.z)
    return world_T_mid360 @ np.linalg.inv(pelvis_T_mid360)


def _quat_mul(a: NDArray[Any], b: NDArray[Any]) -> NDArray[Any]:
    """Hamilton product, wxyz, batched."""
    aw, ax, ay, az = a.T
    bw, bx, by, bz = b.T
    return np.stack(
        [
            aw * bw - ax * bx - ay * by - az * bz,
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
        ],
        -1,
    )


def _interp_rows(t: NDArray[Any], ts: NDArray[Any], rows: NDArray[Any]) -> NDArray[Any]:
    return np.stack([np.interp(t, ts, rows[:, i]) for i in range(rows.shape[1])], 1)


def _interp_quat(t: NDArray[Any], ts: NDArray[Any], q: NDArray[Any]) -> NDArray[Any]:
    """Linear interpolation of wxyz quaternions, sign-continuous, renormalised."""
    q = q.copy()
    flip = np.cumsum(np.einsum("ij,ij->i", q[1:], q[:-1]) < 0) % 2 == 1
    q[1:][flip] *= -1.0
    out = _interp_rows(t, ts, q)
    normalised: NDArray[Any] = out / np.linalg.norm(out, axis=1, keepdims=True)
    return normalised


def segments_from_commands(
    t: NDArray[Any], cmd: NDArray[Any], t_start: float
) -> tuple[NDArray[Any], tuple[str, ...]]:
    """``(seg_t, seg_mode)`` from the velocity commands: walk above the threshold, else stand.

    Runs shorter than ``MIN_SEGMENT_S`` merge into the run before them; a
    leading ``stand`` covers the time before the first command.
    """
    if len(t) == 0:
        return np.array([t_start]), ("stand",)
    walking = np.linalg.norm(cmd, axis=1) > WALK_THRESHOLD
    starts = [t_start]
    modes = ["stand"]
    for i in range(len(t)):
        mode = "walk" if walking[i] else "stand"
        if mode != modes[-1]:
            starts.append(float(t[i]))
            modes.append(mode)
    # Merge short runs backwards: a run that never lasted MIN_SEGMENT_S is
    # absorbed by the run before it.
    keep_t, keep_m = [starts[0]], [modes[0]]
    for i in range(1, len(starts)):
        end = starts[i + 1] if i + 1 < len(starts) else float(t[-1])
        if end - starts[i] < MIN_SEGMENT_S:
            continue
        if modes[i] == keep_m[-1]:
            continue
        keep_t.append(starts[i])
        keep_m.append(modes[i])
    return np.array(keep_t), tuple(keep_m)


def _read_streams_uncached(path: Path) -> Streams:
    from dimos.memory.cli.dataset import open_store

    with open_store(path) as store:

        def rows(name: str, take: Any) -> tuple[NDArray[Any], NDArray[Any]]:
            ts, vals = [], []
            o: Any
            for o in store.stream(name):
                ts.append(float(o.ts))
                vals.append(take(o.data))
            if not ts:
                return np.zeros(0), np.zeros((0, 0))
            order = np.argsort(ts, kind="stable")
            return np.asarray(ts)[order], np.asarray(vals, dtype=np.float64)[order]

        first: Any = next(iter(store.stream("motor_states"))).data
        if tuple(n.split("/", 1)[-1] for n in first.name) != JOINT_NAMES:
            raise ValueError("motor_states joint order is not the G1 actuator order")

        lt, L = rows(
            "motor_states",
            lambda d: [*d.position, *d.velocity, *d.effort],
        )
        it, I = rows(
            "imu",
            lambda d: [
                d.orientation.w,
                d.orientation.x,
                d.orientation.y,
                d.orientation.z,
                d.angular_velocity.x,
                d.angular_velocity.y,
                d.angular_velocity.z,
                d.linear_acceleration.x,
                d.linear_acceleration.y,
                d.linear_acceleration.z,
            ],
        )
        ct, C = rows("motor_command", lambda d: [*d.q, *d.dq, *d.kp, *d.kd, *d.tau])
        wt, W = rows("cmd_vel", lambda d: [d.linear.x, d.linear.y, d.angular.z])
        if len(wt) == 0:
            wt, W = rows("tele_cmd_vel", lambda d: [d.linear.x, d.linear.y, d.angular.z])
        ot, odoms = [], []
        obs: Any
        for obs in store.stream("pointlio_odometry"):
            ot.append(float(obs.ts))
            odoms.append(obs.data)

    n = NUM_JOINTS
    epoch = float(wt[0]) if len(wt) else float(ct[0])
    lq, ldq, ltau = L[:, :n], L[:, n : 2 * n], L[:, 2 * n : 3 * n]

    # IMU onto the lowstate clock; torso attitude -> pelvis through the waist.
    q_torso = _interp_quat(lt, it, I[:, 0:4])
    waist = lq[:, _WAIST]
    r_wt = waist_rotation(waist[:, 0], waist[:, 1], waist[:, 2])
    q_wt = np.stack([mat_to_quat(r) for r in r_wt])
    q_wt_conj = q_wt * np.array([1.0, -1.0, -1.0, -1.0])
    lquat = _quat_mul(q_torso, q_wt_conj)
    lquat /= np.linalg.norm(lquat, axis=1, keepdims=True)

    # Commands: distinct target rows only (the wire repeats each 50 Hz row).
    changed = np.ones(len(ct), dtype=bool)
    changed[1:] = np.any(np.diff(C[:, :n], axis=0) != 0.0, axis=1)
    ct, C = ct[changed], C[changed]

    st = Streams(
        lt=lt - epoch,
        lq=lq,
        ldq=ldq,
        ltau=ltau,
        lquat=lquat,
        lgyro=_interp_rows(lt, it, I[:, 4:7]),
        lacc=_interp_rows(lt, it, I[:, 7:10]),
        ct=ct - epoch + CMD_TIME_OFFSET_S,
        cq=C[:, :n],
        cdq=C[:, n : 2 * n],
        ckp=C[:, 2 * n : 3 * n],
        ckd=C[:, 3 * n : 4 * n],
        ctau=C[:, 4 * n : 5 * n],
        wt=wt - epoch,
        wcmd=W.reshape(-1, 3),
    )
    st.seg_t, st.seg_mode = segments_from_commands(st.wt, st.wcmd, float(st.lt[0]))

    if ot:
        # Point-LIO -> pelvis at the measured waist pose, on a uniform grid.
        ots = np.asarray(ot)
        wi = np.clip(np.searchsorted(lt, ots, "right") - 1, 0, len(lt) - 1)
        poses = np.stack(
            [world_T_pelvis(od, tuple(waist[i])) for od, i in zip(odoms, wi, strict=True)]
        )
        t_rel = ots - epoch
        grid = np.arange(t_rel[0], t_rel[-1], 1.0 / POSE_HZ)
        st.vt = grid
        st.vp = _interp_rows(grid, t_rel, poses[:, :3, 3])
        st.vq = _interp_quat(grid, t_rel, np.stack([mat_to_quat(p[:3, :3]) for p in poses]))
    return st


@dataclass(frozen=True)
class G1DbReader:
    """The G1 recording reader. Stateless and picklable, like a backend."""

    @property
    def cache_tag(self) -> str:
        return f"g1v{_CACHE_VERSION}"

    def read(self, path: Path) -> Streams:
        return _read_streams_uncached(path)


G1_READER = G1DbReader()


def read_streams(path: str | Path, *, cache: bool = True) -> Streams:
    return read_recording(path, G1_READER, cache=cache)


def main() -> None:
    """Print what the reader made of a recording: shapes, rates, segments."""
    import argparse

    from dimos.simulation.sysid.regimes import propose_suspended

    ap = argparse.ArgumentParser(prog="g1.sim.sysid.ingest", description=main.__doc__)
    ap.add_argument("recording")
    ap.add_argument("--no-cache", action="store_true")
    args = ap.parse_args()
    st = read_streams(args.recording, cache=not args.no_cache)

    def rate(t: NDArray[Any]) -> str:
        return f"{1.0 / np.median(np.diff(t)):.1f} Hz" if len(t) > 1 else "empty"

    print(f"lowstate {st.lq.shape}  {rate(st.lt)}  t={st.lt[0]:.1f}..{st.lt[-1]:.1f}s")
    print(f"commands {st.cq.shape}  {rate(st.ct)}  kp[:6]={st.ckp[0, :6]}")
    print(f"base pose {st.vp.shape}  {rate(st.vt)}  (Point-LIO -> pelvis)")
    print(f"velocity cmds {st.wcmd.shape}  {rate(st.wt)}")
    print(f"|acc| median {np.median(np.linalg.norm(st.lacc, axis=1)):.2f} m/s^2")
    print(f"propose_suspended: {propose_suspended(st)}")
    print(f"{'idx':>4s} {'mode':>6s} {'start':>8s} {'end':>8s} {'dur':>7s}")
    for i, mode, a, b in st.segments():
        print(f"{i:4d} {mode:>6s} {a:8.2f} {b:8.2f} {b - a:7.2f}")


if __name__ == "__main__":
    main()
