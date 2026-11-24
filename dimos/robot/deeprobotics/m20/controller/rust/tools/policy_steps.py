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

"""rl_deploy's policy steps from a recorded session: per step what rl_deploy's policy saw and what it sent, exact (the
joint state and command as the ECAN frames carry them, the body IMU samples as DDS carried them).

    python tools/policy_steps.py windows PLAN.json IMU.npz ECAN_DUMP.bin JOINTS.npz OUT.bin.gz RUNNER:FIRST_ROW:N ...
    python tools/policy_steps.py session SESSION_DIR ECAN_DUMP.bin CACHE_DIR OUT.bin [NET_OFFSET_S]

windows: the compact fixture of tests/policy_20261120.rs. session: every step of the session, for
examples/policy_session.rs (layout in session()).

PLAN.json, IMU.npz and JOINTS.npz are dimos.simulation.m20_policy_sim.replay's (plan.json; cache/imu.npz and
cache/joints.npz), ECAN_DUMP.bin examples/ecan_dump.rs's records of the session's ECAN capture. Each window is the runner's
policy steps from FIRST_ROW, N of them, after the runner's steps before it (their
switches and axes only: its command slew and phase) and 5 that fill its history.

Layout (gzip, little-endian): "M20PS1", u32 windows; per window: u8 name length, name, u32 prefix steps, per prefix
step u8 reset, u8 activate, 3 f32 axes; u32 steps, per step 3 f32 axes, 16 f32 q, 16 f32 dq, 3 f32 IMU roll / pitch /
yaw (deg), 3 f32 gyro, 3 f32 accel, 16 f32 commanded motor positions and 4 f32 wheel velocity commands as sent.
"""

import gzip
import json
import os
import struct
import sys

import numpy as np

COMMAND_LEAD_S = 0.0015


def ecan_records(path):
    """examples/ecan_dump.rs's records: the states (16 x q, dq, tau) and the commands' times and (16 x q, dq, tau)."""
    b = open(path, "rb").read()
    o, st, cm_t, cmq = 0, [], [], []
    while o < len(b):
        t, kind = struct.unpack_from("<QB", b, o)
        o += 9
        if kind == 0:
            st.append(np.frombuffer(b, "<f4", 48, o).reshape(16, 3))
            o += 192
        elif kind == 1:
            rows = np.frombuffer(b, np.uint8, 256, o + 2).reshape(16, 16)
            cm_t.append(t)
            cmq.append(rows[:, :12].copy().view("<f4").reshape(16, 3))
            o += 258
    return np.array(st), np.array(cm_t), np.array(cmq)


def windows(plan_path, imu_path, dump_path, joints_path, out_path, *windows):
    steps = json.load(open(plan_path))["steps"]
    imu = np.load(imu_path)
    j = np.load(joints_path)
    t = j["t"] / 1e9
    imu_row = np.searchsorted(imu["t"], t, side="right") - 1
    st, cm_t, cmq = ecan_records(dump_path)
    out = bytearray(b"M20PS1")
    out += struct.pack("<I", len(windows))
    for w in windows:
        name, first, n = w.split(":")
        first, n = int(first), int(n)
        mine = [s for s in steps if s["runner"] == name]
        k0 = next(i for i, s in enumerate(mine) if s["row"] >= first)
        start = k0 - 5
        # every step of the runner before the window: their axes run its command slew and phase
        prefix = mine[:start]
        out += struct.pack("<B", len(name)) + name.encode()
        out += struct.pack("<I", len(prefix))
        for s in prefix:
            out += struct.pack("<BB3f", s["reset"], s["activate"], *s["axes"])
        body = mine[start : k0 + n]
        out += struct.pack("<I", len(body))
        for s in body:
            row = s["row"]
            k = max(int(imu_row[row - 1]), 0)
            c = np.searchsorted(cm_t, j["t"][row])
            wheels = cmq[c, [3, 7, 11, 15], 1]
            out += struct.pack("<3f", *s["axes"])
            out += (
                st[row - 1, :, 0].astype("<f4").tobytes()
                + st[row - 1, :, 1].astype("<f4").tobytes()
            )
            out += np.asarray(imu["rpy_deg"][k], "<f4").tobytes()
            out += (
                np.asarray(imu["gyro"][k], "<f4").tobytes()
                + np.asarray(imu["accel"][k], "<f4").tobytes()
            )
            out += cmq[c, :, 0].astype("<f4").tobytes() + wheels.astype("<f4").tobytes()
            if s["reset"] or s["activate"]:
                raise SystemExit(f"{name}: a switch at row {row} inside the window")
    with gzip.open(out_path, "wb", 9) as f:
        f.write(bytes(out))
    print(f"{len(out)} bytes")


def session(session_dir, dump_path, cache, out_path, net_offset_s="0"):
    """Layout (little-endian): "M20SS2"; u32 rows, f64 time (s) per joint-frame row; u32 IMU samples, per sample f64
    arrival time (s), 9 f32 (roll / pitch / yaw deg, gyro, accel); u32 segments, per segment u8 name length, name, u32
    start row, u32 end row; u32 steps, per step u8 runner length, runner, u32 row, u32 state row, u32 gait, u8 reset, u8 activate,
    3 f32 axes, u8 params, per param u8 name length, name, f64 value, 16 f32 q and 16 f32 dq (the state row's reply), 16 f32 commanded motor positions and 4 f32 wheel velocity commands as sent after it.

    NET_OFFSET_S: added to the IMU samples' net.pcap times, onto the ECAN capture's clock as rl_deploy saw them (the
    commands' times keep COMMAND_LEAD_S: shifting them too fits worse): the middle of the range of IMU offsets over which the steps match (14:16: 0, -0.5 .. 0.5 ms;
    16:36: -1.7 ms, -3.0 .. -0.4)."""
    from pathlib import Path

    from dimos.simulation.m20_policy_sim import replay
    from dimos.simulation.m20_policy_sim.config import load_config

    cfg = load_config()
    cfg["ecan_enable_word"] = 2
    if "M20_STEER_TIMEOUT_S" in os.environ:
        replay.COMMAND_TIMEOUT_S = float(os.environ["M20_STEER_TIMEOUT_S"])
    s = replay.load_session(Path(session_dir), Path(cache))
    # a command takes effect COMMAND_LEAD_S before the step's row: rl_deploy's Run copies UserCommand at the start of
    # the tick that reads the reply before the step (row - 1), and a DDS sample reaches UserCommand ~0.5 ms after the
    # capture saw it (best fit, 14:16: 1.5-2 ms)
    lead = float(os.environ.get("M20_COMMAND_LEAD_S", COMMAND_LEAD_S))
    s.events["nav"] = [(e[0] + lead, *e[1:]) for e in s.events["nav"]]
    segments, steps = replay.build_plan(s, cfg, 1, None, True)
    st, cm_t, cmq = ecan_records(dump_path)
    if len(st) != len(s.t):
        raise SystemExit(f"{len(st)} ECAN states, {len(s.t)} joint rows")
    t_ns = np.load(Path(cache) / "joints.npz")["t"]
    out = bytearray(b"M20SS2")
    out += struct.pack("<I", len(s.t)) + np.asarray(s.t, "<f8").tobytes()
    imu = s.imu
    out += struct.pack("<I", len(imu["t"]))
    for k in range(len(imu["t"])):
        out += struct.pack("<d", imu["t"][k] + float(net_offset_s))
        out += np.r_[imu["rpy_deg"][k], imu["gyro"][k], imu["accel"][k]].astype("<f4").tobytes()
    out += struct.pack("<I", len(segments))
    for g in segments:
        out += (
            struct.pack("<B", len(g["state"]))
            + g["state"].encode()
            + struct.pack("<II", g["start"], g["end"])
        )
    out += struct.pack("<I", len(steps))
    # rl_deploy's policy thread keeps a 20-tick cadence and reads the state when it wakes; a command that reached the
    # bus a tick or two late was computed from the state of its on-time tick
    state_rows, cadence = [], None
    for x in steps:
        row = x["row"]
        late = (row - cadence) % 20 if cadence is not None else 0
        if late > 3 or x["reset"] or cadence is None:
            late = 0
        cadence = row - late
        state_rows.append(cadence - 1)
    for x, state_row in zip(steps, state_rows, strict=True):
        row = x["row"]
        c = min(np.searchsorted(cm_t, t_ns[row]), len(cm_t) - 1)
        out += struct.pack("<B", len(x["runner"])) + x["runner"].encode()
        out += struct.pack(
            "<IIIBB3f", row, state_row, x["gait"], x["reset"], x["activate"], *x["axes"]
        )
        out += struct.pack("<B", len(x["params"]))
        for k, v in x["params"].items():
            out += struct.pack("<B", len(k)) + k.encode() + struct.pack("<d", v)
        out += (
            st[state_row, :, 0].astype("<f4").tobytes()
            + st[state_row, :, 1].astype("<f4").tobytes()
        )
        out += (
            cmq[c, :, 0].astype("<f4").tobytes() + cmq[c, [3, 7, 11, 15], 1].astype("<f4").tobytes()
        )
    open(out_path, "wb").write(bytes(out))
    print(f"{len(steps)} steps, {len(out)} bytes")


if __name__ == "__main__":
    {"windows": windows, "session": session}[sys.argv[1]](*sys.argv[2:])
