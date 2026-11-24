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

"""Windows of rl_deploy's policy steps from a recorded session, for tests/policy_20261120.rs: per step what rl_deploy's
policy saw and what it sent, exact (the joint state and command as the ECAN frames carry them, the body IMU sample as
DDS carried it).

    python tools/policy_steps.py PLAN.json IMU.npz ECAN_DUMP.bin JOINTS.npz OUT.bin.gz RUNNER:FIRST_ROW:N ...

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
import struct
import sys

import numpy as np


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


def main():
    plan_path, imu_path, dump_path, joints_path, out_path, *windows = sys.argv[1:]
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


if __name__ == "__main__":
    main()
