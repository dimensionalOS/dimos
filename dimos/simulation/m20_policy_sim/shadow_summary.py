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

"""Agreement between the shadow controller (m20_shadow) and rl_deploy, per phase and joint, from its logs.

    python -m dimos.simulation.m20_policy_sim.shadow_summary LOG_DIR_OR_FILES... [--html out.html] [--json out.json]

A log file is "M20SHDW1", a u32 header length, a JSON header (record_size, joints, runners, states), then one
fixed-size little-endian record per control cycle (layout in m20_shadow's log module and RECORD below).
"""

from __future__ import annotations

import argparse
import itertools
import json
from pathlib import Path
import struct

import numpy as np

from dimos.simulation.m20_policy_sim.replay import JOINT_NAMES, clean, compose, gain_quant

MAGIC = b"M20SHDW1"
FLAG_OURS = 1
FLAG_OURS_ENABLE = 2
FLAG_RL_ENABLE = 4
FLAG_POLICY_STEP = 8
FLAG_IMU = 16
RECORD = np.dtype(
    [
        ("t_ns", "<u8"),
        ("flags", "<u4"),
        ("rl_word", "<u2"),
        ("state", "u1"),
        ("runner", "u1"),
        ("gait", "<u4"),
        ("reserved", "<u4"),
        ("quat", "<f4", 4),
        ("gyro", "<f4", 3),
        ("accel", "<f4", 3),
        ("q", "<f4", 16),
        ("dq", "<f4", 16),
        ("axes", "<f4", 3),
        ("ours", "<f4", (4, 16)),
        ("rl", "<f4", (4, 16)),
    ]
)
KP, Q, KD, DQ = range(4)


def read_log(path: Path) -> tuple[dict, np.ndarray]:
    data = path.read_bytes()
    if data[:8] != MAGIC:
        raise ValueError(f"{path}: not a shadow log")
    (n,) = struct.unpack("<I", data[8:12])
    header = json.loads(data[12 : 12 + n])
    if header["record_size"] != RECORD.itemsize:
        raise ValueError(
            f"{path}: record size {header['record_size']}, this tool reads {RECORD.itemsize}"
        )
    body = data[12 + n :]
    usable = len(body) // RECORD.itemsize * RECORD.itemsize  # a file being written ends mid-record
    return header, np.frombuffer(body[:usable], RECORD)


def load(paths: list[Path]) -> tuple[dict, np.ndarray]:
    files = sorted(p for x in paths for p in (sorted(x.glob("*.m20shadow")) if x.is_dir() else [x]))
    header, parts = None, []
    for f in files:
        h, r = read_log(f)
        header = header or h
        parts.append(r)
    if not parts:
        raise SystemExit("no shadow logs found")
    return header, np.concatenate(parts)


def phases(header: dict, rec: np.ndarray) -> list[tuple[str, int, int]]:
    """Runs of the same (state, runner) in the log."""
    key = rec["state"].astype(np.int64) * 256 + rec["runner"]
    edges = [0, *(np.where(np.diff(key) != 0)[0] + 1), len(rec)]
    out = []
    for a, b in itertools.pairwise(edges):
        code = rec["state"][a]
        state = header["states"][code] if code < len(header["states"]) else "unknown"
        runner = (
            header["runners"][rec["runner"][a]] if rec["runner"][a] < len(header["runners"]) else ""
        )
        out.append((f"{state} {runner}".strip(), a, b))
    return out


def summarize(header: dict, rec: np.ndarray) -> list[dict]:
    legs = np.arange(16) % 4 != 3
    report = []
    t0 = rec["t_ns"][0]
    both = (rec["flags"] & FLAG_OURS) != 0
    for name, a, b in phases(header, rec):
        rows = np.arange(a, b)
        rows = rows[both[rows] & ((rec["flags"][rows] & (FLAG_OURS_ENABLE | FLAG_RL_ENABLE)) == 0)]
        e = {
            "phase": name,
            "start_s": round((rec["t_ns"][a] - t0) / 1e9, 3),
            "cycles": int(b - a),
            "compared": len(rows),
        }
        enable_rows = np.arange(a, b)[
            (rec["flags"][a:b] & (FLAG_OURS_ENABLE | FLAG_RL_ENABLE)) != 0
        ]
        if len(enable_rows):
            f = rec["flags"][enable_rows]
            e["enable_match"] = float(
                np.mean(((f & FLAG_OURS_ENABLE) != 0) == ((f & FLAG_RL_ENABLE) != 0))
            )
        if len(rows):
            ours, rl = rec["ours"][rows].astype(np.float64), rec["rl"][rows].astype(np.float64)
            dq = ours[:, Q][:, legs] - rl[:, Q][:, legs]
            w = ours[:, DQ][:, ~legs] - rl[:, DQ][:, ~legs]
            e["q_rms"] = np.sqrt(np.mean(dq**2, 0)).round(5).tolist()
            e["q_max"] = np.abs(dq).max(0).round(5).tolist()
            e["q_worst_joint_median"] = float(np.median(np.abs(dq).max(1)).round(5))
            e["q_worst_joint_p95"] = float(np.percentile(np.abs(dq).max(1), 95).round(5))
            e["wheel_dq_rms"] = np.sqrt(np.mean(w**2, 0)).round(4).tolist()
            # the bus carries kp / kd in 0.1 steps: compare those steps
            on_bus = lambda v: np.round(v * 10.0)  # noqa: E731
            e["kp_match"] = float(
                np.mean(np.all(on_bus(gain_quant(ours[:, KP])) == on_bus(rl[:, KP]), 1))
            )
            e["kd_match"] = float(
                np.mean(np.all(on_bus(gain_quant(ours[:, KD])) == on_bus(rl[:, KD]), 1))
            )
            e["exact_q"] = float(np.mean(np.abs(dq).max(1) < 1e-4))
        report.append(e)
    return report


def page(header: dict, rec: np.ndarray, report: list[dict], path: Path) -> None:
    rows = np.arange(0, len(rec), 20)
    t = ((rec["t_ns"][rows] - rec["t_ns"][0]) / 1e9).round(3).tolist()
    series = {"t": t, "joints": JOINT_NAMES, "recorded": {}, "ours": {"shadow": {}}}
    for j, name in enumerate(JOINT_NAMES):
        field = DQ if j % 4 == 3 else Q
        series["recorded"][name] = clean(rec["rl"][rows, field, j].astype(np.float64))
        ok = (rec["flags"][rows] & FLAG_OURS) != 0
        series["ours"]["shadow"][name] = clean(
            np.where(ok, rec["ours"][rows, field, j], np.nan).astype(np.float64)
        )
    series["recorded"]["kp"] = clean(rec["rl"][rows, KP, 1].astype(np.float64))
    series["ours"]["shadow"]["kp"] = clean(gain_quant(rec["ours"][rows, KP, 1].astype(np.float64)))
    table = "".join(
        f"<tr><td>{e['phase']}</td><td>{e['start_s']}</td><td>{e['compared']}</td>"
        f"<td>{e.get('q_worst_joint_median', 0):.5f}</td><td>{e.get('q_worst_joint_p95', 0):.4f}</td>"
        f"<td>{e.get('exact_q', 1):.3f}</td><td>{max(e.get('wheel_dq_rms', [0])):.3f}</td>"
        f"<td>{e.get('kp_match', 1):.3f} / {e.get('kd_match', 1):.3f}</td><td>shadow</td></tr>"
        for e in report
    )
    # the page takes a list of sections (replay.py's compose), one here
    section = {"name": "shadow", "series": series, "table": table, "summary": ""}
    compose([section], path, "M20 shadow vs rl_deploy")


def main() -> None:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("logs", nargs="+", type=Path)
    parser.add_argument("--html", type=Path)
    parser.add_argument("--json", type=Path)
    args = parser.parse_args()
    header, rec = load(args.logs)
    report = summarize(header, rec)
    for e in report:
        if "q_rms" in e:
            print(
                f"{e['phase']:<30} t={e['start_s']:>9} n={e['compared']:>7} exact={e['exact_q']:.3f} "
                f"med={e['q_worst_joint_median']:.5f} p95={e['q_worst_joint_p95']:.4f} q_max={max(e['q_max']):.4f} "
                f"kp={e['kp_match']:.3f} kd={e['kd_match']:.3f}"
            )
        else:
            enable = (
                f"enable frames match {e['enable_match']:.3f}"
                if "enable_match" in e
                else "nothing compared"
            )
            print(f"{e['phase']:<30} t={e['start_s']:>9} n={e['cycles']:>7} ({enable})")
    if args.json:
        args.json.write_text(json.dumps(report, indent=1))
    if args.html:
        page(header, rec, report, args.html)
        print(f"wrote {args.html}")


if __name__ == "__main__":
    main()
