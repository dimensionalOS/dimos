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

"""Open-loop replay of a recorded rl_deploy session: feed our pipeline exactly what rl_deploy saw, compare what it sent.

    python -m dimos.simulation.m20_policy_sim.replay SESSION_DIR --out DIR [--rust PATH_TO_replay_BINARY]

A session (ecan_capture record.sh) has joints.csv (1 kHz joint state and the command rl_deploy sent, decoded by
m20-ecan), net.pcap (DDS traffic, including the body IMU), journal.log (rl_deploy's state / gait log) and
timeline.csv (/NAV_CMD velocity commands). Per row k the command rl_deploy sent is compared with ours computed from
the joint state of row k - 1 (k - lag for the policy). Non-policy states run through the Controller (forced into the
recorded state at the recorded tick); policy steps run the runner at the ticks rl_deploy's targets changed.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import datetime as dt
import itertools
import json
import math
from pathlib import Path
import re
import struct
import subprocess
from typing import Any

import numpy as np

from dimos.simulation.m20_policy_sim.config import (
    IS_WHEEL,
    JOINTS,
    load_config,
    policy_dir,
    runner_for_gait,
)
from dimos.simulation.m20_policy_sim.controller import Controller, UserCommand
from dimos.simulation.m20_policy_sim.runner import RobotState, Runner

JOINT_NAMES = [
    f"{leg}_{part}_joint"
    for leg in ("fl", "fr", "hl", "hr")
    for part in ("hipx", "hipy", "knee", "wheel")
]
FIELDS = ("q", "dq", "q_cmd", "dq_cmd", "kp", "kd", "word")
# the body IMU on the AOS board's DDS: a 56-byte sample (sec, nsec stamps, roll / pitch / yaw in degrees, gyro rad/s,
# accelerometer m/s^2) published at 200 Hz to UDP 7441
IMU_PORT = 7441
IMU_SAMPLE_BYTES = 56
# velocity commands seen on the AOS board's DDS from NOS, by writer entity id: (x, y, yaw, z float index, metric).
# The operator's /NAV_CMD (stamp, then x, y, yaw f32) is m/s and rad/s (rl_deploy's /REAL_STEER path, divided by the
# gait's real_steer_divisor); the charge manager's steer (stamp, then x, y, z, roll, pitch, yaw f32) is normalized
# axes (/STEER). Which is which was settled by this replay (the other reading doubles the error).
COMMAND_WRITERS = {
    "00000503": (0, 1, 2, None, None, None, True),
    "00000803": (0, 1, 5, 2, 3, 4, False),
}
# the stand gaits read /STEER as pose inputs (rl_deploy HandlerSteer): param <- scale x axis (x, y, z, roll, pitch, yaw);
# 0xf001: yaw, -x, -y, roll, pitch, height = max(-z, 0); 0x3004: 7 yaw, -24 x, -18 y, height = max(-z, 0)
STEER_POSE = {
    0xF001: {
        "yaw": ("yaw", 1.0),
        "dx": ("x", -1.0),
        "dy": ("y", -1.0),
        "roll": ("roll", 1.0),
        "pitch": ("pitch", 1.0),
    },
    0x3004: {"yaw": ("yaw", 7.0), "dx": ("x", -24.0), "dy": ("y", -18.0)},
}
# rl_deploy's StateName codes as journal.log names them
STATE_NAMES = {
    "idle_state": "passive",
    "standup_state": "stand_up",
    "liedown_state": "lie_down",
    "rl_control": "rl_control",
    "joint_damping": "joint_damping",
}
STAND_IN_GAIT = 0xF001
# NAV_CMD stops counting after this long without a sample (rl_deploy's /STEER timeout)
COMMAND_TIMEOUT_S = 0.15


# ---------------------------------------------------------------------------------------------------- session input


def load_joints(session: Path, cache: Path) -> dict[str, np.ndarray]:
    path = cache / "joints.npz"
    if path.exists():
        return dict(np.load(path))
    import pandas as pd

    df = pd.read_csv(session / "joints.csv")
    out = {"t": df["t_ns"].to_numpy(np.int64)}
    for f in FIELDS:
        out[f] = np.stack([df[f"{j}.{f}"].to_numpy(np.float64) for j in JOINT_NAMES], 1)
    np.savez(path, **out)
    return out


def pcap_udp(path: Path):
    """(time s, dst port, payload) per IPv4 UDP packet of a pcap (Ethernet or Linux cooked capture)."""
    with open(path, "rb") as f:
        head = f.read(24)
        nano = struct.unpack("<I", head[:4])[0] == 0xA1B23C4D
        link = struct.unpack("<I", head[20:24])[0]
        off = 16 if link == 113 else 14
        while True:
            h = f.read(16)
            if len(h) < 16:
                return
            sec, frac, incl, _ = struct.unpack("<IIII", h)
            d = f.read(incl)
            if d[off - 2 : off] != b"\x08\x00" or d[off + 9] != 17:
                continue
            ihl = (d[off] & 15) * 4
            dport = struct.unpack(">H", d[off + ihl + 2 : off + ihl + 4])[0]
            yield sec + frac * (1e-9 if nano else 1e-6), dport, d[off + ihl + 8 :]


def rtps_data(payload: bytes):
    """Serialized payloads of the DATA submessages of one RTPS message."""
    if payload[:4] != b"RTPS":
        return
    i = 20
    while i + 4 <= len(payload):
        sid, flags = payload[i], payload[i + 1]
        n = struct.unpack("<H" if flags & 1 else ">H", payload[i + 2 : i + 4])[0]
        if sid == 0x15:
            yield payload[i + 4 + 20 : i + 4 + n]
        if n == 0:
            return
        i += 4 + n


def load_imu(session: Path, cache: Path) -> dict[str, np.ndarray]:
    path = cache / "imu.npz"
    if path.exists():
        return dict(np.load(path))
    rows: dict[float, tuple] = {}
    for t, dport, p in pcap_udp(session / "net.pcap"):
        if dport != IMU_PORT:
            continue
        for d in rtps_data(p):
            if len(d) != IMU_SAMPLE_BYTES:
                continue
            _, _, sec, nsec = struct.unpack("<4I", d[4:20])
            stamp = sec + nsec * 1e-9
            if stamp not in rows:  # each sample goes to two readers
                rows[stamp] = (t, *struct.unpack("<9f", d[20:56]))
    a = np.array([rows[k] for k in sorted(rows)])
    out = {"t": a[:, 0], "rpy_deg": a[:, 1:4], "gyro": a[:, 4:7], "accel": a[:, 7:10]}
    np.savez(path, **out)
    return out


def quat_from_rpy(roll: float, pitch: float, yaw: float) -> np.ndarray:
    cr, sr, cp, sp, cy, sy = (
        math.cos(roll / 2),
        math.sin(roll / 2),
        math.cos(pitch / 2),
        math.sin(pitch / 2),
        math.cos(yaw / 2),
        math.sin(yaw / 2),
    )
    return np.array(
        [
            cr * cp * cy + sr * sp * sy,
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
        ]
    )


def journal_time(line: str) -> float:
    stamp = line.split(" ", 1)[0]
    return dt.datetime.fromisoformat(stamp).timestamp()


def load_commands(session: Path, cache: Path) -> np.ndarray:
    """(t, x, y, yaw, z, roll, pitch, metric) per command sample from either command writer, time-sorted."""
    path = cache / "commands_v3.npy"
    if path.exists():
        return np.load(path)
    writers = {bytes.fromhex(k): v for k, v in COMMAND_WRITERS.items()}
    seen, rows = set(), []
    for t, _dport, p in pcap_udp(session / "net.pcap"):
        if p[:4] != b"RTPS":
            continue
        i = 20
        while i + 4 <= len(p):
            sid, flags = p[i], p[i + 1]
            n = struct.unpack("<H" if flags & 1 else ">H", p[i + 2 : i + 4])[0]
            body = p[i + 4 : i + 4 + n]
            layout = writers.get(body[8:12]) if sid == 0x15 else None
            if layout and len(body) >= 40:
                key = (body[8:12], body[12:20])
                data = body[20:]
                f = struct.unpack(
                    f"<{(len(data) - 20) // 4}f", data[20 : 20 + 4 * ((len(data) - 20) // 4)]
                )
                x, y, yaw, z, roll, pitch, metric = layout
                if key not in seen and len(f) > max(x, y, yaw, z or 0):
                    seen.add(key)
                    extra = [f[i] if i is not None else 0.0 for i in (z, roll, pitch)]
                    rows.append((t, f[x], f[y], f[yaw], *extra, float(metric)))
            if n == 0:
                break
            i += 4 + n
    a = np.array(sorted(rows)) if rows else np.zeros((0, 8))
    np.save(path, a)
    return a


def load_events(session: Path) -> dict[str, list]:
    """rl_deploy's state transitions, actual gait switches and height inputs (journal.log)."""
    states, gaits, heights = [], [], []
    for line in (session / "journal.log").read_text().splitlines():
        if line.startswith("--"):
            continue
        t = journal_time(line)
        if m := re.search(r"(\w+) -{6,}> (\w+)", line):
            states.append((t, STATE_NAMES.get(m.group(2), m.group(2))))
        elif m := re.search(r"target_gait --- (\w+), last_target_gait", line):
            gaits.append((t, int(m.group(1), 16)))
        elif m := re.search(r"target_height_scale: ([-\d.]+)", line):
            heights.append((t, float(m.group(1))))
    return {"states": states, "gaits": gaits, "heights": heights}


@dataclass
class Session:
    t: np.ndarray  # row times (s)
    rec: dict[str, np.ndarray]  # recorded joint arrays (rows x 16)
    imu: dict[str, np.ndarray]
    imu_row: np.ndarray  # latest IMU sample index at or before each row
    events: dict[str, list]

    def missing_before(self, row: int) -> int:
        """Control ticks with no recorded frame between row - 1 and row."""
        return max(0, round((self.t[row] - self.t[row - 1]) * 1000.0) - 1)

    def state(self, row: int) -> RobotState:
        row = max(row, 0)
        k = max(int(self.imu_row[row]), 0)
        r, p, y = np.radians(self.imu["rpy_deg"][k])
        return RobotState(
            quat=quat_from_rpy(r, p, y),
            gyro=self.imu["gyro"][k].astype(np.float64),
            accel=self.imu["accel"][k].astype(np.float64),
            q=self.rec["q"][row].copy(),
            dq=self.rec["dq"][row].copy(),
            stamp=float(self.t[row]),
        )


def load_session(session: Path, cache: Path) -> Session:
    cache.mkdir(parents=True, exist_ok=True)
    rec = load_joints(session, cache)
    imu = load_imu(session, cache)
    t = rec.pop("t") / 1e9
    events = load_events(session)
    events["nav"] = [tuple(r) for r in load_commands(session, cache)]
    return Session(
        t=t, rec=rec, imu=imu, imu_row=np.searchsorted(imu["t"], t, side="right") - 1, events=events
    )


# ------------------------------------------------------------------------------------------------------- the plan


def gains_runs(s: Session) -> list[tuple[int, int, tuple]]:
    """Runs of rows with the same (leg kp, leg kd, wheel kd, control word)."""
    g = np.nan_to_num(
        np.c_[s.rec["kp"][:, 1], s.rec["kd"][:, 1], s.rec["kd"][:, 3], s.rec["word"][:, 0]], nan=-1
    )
    edges = [0, *(np.where(np.any(np.diff(g, axis=0) != 0, axis=1))[0] + 1), len(g)]
    return [(a, b, tuple(g[a])) for a, b in itertools.pairwise(edges)]


def last_before(events: list, t: float, default: Any = None) -> Any:
    out = default
    for e in events:
        if e[0] > t:
            break
        out = e
    return out


def gains_match(s: Session, cfg: dict[str, Any], runner: str, row: int) -> bool:
    """Whether rl_deploy's kp / kd on `row` are this runner's (or its start-up multiple on the legs)."""
    spec = cfg["runners"][runner]
    kp = np.resize(np.asarray(spec["kp"], float), JOINTS)
    kd = np.resize(np.asarray(spec["kd"], float), JOINTS)
    legs = ~IS_WHEEL
    for k in (1.0, spec.get("start_gain_scale", 1.0)):
        if (
            np.allclose(gain_quant(kp[legs] * k), s.rec["kp"][row, legs])
            and np.allclose(gain_quant(kd[legs] * k), s.rec["kd"][row, legs])
            and np.isclose(kd[3], s.rec["kd"][row, 3])
        ):
            return True
    return False


def build_plan(
    s: Session, cfg: dict[str, Any], lag: int, nav_normalized: bool | None, persist: bool = True
):
    """State segments (forced into the Controller at their first row) and policy steps (row, runner, inputs)."""
    rows_t = s.t
    enable_rows = [a for a, b, g in gains_runs(s) if g[3] == cfg["ecan_enable_word"]]
    segments, steps = [], []
    # state segments start at the enable frames rl_deploy sends on entering a state, or at the recording's start
    starts = [0, *enable_rows]
    for k, a in enumerate(starts):
        b = starts[k + 1] if k + 1 < len(starts) else len(rows_t)
        name = last_before(s.events["states"], rows_t[a] + 0.01, (0, "passive"))[1]
        segments.append({"state": name, "start": int(a), "end": int(b)})
    # policy steps: rows in rl_control where the leg targets changed
    qc = np.nan_to_num(s.rec["q_cmd"][:, [i for i in range(JOINTS) if i % 4 != 3]])
    change = np.r_[False, np.any(np.abs(np.diff(qc, axis=0)) > 0, axis=1)]
    rl = np.zeros(len(rows_t), bool)
    for seg in segments:
        if seg["state"] == "rl_control":
            first = seg["start"] + cfg["states"]["enable"]["ticks"]
            rl[first : seg["end"]] = True
            change[first] = True  # the first targets after the enable frames
    prev_runner, used, pose, prev_gait, stand_in_start = None, set(), {}, 0, 0.0
    # a policy step that left the targets unchanged (a warming-up sequence runner) still happened: keep the 20-tick
    # cadence between changes
    rows = []
    period = cfg["timing"]["policy_decimation"]
    changes = np.where(change & rl)[0]
    for k, row in enumerate(changes):
        rows.append(row)
        nxt = changes[k + 1] if k + 1 < len(changes) else None
        seg_end = next(g["end"] for g in segments if g["start"] <= row < g["end"])
        stop = min(nxt if nxt is not None else seg_end, seg_end)
        rows.extend(range(row + period, stop - period // 2, period))
    for row in rows:
        t = rows_t[row]
        gait = last_before(s.events["gaits"], t, (0, 0))[1]
        if gait == 0:
            continue
        runner = runner_for_gait(cfg, gait) or cfg["states"]["rl_control"]["fallback_runner"]
        # rl_deploy logs the gait it was asked for even while it forces the stand-in ("protect to sixaxis gait!"):
        # when the gains on the bus are the stand-in runner's and not the logged one's, the stand-in is running
        stand_in_runner = runner_for_gait(cfg, STAND_IN_GAIT)
        stand_in_for = None
        if (
            gait != STAND_IN_GAIT
            and stand_in_runner
            and not gains_match(s, cfg, runner, row)
            and gains_match(s, cfg, stand_in_runner, row)
        ):
            stand_in_for = gait
            gait, runner = STAND_IN_GAIT, stand_in_runner
        params: dict[str, float] = {}
        if gait == STAND_IN_GAIT:
            # the stand-in's params come from the gait it hands over to: params for its first first_s, then
            # settle_params until the handover
            if prev_gait != STAND_IN_GAIT:
                stand_in_start = t
            nxt = (
                (t, stand_in_for)
                if stand_in_for
                else next((e for e in s.events["gaits"] if e[0] > t), None)
            )
            spec = next(
                (
                    si
                    for si in cfg["states"]["rl_control"]["stand_in"]
                    if nxt and si["gait"] == nxt[1]
                ),
                None,
            )
            if spec is not None:
                first = spec["first_s"] if nxt[1] == 0x1001 else spec["later_s"]
                params.update(
                    spec["params"] if t - stand_in_start < first else spec["settle_params"]
                )
        prev_gait = gait
        nav = last_before(s.events["nav"], t)
        fresh = nav and t - nav[0] <= COMMAND_TIMEOUT_S
        vel = np.array(nav[1:4]) if fresh else np.zeros(3)
        metric = bool(nav[7]) if fresh else False
        if nav_normalized is not None:
            metric = not nav_normalized
        div = np.array(
            cfg["runners"][runner].get("real_steer_divisor", [1.0, 1.0, 1.0]), np.float64
        )
        axes = np.divide(vel, div, out=np.zeros(3), where=div > 0) if metric else vel
        if gait in STEER_POSE:
            # pose inputs stay where the last steer sample put them (the steer timeout only zeroes velocity)
            if fresh and not metric:
                axis = dict(zip(("x", "y", "yaw", "z", "roll", "pitch"), nav[1:7], strict=True))
                pose[gait] = {
                    param: k * axis[name] for param, (name, k) in STEER_POSE[gait].items()
                }
            params.update(pose.get(gait, {}))
            axes = np.zeros(3)
        if runner == "finetune_stand":
            h = last_before(s.events["heights"], t)
            params["height"] = h[1] if h else 0.0
        # rl_deploy keeps one object per runner: history and last action carry over to its next use
        reset = runner not in used if persist else runner != prev_runner
        used.add(runner)
        steps.append(
            {
                "row": int(row),
                "state_row": int(row - lag),
                "runner": runner,
                "reset": reset,
                "activate": runner != prev_runner,
                "axes": axes.tolist(),
                "params": params,
                "gait": gait,
            }
        )
        prev_runner = runner
    return segments, steps


# ------------------------------------------------------------------------------------------------------ our replay


def recorded_action(s: Session, spec: dict[str, Any], row: int) -> np.ndarray:
    """The network output that maps to the command rl_deploy sent on `row` (exact unless a target was clamped)."""
    act = spec["action"]
    q, dq = s.rec["q_cmd"][row], s.rec["dq_cmd"][row]
    default = np.array(spec["default_pose"], np.float64)
    scale = np.resize(np.asarray(act["leg_scale"], np.float64), 12)
    base = default if act["leg_mode"] == "offset" else np.zeros(JOINTS)
    a = np.zeros(JOINTS)
    if act["layout"] == "per_leg":
        legs = [i for i in range(JOINTS) if i % 4 != 3]
        a[legs] = (q[legs] - base[legs]) / scale
        a[3::4] = dq[3::4] / act["wheel_scale"]
    else:
        by_type = [0, 4, 8, 12, 1, 5, 9, 13, 2, 6, 10, 14]
        a[:12] = (q[by_type] - base[by_type]) / scale
        a[12:] = dq[3::4] / act["wheel_scale"]
    return a


def add_teacher_actions(s: Session, cfg: dict[str, Any], steps) -> None:
    """One-step mode: each step after the first of a run gets rl_deploy's previous action as its last action."""
    for k, st in enumerate(steps):
        prev = steps[k - 1] if k else None
        if prev is not None and prev["runner"] == st["runner"] and st["row"] - prev["row"] <= 40:
            st["last_action"] = recorded_action(
                s, cfg["runners"][st["runner"]], st["row"] - 1
            ).tolist()


def run_python(s: Session, cfg: dict[str, Any], segments, steps) -> dict[str, np.ndarray]:
    n = len(s.t)
    out = {f: np.full((n, JOINTS), np.nan) for f in ("kp", "q", "kd", "dq")}
    out["enable"] = np.zeros(n, bool)
    controller = Controller(cfg, policy_dir())
    controller.check_faults = (
        lambda _state: ""
    )  # the recorded states are what rl_deploy did; faults are its business
    targets_for = {
        "stand_up": "stand",
        "lie_down": "lie_down",
        "passive": "passive",
        "joint_damping": "joint_damping",
        "start_damping": "passive",
    }
    for seg in segments:
        if seg["state"] == "rl_control":
            continue
        cmd = UserCommand(target=targets_for.get(seg["state"], "passive"))
        controller.force(seg["state"], s.state(seg["start"] - 1), cmd, enable=seg["start"] > 0)
        controller.time = 1e9  # past every start-up gate
        for row in range(seg["start"], seg["end"]):
            # a tick whose frame was replaced (charge-enable requests) has no row: tick through it
            for _ in range(s.missing_before(row) if row > seg["start"] else 0):
                controller.tick(s.state(row - 1), cmd)
            c = controller.tick(s.state(row - 1), cmd)
            controller.state = seg[
                "state"
            ]  # hold the recorded state; transitions are compared separately
            store(out, row, c.kp, c.q, c.kd, c.dq, c.enable)
    # enable frames at rl_control entry
    for seg in segments:
        if seg["state"] == "rl_control":
            for row in range(
                seg["start"], min(seg["start"] + cfg["states"]["enable"]["ticks"], seg["end"])
            ):
                z = np.zeros(JOINTS)
                store(out, row, z, z, z, z, True)
    runners: dict[str, Runner] = {}
    for k, st in enumerate(steps):
        r = runners.get(st["runner"])
        if r is None:
            r = runners[st["runner"]] = Runner(
                st["runner"], cfg["runners"][st["runner"]], cfg["robot"], policy_dir()
            )
            r.policy_dt = cfg["timing"]["policy_decimation"] / cfg["timing"]["control_hz"]
        if st["reset"]:
            r.reset()
        if st["activate"]:
            r.activate()
        r.params.update(st["params"])
        if "last_action" in st and r.collected >= r.spec.get("warmup_steps", 0):
            r.last_action = np.array(st["last_action"])
        scale = np.where(IS_WHEEL, 1.0, r.gain_scale())
        t = r.step(s.state(st["state_row"]), np.array(st["axes"]))
        kp, kd = r.kp * scale, r.kd * scale
        kp[IS_WHEEL] = 0.0
        q = t.q.copy()
        q[IS_WHEEL] = 0.0
        dq = np.zeros(JOINTS)
        dq[IS_WHEEL] = t.wheel_velocity
        end = (
            steps[k + 1]["row"]
            if k + 1 < len(steps)
            else st["row"] + cfg["timing"]["policy_decimation"]
        )
        seg_end = next(g["end"] for g in segments if g["start"] <= st["row"] < g["end"])
        for row in range(st["row"], min(end, seg_end)):
            store(out, row, kp, q, kd, dq, False)
    return out


def store(out, row, kp, q, kd, dq, enable) -> None:
    out["kp"][row], out["q"][row], out["kd"][row], out["dq"][row] = kp, q, kd, dq
    out["enable"][row] = enable


def export_for_rust(s: Session, cfg: dict[str, Any], segments, steps, path: Path) -> None:
    """ticks.bin: per row f64 stamp, quat 4, gyro 3, accel 3, q 16, dq 16; plan.json: segments and steps."""
    path.mkdir(parents=True, exist_ok=True)
    rows = np.zeros((len(s.t), 43))
    for row in range(len(s.t)):
        st = s.state(row)
        rows[row] = np.r_[st.stamp, st.quat, st.gyro, st.accel, st.q, st.dq]
    rows.astype("<f8").tofile(path / "ticks.bin")
    (path / "plan.json").write_text(json.dumps({"segments": segments, "steps": steps}))


def load_rust(path: Path, n: int) -> dict[str, np.ndarray]:
    a = np.fromfile(path, "<f4").reshape(n, 65)
    return {
        "kp": a[:, 0:16],
        "q": a[:, 16:32],
        "kd": a[:, 32:48],
        "dq": a[:, 48:64],
        "enable": a[:, 64] > 0.5,
    }


# ----------------------------------------------------------------------------------------------------- comparison


def gain_quant(v: np.ndarray) -> np.ndarray:
    """kp / kd as the bus carries them: x10, f32, truncated."""
    return np.trunc(v.astype(np.float32) * np.float32(10.0)).astype(np.float64) / 10.0


def segment_label(seg, steps) -> str:
    if seg["state"] != "rl_control":
        return seg["state"]
    gaits = sorted({st["gait"] for st in steps if seg["start"] <= st["row"] < seg["end"]})
    return "rl_control " + "/".join(f"{g:#06x}" for g in gaits)


def compare(s: Session, ours: dict[str, np.ndarray], segments, steps, cfg) -> list[dict[str, Any]]:
    rec = s.rec
    rec_enable = rec["word"][:, 0] == cfg["ecan_enable_word"]
    report = []
    groups: list[tuple[str, int, int]] = []
    for seg in segments:
        if seg["state"] == "rl_control":
            a = seg["start"]
            runs = [
                (st["row"], st["runner"]) for st in steps if seg["start"] <= st["row"] < seg["end"]
            ]
            cur = None
            for row, runner in runs:
                if runner != cur:
                    if cur is not None:
                        groups.append((f"rl_control {cur}", a, row))
                    cur, a = runner, row
            groups.append((f"rl_control {cur}", a, seg["end"]))
            groups.append(("enable", seg["start"], seg["start"] + cfg["states"]["enable"]["ticks"]))
        else:
            groups.append((seg["state"], seg["start"], seg["end"]))
    for name, a, b in groups:
        rows = np.arange(a, b)
        rows = rows[~np.isnan(rec["q_cmd"][rows, 0]) & ~np.isnan(ours["q"][rows, 0])]
        if name != "enable":
            rows = rows[~rec_enable[rows]]
        if len(rows) == 0:
            continue
        e = {"segment": name, "start_s": round(float(s.t[a] - s.t[0]), 3), "rows": len(rows)}
        if name == "enable":
            # an enable frame is raw zeros on the bus (m20-ecan shows that as the calibration offsets): compare flags
            e.update(
                q_rms=[0.0] * 12,
                q_max=[0.0] * 12,
                q_worst_joint_median=0.0,
                q_worst_joint_p95=0.0,
                wheel_dq_rms=[0.0] * 4,
                wheel_dq_max=[0.0] * 4,
                leg_dq_max=0.0,
                kp_match=1.0,
                kd_match=1.0,
                enable_match=float(np.mean(ours["enable"][rows] & rec_enable[rows])),
            )
            report.append(e)
            continue
        dq_err = ours["q"][rows] - rec["q_cmd"][rows]
        legs = ~IS_WHEEL
        e["q_rms"] = np.sqrt(np.mean(dq_err[:, legs] ** 2, 0)).round(5).tolist()
        e["q_max"] = np.abs(dq_err[:, legs]).max(0).round(5).tolist()
        worst = np.abs(dq_err[:, legs]).max(1)
        e["q_worst_joint_median"] = float(np.median(worst).round(5))
        e["q_worst_joint_p95"] = float(np.percentile(worst, 95).round(5))
        e["exact"] = float(np.mean(worst < 1e-4))
        w_err = ours["dq"][rows][:, IS_WHEEL] - rec["dq_cmd"][rows][:, IS_WHEEL]
        e["wheel_dq_rms"] = np.sqrt(np.mean(w_err**2, 0)).round(4).tolist()
        e["wheel_dq_max"] = np.abs(w_err).max(0).round(4).tolist()
        l_err = ours["dq"][rows][:, legs] - rec["dq_cmd"][rows][:, legs]
        e["leg_dq_max"] = float(np.abs(l_err).max().round(5))
        e["kp_match"] = float(np.mean(np.all(gain_quant(ours["kp"][rows]) == rec["kp"][rows], 1)))
        e["kd_match"] = float(np.mean(np.all(gain_quant(ours["kd"][rows]) == rec["kd"][rows], 1)))
        e["enable_match"] = float(np.mean(ours["enable"][rows] == rec_enable[rows]))
        report.append(e)
    return report


# ----------------------------------------------------------------------------------------------------------- page


def write_page(
    s: Session,
    ours: dict[str, dict[str, np.ndarray]],
    report,
    path: Path,
    title: str,
    summary: str = "",
    name: str = "",
    also: list[Path] | None = None,
) -> None:
    """Interactive time plot: recorded vs our q_cmd per joint (50 Hz), wheel velocity targets, kp; earlier replays'
    page.json (in `also`) become more sessions on the same page."""
    also = also or []
    rows = np.arange(0, len(s.t), 20)
    t = (s.t[rows] - s.t[0]).round(3).tolist()
    series = {"t": t, "joints": JOINT_NAMES, "recorded": {}, "ours": {k: {} for k in ours}}
    for j, name in enumerate(JOINT_NAMES):
        field_rec, field_ours = ("dq_cmd", "dq") if j % 4 == 3 else ("q_cmd", "q")
        series["recorded"][name] = clean(s.rec[field_rec][rows, j])
        for k, o in ours.items():
            series["ours"][k][name] = clean(o[field_ours][rows, j])
    series["recorded"]["kp"] = clean(s.rec["kp"][rows, 1])
    for k, o in ours.items():
        series["ours"][k]["kp"] = clean(gain_quant(o["kp"][rows, 1]))
    table = "".join(
        f"<tr><td>{e['segment']}</td><td>{e['start_s']}</td><td>{e['rows']}</td>"
        f"<td>{e['q_worst_joint_median']:.5f}</td><td>{e['q_worst_joint_p95']:.4f}</td>"
        f"<td>{e.get('exact', 1.0):.3f}</td><td>{max(e['wheel_dq_rms']):.3f}</td>"
        f"<td>{e['kp_match']:.3f} / {e['kd_match']:.3f}</td><td>{e['impl']}</td></tr>"
        for e in report
    )
    section = {"name": name or title, "series": series, "table": table, "summary": summary}
    (path.parent / "page.json").write_text(json.dumps(section))
    sections = [json.loads((Path(d) / "page.json").read_text()) for d in also] + [section]
    compose(sections, path, title)


def compose(sections: list[dict], path: Path, title: str) -> None:
    """One page, a session picker over each replay's plot, summary and table."""
    html = PAGE.replace("__TITLE__", title).replace("__DATA__", json.dumps(sections))
    path.write_text(html)


def clean(a: np.ndarray) -> list:
    return [None if not np.isfinite(v) else round(float(v), 4) for v in a]


PAGE = """<title>__TITLE__</title>
<meta name="viewport" content="width=device-width, initial-scale=1">
<script src="https://cdn.jsdelivr.net/npm/plotly.js-dist-min@2.35.2/plotly.min.js"></script>
<style>
:root{--bg:#f7f8f6;--fg:#1c2421;--muted:#5d6b66;--line:#d5dbd8;--rec:#1c2421;--a:#1f7a8c;--b:#d1793b;--c:#6b9e3f;--d:#8a5fbf}
@media (prefers-color-scheme:dark){:root:not([data-theme="light"]){color-scheme:dark;--bg:#151a18;--fg:#e8eeeb;--muted:#9aa8a2;--line:#2e3734;--rec:#e8eeeb;--a:#4fb3c7;--b:#f0a163;--c:#9bcf6c;--d:#b48ce6}}
:root[data-theme="dark"]{color-scheme:dark;--bg:#151a18;--fg:#e8eeeb;--muted:#9aa8a2;--line:#2e3734;--rec:#e8eeeb;--a:#4fb3c7;--b:#f0a163;--c:#9bcf6c;--d:#b48ce6}
body{background:var(--bg);color:var(--fg);font:14px/1.5 -apple-system,system-ui,"Segoe UI",sans-serif;margin:0;padding-inline:16px;padding-block:20px}
main{max-width:1100px;margin:0 auto;display:flex;flex-direction:column;gap:14px}
h1{font-size:22px;margin:0;text-wrap:balance} p{color:var(--muted);margin:0;max-width:75ch}
.wrap{overflow-x:auto} table{border-collapse:collapse;font-size:12px;font-variant-numeric:tabular-nums}
td,th{border-bottom:1px solid var(--line);padding:4px 8px;text-align:left;white-space:nowrap}
label{font-weight:600} select{font-size:14px;margin-left:6px}
#plot{width:100%;height:480px}
</style>
<main>
<h1>__TITLE__</h1>
<div><label for="session">Session</label><select id="session"></select></div>
<div id="summary"></div>
<div><label for="joint">Joint</label><select id="joint"></select></div>
<div id="plot"></div>
<p>rl_deploy's command (solid) vs ours, sampled at 50 Hz. Legs: position target (rad); wheels: velocity target (rad/s).
Lines on top of each other is the goal.</p>
<p>Per segment: the worst joint's typical (median) and 95th-percentile position-target error in rad, the share of ticks
within 0.0001 rad, the worst wheel velocity-target RMS (rad/s), and the share of ticks whose kp / kd match exactly.
Lower errors and 1.000 shares are better.</p>
<div class="wrap"><table><thead><tr><th>segment</th><th>start (s)</th><th>ticks</th><th>median</th><th>p95</th><th>exact</th><th>wheel RMS</th><th>kp / kd match</th><th>impl</th></tr></thead><tbody id="rows"></tbody></table></div>
</main>
<script>
const ALL=__DATA__;
let D=ALL[ALL.length-1].series;
const sel=document.getElementById('joint'), ses=document.getElementById('session');
ALL.forEach((x,i)=>{const o=document.createElement('option');o.value=i;o.textContent=x.name;ses.appendChild(o)});
ses.value=ALL.length-1;
[...D.joints,'kp'].forEach(j=>{const o=document.createElement('option');o.value=j;o.textContent=j==='kp'?'kp (fl_hipy)':j;sel.appendChild(o)});
function pick(){const x=ALL[ses.value];D=x.series;document.getElementById('summary').innerHTML=x.summary;
 document.getElementById('rows').innerHTML=x.table;draw();}
ses.onchange=pick;
function css(v){return getComputedStyle(document.documentElement).getPropertyValue(v).trim()}
function draw(){const j=sel.value;const fg=css('--fg');
 const tr=[{x:D.t,y:D.recorded[j],name:'rl_deploy',line:{color:css('--rec'),width:2.5}}];
 Object.keys(D.ours).forEach((k,i)=>tr.push({x:D.t,y:D.ours[k][j],name:k,line:{color:css(['--a','--b','--c','--d'][i%4]),width:1.2,dash:k.includes('one-step')?'dot':'solid'}}));
 Plotly.react('plot',tr,{margin:{t:10,r:10,b:40,l:50},paper_bgcolor:'rgba(0,0,0,0)',plot_bgcolor:'rgba(0,0,0,0)',
  font:{color:fg},xaxis:{title:'time since recording start (s)',gridcolor:css('--line')},yaxis:{gridcolor:css('--line')},legend:{orientation:'h'}},{responsive:true});}
sel.onchange=draw; pick();
matchMedia('(prefers-color-scheme: dark)').addEventListener('change',draw);
</script>"""


# ------------------------------------------------------------------------------------------------------------ main


def main() -> None:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("session", type=Path)
    parser.add_argument(
        "--out", type=Path, required=True, help="cache, Rust inputs / outputs, report and page"
    )
    parser.add_argument("--lag", type=int, default=1, help="policy state row = target row - lag")
    parser.add_argument(
        "--commands",
        choices=("auto", "metric", "normalized"),
        default="auto",
        help="read every command writer as m/s or as normalized axes (default: per writer)",
    )
    parser.add_argument(
        "--reset-on-switch", action="store_true", help="fresh runner state on every gait switch"
    )
    parser.add_argument("--rust", type=Path, help="the controller's replay binary")
    parser.add_argument("--summary", type=Path, help="HTML fragment shown above the plot")
    parser.add_argument("--name", help="this session's name on the page")
    parser.add_argument(
        "--also", type=Path, nargs="*", default=[], help="earlier replay --out dirs to include"
    )
    args = parser.parse_args()
    cfg = load_config()
    cfg["ecan_enable_word"] = 2
    s = load_session(args.session, args.out / "cache")
    normalized = {"auto": None, "metric": False, "normalized": True}[args.commands]
    segments, steps = build_plan(s, cfg, args.lag, normalized, not args.reset_on_switch)
    results = {"python": run_python(s, cfg, segments, steps)}
    if args.rust:
        export_for_rust(s, cfg, segments, steps, args.out / "rust")
        subprocess.run([str(args.rust), str(args.out / "rust")], check=True)
        results["rust"] = load_rust(args.out / "rust" / "out.bin", len(s.t))
    # one-step: the same, with rl_deploy's own previous action fed back (isolates the per-step pipeline)
    add_teacher_actions(s, cfg, steps)
    results["python one-step"] = run_python(s, cfg, segments, steps)
    if args.rust:
        export_for_rust(s, cfg, segments, steps, args.out / "rust")
        subprocess.run([str(args.rust), str(args.out / "rust")], check=True)
        results["rust one-step"] = load_rust(args.out / "rust" / "out.bin", len(s.t))
    report = []
    for impl, o in results.items():
        for e in compare(s, o, segments, steps, cfg):
            e["impl"] = impl
            report.append(e)
    (args.out / "report.json").write_text(json.dumps(report, indent=1))
    for e in report:
        print(
            f"{e['impl']:<15} {e['segment']:<28} t={e['start_s']:>8} n={e['rows']:>6} q_rms_max={max(e['q_rms']):.5f} "
            f"med={e['q_worst_joint_median']:.5f} p95={e['q_worst_joint_p95']:.5f} "
            f"q_max={max(e['q_max']):.5f} wheel_rms_max={max(e['wheel_dq_rms']):.4f} leg_dq_max={e['leg_dq_max']:.4f} "
            f"kp={e['kp_match']:.3f} kd={e['kd_match']:.3f} enable={e['enable_match']:.3f}"
        )
    summary = args.summary.read_text() if args.summary else ""
    write_page(
        s,
        results,
        report,
        args.out / "replay.html",
        "M20 rl_deploy replay",
        summary,
        name=args.name or args.session.name,
        also=args.also,
    )
    print(f"wrote {args.out / 'replay.html'}")


if __name__ == "__main__":
    main()
