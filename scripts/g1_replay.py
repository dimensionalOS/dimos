#!/usr/bin/env python3
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

# Copyright 2025-2026 Dimensional Inc.
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
"""Replay a recorded 29-DOF G1 joint trajectory through the coordinator.

Publishes JointState to /g1/joint_command. Flow:

    g1_replay.py
        | JointState (29 names + 29 positions)
        v  /g1/joint_command  (LCM)
    ControlCoordinator (servo_g1 task, tick_rate=500)
        |  per-joint position commands → MotorCommand with built-in PD gains
        v  TransportWholeBodyAdapter
        |  MotorCommandArray
        v  /g1/motor_command  (LCM)
    G1WholeBodyConnection
        v  DDS to robot

Joint name remapping: the trajectory file (recorded on the playback branch) uses
``g1_LeftHipPitch`` style names, but this branch's coordinator expects
``g1/left_hip_pitch``. Both lists share the same positional ordering — defined
once in ``_HUMANOID_29DOF_JOINTS`` — so we ignore the file's joint_names entirely
and zip its position arrays onto the canonical list from
``make_humanoid_joints("g1")``.

Trajectory file format:
    {
        "joint_names": [...29 names...],   # ignored — only used for length check
        "samples": [{"ts": float, "position": [...29 floats...]}, ...]
    }

Usage:
    # Terminal 1:
    ROBOT_INTERFACE=enp194s0 dimos run unitree-g1-coordinator
    # Terminal 2:
    python scripts/g1_replay.py --file /tmp/g1_traj.json
    python scripts/g1_replay.py --file /tmp/g1_traj.json --loop
    python scripts/g1_replay.py --file /tmp/g1_traj.json --dry-run
"""

from __future__ import annotations

import argparse
import json
import logging
from pathlib import Path
import threading
import time

from dimos.control.components import make_humanoid_joints
from dimos.core.transport import LCMTransport
from dimos.msgs.sensor_msgs.JointState import JointState

NUM_DOF = 29
CANONICAL_JOINTS = make_humanoid_joints("g1")
assert len(CANONICAL_JOINTS) == NUM_DOF

logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(levelname)s] %(message)s")
logger = logging.getLogger("g1_replay")


def load_trajectory(path: Path) -> tuple[list[float], list[list[float]]]:
    data = json.loads(path.read_text())
    joint_names = data.get("joint_names", [])
    samples = data.get("samples", [])
    if len(joint_names) != NUM_DOF:
        raise ValueError(f"trajectory has {len(joint_names)} joint_names, expected {NUM_DOF}")
    if not samples:
        raise ValueError("trajectory has zero samples")
    for i, s in enumerate(samples):
        if len(s["position"]) != NUM_DOF:
            raise ValueError(f"sample {i} has {len(s['position'])} positions, expected {NUM_DOF}")
    t0 = samples[0]["ts"]
    rel_ts = [s["ts"] - t0 for s in samples]
    positions = [list(s["position"]) for s in samples]
    return rel_ts, positions


def make_joint_state(positions: list[float]) -> JointState:
    return JointState(
        name=list(CANONICAL_JOINTS),
        position=list(positions),
        velocity=[0.0] * NUM_DOF,
        effort=[0.0] * NUM_DOF,
    )


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--file", default="/tmp/g1_traj.json", help="trajectory JSON path")
    p.add_argument(
        "--ramp",
        type=float,
        default=3.0,
        help="seconds to interpolate from current pose to first sample",
    )
    p.add_argument("--loop", action="store_true", help="repeat trajectory until Ctrl+C")
    p.add_argument(
        "--dry-run", action="store_true", help="subscribe + log but do NOT publish commands"
    )
    args = p.parse_args()

    rel_ts, positions = load_trajectory(Path(args.file))
    native_rate = len(rel_ts) / rel_ts[-1] if rel_ts[-1] > 0 else 0.0
    logger.info(
        f"loaded {len(rel_ts)} samples, duration={rel_ts[-1]:.2f}s, native_rate={native_rate:.1f}Hz"
    )
    logger.info(f"first sample[0:3]={positions[0][:3]}  last sample[0:3]={positions[-1][:3]}")
    logger.info(
        f"will publish to /g1/joint_command using canonical joint names ({CANONICAL_JOINTS[0]} ...)"
    )

    # Subscribe to coordinator's joint_state output to capture the current pose.
    # Coordinator joint state names are {hardware_id}/{joint} so they align with
    # CANONICAL_JOINTS, lookup is straightforward.
    state_lock = threading.Lock()
    current_q: dict[str, float] = {}
    state_event = threading.Event()

    def on_state(msg: JointState) -> None:
        with state_lock:
            current_q.clear()
            for n, q in zip(msg.name, msg.position, strict=True):
                current_q[n] = q
        state_event.set()

    state_sub = LCMTransport("/coordinator/joint_state", JointState)
    state_unsub = state_sub.subscribe(on_state)
    cmd_pub = LCMTransport("/g1/joint_command", JointState)

    try:
        logger.info("waiting up to 10s for /coordinator/joint_state ...")
        if not state_event.wait(timeout=10.0):
            logger.error("no joint_state received — is `dimos run unitree-g1-coordinator` running?")
            return
        with state_lock:
            start_q = [current_q.get(n, 0.0) for n in CANONICAL_JOINTS]
            missing = [n for n in CANONICAL_JOINTS if n not in current_q]
        if missing:
            logger.warning(
                f"coordinator joint_state missing {len(missing)} of {NUM_DOF} canonical joints: {missing[:3]}..."
            )
            logger.warning("will treat missing joints as 0.0 — check joint name conventions")
        logger.info(f"current pose[0:3]={start_q[0:3]}")

        if args.dry_run:
            logger.info("--dry-run set — exiting before publish phase")
            return

        # ---------------- Ramp ----------------
        logger.info(f"ramping current → trajectory[0] over {args.ramp:.2f}s")
        ramp_period = 1.0 / 100.0  # 100 Hz during ramp
        ramp_start = time.perf_counter()
        target_q = positions[0]
        while True:
            elapsed = time.perf_counter() - ramp_start
            a = min(elapsed / args.ramp, 1.0)
            interp = [start_q[i] + a * (target_q[i] - start_q[i]) for i in range(NUM_DOF)]
            cmd_pub.publish(make_joint_state(interp))
            if a >= 1.0:
                break
            time.sleep(ramp_period)

        # ---------------- Replay ----------------
        passes = 0
        while True:
            passes += 1
            logger.info(f"playback pass {passes} (Ctrl+C to stop)")
            replay_start = time.perf_counter()
            last_log = replay_start
            for i, (sample_t, q) in enumerate(zip(rel_ts, positions, strict=True)):
                wait = sample_t - (time.perf_counter() - replay_start)
                if wait > 0:
                    time.sleep(wait)
                cmd_pub.publish(make_joint_state(q))
                now = time.perf_counter()
                if now - last_log >= 2.0:
                    logger.info(f"  t={sample_t:6.2f}s  sample={i}/{len(rel_ts)}")
                    last_log = now
            if not args.loop:
                break

        logger.info("trajectory complete (last pose held by coordinator's last command)")

    except KeyboardInterrupt:
        logger.info("Ctrl+C — exiting; coordinator holds last published target")
    finally:
        state_unsub()
        state_sub.stop()
        cmd_pub.stop()


if __name__ == "__main__":
    main()
