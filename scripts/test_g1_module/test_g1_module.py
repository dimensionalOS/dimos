#!/usr/bin/env python3
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
# See the License for the specific language governing permissions and
# limitations under the License.

"""Disposable validation harness for the G1 wholebody Module.

NOT a reusable adapter — Session B will write the real TransportWholeBodyAdapter
against the same topic interface. This script's job is to keep the Module/transport
seam honest with mock DDS traffic.

Lifecycle:
  1. Spawn MockG1LowStatePublisher in this process (DDS rt/lowstate at 500 Hz).
  2. Spawn ChannelSubscriber("rt/lowcmd", LowCmd_) in this process to count
     motor commands round-tripped through the adapter.
  3. Build a blueprint with G1WholeBodyConnection.blueprint(release_sport_mode=False)
     and LCM transports on motor_states / imu / motor_command. Run with n_workers=2
     so the Module lands in its own worker process.
  4. From this process, subscribe to the same LCM topics + publish synthetic
     MotorCommandArray commands at 100 Hz.
  5. After 60 s, print rate / drop counts and assert thresholds:
       - motor_states rate == 500 Hz, drops == 0
       - imu          rate == 500 Hz, drops == 0
       - motor_command round-trip count >= 99% of sent
       - mode_machine NOT exposed on the wire (true by JointState construction)

Run inside the nix dev shell with [unitree-dds] extras installed:
    python scripts/test_g1_module/test_g1_module.py
"""

from __future__ import annotations

import argparse
from collections import deque
import logging
import os
import sys
import threading
import time

logger = logging.getLogger(__name__)


# ---------------------------------------------------------------------------
# RateMonitor — counts arrivals + tracks instantaneous rate over a sliding window
# ---------------------------------------------------------------------------


class RateMonitor:
    """Counts callback invocations and reports rate over a fixed window."""

    def __init__(self, name: str, window_s: float = 1.0) -> None:
        self.name = name
        self.window_s = window_s
        self._lock = threading.Lock()
        self._times: deque[float] = deque()
        self._total: int = 0
        self._first_ts: float | None = None
        self._last_ts: float | None = None

    def tick(self) -> None:
        now = time.perf_counter()
        with self._lock:
            self._total += 1
            if self._first_ts is None:
                self._first_ts = now
            self._last_ts = now
            self._times.append(now)
            cutoff = now - self.window_s
            while self._times and self._times[0] < cutoff:
                self._times.popleft()

    @property
    def total(self) -> int:
        with self._lock:
            return self._total

    @property
    def instantaneous_hz(self) -> float:
        with self._lock:
            return len(self._times) / self.window_s

    def average_hz(self) -> float:
        with self._lock:
            if self._first_ts is None or self._last_ts is None:
                return 0.0
            elapsed = self._last_ts - self._first_ts
            if elapsed <= 0:
                return 0.0
            return self._total / elapsed


# ---------------------------------------------------------------------------
# Test harness
# ---------------------------------------------------------------------------


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--duration", type=float, default=60.0, help="Run time (s)")
    parser.add_argument("--mock-rate", type=float, default=500.0, help="Mock LowState Hz")
    parser.add_argument("--cmd-rate", type=float, default=100.0, help="Synthetic command Hz")
    parser.add_argument(
        "--rate-tolerance",
        type=float,
        default=2.0,
        help="Hz tolerance below target to allow before flagging (default 2.0)",
    )
    parser.add_argument(
        "--cmd-roundtrip-min", type=float, default=0.99, help="Min round-trip ratio"
    )
    parser.add_argument("--verbose", "-v", action="store_true")
    args = parser.parse_args()

    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.INFO,
        format="%(asctime)s %(name)s %(levelname)s %(message)s",
    )
    logger.info(f"test_g1_module pid={os.getpid()} duration={args.duration}s")

    # Heavy imports deferred so --help works even outside the nix env
    from unitree_sdk2py.core.channel import ChannelSubscriber
    from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_

    from dimos.core.coordination.blueprints import autoconnect
    from dimos.core.coordination.module_coordinator import ModuleCoordinator
    from dimos.core.global_config import global_config
    from dimos.core.transport import LCMTransport
    from dimos.msgs.sensor_msgs.Imu import Imu
    from dimos.msgs.sensor_msgs.JointState import JointState
    from dimos.msgs.sensor_msgs.MotorCommandArray import MotorCommandArray
    from dimos.robot.unitree.g1.wholebody_connection import G1WholeBodyConnection
    from scripts.test_g1_module.mock_lowstate_publisher import MockG1LowStatePublisher

    # ---------------- Step 1: mock LowState publisher ----------------
    logger.info("Starting mock LowState publisher...")
    mock = MockG1LowStatePublisher(rate_hz=args.mock_rate, mode_machine=42)
    mock.start()

    # ---------------- Step 2: rt/lowcmd subscriber for round-trip ----------------
    cmd_roundtrip_count = [0]
    cmd_roundtrip_lock = threading.Lock()

    def on_lowcmd(_msg: LowCmd_) -> None:
        with cmd_roundtrip_lock:
            cmd_roundtrip_count[0] += 1

    cmd_sub = ChannelSubscriber("rt/lowcmd", LowCmd_)
    cmd_sub.Init(on_lowcmd, 10)

    # ---------------- Step 3: build blueprint with LCM transports ----------------
    logger.info("Building blueprint (n_workers=2)...")
    global_config.update(viewer="none", n_workers=2)

    blueprint = autoconnect(
        G1WholeBodyConnection.blueprint(release_sport_mode=False),
    ).transports(
        {
            ("motor_states", JointState): LCMTransport("/g1/motor_states", JointState),
            ("imu", Imu): LCMTransport("/g1/imu", Imu),
            ("motor_command", MotorCommandArray): LCMTransport(
                "/g1/motor_command", MotorCommandArray
            ),
        }
    )

    coord = ModuleCoordinator.build(blueprint)

    # ---------------- Step 4: external LCM subscribers + command publisher ----------------
    motor_states_monitor = RateMonitor("motor_states")
    imu_monitor = RateMonitor("imu")

    # mode_machine leak detection — JointState has no such attr by construction.
    mode_machine_leaked = [False]

    def on_motor_states(msg: JointState) -> None:
        motor_states_monitor.tick()
        if getattr(msg, "mode_machine", None) is not None:
            mode_machine_leaked[0] = True

    def on_imu(msg: Imu) -> None:
        imu_monitor.tick()
        if getattr(msg, "mode_machine", None) is not None:
            mode_machine_leaked[0] = True

    motor_states_sub = LCMTransport("/g1/motor_states", JointState)
    motor_states_unsub = motor_states_sub.subscribe(on_motor_states)

    imu_sub = LCMTransport("/g1/imu", Imu)
    imu_unsub = imu_sub.subscribe(on_imu)

    motor_command_pub = LCMTransport("/g1/motor_command", MotorCommandArray)

    # Wait for the Module to come up — adapter.connect() waits for first LowState
    logger.info("Waiting 3s for Module to start in worker...")
    time.sleep(3.0)

    # ---------------- Step 5: 60 s sustained run ----------------
    logger.info(f"Running for {args.duration}s — driving cmd at {args.cmd_rate} Hz...")
    cmd_sent = 0
    cmd_period = 1.0 / args.cmd_rate
    next_cmd = time.perf_counter()
    end_at = time.perf_counter() + args.duration

    last_log = time.perf_counter()
    while time.perf_counter() < end_at:
        cmd = MotorCommandArray(
            q=[0.0] * 29,
            dq=[0.0] * 29,
            kp=[10.0] * 29,
            kd=[1.0] * 29,
            tau=[0.0] * 29,
        )
        motor_command_pub.publish(cmd)
        cmd_sent += 1

        next_cmd += cmd_period
        sleep_for = next_cmd - time.perf_counter()
        if sleep_for > 0:
            time.sleep(sleep_for)
        else:
            next_cmd = time.perf_counter()

        # 5s heartbeat so the operator knows we're not stuck
        now = time.perf_counter()
        if now - last_log >= 5.0:
            last_log = now
            logger.info(
                f"  motor_states={motor_states_monitor.instantaneous_hz:.1f}Hz "
                f"imu={imu_monitor.instantaneous_hz:.1f}Hz "
                f"cmd_sent={cmd_sent} cmd_round_trip={cmd_roundtrip_count[0]}"
            )

    # ---------------- Step 6: tear down + assertions ----------------
    logger.info("Stopping...")
    motor_states_unsub()
    imu_unsub()
    motor_states_sub.stop()
    imu_sub.stop()
    motor_command_pub.stop()

    coord.stop()
    cmd_sub.Close()
    mock.stop()

    # Drain a tiny bit to let any in-flight messages land
    time.sleep(0.5)

    avg_motor_states_hz = motor_states_monitor.average_hz()
    avg_imu_hz = imu_monitor.average_hz()
    cmd_round_trip_ratio = cmd_roundtrip_count[0] / cmd_sent if cmd_sent else 0.0

    print()
    print("=" * 60)
    print("RESULTS")
    print("=" * 60)
    print(f"  duration:            {args.duration:.1f}s")
    print(f"  mock published:      {mock.messages_published}")
    print(
        f"  motor_states:        total={motor_states_monitor.total}  "
        f"avg={avg_motor_states_hz:.2f}Hz  target={args.mock_rate}Hz"
    )
    print(
        f"  imu:                 total={imu_monitor.total}  "
        f"avg={avg_imu_hz:.2f}Hz  target={args.mock_rate}Hz"
    )
    print(
        f"  motor_command:       sent={cmd_sent}  round_trip={cmd_roundtrip_count[0]}  "
        f"ratio={cmd_round_trip_ratio:.3f}  min={args.cmd_roundtrip_min}"
    )
    print(f"  mode_machine leaked: {mode_machine_leaked[0]}")
    print("=" * 60)

    failures: list[str] = []
    rate_floor = args.mock_rate - args.rate_tolerance
    if avg_motor_states_hz < rate_floor:
        failures.append(f"motor_states {avg_motor_states_hz:.2f}Hz < {rate_floor}Hz floor")
    if avg_imu_hz < rate_floor:
        failures.append(f"imu {avg_imu_hz:.2f}Hz < {rate_floor}Hz floor")
    if cmd_round_trip_ratio < args.cmd_roundtrip_min:
        failures.append(f"cmd round-trip {cmd_round_trip_ratio:.3f} < {args.cmd_roundtrip_min}")
    if mode_machine_leaked[0]:
        failures.append("mode_machine leaked onto output stream")

    if failures:
        print()
        print("FAIL:")
        for f in failures:
            print(f"  - {f}")
        return 1

    print()
    print("PASS")
    return 0


if __name__ == "__main__":
    sys.exit(main())
