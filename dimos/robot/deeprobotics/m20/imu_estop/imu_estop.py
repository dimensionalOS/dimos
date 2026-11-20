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

"""Temporary high-g e-stop for the M20: watches both Airy lidar IMUs and latches an e-stop on a high-g or high-rate run.

Standalone, stdlib only, Python 3.8 (AOS's system python), so it runs beside whatever blueprint is up without a
second ``dimos run``. It reads the Airys' IMU multicast (51-byte packets, 200 Hz each) like any other listener.

A sample is "over" when its specific-force magnitude (g) or gyro norm (rad/s) passes its threshold; a stream that
has ``consecutive_samples`` overs in a row trips. A trip latches (in memory and in ``latch_file``) until ``reset``.
Unarmed (the default) a trip is only logged to ``event_log``. Armed, every trip:
  - sends basic_server the vendor soft e-stop (Type 2 / Command 22, MotionParam 2: joint damping, then Idle after
    3 s), which does nothing while rl_deploy is stopped, since nothing else drives the motors then, and
  - SIGKILLs every process whose name contains one of ``kill_process_names`` (the motor-bus test senders).

    sudo python3 imu_estop.py run [--config c.json] [--armed]
    sudo python3 imu_estop.py reset
"""

from __future__ import annotations

import argparse
from dataclasses import asdict, dataclass, field
from datetime import datetime
import json
import math
import os
import signal
import socket
import struct
import sys
import time

AIRY_IMU_LEN = 51
AIRY_IMU_ID = bytes([0xAA, 0x55, 0x5A, 0x05])


@dataclass
class EstopConfig:
    armed: bool = False
    # walking peaks of the 2026-11-20 session's body IMU over 4 samples (20 ms): 2.49 g, 1.42 rad/s
    accel_threshold_g: float = 3.8
    gyro_threshold_rad_s: float | None = 4.0
    consecutive_samples: int = 4
    # (name, multicast group, port) per Airy IMU, from the vendor node_driver config.yaml
    streams: list[tuple[str, str, int]] = field(
        default_factory=lambda: [
            ("front_lidar", "224.10.10.201", 6681),
            ("rear_lidar", "224.10.10.202", 6682),
        ]
    )
    # AOS's lidar-side address (eth0), where the multicast groups are joined
    interface_ip: str = "10.21.33.103"
    basic_server_host: str = "10.21.31.103"
    basic_server_port: int = 30000
    kill_process_names: list[str] = field(
        default_factory=lambda: ["send_charge_enable", "zero_gain"]
    )
    state_dir: str = "/var/opt/robot/data/imu_estop"

    @property
    def latch_file(self) -> str:
        return os.path.join(self.state_dir, "tripped.json")

    @property
    def event_log(self) -> str:
        return os.path.join(self.state_dir, "events.jsonl")

    @property
    def pid_file(self) -> str:
        return os.path.join(self.state_dir, "imu_estop.pid")


class HighGLatch:
    """Per-stream run counters and the latch: ``update`` returns a trip event when a stream completes a run."""

    def __init__(self, config: EstopConfig) -> None:
        self.config = config
        self.runs: dict[str, int] = {}
        self.tripped: dict | None = None

    def update(
        self,
        stream: str,
        accel_g: tuple[float, float, float],
        gyro_rad_s: tuple[float, float, float],
    ) -> dict | None:
        accel = math.sqrt(sum(v * v for v in accel_g))
        gyro = math.sqrt(sum(v * v for v in gyro_rad_s))
        limit = self.config.gyro_threshold_rad_s
        over = accel > self.config.accel_threshold_g or (limit is not None and gyro > limit)
        run = self.runs.get(stream, 0) + 1 if over else 0
        if run < self.config.consecutive_samples:
            self.runs[stream] = run
            return None
        self.runs[stream] = 0
        event = {
            "stream": stream,
            "accel_g": round(accel, 3),
            "gyro_rad_s": round(gyro, 3),
            "first": self.tripped is None,
        }
        if self.tripped is None:
            self.tripped = event
        return event

    def reset(self) -> None:
        self.tripped = None
        self.runs.clear()


def parse_airy_imu(
    packet: bytes,
) -> tuple[tuple[float, float, float], tuple[float, float, float]] | None:
    """(accel g, gyro rad/s) of an Airy IMU packet, scaled as rs_driver / robosense-airy do, or None."""
    if len(packet) != AIRY_IMU_LEN or packet[:4] != AIRY_IMU_ID:
        return None
    accel_unit = ((1 << (packet[43] + 1)) & 0xFF) / 32768.0
    gyro_unit = ((250 * (1 << packet[44])) & 0xFFFF) / 32768.0 * math.pi / 180.0
    raw = struct.unpack(">3i3i", packet[14:38])
    accel = (raw[0] * accel_unit, raw[1] * accel_unit, raw[2] * accel_unit)
    gyro = (raw[3] * gyro_unit, raw[4] * gyro_unit, raw[5] * gyro_unit)
    return accel, gyro


def soft_estop_frame(message_id: int) -> bytes:
    """basic_server's motion-state switch to Soft Emergency Stop (dev guide 1.2.3), as m20tool frames its commands."""
    payload = json.dumps(
        {
            "PatrolDevice": {
                "Type": 2,
                "Command": 22,
                "Time": datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
                "Items": {"MotionParam": 2},
            }
        },
        separators=(",", ":"),
    ).encode()
    return (
        struct.pack(
            "<4sHHB7s", bytes.fromhex("eb91eb90"), len(payload), message_id & 0xFFFF, 1, b"\0" * 7
        )
        + payload
    )


def kill_processes(names: list[str]) -> list[str]:
    """SIGKILL every process (not this one) whose comm or argv[0] basename contains one of ``names``."""
    killed = []
    for pid in filter(str.isdigit, os.listdir("/proc")):
        if int(pid) == os.getpid():
            continue
        try:
            with open(f"/proc/{pid}/cmdline", "rb") as f:
                argv0 = os.path.basename(f.read().split(b"\0")[0].decode(errors="replace"))
            with open(f"/proc/{pid}/comm") as f:
                comm = f.read().strip()
        except OSError:
            continue
        if any(name in argv0 or name in comm for name in names):
            try:
                os.kill(int(pid), signal.SIGKILL)
                killed.append(f"{pid}:{argv0 or comm}")
            except OSError as exc:
                killed.append(f"{pid}:{argv0 or comm} failed: {exc}")
    return killed


class Estop:
    def __init__(self, config: EstopConfig) -> None:
        self.config = config
        self.latch = HighGLatch(config)
        self.sent = 0
        self.receiving: set = set()
        os.makedirs(config.state_dir, exist_ok=True)
        if os.path.exists(config.latch_file):
            with open(config.latch_file) as f:
                self.latch.tripped = json.load(f)
            self.log({"event": "start_latched", "tripped": self.latch.tripped})

    def log(self, record: dict) -> None:
        record = {"time": datetime.now().isoformat(timespec="milliseconds"), **record}
        line = json.dumps(record)
        print(line, flush=True)
        with open(self.config.event_log, "a") as f:
            f.write(line + "\n")

    def on_sample(
        self,
        stream: str,
        accel_g: tuple[float, float, float],
        gyro_rad_s: tuple[float, float, float],
    ) -> None:
        if stream not in self.receiving:
            self.receiving.add(stream)
            self.log(
                {
                    "event": "receiving",
                    "stream": stream,
                    "accel_g": accel_g,
                    "gyro_rad_s": gyro_rad_s,
                }
            )
        event = self.latch.update(stream, accel_g, gyro_rad_s)
        if event is None:
            return
        if event["first"]:
            with open(self.config.latch_file, "w") as f:
                json.dump(event, f)
        actions = self.trip() if self.config.armed else ["unarmed: logged only"]
        self.log({"event": "estop", "armed": self.config.armed, **event, "actions": actions})

    def trip(self) -> list[str]:
        actions = []
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as udp:
                self.sent += 1
                udp.sendto(
                    soft_estop_frame(self.sent),
                    (self.config.basic_server_host, self.config.basic_server_port),
                )
            actions.append("soft e-stop sent to basic_server")
        except OSError as exc:
            actions.append(f"soft e-stop send failed: {exc}")
        actions.append(f"killed {kill_processes(self.config.kill_process_names)}")
        return actions

    def reset(self) -> None:
        self.latch.reset()
        if os.path.exists(self.config.latch_file):
            os.remove(self.config.latch_file)
        self.log({"event": "reset"})


def _open_streams(config: EstopConfig) -> dict[socket.socket, str]:
    sockets = {}
    for name, group, port in config.streams:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind((group, port))
        membership = socket.inet_aton(group) + socket.inet_aton(config.interface_ip)
        sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, membership)
        sockets[sock] = name
    return sockets


def run(config: EstopConfig) -> None:
    import selectors

    estop = Estop(config)
    with open(config.pid_file, "w") as f:
        f.write(str(os.getpid()))
    signal.signal(signal.SIGUSR1, lambda *_: estop.reset())
    estop.log({"event": "start", "pid": os.getpid(), "config": asdict(config)})
    selector = selectors.DefaultSelector()
    for sock, name in _open_streams(config).items():
        selector.register(sock, selectors.EVENT_READ, name)
    while True:
        for key, _mask in selector.select():
            sample = parse_airy_imu(key.fileobj.recv(2048))
            if sample is not None:
                estop.on_sample(key.data, *sample)


def load_config(path: str | None, armed: bool) -> EstopConfig:
    overrides = {}
    if path:
        with open(path) as f:
            overrides = json.load(f)
    config = EstopConfig(**overrides)
    config.streams = [tuple(s) for s in config.streams]
    config.armed = config.armed or armed
    return config


def main(argv: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("command", choices=["run", "reset"])
    parser.add_argument("--config", help="json of EstopConfig overrides")
    parser.add_argument(
        "--armed", action="store_true", help="act on a trip (soft e-stop + kill test senders)"
    )
    args = parser.parse_args(argv)
    config = load_config(args.config, args.armed)
    if args.command == "run":
        run(config)
        return
    try:
        with open(config.pid_file) as f:
            os.kill(int(f.read()), signal.SIGUSR1)
    except (OSError, ValueError):
        # not running: clearing the file is the whole reset
        if os.path.exists(config.latch_file):
            os.remove(config.latch_file)
    time.sleep(0.2)
    print("latched:", os.path.exists(config.latch_file))


if __name__ == "__main__":
    sys.exit(main())
