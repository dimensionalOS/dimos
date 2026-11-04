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

"""The native binary replays a synthetic RS-Airy capture and publishes clouds and IMU.

The decoder itself is tested against a real Airy capture in the robosense-lidar crate;
this checks the module's wiring: config over stdin, pcap source, LCM output layout.
"""

import json
import math
import os
from pathlib import Path
import signal
import struct
import subprocess
import time

import lcm as lcm_module
import pytest

from dimos.constants import DIMOS_PROJECT_ROOT
from dimos.hardware.sensors.lidar.robosense.module import RoboSenseLidarConfig
from dimos.msgs.sensor_msgs.Imu import Imu
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2

_RUST_DIR = DIMOS_PROJECT_ROOT / "dimos" / "hardware" / "sensors" / "lidar" / "robosense" / "rust"
MSOP_PORT, DIFOP_PORT, IMU_PORT = 6699, 7788, 6688
START = 1_700_000_000.0
# one 10 Hz rotation of an Airy is 225 packets of 4 firings 0.4 deg apart
PACKETS_PER_SCAN = 225
VERTICAL_CDEG = [-1500 + 30 * i for i in range(96)]


def _difop():
    packet = bytearray(1248)
    packet[0:8] = bytes([0xA5, 0xFF, 0x00, 0x5A, 0x11, 0x11, 0x55, 0x55])
    struct.pack_into(">H", packet, 8, 600)
    struct.pack_into(">HH", packet, 32, 0, 36000)
    packet[289] = 2  # Airy-M install mode
    for i, angle in enumerate(VERTICAL_CDEG):
        struct.pack_into(">BH", packet, 468 + 3 * i, int(angle < 0), abs(angle))
    struct.pack_into(">4f", packet, 1092, 0.0, 0.0, 0.0, 1.0)  # IMU aligned with the lidar
    return bytes(packet)


def _stamp(ts):
    return struct.pack(">Q", int(ts))[2:] + struct.pack(">I", round((ts % 1) * 1e6))


def _msop(index, ts):
    packet = bytearray(1248)
    packet[0:4] = bytes([0x55, 0xAA, 0x05, 0x5A])
    struct.pack_into(">I", packet, 12, index)
    packet[20:30] = _stamp(ts)
    packet[32] = 0x02  # 96 lines
    for block in range(8):
        azimuth = ((index * 4 + block // 2) * 160 // 4) % 36000
        base = 42 + block * 148
        struct.pack_into(">BBH", packet, base, 0xFF, 0xEE, azimuth)
        for channel in range(48):
            struct.pack_into(">HB", packet, base + 4 + channel * 3, 2000, 100)  # 10 m
    return bytes(packet)


def _imu(ts):
    packet = bytearray(51)
    packet[0:4] = bytes([0xAA, 0x55, 0x5A, 0x05])
    packet[4:14] = _stamp(ts)
    struct.pack_into(">i", packet, 22, 8192)  # z = 1 g at +/- 4 g
    packet[43] = 1
    return bytes(packet)


def _record(ts, port, payload):
    udp = struct.pack(">HHHH", 6699, port, 8 + len(payload), 0) + payload
    ip = struct.pack(
        ">BBHHHBBH4s4s",
        0x45,
        0,
        20 + len(udp),
        0,
        0,
        64,
        17,
        0,
        bytes([192, 168, 1, 200]),
        bytes([192, 168, 1, 102]),
    )
    frame = ip + udp
    return struct.pack("<IIII", int(ts), round((ts % 1) * 1e6), len(frame), len(frame)) + frame


@pytest.fixture(scope="module")
def airy_pcap(tmp_path_factory):
    records = [(START, DIFOP_PORT, _difop())]
    for i in range(4 * PACKETS_PER_SCAN):
        ts = START + 0.001 + i * 0.1 / PACKETS_PER_SCAN
        records.append((ts, MSOP_PORT, _msop(i, ts)))
    for i in range(60):
        ts = START + 0.001 + i * 0.005
        records.append((ts, IMU_PORT, _imu(ts)))
    records.sort(key=lambda r: r[0])
    header = struct.pack("<IHHiIII", 0xA1B2C3D4, 2, 4, 0, 0, 65535, 101)  # raw IPv4
    path = tmp_path_factory.mktemp("robosense_e2e") / "synthetic_airy.pcap"
    path.write_bytes(header + b"".join(_record(*r) for r in records))
    return path


@pytest.fixture
def native_process(tmp_path, airy_pcap):
    binary = _RUST_DIR / "result" / "bin" / "robosense_native"
    if not binary.exists():
        pytest.fail(f"{binary} missing; run: (cd {_RUST_DIR} && nix build -L path:.)")
    config = RoboSenseLidarConfig(pcap=str(airy_pcap), replay_rate=1.0, replay_delay=0.5)
    blob = {
        "topics": {
            "lidar": "/robosense_e2e_lidar#sensor_msgs.PointCloud2",
            "imu": "/robosense_e2e_imu#sensor_msgs.Imu",
        },
        "config": config.to_config_dict(),
        "session": {"id": "robosense-e2e", "links": []},
    }
    stderr_path = tmp_path / "robosense_native.stderr"
    with stderr_path.open("wb") as stderr:
        process = subprocess.Popen(
            [str(binary)],
            stdin=subprocess.PIPE,
            stdout=subprocess.DEVNULL,
            stderr=stderr,
            env={**os.environ, "DIMOS_TRANSPORT": "lcm"},
        )
    process.stdin.write(json.dumps(blob).encode() + b"\n")
    process.stdin.flush()
    yield process, stderr_path
    if process.poll() is None:
        process.send_signal(signal.SIGTERM)
    try:
        process.wait(timeout=5)
    except subprocess.TimeoutExpired:
        process.kill()


@pytest.mark.native_e2e
def test_synthetic_airy_replay_publishes_clouds_and_imu(native_process):
    process, stderr_path = native_process
    received = {"lidar": [], "imu": []}
    lcm = lcm_module.LCM()
    subscriptions = [
        lcm.subscribe(
            "/robosense_e2e_lidar#sensor_msgs.PointCloud2",
            lambda _c, data: received["lidar"].append(data),
        ),
        lcm.subscribe(
            "/robosense_e2e_imu#sensor_msgs.Imu", lambda _c, data: received["imu"].append(data)
        ),
    ]
    try:
        end = time.monotonic() + 10
        while time.monotonic() < end and process.poll() is None:
            lcm.handle_timeout(200)
    finally:
        for subscription in subscriptions:
            lcm.unsubscribe(subscription)

    stderr = Path(stderr_path).read_text(errors="replace")[-2000:]
    clouds = [PointCloud2.lcm_decode(data) for data in received["lidar"]]
    imus = [Imu.lcm_decode(data) for data in received["imu"]]
    # four rotations: the first counts as partial (the decoder cannot know it began at 0 deg)
    # and the last is never closed by a following one
    assert len(clouds) == 2, stderr
    for cloud in clouds:
        assert cloud.frame_id == "lidar_link"
        points, _ = cloud.as_numpy()
        assert len(points) == PACKETS_PER_SCAN * 8 * 48
        ranges = (points**2).sum(axis=1) ** 0.5
        assert ranges.min() == pytest.approx(10.0, abs=0.06)
        assert ranges.max() == pytest.approx(10.0, abs=0.06)
    assert len(imus) == 60, stderr
    for imu in imus:
        acceleration = imu.linear_acceleration
        assert math.sqrt(
            acceleration.x**2 + acceleration.y**2 + acceleration.z**2
        ) == pytest.approx(9.80665)
        assert acceleration.z == pytest.approx(9.80665)
