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

"""End-to-end tests for the Mid-360 rust driver on a synthesized capture.

Both tests run the real release binaries. The pcap test exercises the
pipeline through the module boundary; the loopback test additionally
exercises the live path (handshake, sockets) against virtual_mid360.

The capture is synthesized per session: one second of SDK2 point and IMU
packets wrapped in a classic pcap. The layouts mirror rust/src/wire.rs;
if they drift, the driver fails to parse and the test fails with it.

Run locally with::
    cargo build --release -p dimos-livox -p virtual-mid360
    pytest -m pointlio_e2e dimos/hardware/sensors/lidar/livox/test_e2e.py
"""

from __future__ import annotations

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
from dimos.hardware.sensors.lidar.livox.module import Mid360Config
from dimos.msgs.sensor_msgs.Imu import Imu
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2

pytestmark = pytest.mark.pointlio_e2e

_RELEASE = DIMOS_PROJECT_ROOT / "target" / "release"

_LIDAR_POINT_PORT = 56300
_LIDAR_IMU_PORT = 56400
_HOST_POINT_PORT = 56301


def _data_packet(data_type: int, time_interval: int, ts_ns: int, payload: bytes) -> bytes:
    count = len(payload) // (14 if data_type == 1 else 24)
    header = struct.pack(
        "<BHHHHBBB12xIQ",
        0,  # version
        36 + len(payload),
        time_interval,  # 0.1 us units
        count,
        0,  # udp_cnt
        0,  # frame_cnt
        data_type,
        0,  # time_type
        0,  # crc32 (unverified on data packets)
        ts_ns,
    )
    return header + payload


def _udp_record(ts: float, src_port: int, payload: bytes) -> bytes:
    udp = struct.pack(">HHHH", src_port, _HOST_POINT_PORT, 8 + len(payload), 0) + payload
    ip = bytes([0x45]) + bytes(8) + bytes([17]) + bytes(10)
    frame = bytes(12) + b"\x08\x00" + ip + udp
    return struct.pack("<IIII", int(ts), int((ts % 1) * 1e6), len(frame), len(frame)) + frame


@pytest.fixture(scope="session")
def synth_pcap(tmp_path_factory: pytest.TempPathFactory) -> Path:
    """One second of a ring of points at 10 Hz frames plus 200 Hz gravity IMU."""
    base_ts = 1_000_000.0
    base_ns = int(base_ts * 1e9)
    records = []
    # 200 point packets, one every 5 ms, 100 points each: ~2000 points/frame.
    for i in range(200):
        offset_ns = i * 5_000_000
        points = b"".join(
            struct.pack(
                "<iiiBB",
                int(5000 * math.cos((i * 100 + j) / 300.0)),
                int(5000 * math.sin((i * 100 + j) / 300.0)),
                1000 + j,
                128,
                0,
            )
            for j in range(100)
        )
        packet = _data_packet(1, 1000, base_ns + offset_ns, points)
        records.append(_udp_record(base_ts + offset_ns / 1e9, _LIDAR_POINT_PORT, packet))
    # 200 IMU packets at 200 Hz: gravity on z, slow roll on x.
    for i in range(200):
        offset_ns = i * 5_000_000
        sample = struct.pack("<6f", 0.01, 0.0, 0.0, 0.0, 0.0, 1.0)
        packet = _data_packet(0, 0, base_ns + offset_ns, sample)
        records.append(_udp_record(base_ts + offset_ns / 1e9, _LIDAR_IMU_PORT, packet))

    records.sort(key=lambda r: struct.unpack("<II", r[:8]))
    pcap_header = struct.pack("<IHHiIII", 0xA1B2_C3D4, 2, 4, 0, 0, 0, 1)
    path = tmp_path_factory.mktemp("livox_e2e") / "synth_mid360.pcap"
    path.write_bytes(pcap_header + b"".join(records))
    return path


def _require_binary(name: str) -> Path:
    binary = _RELEASE / name
    if not binary.exists():
        pytest.skip(
            f"{binary} missing; run: cargo build --release -p dimos-livox -p virtual-mid360"
        )
    return binary


def _spawn(binary: Path, blob: dict) -> subprocess.Popen[bytes]:
    process = subprocess.Popen(
        [str(binary)],
        stdin=subprocess.PIPE,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        env={**os.environ, "DIMOS_TRANSPORT": "lcm"},
    )
    assert process.stdin is not None
    process.stdin.write(json.dumps(blob).encode() + b"\n")
    process.stdin.flush()
    return process


def _collect(topics: dict[str, type], seconds: float) -> dict[str, list]:
    """Subscribe to `topic -> msg type` and decode everything for `seconds`."""
    raw: dict[str, list[bytes]] = {topic: [] for topic in topics}
    lc = lcm_module.LCM()
    for topic in topics:
        # The handler only buffers bytes: decoding a 400 KB cloud inline is
        # slow enough to overflow the receive buffer and drop packets.
        def on_msg(_ch: str, data: bytes, topic=topic) -> None:
            raw[topic].append(data)

        msg_type = topics[topic]
        lc.subscribe(f"{topic}#sensor_msgs.{msg_type.__name__}", on_msg)
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        lc.handle_timeout(200)
    return {topic: [topics[topic].lcm_decode(data) for data in raw[topic]] for topic in topics}


def _terminate(*processes: subprocess.Popen[bytes]) -> None:
    for process in processes:
        if process.poll() is None:
            process.send_signal(signal.SIGTERM)
    for process in processes:
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            process.kill()


def _assert_stream_contents(clouds: list, imus: list) -> None:
    # One second of capture: ~10 frames at 10 Hz, ~200 IMU samples at 200 Hz.
    # Fragmented ~400 KB clouds drop readily under the kernel's default
    # receive buffer, so the cloud threshold only proves the stream flows;
    # the small IMU packets carry the rate assertion.
    assert len(clouds) >= 3, f"expected >=3 clouds, got {len(clouds)}"
    assert len(imus) >= 120, f"expected >=120 imu samples, got {len(imus)}"

    cloud = clouds[len(clouds) // 2]
    assert cloud.frame_id == "lidar_link"
    points, _ = cloud.as_numpy()
    assert len(points) > 1000
    # Full point format carries per-point deskew offsets within one frame.
    offsets = cloud.offset_times_u32()
    assert offsets is not None
    assert offsets.max() < 150_000_000, "offsets exceed one frame interval"

    # The synthesized IMU reports exactly 1 g on z.
    sample = imus[len(imus) // 2]
    acc = sample.linear_acceleration
    magnitude = math.sqrt(acc.x**2 + acc.y**2 + acc.z**2)
    assert 6.0 < magnitude < 14.0, f"accel magnitude {magnitude}"
    assert sample.orientation_covariance[0] == -1.0
    assert sample.frame_id == "imu_link"


def test_pcap_replay_publishes_streams(synth_pcap: Path) -> None:
    binary = _require_binary("mid360_native")
    config = Mid360Config(
        pcap=str(synth_pcap),
        replay_rate=0.5,
        multicast_ip=None,
        point_format="full",
    )
    blob = {
        "topics": {
            "lidar": "/e2e_pcap_lidar#sensor_msgs.PointCloud2",
            "imu": "/e2e_pcap_imu#sensor_msgs.Imu",
        },
        "config": config.to_config_dict(),
        "session": {"id": "pointlio-e2e", "links": []},
    }
    process = _spawn(binary, blob)
    try:
        received = _collect({"/e2e_pcap_lidar": PointCloud2, "/e2e_pcap_imu": Imu}, seconds=4.0)
    finally:
        _terminate(process)
    _assert_stream_contents(received["/e2e_pcap_lidar"], received["/e2e_pcap_imu"])


def test_live_loopback_handshake_and_stream(synth_pcap: Path) -> None:
    driver_binary = _require_binary("mid360_native")
    virtual_binary = _require_binary("virtual_mid360")

    virtual_blob = {
        "topics": {},
        "config": {
            "pcap": str(synth_pcap),
            "rate": 0.5,
            "delay": 1.0,
            "lidar_ip": "127.0.0.1",
            "host_ip": "127.0.0.1",
            "lidar_netns": "",
            "mcast_data": "224.1.1.5",
        },
        "session": {"id": "pointlio-e2e", "links": []},
    }
    config = Mid360Config(
        host_ip="127.0.0.1",
        lidar_ip="127.0.0.1",
        multicast_ip=None,
        point_format="full",
    )
    driver_blob = {
        "topics": {
            "lidar": "/e2e_live_lidar#sensor_msgs.PointCloud2",
            "imu": "/e2e_live_imu#sensor_msgs.Imu",
        },
        "config": config.to_config_dict(),
        "session": {"id": "pointlio-e2e", "links": []},
    }

    driver = _spawn(driver_binary, driver_blob)
    virtual = _spawn(virtual_binary, virtual_blob)
    try:
        # Handshake arms the virtual within ~1 s, then 2 s of half-rate stream.
        received = _collect({"/e2e_live_lidar": PointCloud2, "/e2e_live_imu": Imu}, seconds=8.0)
    finally:
        _terminate(driver, virtual)
    _assert_stream_contents(received["/e2e_live_lidar"], received["/e2e_live_imu"])
