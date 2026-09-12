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

"""End-to-end tests for the Mid-360 rust driver.

self_hosted replays a real recording. native_e2e runs the live path
against virtual_mid360 over loopback.
"""

from __future__ import annotations

from collections.abc import Callable, Generator
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
from dimos.hardware.sensors.lidar.livox.ports import (
    SDK_HOST_POINT_DATA_PORT,
    SDK_IMU_DATA_PORT,
    SDK_MULTICAST_GROUP,
    SDK_POINT_DATA_PORT,
)
from dimos.hardware.sensors.lidar.virtual_mid360.module import VirtualMid360Config
from dimos.msgs.sensor_msgs.Imu import Imu
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.utils.data import get_data

_RELEASE = DIMOS_PROJECT_ROOT / "target" / "release"

GRAVITY_MS2 = 9.80665

# Livox data-plane wire sizes, matching rust/src/wire.rs.
POINT_SIZE = 14
IMU_SAMPLE_SIZE = 24
DATA_HEADER_SIZE = 36
UDP_PROTOCOL = 17

Spawn = Callable[[Path, dict[str, object]], "subprocess.Popen[bytes]"]


def _data_packet(data_type: int, time_interval: int, ts_ns: int, payload: bytes) -> bytes:
    count = len(payload) // (POINT_SIZE if data_type == 1 else IMU_SAMPLE_SIZE)
    header = struct.pack(
        "<BHHHHBBB12xIQ",
        0,  # version
        DATA_HEADER_SIZE + len(payload),
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
    udp = struct.pack(">HHHH", src_port, SDK_HOST_POINT_DATA_PORT, 8 + len(payload), 0) + payload
    ip = struct.pack(">BBHHHBBH", 0x45, 0, 20 + len(udp), 0, 0, 64, UDP_PROTOCOL, 0) + bytes(8)
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
        records.append(_udp_record(base_ts + offset_ns / 1e9, SDK_POINT_DATA_PORT, packet))
    # 200 IMU packets at 200 Hz: gravity on z, slow roll on x.
    for i in range(200):
        offset_ns = i * 5_000_000
        sample = struct.pack("<6f", 0.01, 0.0, 0.0, 0.0, 0.0, 1.0)
        packet = _data_packet(0, 0, base_ns + offset_ns, sample)
        records.append(_udp_record(base_ts + offset_ns / 1e9, SDK_IMU_DATA_PORT, packet))

    records.sort(key=lambda r: struct.unpack("<II", r[:8]))
    pcap_header = struct.pack("<IHHiIII", 0xA1B2_C3D4, 2, 4, 0, 0, 0, 1)
    path = tmp_path_factory.mktemp("livox_e2e") / "synth_mid360.pcap"
    path.write_bytes(pcap_header + b"".join(records))
    return path


def _require_binary(name: str) -> Path:
    binary = _RELEASE / name
    if not binary.exists():
        pytest.fail(
            f"{binary} missing; run: cargo build --release -p dimos-livox -p dimos-virtual-mid360"
        )
    return binary


@pytest.fixture
def spawn() -> Generator[Spawn]:
    """Spawn a native module binary. Everything spawned is torn down."""
    processes: list[subprocess.Popen[bytes]] = []

    def factory(binary: Path, blob: dict[str, object]) -> subprocess.Popen[bytes]:
        process = subprocess.Popen(
            [str(binary)],
            stdin=subprocess.PIPE,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            env={**os.environ, "DIMOS_TRANSPORT": "lcm"},
        )
        processes.append(process)
        assert process.stdin is not None
        process.stdin.write(json.dumps(blob).encode() + b"\n")
        process.stdin.flush()
        return process

    yield factory

    for process in processes:
        if process.poll() is None:
            process.send_signal(signal.SIGTERM)
    for process in processes:
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            process.kill()


def _collect(
    topics: dict[str, type[PointCloud2 | Imu]], seconds: float, enough: dict[str, int]
) -> dict[str, list[PointCloud2 | Imu]]:
    """Decode each topic until every count in enough is met or time runs out."""
    raw: dict[str, list[bytes]] = {topic: [] for topic in topics}
    lc = lcm_module.LCM()
    for topic in topics:
        # Buffer bytes only. Decoding inline drops packets.
        def on_msg(_ch: str, data: bytes, topic: str = topic) -> None:
            raw[topic].append(data)

        msg_type = topics[topic]
        lc.subscribe(f"{topic}#sensor_msgs.{msg_type.__name__}", on_msg)
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        if all(len(raw[topic]) >= count for topic, count in enough.items()):
            break
        lc.handle_timeout(200)
    return {topic: [topics[topic].lcm_decode(data) for data in raw[topic]] for topic in topics}


def _magnitude(sample: Imu) -> float:
    acc = sample.linear_acceleration
    return math.sqrt(acc.x**2 + acc.y**2 + acc.z**2)


def _assert_cloud_shape(cloud: PointCloud2, min_points: int) -> None:
    assert cloud.frame_id == "lidar_link"
    points, _ = cloud.as_numpy()
    assert len(points) > min_points
    # Full point format carries per-point deskew offsets within one frame.
    offsets = cloud.offset_times_u32()
    assert offsets is not None
    assert offsets.max() < 150_000_000, "offsets exceed one frame interval"


@pytest.mark.self_hosted
def test_real_capture_replay_publishes_streams(spawn: Spawn) -> None:
    """Replay a real Mid-360 recording: the non-circular wire-format check."""
    binary = _require_binary("mid360_native")
    pcap = get_data("mid360_shake_stairs/mid360_shake_stairs.pcap")
    config = Mid360Config(
        pcap=str(pcap),
        replay_rate=1.0,
        multicast_ip=None,
        point_format="full",
    )
    blob = {
        "topics": {
            "lidar": "/e2e_real_lidar#sensor_msgs.PointCloud2",
            "imu": "/e2e_real_imu#sensor_msgs.Imu",
        },
        "config": config.to_config_dict(),
        "session": {"id": "pointlio-e2e", "links": []},
    }
    spawn(binary, blob)
    received = _collect(
        {"/e2e_real_lidar": PointCloud2, "/e2e_real_imu": Imu},
        seconds=15.0,
        enough={"/e2e_real_lidar": 20, "/e2e_real_imu": 800},
    )

    clouds = received["/e2e_real_lidar"]
    imus = received["/e2e_real_imu"]
    # Thresholds leave headroom for receive-buffer drops on the big clouds.
    assert len(clouds) >= 20, f"expected >=20 clouds, got {len(clouds)}"
    assert len(imus) >= 800, f"expected >=800 imu samples, got {len(imus)}"
    _assert_cloud_shape(clouds[len(clouds) // 2], min_points=5000)

    # The rig shakes, so single samples spike. The median stays near 1 g.
    magnitudes = sorted(_magnitude(sample) for sample in imus)
    median = magnitudes[len(magnitudes) // 2]
    assert 5.0 < median < 15.0, f"median accel magnitude {median}"
    assert imus[0].orientation_covariance[0] == -1.0
    assert imus[0].frame_id == "imu_link"


@pytest.mark.native_e2e
def test_live_loopback_handshake_and_stream(spawn: Spawn, synth_pcap: Path) -> None:
    driver_binary = _require_binary("mid360_native")
    virtual_binary = _require_binary("virtual_mid360")

    virtual_config = VirtualMid360Config(
        pcap=str(synth_pcap),
        rate=0.5,
        delay=1.0,
        lidar_ip="127.0.0.1",
        host_ip="127.0.0.1",
        mcast_data=SDK_MULTICAST_GROUP,
    )
    virtual_blob = {
        "topics": {},
        "config": virtual_config.to_config_dict(),
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

    spawn(driver_binary, driver_blob)
    spawn(virtual_binary, virtual_blob)
    # Handshake arms the virtual within ~1 s, then 2 s of half-rate stream.
    received = _collect(
        {"/e2e_live_lidar": PointCloud2, "/e2e_live_imu": Imu},
        seconds=10.0,
        enough={"/e2e_live_lidar": 3, "/e2e_live_imu": 120},
    )

    clouds = received["/e2e_live_lidar"]
    imus = received["/e2e_live_imu"]
    # Large clouds drop under the default receive buffer, so the IMU packets
    # carry the rate assertion.
    assert len(clouds) >= 3, f"expected >=3 clouds, got {len(clouds)}"
    assert len(imus) >= 120, f"expected >=120 imu samples, got {len(imus)}"
    _assert_cloud_shape(clouds[len(clouds) // 2], min_points=1000)

    # The synthetic capture is a 5 m ring at z ~1 m, so a decode regression
    # in scaling or field layout shows up as broken geometry here.
    points, _ = clouds[len(clouds) // 2].as_numpy()
    radii = sorted(math.hypot(x, y) for x, y, _z in points)
    assert abs(radii[len(radii) // 2] - 5.0) < 0.1, f"median ring radius {radii[len(radii) // 2]}"
    z_values = sorted(z for _x, _y, z in points)
    assert 0.9 < z_values[len(z_values) // 2] < 1.2

    # The synthesized IMU reports exactly 1 g on z.
    sample = imus[len(imus) // 2]
    assert abs(_magnitude(sample) - GRAVITY_MS2) < 1e-3
    assert sample.orientation_covariance[0] == -1.0
    assert sample.frame_id == "imu_link"
