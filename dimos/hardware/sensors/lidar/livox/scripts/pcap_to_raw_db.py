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

"""Offline Mid-360 pcap -> raw PointCloud2 + Imu memory2 db, no network, no SDK.

Decodes the Livox point/IMU UDP payloads straight out of a pcap and writes the
same messages the live mid360_native driver would publish, into a replayable
memory2 SQLite db with two streams:

    lidar   PointCloud2 frames accumulated at --frequency (sensor time)
    imu     one Imu per raw sample (~200 Hz on a Mid-360)

Conventions mirror the driver (dimos/hardware/sensors/lidar/livox/cpp/main.cpp)
exactly: mm/1000 (high precision) or cm/100 (low), reflectivity/255 as
intensity, frame ts = first packet ts in the frame, IMU accel scaled g ->
m/s^2 by 9.80665, identity orientation with orientation_covariance[0] = -1.
Packet layout is from the public Livox-SDK2 header (packed, 36-byte header,
u64-LE nanosecond timestamp at offset 28).

Unlike scripts like pointlio/scripts/pcap_to_db.py this needs no virtual NIC,
sudo, or SDK — the pcap is parsed directly, so the output is deterministic.

Usage:
    uv run python -m dimos.hardware.sensors.lidar.livox.scripts.pcap_to_raw_db \\
        --pcap data/mid360_shake_stairs/mid360_shake_stairs.pcap
"""

from __future__ import annotations

import argparse
from pathlib import Path
import struct

import numpy as np

from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.Imu import Imu
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2

# Livox SDK2 wire constants (dimos/hardware/sensors/lidar/common/livox_sdk_config.hpp
# + livox/ports.py). Both lidar-side and host-side ports accepted: captures can be
# taken on either side of the link.
POINT_PORTS = {56300, 56301}
IMU_PORTS = {56400, 56401}
DATA_TYPE_IMU = 0x00
DATA_TYPE_CARTESIAN_HIGH = 0x01
DATA_TYPE_CARTESIAN_LOW = 0x02
GRAVITY_MS2 = 9.80665

# Packed LivoxLidarEthernetPacket header (Livox-SDK2 livox_lidar_def.h):
# version u8 | length u16 | time_interval u16 | dot_num u16 | udp_cnt u16 |
# frame_cnt u8 | data_type u8 | time_type u8 | rsvd[12] | crc32 u32 | ts u64
LIVOX_HEADER_LEN = 36
_DOT_NUM_OFF = 5
_DATA_TYPE_OFF = 10
_TIMESTAMP_OFF = 28

_HIGH_DTYPE = np.dtype([("x", "<i4"), ("y", "<i4"), ("z", "<i4"), ("refl", "u1"), ("tag", "u1")])
_LOW_DTYPE = np.dtype([("x", "<i2"), ("y", "<i2"), ("z", "<i2"), ("refl", "u1"), ("tag", "u1")])
_IMU_DTYPE = np.dtype("<f4")  # gyro xyz then acc xyz


def iter_udp_payloads(pcap: bytes):
    """Yield (dst_port, payload) for every UDP packet in a classic pcap."""
    if len(pcap) < 24:
        raise SystemExit("not a pcap: file shorter than the global header")
    magic = struct.unpack_from("<I", pcap, 0)[0]
    if magic not in (0xA1B2C3D4, 0xA1B23C4D):  # classic LE (usec / nsec variants)
        raise SystemExit(f"unsupported pcap magic 0x{magic:08x} (need classic little-endian)")
    linktype = struct.unpack_from("<I", pcap, 20)[0]
    if linktype != 1:  # LINKTYPE_ETHERNET
        raise SystemExit(f"unsupported linktype {linktype} (need Ethernet)")

    off = 24
    n = len(pcap)
    while off + 16 <= n:
        incl_len = struct.unpack_from("<I", pcap, off + 8)[0]
        frame_off = off + 16
        off = frame_off + incl_len
        if off > n:
            break
        # Ethernet -> IPv4 -> UDP. Anything else is control traffic we skip.
        if incl_len < 14 + 20 + 8:
            continue
        ethertype = (pcap[frame_off + 12] << 8) | pcap[frame_off + 13]
        if ethertype != 0x0800:
            continue
        ip_off = frame_off + 14
        ihl = (pcap[ip_off] & 0x0F) * 4
        if pcap[ip_off + 9] != 17:  # UDP
            continue
        udp_off = ip_off + ihl
        dst_port = (pcap[udp_off + 2] << 8) | pcap[udp_off + 3]
        udp_len = (pcap[udp_off + 4] << 8) | pcap[udp_off + 5]
        payload = pcap[udp_off + 8 : udp_off + max(udp_len, 8)]
        yield dst_port, payload


def parse_livox_header(payload: bytes) -> tuple[int, int, float] | None:
    """(dot_num, data_type, ts_seconds) or None if too short."""
    if len(payload) < LIVOX_HEADER_LEN:
        return None
    dot_num = struct.unpack_from("<H", payload, _DOT_NUM_OFF)[0]
    data_type = payload[_DATA_TYPE_OFF]
    ts_ns = struct.unpack_from("<Q", payload, _TIMESTAMP_OFF)[0]
    return dot_num, data_type, ts_ns * 1e-9


def parse_args() -> argparse.Namespace:
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    ap.add_argument("--pcap", required=True, help="Mid-360 capture (classic pcap, Ethernet)")
    ap.add_argument("--out", default=None, help="output db (default: <pcap dir>/<stem>_raw.db)")
    ap.add_argument("--frequency", type=float, default=10.0, help="lidar frame rate (Hz)")
    ap.add_argument("--frame-id", default="lidar_link")
    ap.add_argument("--imu-frame-id", default="imu_link")
    return ap.parse_args()


def main() -> None:
    args = parse_args()
    # Deferred: pulls in the heavy dimos import chain.
    from dimos.memory2.store.sqlite import SqliteStore

    pcap_path = Path(args.pcap)
    out_path = Path(args.out) if args.out else pcap_path.with_name(pcap_path.stem + "_raw.db")
    if out_path.exists():
        out_path.unlink()  # regenerate deterministically; -wal/-shm follow the db
        for suffix in ("-wal", "-shm"):
            side = Path(str(out_path) + suffix)
            if side.exists():
                side.unlink()

    print(f"reading {pcap_path} ({pcap_path.stat().st_size / 1e6:.0f} MB)")
    pcap = pcap_path.read_bytes()

    store = SqliteStore(path=str(out_path))
    store.start()
    lidar_stream = store.stream("lidar", PointCloud2)
    imu_stream = store.stream("imu", Imu)

    frame_period = 1.0 / args.frequency
    frame_xyz: list[np.ndarray] = []
    frame_intensity: list[np.ndarray] = []
    frame_ts: float | None = None

    counts = {"point_pkts": 0, "imu_pkts": 0, "frames": 0, "imu_msgs": 0, "skipped": 0}
    first_ts = last_ts = None
    imu_acc_norms: list[float] = []
    # Monotonicity is a per-stream property: point and IMU packets interleave
    # in capture order with different in-packet clock semantics.
    ts_regressions = {"point": 0, "imu": 0}
    prev_stream_ts = {"point": None, "imu": None}

    def emit_frame() -> None:
        nonlocal frame_ts
        if frame_ts is None or not frame_xyz:
            return
        xyz = np.concatenate(frame_xyz)
        intensity = np.concatenate(frame_intensity)
        cloud = PointCloud2.from_numpy(
            xyz, frame_id=args.frame_id, timestamp=frame_ts, intensities=intensity
        )
        lidar_stream.append(cloud, ts=frame_ts)
        counts["frames"] += 1
        frame_xyz.clear()
        frame_intensity.clear()
        frame_ts = None

    for dst_port, payload in iter_udp_payloads(pcap):
        is_point = dst_port in POINT_PORTS
        is_imu = dst_port in IMU_PORTS
        if not (is_point or is_imu):
            continue
        header = parse_livox_header(payload)
        if header is None:
            counts["skipped"] += 1
            continue
        dot_num, data_type, ts = header

        if first_ts is None:
            first_ts = ts
        last_ts = ts
        stream_key = "point" if is_point else "imu"
        prev = prev_stream_ts[stream_key]
        if prev is not None and ts < prev:
            ts_regressions[stream_key] += 1
        prev_stream_ts[stream_key] = ts

        if is_point and data_type in (DATA_TYPE_CARTESIAN_HIGH, DATA_TYPE_CARTESIAN_LOW):
            counts["point_pkts"] += 1
            # Cut the frame on sensor time; stamp with its first packet's ts
            # (the driver's frame_has_ts_ convention, made deterministic).
            if frame_ts is not None and ts - frame_ts >= frame_period:
                emit_frame()
            if frame_ts is None:
                frame_ts = ts

            if data_type == DATA_TYPE_CARTESIAN_HIGH:
                pts = np.frombuffer(payload, _HIGH_DTYPE, count=dot_num, offset=LIVOX_HEADER_LEN)
                scale = 1e-3  # mm -> m
            else:
                pts = np.frombuffer(payload, _LOW_DTYPE, count=dot_num, offset=LIVOX_HEADER_LEN)
                scale = 1e-2  # cm -> m
            xyz = np.stack([pts["x"], pts["y"], pts["z"]], axis=1).astype(np.float32) * np.float32(
                scale
            )
            frame_xyz.append(xyz)
            frame_intensity.append(pts["refl"].astype(np.float32) / np.float32(255.0))

        elif is_imu and data_type == DATA_TYPE_IMU:
            counts["imu_pkts"] += 1
            samples = np.frombuffer(payload, _IMU_DTYPE, count=dot_num * 6, offset=LIVOX_HEADER_LEN)
            for i in range(dot_num):
                gx, gy, gz, ax, ay, az = (float(v) for v in samples[i * 6 : i * 6 + 6])
                msg = Imu(
                    angular_velocity=Vector3(gx, gy, gz),
                    linear_acceleration=Vector3(
                        ax * GRAVITY_MS2, ay * GRAVITY_MS2, az * GRAVITY_MS2
                    ),
                    orientation=Quaternion(0.0, 0.0, 0.0, 1.0),
                    orientation_covariance=[-1.0] + [0.0] * 8,
                    frame_id=args.imu_frame_id,
                    ts=ts,
                )
                imu_stream.append(msg, ts=ts)
                counts["imu_msgs"] += 1
                imu_acc_norms.append((ax * ax + ay * ay + az * az) ** 0.5 * GRAVITY_MS2)

    emit_frame()  # trailing partial frame
    store.stop()

    duration = (last_ts - first_ts) if (first_ts is not None and last_ts is not None) else 0.0
    print(f"\nwrote {out_path}")
    print(f"sensor-time span: {duration:.1f}s")
    print(
        f"lidar: {counts['frames']} frames from {counts['point_pkts']} packets "
        f"({counts['frames'] / duration:.2f} Hz)"
        if duration
        else "lidar: no data"
    )
    print(
        f"imu:   {counts['imu_msgs']} msgs from {counts['imu_pkts']} packets "
        f"({counts['imu_msgs'] / duration:.1f} Hz)"
        if duration
        else "imu: no data"
    )
    if imu_acc_norms:
        mean_norm = sum(imu_acc_norms) / len(imu_acc_norms)
        print(f"imu |acc| mean: {mean_norm:.2f} m/s^2 (gravity check; ~9.81 if mostly gentle)")
    print(
        f"per-stream timestamp regressions: point={ts_regressions['point']} "
        f"imu={ts_regressions['imu']}"
    )
    if counts["skipped"]:
        print(f"skipped {counts['skipped']} short payloads")


if __name__ == "__main__":
    main()
