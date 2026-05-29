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

"""Decode a Mid-360 pcap recorded by `record_pcap.py`.

Parses UDP payloads as Livox-SDK2 ethernet packets (point/IMU/cmd/push/log
streams) without depending on the SDK or scapy. Prints per-port summaries,
checks udp_cnt continuity for dropped packets, and (optionally) dumps the
first N packets per stream for inspection.

Use this to determine whether a fastlio anomaly originates upstream of the
SDK (visible in raw pcap → hardware/cable/EMI) or downstream of it (clean
pcap, dirty fastlio output → SDK / module bug).

Reference: Livox-SDK2 livox_lidar_def.h, LivoxLidarEthernetPacket.

Usage:
    uv run python -m dimos.hardware.sensors.lidar.fastlio2.tests.decode_pcap \
        tests/data/mid360_pcap/mid360_<timestamp>.pcap
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass, field
from pathlib import Path
import statistics
import struct
import sys

PCAP_MAGIC_LE_US = 0xA1B2C3D4
PCAP_MAGIC_BE_US = 0xD4C3B2A1
PCAP_MAGIC_LE_NS = 0xA1B23C4D
PCAP_MAGIC_BE_NS = 0x4D3CB2A1

LINKTYPE_ETHERNET = 1
LINKTYPE_LINUX_SLL = 113
LINKTYPE_LINUX_SLL2 = 276

ETHERTYPE_IPV4 = 0x0800
IPPROTO_UDP = 17

LIVOX_ETH_HEADER_LEN = 36
LIVOX_POINT_HIGH_LEN = 14
LIVOX_IMU_RAW_LEN = 24

DATA_TYPE_IMU = 0
DATA_TYPE_CART_HIGH = 1
DATA_TYPE_CART_LOW = 2
DATA_TYPE_SPHER = 3
DATA_TYPE_NAMES = {
    0: "imu",
    1: "cartesian_high",
    2: "cartesian_low",
    3: "spherical",
}

MID360_HOST_PORTS = {
    56101: "cmd",
    56201: "push",
    56301: "point",
    56401: "imu",
    56501: "log",
}


@dataclass
class PortStats:
    port: int
    label: str
    packet_count: int = 0
    total_payload_bytes: int = 0
    first_ts: float | None = None
    last_ts: float | None = None
    data_type_counts: dict[int, int] = field(default_factory=dict)
    dot_num_samples: list[int] = field(default_factory=list)
    udp_cnt_prev: int | None = None
    udp_cnt_gaps: int = 0
    udp_cnt_drops: int = 0
    bad_payload_count: int = 0
    imu_gyro_x: list[float] = field(default_factory=list)
    imu_gyro_y: list[float] = field(default_factory=list)
    imu_gyro_z: list[float] = field(default_factory=list)
    imu_acc_x: list[float] = field(default_factory=list)
    imu_acc_y: list[float] = field(default_factory=list)
    imu_acc_z: list[float] = field(default_factory=list)


@dataclass
class PcapHeader:
    magic: int
    little_endian: bool
    nanos: bool
    snaplen: int
    linktype: int


def read_pcap_header(buf: bytes) -> PcapHeader:
    if len(buf) < 24:
        raise ValueError("pcap file too small for global header")
    magic = struct.unpack("<I", buf[:4])[0]
    if magic in (PCAP_MAGIC_LE_US, PCAP_MAGIC_LE_NS):
        little = True
    elif magic in (PCAP_MAGIC_BE_US, PCAP_MAGIC_BE_NS):
        little = False
    else:
        raise ValueError(f"unrecognized pcap magic 0x{magic:08x}; not a classic pcap file")
    nanos = magic in (PCAP_MAGIC_LE_NS, PCAP_MAGIC_BE_NS)
    endian = "<" if little else ">"
    snaplen, linktype = struct.unpack(endian + "II", buf[16:24])
    return PcapHeader(
        magic=magic, little_endian=little, nanos=nanos, snaplen=snaplen, linktype=linktype
    )


def iter_pcap_records(path: Path) -> tuple[PcapHeader, list[tuple[float, bytes]]]:
    data = path.read_bytes()
    hdr = read_pcap_header(data)
    endian = "<" if hdr.little_endian else ">"
    records: list[tuple[float, bytes]] = []
    off = 24
    rec_fmt = endian + "IIII"
    while off + 16 <= len(data):
        ts_sec, ts_sub, incl_len, _orig_len = struct.unpack(rec_fmt, data[off : off + 16])
        off += 16
        if off + incl_len > len(data):
            break
        pkt = data[off : off + incl_len]
        off += incl_len
        ts = ts_sec + (ts_sub / 1e9 if hdr.nanos else ts_sub / 1e6)
        records.append((ts, pkt))
    return hdr, records


def extract_udp(pkt: bytes, linktype: int) -> tuple[int, int, bytes] | None:
    """Return (src_port, dst_port, udp_payload) or None if not UDP/IPv4."""
    if linktype == LINKTYPE_ETHERNET:
        if len(pkt) < 14:
            return None
        ethertype = struct.unpack("!H", pkt[12:14])[0]
        if ethertype != ETHERTYPE_IPV4:
            return None
        ip_off = 14
    elif linktype == LINKTYPE_LINUX_SLL:
        if len(pkt) < 16:
            return None
        ethertype = struct.unpack("!H", pkt[14:16])[0]
        if ethertype != ETHERTYPE_IPV4:
            return None
        ip_off = 16
    elif linktype == LINKTYPE_LINUX_SLL2:
        if len(pkt) < 20:
            return None
        ethertype = struct.unpack("!H", pkt[0:2])[0]
        if ethertype != ETHERTYPE_IPV4:
            return None
        ip_off = 20
    else:
        return None

    if len(pkt) < ip_off + 20:
        return None
    vihl = pkt[ip_off]
    version = vihl >> 4
    ihl = (vihl & 0x0F) * 4
    if version != 4 or ihl < 20:
        return None
    proto = pkt[ip_off + 9]
    if proto != IPPROTO_UDP:
        return None
    udp_off = ip_off + ihl
    if len(pkt) < udp_off + 8:
        return None
    src_port, dst_port, udp_len, _ = struct.unpack("!HHHH", pkt[udp_off : udp_off + 8])
    payload_off = udp_off + 8
    payload_end = min(len(pkt), udp_off + udp_len)
    payload = pkt[payload_off:payload_end]
    return src_port, dst_port, payload


@dataclass
class LivoxHeader:
    version: int
    length: int
    time_interval: int
    dot_num: int
    udp_cnt: int
    frame_cnt: int
    data_type: int
    time_type: int
    crc32: int
    timestamp: bytes


def parse_livox_header(payload: bytes) -> LivoxHeader | None:
    if len(payload) < LIVOX_ETH_HEADER_LEN:
        return None
    (
        version,
        length,
        time_interval,
        dot_num,
        udp_cnt,
        frame_cnt,
        data_type,
        time_type,
    ) = struct.unpack("<BHHHHBBB", payload[:12])
    # 12 bytes rsvd at offset 12..23
    crc32 = struct.unpack("<I", payload[24:28])[0]
    timestamp = payload[28:36]
    return LivoxHeader(
        version=version,
        length=length,
        time_interval=time_interval,
        dot_num=dot_num,
        udp_cnt=udp_cnt,
        frame_cnt=frame_cnt,
        data_type=data_type,
        time_type=time_type,
        crc32=crc32,
        timestamp=timestamp,
    )


def decode_imu_point(payload: bytes) -> tuple[float, float, float, float, float, float] | None:
    body = payload[LIVOX_ETH_HEADER_LEN:]
    if len(body) < LIVOX_IMU_RAW_LEN:
        return None
    return struct.unpack("<ffffff", body[:LIVOX_IMU_RAW_LEN])


def fmt_range(samples: list[float]) -> str:
    if not samples:
        return "no samples"
    return (
        f"min={min(samples):+.3f}  max={max(samples):+.3f}  mean={statistics.fmean(samples):+.3f}"
    )


def summarize(stats: dict[int, PortStats], duration: float) -> None:
    print()
    print(f"capture duration: {duration:.3f} s")
    print()
    for port in sorted(stats.keys()):
        s = stats[port]
        rate = s.packet_count / duration if duration > 0 else 0.0
        bps = (s.total_payload_bytes * 8) / duration if duration > 0 else 0.0
        print(f"port {port}  ({s.label}):")
        print(
            f"  packets={s.packet_count:>7}   rate={rate:>7.1f} pkt/s"
            f"   payload={s.total_payload_bytes / 1e6:>6.2f} MB"
            f"   ~{bps / 1e6:>5.2f} Mbps"
        )
        if s.bad_payload_count:
            print(f"  WARN: {s.bad_payload_count} payloads too short to parse")
        if s.data_type_counts:
            parts = []
            for dt, n in sorted(s.data_type_counts.items()):
                name = DATA_TYPE_NAMES.get(dt, f"type=0x{dt:02x}")
                parts.append(f"{name}={n}")
            print(f"  data_types: {', '.join(parts)}")
        if s.udp_cnt_prev is not None:
            print(
                f"  udp_cnt: {s.udp_cnt_gaps} gap(s), {s.udp_cnt_drops} estimated dropped packet(s)"
            )
        if s.dot_num_samples:
            dn = s.dot_num_samples
            total_dots = sum(dn)
            dn_rate = total_dots / duration if duration > 0 else 0.0
            print(
                f"  dot_num per packet: min={min(dn)}  max={max(dn)}  "
                f"mean={statistics.fmean(dn):.1f}  total={total_dots}  "
                f"({dn_rate / 1000:.1f} kpt/s)"
            )
        if s.imu_gyro_x:
            print(f"  gyro_x [rad/s]: {fmt_range(s.imu_gyro_x)}")
            print(f"  gyro_y [rad/s]: {fmt_range(s.imu_gyro_y)}")
            print(f"  gyro_z [rad/s]: {fmt_range(s.imu_gyro_z)}")
            print(f"  acc_x  [g]:    {fmt_range(s.imu_acc_x)}")
            print(f"  acc_y  [g]:    {fmt_range(s.imu_acc_y)}")
            print(f"  acc_z  [g]:    {fmt_range(s.imu_acc_z)}")
        print()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("pcap", type=Path, help="Path to pcap file")
    parser.add_argument(
        "--head",
        type=int,
        default=0,
        help=("Dump the first N Livox headers per port for inspection (default: 0 = summary only)"),
    )
    args = parser.parse_args()

    if not args.pcap.exists():
        print(f"[decode_pcap] file not found: {args.pcap}", file=sys.stderr)
        return 1

    hdr, records = iter_pcap_records(args.pcap)
    print(f"[decode_pcap] {args.pcap} ({args.pcap.stat().st_size / 1e6:.2f} MB)")
    print(
        f"[decode_pcap] linktype={hdr.linktype}  "
        f"{'nanosecond' if hdr.nanos else 'microsecond'} timestamps  "
        f"{len(records)} records"
    )

    if not records:
        print("[decode_pcap] empty capture")
        return 1

    first_ts = records[0][0]
    last_ts = records[-1][0]
    duration = max(0.0, last_ts - first_ts)

    stats: dict[int, PortStats] = {}
    head_remaining = {port: args.head for port in MID360_HOST_PORTS}
    non_udp = 0
    other_ports: dict[int, int] = {}

    for ts, pkt in records:
        udp = extract_udp(pkt, hdr.linktype)
        if udp is None:
            non_udp += 1
            continue
        _src, dst, payload = udp
        if dst not in MID360_HOST_PORTS:
            other_ports[dst] = other_ports.get(dst, 0) + 1
            continue
        s = stats.get(dst)
        if s is None:
            s = PortStats(port=dst, label=MID360_HOST_PORTS[dst])
            stats[dst] = s
        s.packet_count += 1
        s.total_payload_bytes += len(payload)
        if s.first_ts is None:
            s.first_ts = ts
        s.last_ts = ts

        lh = parse_livox_header(payload)
        if lh is None:
            s.bad_payload_count += 1
            continue
        s.data_type_counts[lh.data_type] = s.data_type_counts.get(lh.data_type, 0) + 1

        if s.udp_cnt_prev is not None:
            expected = (s.udp_cnt_prev + 1) & 0xFFFF
            if lh.udp_cnt != expected:
                s.udp_cnt_gaps += 1
                gap = (lh.udp_cnt - expected) & 0xFFFF
                if 0 < gap < 1024:
                    s.udp_cnt_drops += gap
        s.udp_cnt_prev = lh.udp_cnt

        if lh.data_type == DATA_TYPE_CART_HIGH:
            s.dot_num_samples.append(lh.dot_num)
        elif lh.data_type == DATA_TYPE_IMU:
            imu = decode_imu_point(payload)
            if imu is not None:
                gx, gy, gz, ax, ay, az = imu
                s.imu_gyro_x.append(gx)
                s.imu_gyro_y.append(gy)
                s.imu_gyro_z.append(gz)
                s.imu_acc_x.append(ax)
                s.imu_acc_y.append(ay)
                s.imu_acc_z.append(az)

        if head_remaining.get(dst, 0) > 0:
            head_remaining[dst] -= 1
            rel = ts - first_ts
            print(
                f"  [{rel:8.4f}s] port={dst:<5} type={DATA_TYPE_NAMES.get(lh.data_type, lh.data_type)}"
                f"  udp_cnt={lh.udp_cnt:>5} frame_cnt={lh.frame_cnt:>3}"
                f"  dot_num={lh.dot_num:<4} length={lh.length:<5}"
                f"  time_type={lh.time_type}  ts_bytes={lh.timestamp.hex()}"
            )

    summarize(stats, duration)
    if non_udp:
        print(f"non-UDP/IPv4 packets skipped: {non_udp}")
    if other_ports:
        extras = ", ".join(f"{p}:{n}" for p, n in sorted(other_ports.items()))
        print(f"UDP on unexpected dst ports: {extras}")

    if not stats:
        print("WARNING: no Mid-360 traffic decoded — check filter / capture")
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
