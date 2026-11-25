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

# Copyright 2026 Dimensional Inc.
# SPDX-License-Identifier: Apache-2.0
"""Line up one record.sh session into a single timeline: motor-bus frames (ecan.pcapng), the commands sent to the
robot (basic_server UDP/TCP, DDS samples incl. /NAV_CMD, m20tool runs in net.pcap and the m20tool logs) and the
vendor logs. Standard library only (runs on the robot's Python 3.8).

usage: timeline.py SESSION_DIR [--gap-ms 3] [--no-heartbeats]
writes into SESSION_DIR:
  timeline.csv     t_ns, time, source, event, detail: every command and every change of the motor-bus state, sorted
  joints.csv       one row per motor-board reply (1 kHz): each joint's URDF position/velocity/torque, status word,
                   and the command rl_deploy sent just before it (URDF position, velocity, torque, kp, kd, word)
  dds_writers.csv  every DDS writer seen: host, writer GUID, topic (when its discovery was captured), samples, sizes
"""

from __future__ import annotations

import argparse
import collections
import csv
from dataclasses import dataclass, field
import datetime
import json
import math
import os
import re
import struct
from typing import TYPE_CHECKING, Any

if TYPE_CHECKING:
    from collections.abc import Iterator

    # one struct-unpacked joint unit (JOINT's fields)
    Unit = tuple[Any, ...]
    # (t_ns, source, event, detail): one timeline.csv row
    Event = tuple[int, str, str, str]


LEGS = ("fl", "fr", "hl", "hr")
KINDS = ("hipx", "hipy", "knee", "wheel")
JOINT_NAMES = [f"{leg}_{kind}_joint" for leg in LEGS for kind in KINDS]
# vendor sdk_deploy m20_interface.hpp: urdf = motor * direction + offset (deg); hips/knees wrapped into (-pi, pi]
OFFSET_RAD = [
    math.radians(d)
    for d in (-25, -131, 160, 0, 25, -131, 160, 0, -25, 131, -160, 0, 25, 131, -160, 0)
]
DIRECTION = (1, 1, -1, 1, 1, -1, 1, -1, -1, 1, -1, 1, -1, -1, 1, -1)
JOINT = struct.Struct("<BBHfffhhHH")
BATTERY = struct.Struct("<HhHHHHHHHBBBBB")
TYPES = {
    0x81: "type81",
    0x84: "type84",
    0x86: "type86",
    0x87: "battery",
    0x89: "joints",
    0x8A: "ecan_status",
    0xA3: "typeA3",
    0xB1: "version",
    0xB3: "charge_enable",
}


def urdf(index: int, motor: float) -> float:
    value = motor * DIRECTION[index] + OFFSET_RAD[index]
    if index % 4 == 3:
        return value
    return math.remainder(value, math.tau)


def ecan_frame(frame: bytes) -> tuple[int, bytes] | None:
    """(frame type, payload) of an Ethernet frame, or None."""
    if len(frame) < 24 or frame[12:14] != b"\x44\x52":
        return None
    length = struct.unpack_from("<H", frame, 20)[0]
    return frame[19], frame[24 : 24 + length]


def joint_units(payload: bytes) -> dict[int, Unit]:
    out: dict[int, Unit] = {}
    for at in range(0, len(payload) - 23, 24):
        unit = JOINT.unpack_from(payload, at)
        leg, kind = unit[1], unit[2] - 2
        if 0 <= leg < 4 and 0 <= kind < 4:
            out[leg * 4 + kind] = unit
    return out


def read_packets(path: str) -> Iterator[tuple[int, int, str | None, bytes]]:
    """(t_ns, linktype, direction, bytes) from a pcap or pcapng file; direction 'rx'/'tx'/None (pcapng epb_flags)."""
    data = open(path, "rb").read()
    if len(data) < 24:
        return
    magic = struct.unpack_from("<I", data, 0)[0]
    if magic in (0xA1B2C3D4, 0xA1B23C4D):
        nanos = magic == 0xA1B23C4D
        linktype = struct.unpack_from("<I", data, 20)[0]
        at = 24
        while at + 16 <= len(data):
            sec, frac, caplen, _ = struct.unpack_from("<IIII", data, at)
            yield (
                sec * 1_000_000_000 + (frac if nanos else frac * 1000),
                linktype,
                None,
                data[at + 16 : at + 16 + caplen],
            )
            at += 16 + caplen
        return
    if magic != 0x0A0D0D0A:
        raise ValueError(f"{path}: not pcap or pcapng")
    interfaces: list[tuple[int, int]] = []
    at = 0
    while at + 12 <= len(data):
        block_type, block_len = struct.unpack_from("<II", data, at)
        if block_len < 12:
            break
        if block_type == 1:
            linktype = struct.unpack_from("<H", data, at + 8)[0]
            resolution = 1_000_000
            option = at + 16
            while option + 4 <= at + block_len - 4:
                code, length = struct.unpack_from("<HH", data, option)
                if code == 0:
                    break
                if code == 9:
                    value = data[option + 4]
                    resolution = (1 << (value & 0x7F)) if value & 0x80 else 10**value
                option += 4 + ((length + 3) & ~3)
            interfaces.append((linktype, resolution))
        elif block_type == 6:
            interface, high, low, caplen = struct.unpack_from("<IIII", data, at + 8)
            frame = data[at + 28 : at + 28 + caplen]
            option = at + 28 + ((caplen + 3) & ~3)
            direction: str | None = None
            while option + 4 <= at + block_len - 4:
                code, length = struct.unpack_from("<HH", data, option)
                if code == 0:
                    break
                if code == 2:
                    flags = struct.unpack_from("<I", data, option + 4)[0] & 3
                    direction = {1: "rx", 2: "tx"}.get(flags)
                option += 4 + ((length + 3) & ~3)
            linktype, resolution = (
                interfaces[interface] if interface < len(interfaces) else (1, 1_000_000)
            )
            yield ((high << 32) | low) * 1_000_000_000 // resolution, linktype, direction, frame
        at += block_len


def ip_packet(linktype: int, frame: bytes) -> bytes | None:
    """The IPv4 packet inside a link-layer frame."""
    if linktype == 1:
        ethertype, offset = struct.unpack_from(">H", frame, 12)[0], 14
    elif linktype == 113:
        ethertype, offset = struct.unpack_from(">H", frame, 14)[0], 16
    elif linktype == 276:
        ethertype, offset = struct.unpack_from(">H", frame, 0)[0], 20
    elif linktype in (101, 228):
        ethertype, offset = 0x0800, 0
    else:
        return None
    return frame[offset:] if ethertype == 0x0800 else None


def udp_tcp(packet: bytes) -> tuple[str, str, int, str, int, bytes, int | None] | None:
    """(protocol, src, sport, dst, dport, payload, tcp seq) of an IPv4 UDP/TCP packet."""
    if len(packet) < 20 or packet[0] >> 4 != 4:
        return None
    header = (packet[0] & 0x0F) * 4
    total = struct.unpack_from(">H", packet, 2)[0]
    protocol = packet[9]
    src = ".".join(map(str, packet[12:16]))
    dst = ".".join(map(str, packet[16:20]))
    body = packet[header : max(total, header)] if total else packet[header:]
    if protocol == 17 and len(body) >= 8:
        sport, dport = struct.unpack_from(">HH", body, 0)
        return "udp", src, sport, dst, dport, body[8:], None
    if protocol == 6 and len(body) >= 20:
        sport, dport, seq = struct.unpack_from(">HHI", body, 0)
        offset = (body[12] >> 4) * 4
        return "tcp", src, sport, dst, dport, body[offset:], seq
    return None


APDU_MAGIC = bytes.fromhex("eb91eb90")
APDU = struct.Struct("<4sHHB7s")
# (Type, Command) as the APDU's JSON carries them
BASIC_NAMES: dict[tuple[object, object], str] = {
    (100, 100): "heartbeat",
    (2, 21): "axis command",
    (2, 22): "motion state",
    (2, 23): "gait",
    (2, 24): "charge",
    (1101, 5): "usage mode",
    (1002, 3): "report: error list",
    (1002, 4): "report: motion status",
    (1002, 5): "report: battery",
    (1002, 6): "report: basic status",
}


def apdus(stream: bytes) -> tuple[list[tuple[int, dict[str, Any], bytes]], bytes]:
    """Complete APDUs at the front of a byte buffer; returns (messages, unconsumed bytes)."""
    messages: list[tuple[int, dict[str, Any], bytes]] = []
    while True:
        start = stream.find(APDU_MAGIC)
        if start < 0:
            return messages, stream[-3:]
        stream = stream[start:]
        if len(stream) < APDU.size:
            return messages, stream
        _, length, message_id, encoding, _ = APDU.unpack_from(stream, 0)
        if len(stream) < APDU.size + length:
            return messages, stream
        body = stream[APDU.size : APDU.size + length]
        stream = stream[APDU.size + length :]
        try:
            document = json.loads(body.decode("utf-8", "replace")) if encoding == 1 else {}
        except ValueError:
            document = {}
        device = document.get("PatrolDevice", {}) if isinstance(document, dict) else {}
        messages.append((message_id, device, body))


SEDP_WRITERS = (b"\x00\x00\x03\xc2", b"\x00\x00\x04\xc2")  # publications, subscriptions announcers


def rtps_samples(payload: bytes) -> Iterator[tuple[bytes, bytes, bytes, bool]]:
    """(guid prefix, writer id, serialized payload, is_sedp) for every DATA submessage."""
    if len(payload) < 20 or payload[:4] != b"RTPS":
        return
    prefix = payload[8:20]
    at = 20
    while at + 4 <= len(payload):
        kind, flags = payload[at], payload[at + 1]
        endian = "<" if flags & 1 else ">"
        length = struct.unpack_from(endian + "H", payload, at + 2)[0]
        body = payload[at + 4 : at + 4 + length] if length else payload[at + 4 :]
        if kind == 0x0E and len(body) >= 12:  # INFO_DST
            pass
        elif kind == 0x15 and len(body) >= 20:  # DATA
            to_qos = struct.unpack_from(endian + "H", body, 2)[0]
            writer = body[8:12]
            position = 4 + to_qos
            if flags & 0x02:  # inline QoS
                while position + 4 <= len(body):
                    pid, plen = struct.unpack_from(endian + "HH", body, position)
                    position += 4 + plen
                    if pid == 1:
                        break
            if flags & 0x0C:
                yield prefix, writer, body[position:], writer in SEDP_WRITERS
        if length == 0:
            break
        at += 4 + length


def parameter_list(serialized: bytes) -> tuple[dict[int, bytes], str] | None:
    """({pid: value}, struct endian) of a PL_CDR payload, or None."""
    if len(serialized) < 4:
        return None
    endian = "<" if serialized[1] in (1, 3) else ">"
    out: dict[int, bytes] = {}
    at = 4
    while at + 4 <= len(serialized):
        pid, length = struct.unpack_from(endian + "HH", serialized, at)
        if pid == 1:
            break
        out.setdefault(pid & 0x3FFF, serialized[at + 4 : at + 4 + length])
        at += 4 + length
    return out, endian


def cdr_string(value: bytes, endian: str) -> str:
    if len(value) < 4:
        return ""
    length = struct.unpack_from(endian + "I", value, 0)[0]
    return value[4 : 4 + length].rstrip(b"\x00").decode("utf-8", "replace")


def nav_cmd(serialized: bytes, t_ns: int) -> tuple[float, float, float] | None:
    """(x, y, yaw) of a drdds NavCmd sample (u64 frame_id, i32 sec, u32 nsec, 3 f32), or None."""
    if len(serialized) != 32 or serialized[:2] not in (b"\x00\x01", b"\x00\x07"):
        return None
    _, sec, nsec, x, y, yaw = struct.unpack_from("<QiIfff", serialized, 4)
    if (
        abs(sec - t_ns / 1e9) > 86400
        or nsec >= 1_000_000_000
        or not all(map(math.isfinite, (x, y, yaw)))
    ):
        return None
    return float(x), float(y), float(yaw)


def int_sample(serialized: bytes, t_ns: int) -> int | None:
    """The value of a drdds header + int32 sample (MotionState, Gait), or None."""
    if len(serialized) != 24 or serialized[:2] not in (b"\x00\x01", b"\x00\x07"):
        return None
    _, sec, nsec, value = struct.unpack_from("<QiIi", serialized, 4)
    if abs(sec - t_ns / 1e9) > 86400 or nsec >= 1_000_000_000:
        return None
    return int(value)


def iso(t_ns: int) -> str:
    return datetime.datetime.fromtimestamp(t_ns / 1e9).strftime("%Y-%m-%d %H:%M:%S.%f")


LOG_LINE = re.compile(r"^\[(\d{4}-\d\d-\d\d \d\d:\d\d:\d\d)\.(\d{3})\.(\d{3})\]\s*(.*)$")


def log_events(path: str, source: str) -> Iterator[Event]:
    if not os.path.exists(path):
        return
    for line in open(path, encoding="utf-8", errors="replace"):
        match = LOG_LINE.match(line.rstrip("\n"))
        if not match:
            continue
        stamp = datetime.datetime.strptime(match.group(1), "%Y-%m-%d %H:%M:%S").timestamp()
        t_ns = (
            int(stamp) * 1_000_000_000
            + int(match.group(2)) * 1_000_000
            + int(match.group(3)) * 1000
        )
        yield t_ns, source, "log", match.group(4).strip()


@dataclass
class WriterStats:
    """One DDS writer's samples, for dds_writers.csv."""

    first: int
    last: int
    count: int = 0
    sizes: collections.Counter[int] = field(default_factory=collections.Counter)
    dst: set[str] = field(default_factory=set)


def main() -> None:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("session")
    parser.add_argument(
        "--gap-ms", type=float, default=3.0, help="report command gaps longer than this"
    )
    parser.add_argument(
        "--no-heartbeats", action="store_true", help="leave basic_server heartbeats/reports out"
    )
    args = parser.parse_args()
    session = args.session
    events: list[Event] = []

    # motor bus
    joints_out = csv.writer(open(os.path.join(session, "joints.csv"), "w", newline=""))
    header = ["t_ns", "seq"]
    for name in JOINT_NAMES:
        header += [
            f"{name}.{field}"
            for field in (
                "q",
                "dq",
                "tau",
                "status",
                "q_cmd",
                "dq_cmd",
                "tau_cmd",
                "kp",
                "kd",
                "word",
            )
        ]
    joints_out.writerow(header)
    last_command: dict[int, Unit] = {}
    last_mode: tuple[tuple[Any, Any, Any], ...] | None = None
    last_status: tuple[Any, ...] | None = None
    last_tx: int | None = None
    # (direction, frame type name) -> frames; "-" when the capture recorded no direction
    counts: collections.Counter[tuple[str, str]] = collections.Counter()
    ecan_path = os.path.join(session, "ecan.pcapng")
    first: int | None = None
    last: int | None = None
    if os.path.exists(ecan_path):
        for t_ns, _, direction, frame in read_packets(ecan_path):
            ecan = ecan_frame(frame)
            if ecan is None:
                continue
            first = first if first is not None else t_ns
            last = t_ns
            frame_type, payload = ecan
            counts[(direction or "-", TYPES.get(frame_type, hex(frame_type)))] += 1
            seq = frame[16]
            if frame_type == 0x89 and direction == "tx":
                units = joint_units(payload)
                last_command = units
                if last_tx is not None and t_ns - last_tx > args.gap_ms * 1e6:
                    events.append(
                        (
                            t_ns,
                            "ecan",
                            "command gap",
                            f"{(t_ns - last_tx) / 1e6:.1f} ms without a joint command",
                        )
                    )
                last_tx = t_ns
                mode = tuple((units[i][6], units[i][7], units[i][8]) for i in sorted(units))
                if mode != last_mode:
                    groups = []
                    for k, kind in enumerate(KINDS):
                        kps = sorted({units[i][6] / 10 for i in units if i % 4 == k})
                        kds = sorted({units[i][7] / 10 for i in units if i % 4 == k})
                        words = sorted({units[i][8] for i in units if i % 4 == k})
                        groups.append(
                            f"{kind} kp {'/'.join(map(str, kps))} kd {'/'.join(map(str, kds))} word {'/'.join(map(str, words))}"
                        )
                    events.append((t_ns, "ecan", "command gains", "; ".join(groups)))
                    last_mode = mode
            elif frame_type == 0x89 and direction == "rx" and len(payload) >= 40:
                units = joint_units(payload[40:])
                status = tuple(units[i][8] for i in sorted(units))
                if status != last_status:
                    events.append(
                        (
                            t_ns,
                            "ecan",
                            "status words",
                            " ".join(f"{JOINT_NAMES[i]}={units[i][8]:#x}" for i in sorted(units)),
                        )
                    )
                    last_status = status
                row: list[object] = [t_ns, seq]
                for i in range(16):
                    state = units.get(i)
                    sent = last_command.get(i)
                    row += (
                        [
                            f"{urdf(i, state[3]):.5f}",
                            f"{state[4] * DIRECTION[i]:.5f}",
                            f"{state[5] * DIRECTION[i]:.5f}",
                            state[8],
                        ]
                        if state
                        else ["", "", "", ""]
                    )
                    row += (
                        [
                            f"{urdf(i, sent[3]):.5f}",
                            f"{sent[4] * DIRECTION[i]:.5f}",
                            f"{sent[5] * DIRECTION[i]:.5f}",
                            sent[6] / 10,
                            sent[7] / 10,
                            sent[8],
                        ]
                        if sent
                        else [""] * 6
                    )
                joints_out.writerow(row)
            elif frame_type == 0x87 and direction == "rx":
                packs = []
                for side, at in (("L", 0), ("R", 258)):
                    if len(payload) >= at + 258:
                        v = BATTERY.unpack_from(payload, at)
                        packs.append(
                            f"{side} {v[10]}% {v[0] / 100:.2f} V {v[1] / 100:+.2f} A prot {v[8]:#x} mos {v[11]}"
                        )
                events.append((t_ns, "ecan", "battery", "; ".join(packs)))
            elif frame_type == 0x89:
                pass
            else:
                events.append(
                    (
                        t_ns,
                        "ecan",
                        f"{direction} {TYPES.get(frame_type, hex(frame_type))}",
                        payload[:32].hex(),
                    )
                )
        if first is not None and last is not None:
            events.append((first, "ecan", "first frame", ""))
            events.append(
                (
                    last,
                    "ecan",
                    "last frame",
                    ", ".join(f"{d} {k}: {n}" for (d, k), n in sorted(counts.items())),
                )
            )

    # network: basic_server, m20tool notes, DDS
    tcp_buffers: collections.defaultdict[tuple[str, str, int, str, int], bytes] = (
        collections.defaultdict(bytes)
    )
    # writer GUID -> (topic name, type name), from the captured discovery
    topics: dict[bytes, tuple[str, str]] = {}
    writers: dict[tuple[str, str], WriterStats] = {}
    net_path = os.path.join(session, "net.pcap")
    pending_dds: list[tuple[int, str, bytes, bytes]] = []
    if os.path.exists(net_path):
        for t_ns, linktype, _, frame in read_packets(net_path):
            packet = ip_packet(linktype, frame)
            parsed = udp_tcp(packet) if packet else None
            if parsed is None:
                continue
            protocol, src, sport, dst, dport, payload, _ = parsed
            if protocol == "udp" and dport == 30999 and payload.startswith(b"M20TOOL "):
                events.append(
                    (t_ns, "m20tool", "run", payload[8:].decode("utf-8", "replace").strip())
                )
                continue
            if 30000 in (sport, dport) or 30001 in (sport, dport):
                stream = (protocol, src, sport, dst, dport)
                if protocol == "tcp":
                    tcp_buffers[stream] += payload
                    messages, tcp_buffers[stream] = apdus(tcp_buffers[stream])
                else:
                    messages, _ = apdus(payload)
                to_robot = dport in (30000, 30001)
                for message_id, device, _body in messages:
                    basic_kind = (device.get("Type"), device.get("Command"))
                    name = BASIC_NAMES.get(
                        basic_kind, f"type {basic_kind[0]} command {basic_kind[1]}"
                    )
                    if args.no_heartbeats and (basic_kind == (100, 100) or basic_kind[0] == 1002):
                        continue
                    items = json.dumps(device.get("Items", {}), separators=(",", ":"))
                    if len(items) > 400:
                        items = items[:400] + "..."
                    where = f"{src}:{sport} -> {dst}:{dport} {protocol} id {message_id}"
                    events.append(
                        (
                            t_ns,
                            "basic_server",
                            ("command " if to_robot else "reply ") + name,
                            f"{items} ({where})",
                        )
                    )
                continue
            if protocol == "udp" and 7400 <= dport <= 7700:
                for prefix, writer, serialized, sedp in rtps_samples(payload):
                    if sedp:
                        parameters = parameter_list(serialized)
                        if parameters:
                            values, endian = parameters
                            announced = values.get(0x5A)
                            name = cdr_string(values.get(0x05, b""), endian)
                            if announced and name:
                                topics[announced[:16]] = (
                                    name,
                                    cdr_string(values.get(0x07, b""), endian),
                                )
                        continue
                    info = writers.setdefault(
                        (src, (prefix + writer).hex()), WriterStats(first=t_ns, last=t_ns)
                    )
                    info.count += 1
                    info.sizes[len(serialized)] += 1
                    info.last = t_ns
                    info.dst.add(f"{dst}:{dport}")
                    pending_dds.append((t_ns, src, prefix + writer, serialized))

    # DDS samples, decoded once every captured discovery is known
    for t_ns, src, guid, serialized in pending_dds:
        topic = topics.get(guid, ("", ""))[0]
        if topic in ("", "/NAV_CMD", "rt/NAV_CMD"):
            command = nav_cmd(serialized, t_ns)
            if command is not None:
                label = "NAV_CMD" if topic else "NAV_CMD? (layout match, no discovery captured)"
                events.append(
                    (
                        t_ns,
                        "dds",
                        label,
                        f"x {command[0]:+.3f} y {command[1]:+.3f} yaw {command[2]:+.3f} from {src} writer {guid.hex()}",
                    )
                )
                continue
        if topic and any(word in topic.upper() for word in ("MOTION_STATE", "GAIT")):
            value = int_sample(serialized, t_ns)
            if value is not None:
                events.append((t_ns, "dds", topic, f"{value} ({value:#x}) from {src}"))
    with open(os.path.join(session, "dds_writers.csv"), "w", newline="") as out:
        table = csv.writer(out)
        table.writerow(
            [
                "src",
                "writer_guid",
                "topic",
                "type",
                "samples",
                "per_s",
                "payload_sizes",
                "destinations",
            ]
        )
        for (src, guid_hex), info in writers.items():
            topic, type_name = topics.get(bytes.fromhex(guid_hex), ("", ""))
            span = max(info.last - info.first, 1) / 1e9
            sizes = " ".join(f"{size}x{n}" for size, n in info.sizes.most_common(4))
            table.writerow(
                [
                    src,
                    guid_hex,
                    topic,
                    type_name,
                    info.count,
                    f"{info.count / span:.1f}",
                    sizes,
                    " ".join(sorted(info.dst)),
                ]
            )

    # m20tool logs from the boards, vendor logs
    for name in sorted(os.listdir(session)):
        if name.startswith("m20tool_") and name.endswith(".log"):
            for line in open(os.path.join(session, name), encoding="utf-8", errors="replace"):
                parts = line.split(" ", 1)
                if parts[0].isdigit() and (
                    first is None or first - 60e9 <= int(parts[0]) <= (last or 0) + 60e9
                ):
                    events.append((int(parts[0]), "m20tool", f"run ({name})", parts[1].strip()))
    events += log_events(os.path.join(session, "rl_deploy.log"), "rl_deploy")
    events += log_events(os.path.join(session, "basic_server.log"), "basic_server_log")

    events.sort(key=lambda event: event[0])
    with open(os.path.join(session, "timeline.csv"), "w", newline="") as out:
        table = csv.writer(out)
        table.writerow(["t_ns", "time", "source", "event", "detail"])
        for t_ns, source, event, detail in events:
            table.writerow([t_ns, iso(t_ns), source, event, detail])
    by_source = collections.Counter(event[1] for event in events)
    print(
        f"timeline.csv: {len(events)} events ({', '.join(f'{k} {v}' for k, v in sorted(by_source.items()))}); "
        f"joints.csv; dds_writers.csv: {len(writers)} writers, {len(topics)} with a captured topic name"
    )
    for (direction, kind), n in sorted(counts.items()):
        print(f"  ecan {direction} {kind}: {n}")


if __name__ == "__main__":
    main()
