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

"""The user samples of a DDS capture (tcpdump of UDP 7400-7700), one line each, the first copy of every sample:

    capture_time_ns source_ip writer_guid sequence_number serialized_payload_hex

Every RTPS DATA submessage whose writer is not a built-in (discovery) one, its payload as sent (CDR encapsulation
header included). Each sample reaches several readers and interfaces; only its first copy is kept.

    python tools/dds_samples.py CAPTURE.pcap [WRITER_GUID_PREFIX ...] | gzip -9 > samples.txt.gz
"""

from collections.abc import Iterator
import struct
import sys


def udp_payloads(path: str) -> Iterator[tuple[int, str, bytes]]:
    """(capture time ns, source ip, udp payload) per IPv4 UDP packet (Ethernet or Linux cooked capture)."""
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
            src = ".".join(map(str, d[off + 12 : off + 16]))
            yield sec * 10**9 + frac * (1 if nano else 1000), src, d[off + ihl + 8 :]


def main() -> None:
    path, keep = sys.argv[1], sys.argv[2:]
    seen: set[tuple[str, int]] = set()
    for t, src, p in udp_payloads(path):
        if p[:4] != b"RTPS":
            continue
        prefix = p[8:20].hex()
        i = 20
        while i + 4 <= len(p):
            sid, flags = p[i], p[i + 1]
            little = flags & 1
            n = struct.unpack("<H" if little else ">H", p[i + 2 : i + 4])[0]
            if sid == 0x15:
                body = p[i + 4 : i + 4 + n]
                writer = body[8:12].hex()
                high, low = struct.unpack("<iI" if little else ">iI", body[12:20])
                guid = prefix + writer
                # entity kinds 0xc0-0xc7: the built-in (discovery) endpoints
                if body[11] < 0xC0 and (not keep or any(guid.startswith(k) for k in keep)):
                    key = (guid, (high << 32) | low)
                    if key not in seen:
                        seen.add(key)
                        print(f"{t} {src} {guid} {key[1]} {body[20:].hex()}")
            if n == 0:
                break
            i += 4 + n


if __name__ == "__main__":
    main()
