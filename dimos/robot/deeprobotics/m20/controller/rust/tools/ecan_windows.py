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

"""Cut windows of an ECAN capture (the ecan_tap shim's pcapng) into a compact gzipped fixture.

    python tools/ecan_windows.py IN.pcapng OUT.bin.gz KEEP_WINDOWS FULL_WINDOWS

Windows are "HH:MM:SS.fff-HH:MM:SS.fff" (CST, the capture's day), comma separated. Every frame in a KEEP window is kept
with its time (us). Frames to the board are kept whole (a joint command as its 16 units XOR the previous command's, or
as "repeated"; its 24-byte header is the first command's with this sequence number); the board's replies only in a FULL
window, elsewhere just their time. tests/ecan_dock_rec_20261124.rs reads it back (read_compact).

Layout (gzip): u32 n, 24-byte command header, n kind bytes, then per frame an i64 (first) / i32 us time delta, then the
side data in frame order (S: 452-byte reply, B: 24-byte charge-enable frame, Q: one sequence byte), then the units.
Kinds: S whole reply, s reply time only, B charge-enable frame, C command (units follow), r repeated command, Q (before
a command) its sequence number is not the previous plus one.
"""

import argparse
import gzip
import struct

CST_NS = 8 * 3600 * 10**9


def packets(data: bytes):
    """(time ns, epb_flags direction, frame) per enhanced packet block (nanosecond interface)."""
    i = 0
    while i + 12 <= len(data):
        kind, size = struct.unpack_from("<II", data, i)
        if kind == 6:
            _, hi, lo, cap, _ = struct.unpack_from("<IIIII", data, i + 8)
            frame = data[i + 28 : i + 28 + cap]
            o, direction = i + 28 + ((cap + 3) & ~3), 0
            while o + 4 <= i + size - 4:
                code, n = struct.unpack_from("<HH", data, o)
                if code == 0:
                    break
                if code == 2:
                    direction = struct.unpack_from("<I", data, o + 4)[0] & 3
                o += 4 + ((n + 3) & ~3)
            yield (hi << 32) | lo, direction, frame
        i += size


def windows(spec: str, day_ns: int) -> list[tuple[int, int]]:
    def at(hms: str) -> int:
        h, m, s = hms.split(":")
        return day_ns + (int(h) * 3600 + int(m) * 60) * 10**9 + round(float(s) * 1e9) - CST_NS

    return [tuple(at(x) for x in w.split("-")) for w in spec.split(",") if w]


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("capture")
    ap.add_argument("out")
    ap.add_argument("keep")
    ap.add_argument("full")
    a = ap.parse_args()
    frames = list(packets(open(a.capture, "rb").read()))
    day = (frames[0][0] + CST_NS) // (86400 * 10**9) * 86400 * 10**9
    keep, full = windows(a.keep, day), windows(a.full, day)
    kinds, times, side, units = bytearray(), bytearray(), bytearray(), bytearray()
    header = None
    prev_t = prev_units = prev_seq = None
    for t, direction, frame in frames:
        if not any(x <= t <= y for x, y in keep):
            continue
        us = t // 1000
        times += struct.pack("<q", us) if prev_t is None else struct.pack("<i", us - prev_t)
        prev_t = us
        if direction == 1:
            if any(x <= t <= y for x, y in full):
                kinds += b"S"
                side += frame
            else:
                kinds += b"s"
            continue
        seq = frame[16]
        if prev_seq is None or seq != (prev_seq + 1) % 256:
            kinds += b"Q"
            side.append(seq)
        prev_seq = seq
        if frame[19] == 0xB3:
            kinds += b"B"
            side += frame
        elif frame[24:] == prev_units:
            kinds += b"r"
        else:
            header = header or frame[:24]
            kinds += b"C"
            base = prev_units or bytes(len(frame) - 24)
            units += bytes(x ^ y for x, y in zip(frame[24:], base, strict=False))
            prev_units = frame[24:]
    n = sum(k in b"SsBCr" for k in kinds)
    blob = struct.pack("<I", len(kinds)) + header + kinds + times + side + units
    with open(a.out, "wb") as f:
        f.write(gzip.compress(blob, 9, mtime=0))
    print(f"{n} frames -> {a.out}")


if __name__ == "__main__":
    main()
