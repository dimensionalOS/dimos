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

"""Read a lazy-feed file (see src/main.rs): the header, then GSC's messages and each scan's tracer output in order.

usage: read_feed.py <feed.zst>   prints the header's settings and record counts
"""

from __future__ import annotations

from collections.abc import Iterator
from dataclasses import dataclass
import json
from pathlib import Path
import struct
import sys

import numpy as np
import zstandard

NEW_KEYFRAME, LOOP_CLOSURE_EVENT, SCAN, END = 1, 2, 3, 9


@dataclass
class Scan:
    index: int
    ts: float
    pose: np.ndarray  # corrected body pose: x y z qx qy qz qw
    version: int  # the closure version the tracer's map was in
    cylinder: tuple[float, float, float, float, float] | None  # cx cy radius z_min z_max
    surfaces: (
        np.ndarray | None
    )  # (N, 7) float32: center, unit normal, plane offset (NaN normal without a plane)


@dataclass
class Message:
    tag: int  # NEW_KEYFRAME or LOOP_CLOSURE_EVENT
    index: int  # the scan it came out on
    ts: float
    lcm: bytes  # an LCM-encoded sensor_msgs/PointCloud2


def _exact(stream, n: int) -> bytes:
    data = stream.read(n)
    if len(data) != n:
        raise EOFError(f"feed ended mid-record ({len(data)} of {n} bytes)")
    return data


def read_feed(path: Path) -> tuple[dict, Iterator[Scan | Message | dict]]:
    """The header, and the records in order; the last is the summary dict."""
    stream = zstandard.ZstdDecompressor().stream_reader(open(path, "rb"))
    if _exact(stream, 8) != b"LAZYFEED":
        raise ValueError(f"{path} is not a lazy-feed file")
    (version,) = struct.unpack("<I", _exact(stream, 4))
    if version != 1:
        raise ValueError(f"{path}: format {version}, this reader knows 1")
    (length,) = struct.unpack("<I", _exact(stream, 4))
    header = json.loads(_exact(stream, length))

    def records() -> Iterator[Scan | Message | dict]:
        while True:
            tag, index, ts = struct.unpack("<BQd", _exact(stream, 17))
            if tag in (NEW_KEYFRAME, LOOP_CLOSURE_EVENT):
                (n,) = struct.unpack("<I", _exact(stream, 4))
                yield Message(tag, index, ts, _exact(stream, n))
            elif tag == SCAN:
                pose = np.frombuffer(_exact(stream, 56), "<f8").copy()
                (map_version, has_region) = struct.unpack("<IB", _exact(stream, 5))
                cylinder, surfaces = None, None
                if has_region:
                    cylinder = struct.unpack("<5f", _exact(stream, 20))
                    (n,) = struct.unpack("<I", _exact(stream, 4))
                    surfaces = np.frombuffer(_exact(stream, n * 28), "<f4").reshape(n, 7).copy()
                yield Scan(index, ts, pose, map_version, cylinder, surfaces)
            elif tag == END:
                rest = stream.read()
                yield json.loads(rest)
                return
            else:
                raise ValueError(f"unknown record tag {tag}")

    return header, records()


def main() -> None:
    header, records = read_feed(Path(sys.argv[1]))
    counts = {"scans": 0, "regions": 0, "new_keyframe": 0, "loop_closure_event": 0}
    summary = {}
    for record in records:
        if isinstance(record, Scan):
            counts["scans"] += 1
            counts["regions"] += record.cylinder is not None
        elif isinstance(record, Message):
            counts["new_keyframe" if record.tag == NEW_KEYFRAME else "loop_closure_event"] += 1
        else:
            summary = record
    print(json.dumps({"db": header["db"], "counts": counts, "summary": summary}, indent=1))


if __name__ == "__main__":
    main()
