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
"""Read ort_tap's capture (ort_capture_<time>.bin, policy.bin in a session): every inference rl_deploy ran.

python ort_capture.py CAPTURE              summary: sessions, runs per model, Run time, gaps, drops
python ort_capture.py CAPTURE --check OUT  compare with ort_tap_standin's OUT (the stand-in test)
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass, field
import struct

import numpy as np

NAME_BYTES, MAX_DIMS = 32, 4


@dataclass
class Tensor:
    name: str
    shape: tuple[int, ...]
    data: np.ndarray


@dataclass
class Run:
    seq: int
    session: int
    thread: int
    mono_before_ns: int
    mono_after_ns: int
    realtime_ns: int
    ecan_frames: int
    ecan_last_rx_ns: int
    failed: bool
    inputs: list[Tensor] = field(default_factory=list)
    outputs: list[Tensor] = field(default_factory=list)


def read(path: str) -> tuple[dict[int, str], list[Run]]:
    b = open(path, "rb").read()
    if b[:8] != b"M20ORT1\0":
        raise ValueError(f"{path}: not an ort_tap capture")
    o, sessions, runs = 8, {}, []
    while o + 8 <= len(b):
        kind, length = struct.unpack_from("<II", b, o)
        body = b[o + 8 : o + length]
        o += length
        if kind == 1:
            sessions[struct.unpack_from("<I", body)[0]] = body[4:].decode(errors="replace")
        elif kind == 2:
            seq, session, thread, before, after, realtime, frames, last_rx, failed, n_in, n_out = (
                struct.unpack_from("<QIIQQQQQiII", body)
            )
            run = Run(seq, session, thread, before, after, realtime, frames, last_rx, bool(failed))
            p = struct.calcsize("<QIIQQQQQiII")
            for k in range(n_in + n_out):
                name = body[p : p + NAME_BYTES].split(b"\0")[0].decode()
                dims = struct.unpack_from("<I", body, p + NAME_BYTES)[0]
                shape = struct.unpack_from(f"<{MAX_DIMS}q", body, p + NAME_BYTES + 4)[:dims]
                floats = struct.unpack_from("<I", body, p + NAME_BYTES + 4 + 8 * MAX_DIMS)[0]
                q = p + NAME_BYTES + 8 + 8 * MAX_DIMS
                t = Tensor(name, tuple(shape), np.frombuffer(body, "<f4", floats, q).copy())
                (run.inputs if k < n_in else run.outputs).append(t)
                p = q + 4 * floats
            runs.append(run)
    return sessions, runs


def summary(sessions: dict[int, str], runs: list[Run]) -> None:
    for i, path in sorted(sessions.items()):
        mine = [r for r in runs if r.session == i]
        if not mine:
            continue
        us = np.array([(r.mono_after_ns - r.mono_before_ns) / 1e3 for r in mine])
        gaps = np.diff([r.mono_before_ns for r in mine]) / 1e6
        print(
            f"{path}: {len(mine)} runs, Run median {np.median(us):.0f} us max {us.max():.0f} us, "
            f"gap median {np.median(gaps) if len(gaps) else 0:.1f} ms"
        )
    seqs = np.array([r.seq for r in runs])
    if len(seqs):
        print(
            f"{len(runs)} runs recorded, seq {seqs[0]}..{seqs[-1]} ({seqs[-1] - seqs[0] + 1 - len(seqs)} missing)"
        )


def check(runs: list[Run], out_path: str) -> None:
    """ort_tap_standin's OUT, per Run: u32 kind (0 frames: 318 in, 16 + 3 out; 1 sequence: 1908 + 21 + 1 in, 16 out)."""
    b = open(out_path, "rb").read()
    o, expected = 0, []
    while o < len(b):
        kind = struct.unpack_from("<I", b, o)[0]
        o += 4
        sizes = ([318], [16, 3]) if kind == 0 else ([36 * 53, 21, 1], [16])
        tensors = []
        for n in sizes[0] + sizes[1]:
            tensors.append(np.frombuffer(b, "<f4", n, o))
            o += 4 * n
        expected.append((kind, tensors))
    if len(runs) > len(expected):
        raise SystemExit(f"{len(runs)} runs recorded, the stand-in ran {len(expected)}")
    # the capture starts when the enable file is seen: align on the first recorded run's inputs
    first = runs[0].inputs[0].data
    start = next(
        k
        for k, (_, t) in enumerate(expected)
        if len(t[0]) == len(first) and np.array_equal(t[0], first)
    )
    bad = 0
    for run, (_kind, tensors) in zip(runs, expected[start:], strict=False):
        got = [t.data for t in run.inputs + run.outputs]
        if len(got) != len(tensors) or not all(
            np.array_equal(g, e) for g, e in zip(got, tensors, strict=True)
        ):
            bad += 1
    print(
        f"{len(runs)} recorded runs from stand-in run {start}: {len(runs) - bad} bit-identical, {bad} differ"
    )
    if bad:
        raise SystemExit(1)


def main() -> None:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("capture")
    parser.add_argument("--check", help="ort_tap_standin's OUT file")
    args = parser.parse_args()
    sessions, runs = read(args.capture)
    summary(sessions, runs)
    if args.check:
        check(runs, args.check)


if __name__ == "__main__":
    main()
