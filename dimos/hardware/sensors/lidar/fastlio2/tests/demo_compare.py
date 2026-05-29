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

"""Compare two memory2 SqliteStores of FastLio2 outputs.

Usage:
    demo_compare.py <ground_truth.db> <replay.db>

Walks the `lidar` and `odometry` streams in both stores, aligns by index,
and reports per-stream counts plus the max absolute difference in pose
(odometry) and point counts (lidar). For a deterministic replay we expect
identical counts and very small pose deltas.
"""

from __future__ import annotations

import argparse
import bisect
from pathlib import Path
import sys

import numpy as np

from dimos.memory2.store.sqlite import SqliteStore
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2

# Replay publishes against pcap-capture time; live publishes against
# system-clock-at-publish time. Same algorithm state but with up to a few
# ms of post-capture latency. Pair messages within this window.
_TS_PAIR_TOLERANCE = 0.050


def _pose_tuple(odom: Odometry) -> tuple[float, ...]:
    p = odom.pose.pose
    return (
        p.position.x,
        p.position.y,
        p.position.z,
        p.orientation.x,
        p.orientation.y,
        p.orientation.z,
        p.orientation.w,
    )


def _pair_by_ts(a: list, b: list, tol: float) -> list[tuple[int, int]]:
    """For each item in `a` find the nearest-ts item in `b` within tol."""
    b_ts = [obs.ts for obs in b]
    pairs: list[tuple[int, int]] = []
    for i, obs in enumerate(a):
        t = obs.ts
        j = bisect.bisect_left(b_ts, t)
        candidates = []
        if j < len(b_ts):
            candidates.append(j)
        if j > 0:
            candidates.append(j - 1)
        best_j = -1
        best_d = tol
        for c in candidates:
            d = abs(b_ts[c] - t)
            if d <= best_d:
                best_d = d
                best_j = c
        if best_j >= 0:
            pairs.append((i, best_j))
    return pairs


def compare_odometry(a_db: SqliteStore, b_db: SqliteStore) -> int:
    a = a_db.stream("odometry", Odometry).to_list()
    b = b_db.stream("odometry", Odometry).to_list()
    print(f"odometry: ground_truth={len(a)}  replay={len(b)}")
    pairs = _pair_by_ts(a, b, _TS_PAIR_TOLERANCE)
    print(f"  paired within {_TS_PAIR_TOLERANCE * 1000:.0f}ms: {len(pairs)} / {len(a)}")
    if not pairs:
        return 1
    max_diff = 0.0
    max_diff_at = -1
    for i, j in pairs:
        pa = _pose_tuple(a[i].data)
        pb = _pose_tuple(b[j].data)
        d = max(abs(x - y) for x, y in zip(pa, pb, strict=True))
        if d > max_diff:
            max_diff = d
            max_diff_at = i
    print(f"  max pose abs-diff: {max_diff:.6e} at ground_truth i={max_diff_at}")
    return 0 if max_diff < 1e-3 else 1


def compare_lidar(a_db: SqliteStore, b_db: SqliteStore) -> int:
    a = a_db.stream("lidar", PointCloud2).to_list()
    b = b_db.stream("lidar", PointCloud2).to_list()
    print(f"lidar: ground_truth={len(a)}  replay={len(b)}")
    pairs = _pair_by_ts(a, b, _TS_PAIR_TOLERANCE)
    print(f"  paired within {_TS_PAIR_TOLERANCE * 1000:.0f}ms: {len(pairs)} / {len(a)}")
    if not pairs:
        return 1
    max_pt_delta = 0
    max_xyz_abs_diff = 0.0
    max_xyz_at = -1
    for i, j in pairs:
        xa = a[i].data.as_numpy()[0]
        xb = b[j].data.as_numpy()[0]
        max_pt_delta = max(max_pt_delta, abs(xa.shape[0] - xb.shape[0]))
        common = min(xa.shape[0], xb.shape[0])
        if common == 0:
            continue
        d = float(np.max(np.abs(xa[:common] - xb[:common])))
        if d > max_xyz_abs_diff:
            max_xyz_abs_diff = d
            max_xyz_at = i
    print(f"  max point count abs-diff: {max_pt_delta}")
    print(
        f"  max xyz abs-diff (matched prefix): {max_xyz_abs_diff:.6e} "
        f"at ground_truth i={max_xyz_at}"
    )
    return 0 if max_xyz_abs_diff < 1e-3 else 1


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("ground_truth", type=Path)
    parser.add_argument("replay", type=Path)
    args = parser.parse_args()

    if not args.ground_truth.exists():
        print(f"[demo_compare] missing: {args.ground_truth}", file=sys.stderr)
        return 1
    if not args.replay.exists():
        print(f"[demo_compare] missing: {args.replay}", file=sys.stderr)
        return 1

    print(f"[demo_compare] ground_truth: {args.ground_truth}")
    print(f"[demo_compare] replay:       {args.replay}")
    print()
    a_db = SqliteStore(path=str(args.ground_truth))
    b_db = SqliteStore(path=str(args.replay))

    rc = 0
    rc |= compare_odometry(a_db, b_db)
    print()
    rc |= compare_lidar(a_db, b_db)
    return rc


if __name__ == "__main__":
    raise SystemExit(main())
