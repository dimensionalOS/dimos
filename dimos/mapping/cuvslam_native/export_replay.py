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

"""Flatten a recording into raw files a standalone cuVSLAM harness can read.

    python -m dimos.mapping.cuvslam_native.export_replay <recording.db> <out_dir>

The point is to get dimos out of the measurement. Driving the tracker through the
module puts sqlite, lz4 and an LCM round trip of two 400 kB images per frame in
front of it, and that path freezes partway through every recording -- so what gets
timed is the transport, not cuVSLAM. Writing plain mono8 planes plus two CSVs lets
a C++ program feed the tracker directly with nothing else in the loop.

Deliberately uncompressed: the harness should spend its time in Track(), not in a
decoder, and disk is cheaper than an ambiguous number.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys

import numpy as np

from dimos.memory2.replay import Replay
from dimos.memory2.store.sqlite import SqliteStore

NS_PER_S = 1_000_000_000
# Above this share of unpairable left frames the export is wrong, not merely lossy.
MAX_UNPAIRED_FRACTION = 0.02


def export(db_path: Path, out_dir: Path, limit: int | None = None) -> dict[str, object]:
    replay = Replay(store=SqliteStore(path=str(db_path)))
    (out_dir / "left").mkdir(parents=True, exist_ok=True)
    (out_dir / "right").mkdir(parents=True, exist_ok=True)

    info = next(iter(replay.stream("realsense_infra_left_camera_info").iterate_ts()))[1]
    right_info = next(iter(replay.stream("realsense_infra_right_camera_info").iterate_ts()))[1]
    baseline_m = -right_info.P[3] / right_info.P[0] if right_info.P[0] else 0.0

    right_by_ts = {
        round(message.ts, 4): message
        for _ts, message in replay.stream("realsense_infra_right").iterate_ts()
    }
    right_stamps = np.array(sorted(right_by_ts))

    frames = []
    unpaired = 0
    for index, (_ts, left) in enumerate(replay.stream("realsense_infra_left").iterate_ts()):
        if limit is not None and index >= limit:
            break
        nearest = right_stamps[np.argmin(np.abs(right_stamps - left.ts))]
        right = right_by_ts[nearest]
        # cuVSLAM wants the stereo pair within 1 ms; a wider pair is not a pair.
        if abs(right.ts - left.ts) > 0.001:
            unpaired += 1
            continue
        left.data.tofile(out_dir / "left" / f"{index:06d}.raw")
        right.data.tofile(out_dir / "right" / f"{index:06d}.raw")
        frames.append((index, int(left.ts * NS_PER_S), int(right.ts * NS_PER_S)))

    with (out_dir / "frames.csv").open("w") as handle:
        handle.write("index,left_ns,right_ns\n")
        for index, left_ns, right_ns in frames:
            handle.write(f"{index},{left_ns},{right_ns}\n")

    imu_rows = 0
    with (out_dir / "imu.csv").open("w") as handle:
        handle.write("ts_ns,gx,gy,gz,ax,ay,az\n")
        for _ts, sample in replay.stream("realsense_imu").iterate_ts():
            angular, linear = sample.angular_velocity, sample.linear_acceleration
            handle.write(
                f"{int(sample.ts * NS_PER_S)},{angular.x},{angular.y},{angular.z},"
                f"{linear.x},{linear.y},{linear.z}\n"
            )
            imu_rows += 1

    # Pairing drops a left frame whose partner is missing, and silence there is how
    # a repair that removed rows from only one eye would quietly starve the tracker
    # instead of failing. The count is recorded, and a large loss is an error.
    if unpaired > MAX_UNPAIRED_FRACTION * max(len(frames) + unpaired, 1):
        raise RuntimeError(
            f"{unpaired} of {len(frames) + unpaired} left frames had no right partner "
            "within 1 ms; the stereo streams are not symmetric"
        )

    manifest = {
        "unpaired_left_frames": unpaired,
        "width": int(info.width),
        "height": int(info.height),
        "fx": float(info.K[0]),
        "fy": float(info.K[4]),
        "cx": float(info.K[2]),
        "cy": float(info.K[5]),
        "baseline_m": float(baseline_m),
        # The IR pair is delivered rectified, which is what lets the tracker run
        # with a pinhole model and an identity inter-camera rotation.
        "rectified": True,
        "frames": len(frames),
        "imu_samples": imu_rows,
        "source_db": str(db_path),
    }
    (out_dir / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
    return manifest


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("db", type=Path)
    parser.add_argument("out_dir", type=Path)
    parser.add_argument("--limit", type=int, default=None)
    args = parser.parse_args()
    manifest = export(args.db, args.out_dir, args.limit)
    print(json.dumps(manifest, indent=2))
    return 0


if __name__ == "__main__":
    sys.exit(main())
