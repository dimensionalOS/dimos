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

"""Ivan's run-to-run noise: re-run his PGO N times per recording and give mean +/- 2 sigma.

Each run is scored by the same code as bench.py. gsc is deterministic, so its single result is
compared against Ivan's band. Writes <name>.repeat.json next to the summaries.

usage: repeat.py <runs> <name> [<name> ...]      (dataset names as in $EVAL_LOOP_OUT)
"""

import json
import sys

import bench
import numpy as np
from paths import OUT

METRICS = ("voxels_pct", "tag_spread_m", "end_gap_m")


def metrics(stats: dict, row: dict) -> dict:
    return {
        "voxels_pct": (stats["voxels"] / row["voxels_raw"] - 1) * 100,
        "tag_spread_m": stats.get("tag_spread_m"),
        "end_gap_m": stats["end_gap_m"],
    }


def band(values: list) -> dict | None:
    values = [v for v in values if v is not None]
    if not values:
        return None
    mean, sigma = float(np.mean(values)), float(np.std(values, ddof=1)) if len(values) > 1 else 0.0
    return {
        "mean": mean,
        "sigma": sigma,
        "low": mean - 2 * sigma,
        "high": mean + 2 * sigma,
        "n": len(values),
    }


def main() -> None:
    runs, names = int(sys.argv[1]), sys.argv[2:]
    for name in names:
        row = json.loads((OUT / f"{name}.summary.json").read_text())
        source = bench.source_for(name)
        times, poses, clouds = bench.load(source)
        markers = (
            bench.markers_for(name, times, poses)[0]
            if row.get("tag_spread_raw_m") is not None
            else None
        )
        result: dict = {"runs": runs}
        for method, config in bench.IVAN_CONFIGS.items():
            samples = []
            for _ in range(runs):
                crashed = False
                try:
                    k_raw, k_opt, loops, runtime, _ = bench.run_ivan(source, config)
                except RuntimeError:  # Ivan's PGO raising ends his run; rerun with the ICP guard
                    crashed = True
                    k_raw, k_opt, loops, runtime, _ = bench.run_ivan(source, config, guarded=True)
                maps: dict = {}
                stats = bench.score(
                    k_raw, k_opt, loops, runtime, times, poses, clouds, markers, maps, method
                )
                samples.append(
                    {
                        "loops": loops,
                        "crashed_unguarded": crashed,
                        **metrics(stats, row),
                    }
                )
            result[method] = {
                "samples": samples,
                **{m: band([s[m] for s in samples]) for m in (*METRICS, "loops")},
                "crashed_unguarded": sum(s["crashed_unguarded"] for s in samples),
            }
            print(
                name,
                method,
                {m: result[method][m] for m in ("voxels_pct", "tag_spread_m")},
                flush=True,
            )
        (OUT / f"{name}.repeat.json").write_text(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
