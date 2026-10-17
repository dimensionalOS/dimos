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

"""Re-run only gsc with config overrides on already-benchmarked datasets, scored by the same code.

Adds `summary[<label>]` next to "ivan", "ivan_live" and "gsc" (which stay as they were), so a gsc
setting can be compared without re-running Ivan's PGO. Label "gsc" with '{}' re-runs gsc's current
defaults in place (after a default changes), including the maps render.py draws.

usage: variant.py <label> '<json overrides>' <name> [<name> ...]
       names are dataset summary names in $EVAL_LOOP_OUT
"""

import json
import sys

import bench
import numpy as np
from paths import OUT


def main() -> None:
    label, overrides, names = sys.argv[1], json.loads(sys.argv[2]), sys.argv[3:]
    config_path = OUT / f"{label}.config.json"
    config_path.write_text(json.dumps(overrides))
    for name in names:
        summary_path = OUT / f"{name}.summary.json"
        row = json.loads(summary_path.read_text())
        source = bench.source_for(name)
        times, poses, clouds = bench.load(source)
        markers = None
        if row.get("tag_spread_raw_m") is not None:
            markers = bench.markers_for(name, times, poses)[0]
        g_raw, g_opt, loops, runtime, result = bench.run_gsc(
            OUT / f"{name}.gsc.db", OUT / f"{name}.{label}.json", config=config_path
        )
        maps: dict = {}
        row[label] = bench.score(
            g_raw, g_opt, loops, runtime, times, poses, clouds, markers, maps, label
        )
        row[label]["loops_before_gnc"] = result["closures"]
        np.save(OUT / f"{name}.{label}_path.npy", maps[f"{label}_path"])
        maps_path = OUT / f"{name}.maps.npz"
        if (
            label == "gsc" and maps_path.exists()
        ):  # re-running the defaults: refresh what render.py draws
            np.savez_compressed(maps_path, **{**np.load(maps_path), **maps})
        row[label]["overrides"] = overrides
        summary_path.write_text(json.dumps(row, indent=2))
        print(name, label, {k: row[label][k] for k in ("loops", "voxels", "end_gap_m")}, flush=True)


if __name__ == "__main__":
    main()
