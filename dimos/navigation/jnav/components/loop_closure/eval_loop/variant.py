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

"""Replay a recording live through gsc_pgo (pgo-eval, --gnc live by default) and score it.

Adds `summary[<label>]` to each recording's summary; run.py uses label "gsc_live" with '{}' (the
module's defaults). Other labels with config overrides compare gsc settings on the same input.

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
        row[label]["overrides"] = overrides
        summary_path.write_text(json.dumps(row, indent=2))
        print(name, label, {k: row[label][k] for k in ("loops", "voxels", "end_gap_m")}, flush=True)


if __name__ == "__main__":
    main()
