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

"""Does gsc at an Ivan-equivalent config land inside Ivan's own run-to-run spread?

Ivan's PGO() voxel change across every given run directory gives a min..max per recording, and the
gsc_ivancfg result in the first directory is compared against it. An earlier run only counts if its
raw map is identical (same voxels_raw), so runs on a different prep of the input are left out.

usage: spread.py <main results dir> [<earlier run dir> ...]
"""

import json
from pathlib import Path
import sys

from recordings import DATASETS


def voxels(row: dict, key: str) -> float:
    return (row[key]["voxels"] / row["voxels_raw"] - 1) * 100


def main() -> None:
    dirs = [Path(d) for d in sys.argv[1:]]
    print(
        "| dataset | Ivan PGO() voxels, runs | range | gsc Ivan-config | inside range | gsc defaults |"
    )
    print("|---|---|---|---|---|---|")
    inside_count = total = 0
    for dataset in DATASETS:
        main_path = dirs[0] / f"{dataset.name}.summary.json"
        if not main_path.exists():
            continue
        row = json.loads(main_path.read_text())
        if "gsc_ivancfg" not in row:
            continue
        runs = [voxels(row, "ivan")]
        for earlier in dirs[1:]:
            path = earlier / f"{dataset.name}.summary.json"
            if path.exists():
                earlier_row = json.loads(path.read_text())
                if earlier_row["voxels_raw"] == row["voxels_raw"]:
                    runs.append(voxels(earlier_row, "ivan"))
        low, high = min(runs), max(runs)
        mine = voxels(row, "gsc_ivancfg")
        inside = low <= round(mine, 1) <= high
        inside_count += inside
        total += 1
        print(
            f"| {dataset.name} | {', '.join(f'{v:+.1f}%' for v in runs)} | {low:+.1f}..{high:+.1f}% "
            f"| {mine:+.1f}% | {'yes' if inside else f'no ({min(abs(mine - low), abs(mine - high)):.1f} pts out)'} "
            f"| {voxels(row, 'gsc'):+.1f}% |"
        )
    print(
        f"\ngsc at the Ivan-equivalent config is inside Ivan PGO()'s range on {inside_count} of {total}."
    )


if __name__ == "__main__":
    main()
