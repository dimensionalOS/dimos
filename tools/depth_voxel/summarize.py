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

"""Print the filter sweep as a markdown table, ranked by voxel-overlap IoU."""

from __future__ import annotations

import json

from tools.depth_voxel.run_experiment import RESULTS_DIR

COLUMNS = (
    ("config", "name"),
    ("params", "params"),
    ("depth voxels", "depth_voxels"),
    ("overlap", "intersection"),
    ("IoU", "iou"),
    ("precision", "depth_precision"),
    ("recall", "lidar_recall"),
    ("±1vox precision", "depth_precision_tolerant"),
    ("±1vox recall", "lidar_recall_tolerant"),
)


def main() -> None:
    records = [json.loads(path.read_text()) for path in sorted(RESULTS_DIR.glob("*.json"))]
    records.sort(key=lambda record: record["iou"], reverse=True)
    print("| " + " | ".join(header for header, _ in COLUMNS) + " |")
    print("|" + "|".join("---" for _ in COLUMNS) + "|")
    for record in records:
        cells = []
        for _, key in COLUMNS:
            value = record[key]
            cells.append(f"{value:.4f}" if isinstance(value, float) else str(value))
        print("| " + " | ".join(cells) + " |")
    print(f"\nlidar reference voxels: {records[0]['lidar_voxels']}")


if __name__ == "__main__":
    main()
