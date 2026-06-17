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

"""evaluate PGO against KITTI-360

Usage:
    uv run python -m dimos.navigation.nav_stack.modules.pgo.run_kitti360 \\
        --kitti360-root ~/datasets/kitti360 --sequence 2
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from dimos.navigation.nav_stack.benchmarks.pose_graph_kitti360.runner import (
    run_benchmark,
)
from dimos.navigation.nav_stack.modules.pgo.pgo import PGO

# KITTI native lidar rate.
DEFAULT_PUBLISH_INTERVAL_SEC = 0.1

# Tuned PGO config for the KITTI-360 benchmark. See benchmark_regen.py
# for the script that consumes this when re-generating regression JSONs.
DEFAULT_PGO_KWARGS: dict[str, object] = {
    "scan_context_match_threshold": 0.4,
    # ICP fitness on KITTI urban submaps has a 5-50 m² noise floor;
    # the 0.15 default rejects nearly all true loops. Trust scan-context.
    "loop_score_thresh": 10000.0,
    "loop_search_radius": 1.0,
    "loop_candidate_max_distance_m": 10.0,
    "loop_time_thresh": 50.0,
    "min_loop_detect_duration": 0.0,
    "key_pose_delta_trans": 0.5,
    # CMU's 0.1m + half=5 overflows PCL VoxelGrid int32 on KITTI.
    "submap_resolution": 0.5,
    "loop_submap_half_range": 2,
    "global_map_publish_rate": 0.001,
    "drain_stale_scans": False,
}


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Run the generic pose-graph KITTI-360 benchmark against PGO"
    )
    parser.add_argument("--kitti360-root", type=Path, required=True)
    parser.add_argument("--sequence", type=int, default=2)
    parser.add_argument("--max-scans", type=int, default=None)
    parser.add_argument("--publish-interval-sec", type=float, default=DEFAULT_PUBLISH_INTERVAL_SEC)
    parser.add_argument("--output-json", type=Path, default=None)
    args = parser.parse_args()

    results = run_benchmark(
        module_under_test=PGO,
        module_kwargs=DEFAULT_PGO_KWARGS,
        kitti360_root=args.kitti360_root,
        sequence_id=args.sequence,
        max_scans=args.max_scans,
        publish_interval_sec=args.publish_interval_sec,
    )

    print(json.dumps(results, indent=2))
    if args.output_json is not None:
        args.output_json.parent.mkdir(parents=True, exist_ok=True)
        args.output_json.write_text(json.dumps(results, indent=2))


if __name__ == "__main__":
    main()
