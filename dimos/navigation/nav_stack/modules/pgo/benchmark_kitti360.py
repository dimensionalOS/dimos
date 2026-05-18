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


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Run the generic pose-graph KITTI-360 benchmark against PGO"
    )
    parser.add_argument("--kitti360-root", type=Path, required=True)
    parser.add_argument("--sequence", type=int, default=2)
    parser.add_argument("--max-scans", type=int, default=None)
    parser.add_argument("--scan-context-match-threshold", type=float, default=0.4)
    parser.add_argument("--loop-score-thresh", type=float, default=0.5)
    parser.add_argument("--loop-search-radius-m", type=float, default=1.0)
    parser.add_argument("--key-pose-delta-trans", type=float, default=0.5)
    parser.add_argument("--publish-interval-sec", type=float, default=0.02)
    parser.add_argument("--output-json", type=Path, default=None)
    args = parser.parse_args()

    results = run_benchmark(
        module_under_test=PGO,
        module_kwargs={
            "scan_context_match_threshold": args.scan_context_match_threshold,
            "loop_score_thresh": args.loop_score_thresh,
            "loop_search_radius": args.loop_search_radius_m,
            "key_pose_delta_trans": args.key_pose_delta_trans,
        },
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
