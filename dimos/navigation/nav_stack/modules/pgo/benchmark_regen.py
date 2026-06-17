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

"""Regenerate the PGO KITTI-360 benchmark JSONs used for regression checks.

Runs sequences 2, 4, 8 with ``DEFAULT_PGO_KWARGS`` from
``benchmark_kitti360`` and writes one JSON per sequence to
``modules/pgo/benchmark/kitti360_seq<NN>.json``.

Usage:
    uv run python -m dimos.navigation.nav_stack.modules.pgo.benchmark_regen \\
        --kitti360-root ~/datasets/kitti360
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from dimos.navigation.nav_stack.benchmarks.pose_graph_kitti360.runner import (
    run_benchmark,
)
from dimos.navigation.nav_stack.modules.pgo.benchmark_kitti360 import (
    DEFAULT_PGO_KWARGS,
    DEFAULT_PUBLISH_INTERVAL_SEC,
)
from dimos.navigation.nav_stack.modules.pgo.pgo import PGO

BENCHMARK_SEQUENCES: tuple[int, ...] = (2, 4, 8)
OUTPUT_DIR = Path(__file__).resolve().parent / "benchmark"


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Regenerate PGO KITTI-360 benchmark JSONs (seqs 2, 4, 8)"
    )
    parser.add_argument("--kitti360-root", type=Path, required=True)
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=OUTPUT_DIR,
        help=f"Output directory for the JSON files (default: {OUTPUT_DIR})",
    )
    parser.add_argument(
        "--sequences",
        type=int,
        nargs="+",
        default=list(BENCHMARK_SEQUENCES),
        help=f"Sequences to run (default: {list(BENCHMARK_SEQUENCES)})",
    )
    args = parser.parse_args()

    args.output_dir.mkdir(parents=True, exist_ok=True)
    for sequence_id in args.sequences:
        output_path = args.output_dir / f"kitti360_seq{sequence_id:02d}.json"
        print(f"### sequence {sequence_id} -> {output_path}")
        results = run_benchmark(
            module_under_test=PGO,
            module_kwargs=DEFAULT_PGO_KWARGS,
            kitti360_root=args.kitti360_root,
            sequence_id=sequence_id,
            publish_interval_sec=DEFAULT_PUBLISH_INTERVAL_SEC,
        )
        output_path.write_text(json.dumps(results, indent=2))
        print(json.dumps(results, indent=2))


if __name__ == "__main__":
    main()
