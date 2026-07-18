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

"""Merge the offline CSVs + e2e JSONs into the final Rust-vs-Python report.

Usage (after baseline_mappers.py, bench_mappers, and both e2e_bench runs):
    uv run python scripts/benchmarks/compare.py
"""

import csv
import json
from pathlib import Path
import statistics

HERE = Path(__file__).parent


def load_stage(path: Path, col: str) -> list[float]:
    with open(path) as f:
        return [float(r[col]) for r in csv.DictReader(f) if r[col]]


def pct(vals, p):
    s = sorted(vals)
    return s[min(len(s) - 1, round(p / 100 * (len(s) - 1)))]


def row(name, py, rs):
    speedup = statistics.mean(py) / statistics.mean(rs)
    return (
        f"| {name} | {statistics.mean(py):.2f} | {pct(py, 95):.2f} "
        f"| {statistics.mean(rs):.2f} | {pct(rs, 95):.2f} | **{speedup:.1f}x** |"
    )


def main() -> None:
    py_csv, rs_csv = HERE / "baseline_python.csv", HERE / "baseline_rust.csv"
    print("## Offline per-frame compute (identical 461 replayed frames)\n")
    print("| stage | Python mean (ms) | p95 | Rust mean (ms) | p95 | speedup |")
    print("|---|---|---|---|---|---|")
    for stage in ("add_ms", "emit_ms", "cost_ms"):
        py = load_stage(py_csv, stage)
        rs = load_stage(rs_csv, stage)
        if stage == "cost_ms":
            py = py[1:]  # drop numba JIT warmup call
        label = {"add_ms": "add_frame", "emit_ms": "emit", "cost_ms": "costmap"}[stage]
        print(row(label, py, rs))

    for name, path in (("Python", "e2e_python.json"), ("Rust", "e2e_rust.json")):
        p = HERE / path
        if not p.exists():
            continue
        d = json.loads(p.read_text())
        print(f"\n## End-to-end ({name}: {d['blueprint']})\n")
        print(f"messages: {d['counts']}")
        for hop in ("lidar_to_map", "map_to_cost"):
            s = d.get(hop) or {}
            if s:
                print(
                    f"{hop:12s}: mean {s['mean_ms']:.1f} ms  p50 {s['p50_ms']:.1f}  "
                    f"p95 {s['p95_ms']:.1f}  max {s['max_ms']:.1f}  (n={s['n']})"
                )
        print(
            f"pipeline CPU: mean {d['cpu_pct']['mean']:.0f}%  p95 {d['cpu_pct']['p95']:.0f}%   "
            f"RSS: mean {d['rss_mb']['mean']:.0f} MB  max {d['rss_mb']['max']:.0f} MB"
        )


if __name__ == "__main__":
    main()
