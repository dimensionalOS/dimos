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

"""Run every mapper configuration on this machine and print one comparison table.

    uv run python scripts/benchmarks/run_all.py

Configurations, all fed the identical recorded lidar frames:

    Python CUDA:0   VoxelGrid on the GPU (skipped when no CUDA device is present)
    Python CPU:0    VoxelGrid on the CPU
    Rust            the native-module port (CPU only)

Stage timings are only comparable within one machine, so all three run here and
the machine is printed with the table. The output block is markdown, ready to
paste into a PR comment.

Prerequisites are handled automatically: golden frames are dumped if missing and
the Rust bench binary is built if missing.
"""

from __future__ import annotations

import argparse
from pathlib import Path
import subprocess
import sys

from bench_util import STAGES, growth, machine_info, read_stage, resolve_open3d_device, stage_stats

HERE = Path(__file__).resolve().parent
REPO = HERE.parent.parent
RUST_DIR = REPO / "dimos" / "mapping" / "rust"
RUST_BIN = RUST_DIR / "target" / "release" / "bench_mappers"
GOLDEN_FRAMES = RUST_DIR / "tests" / "golden_data" / "frames.bin"


def run(cmd: list[str], cwd: Path | None = None) -> None:
    print(f"\n$ {' '.join(str(c) for c in cmd)}", flush=True)
    subprocess.run(cmd, cwd=cwd, check=True)


def ensure_prerequisites(db: str, device: str) -> None:
    if not GOLDEN_FRAMES.exists():
        print("golden frames missing; generating them first")
        run([sys.executable, str(HERE / "dump_golden.py"), "--db", db, "--device", device])
    if not RUST_BIN.exists():
        print("rust bench binary missing; building it first")
        run(["cargo", "build", "--release", "--bin", "bench_mappers"], cwd=RUST_DIR)


def fmt(value: float, digits: int = 2) -> str:
    return "—" if value != value else f"{value:.{digits}f}"


def build_table(results: list[tuple[str, Path]]) -> str:
    """Markdown table: one column per configuration, one row per metric."""
    labels = [label for label, _ in results]
    samples = {label: {stage: read_stage(csv, stage) for stage in STAGES} for label, csv in results}

    lines = ["| metric | " + " | ".join(labels) + " |"]
    lines.append("|---" * (len(labels) + 1) + "|")

    def row(title: str, values: list[str]) -> None:
        lines.append(f"| {title} | " + " | ".join(values) + " |")

    for stage, title in (("add_ms", "add_frame"), ("emit_ms", "emit"), ("cost_ms", "costmap")):
        for metric in ("mean", "p95"):
            cells = []
            for label in labels:
                values = samples[label][stage]
                # The first costmap call pays numba's JIT compile in Python; it is
                # reported on its own row rather than skewing the steady state.
                if stage == "cost_ms":
                    values = values[1:]
                stats = stage_stats(values)
                cells.append(fmt(stats.get(metric, float("nan"))))
            row(f"{title} {metric} (ms)", cells)

    row(
        "costmap first call (ms)",
        [
            fmt(samples[label]["cost_ms"][0], 0) if samples[label]["cost_ms"] else "—"
            for label in labels
        ],
    )

    growth_cells = []
    for label in labels:
        first, last = growth(samples[label]["add_ms"])
        growth_cells.append("—" if first != first else f"{fmt(first)} → {fmt(last)}")
    row("add_frame growth, 1st→4th quarter (ms)", growth_cells)

    total_cells = []
    for label in labels:
        total_ms = sum(sum(samples[label][stage]) for stage in STAGES)
        total_cells.append(fmt(total_ms / 1000, 2))
    row("total measured compute (s)", total_cells)

    return "\n".join(lines)


def parse_args() -> argparse.Namespace:
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    ap.add_argument("--db", default="data/go2_short.db", help="memory2 SQLite replay dataset")
    ap.add_argument(
        "--outdir",
        default=str(HERE / "results"),
        help="directory for the per-frame CSVs (default: scripts/benchmarks/results)",
    )
    return ap.parse_args()


def main() -> None:
    args = parse_args()
    outdir = Path(args.outdir)
    outdir.mkdir(parents=True, exist_ok=True)

    _, cuda_available = resolve_open3d_device("CUDA:0")
    ensure_prerequisites(args.db, "CPU:0")

    results: list[tuple[str, Path]] = []

    if cuda_available:
        csv_path = outdir / "python_cuda.csv"
        run(
            [
                sys.executable,
                str(HERE / "baseline_mappers.py"),
                "--device",
                "CUDA:0",
                "--db",
                args.db,
                "--out",
                str(csv_path),
            ]
        )
        results.append(("Python (CUDA:0)", csv_path))

    csv_path = outdir / "python_cpu.csv"
    run(
        [
            sys.executable,
            str(HERE / "baseline_mappers.py"),
            "--device",
            "CPU:0",
            "--db",
            args.db,
            "--out",
            str(csv_path),
        ]
    )
    results.append(("Python (CPU:0)", csv_path))

    rust_csv = outdir / "rust.csv"
    run([str(RUST_BIN), str(rust_csv)])
    results.append(("Rust (CPU)", rust_csv))

    info = machine_info()
    print("\n" + "=" * 72)
    print("paste everything below into the PR")
    print("=" * 72 + "\n")

    print(f"**Machine:** {info['platform']}, {info['processor']}, {info['cpu_count']} cores")
    print(f"**GPU:** {info['gpu'] or 'none detected'}")
    print(f"**Dataset:** `{args.db}`\n")
    if not cuda_available:
        print(
            "> No CUDA device on this machine, so the Python GPU configuration was "
            "skipped — these are CPU-only numbers.\n"
        )
    print(build_table(results))
    print(
        "\nStage timings are only comparable within this table; all configurations "
        "ran on the machine above against the same frames."
    )


if __name__ == "__main__":
    main()
