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

"""Shared helpers for the mapper benchmarks.

One statistics implementation for every configuration (Python CUDA, Python CPU,
Rust), so the numbers in a comparison table can never differ because of how they
were summarized.
"""

from __future__ import annotations

import csv
import os
from pathlib import Path
import platform
import statistics
import subprocess
from typing import Any

# Stage columns present in both the Python and the Rust per-frame CSVs.
STAGES = ("add_ms", "emit_ms", "cost_ms")


def pct(values: list[float], p: float) -> float:
    """Nearest-rank percentile (same convention across all configurations)."""
    if not values:
        return float("nan")
    ordered = sorted(values)
    idx = min(len(ordered) - 1, round(p / 100 * (len(ordered) - 1)))
    return ordered[idx]


def stage_stats(values: list[float]) -> dict[str, float]:
    if not values:
        return {}
    return {
        "n": len(values),
        "mean": statistics.mean(values),
        "p50": pct(values, 50),
        "p95": pct(values, 95),
        "max": max(values),
    }


def read_stage(csv_path: str | Path, column: str) -> list[float]:
    """Per-frame samples for one stage; blank cells (stage not run) are skipped."""
    with open(csv_path) as f:
        return [float(row[column]) for row in csv.DictReader(f) if row.get(column)]


def growth(values: list[float]) -> tuple[float, float]:
    """Mean of the first quarter vs the last quarter — every stage costs more as
    the map grows, which the headline mean hides."""
    if len(values) < 4:
        return (float("nan"), float("nan"))
    q = max(1, len(values) // 4)
    return statistics.mean(values[:q]), statistics.mean(values[-q:])


def gpu_name() -> str | None:
    """First NVIDIA GPU reported by nvidia-smi, or None when there is none."""
    try:
        result = subprocess.run(
            ["nvidia-smi", "--query-gpu=name", "--format=csv,noheader"],
            capture_output=True,
            text=True,
            timeout=10,
        )
    except (OSError, subprocess.SubprocessError):
        return None
    if result.returncode != 0 or not result.stdout.strip():
        return None
    return result.stdout.strip().splitlines()[0].strip()


def machine_info() -> dict[str, Any]:
    """Identifies the machine a run came from. Recorded with every result: stage
    timings are only comparable within a single machine."""
    return {
        "platform": platform.platform(),
        "processor": platform.processor() or platform.machine(),
        "cpu_count": os.cpu_count(),
        "gpu": gpu_name(),
    }


def resolve_open3d_device(requested: str) -> tuple[str, bool]:
    """Return (resolved_device, cuda_available) for an Open3D device string.

    Mirrors ``VoxelGrid.__init__``: a CUDA request silently falls back to CPU
    when no CUDA device is present. Reported so a fallback run can never be
    mistaken for a GPU measurement.
    """
    import open3d.core as o3c  # type: ignore[import-untyped]

    cuda_available = bool(o3c.cuda.is_available())
    resolved = requested if (requested.startswith("CUDA") and cuda_available) else "CPU:0"
    return resolved, cuda_available
