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

"""Data model, JSON serialization, and terminal rendering for eval results."""

from __future__ import annotations

from dataclasses import asdict, dataclass, field
import json

from dimos.eval.hardware import HardwareProfile
from dimos.utils.human import human_bytes


@dataclass
class WorkerFootprint:
    """Aggregated resource footprint of a single worker process over the run."""

    worker_id: int
    modules: list[str]
    dedicated: bool
    cpu_percent_mean: float
    cpu_percent_peak: float
    pss_mean_bytes: int
    pss_peak_bytes: int
    num_threads: int
    num_children: int
    num_fds: int
    io_read_bytes: int
    io_write_bytes: int
    samples: int


@dataclass
class GpuFootprint:
    """Aggregated (system-wide) GPU utilization over the run."""

    index: int
    name: str
    util_percent_mean: float
    util_percent_peak: float
    mem_used_mean_bytes: int
    mem_used_peak_bytes: int
    mem_total_bytes: int
    samples: int


@dataclass
class EvalTotals:
    """Roll-up across every DimOS process (coordinator + workers)."""

    cpu_percent_mean: float
    pss_mean_bytes: int
    pss_peak_bytes: int
    ram_used_pct_of_host: float


@dataclass
class EvalResult:
    """The full result of evaluating one blueprint on one machine."""

    blueprint: str
    run_mode: str  # "replay" | "simulation" | "real"
    status: str  # "ok" | "failed"
    timestamp: str
    hardware: HardwareProfile
    startup_seconds: float = 0.0
    warmup_seconds: float = 0.0
    duration_seconds: float = 0.0
    sample_count: int = 0
    coordinator: WorkerFootprint | None = None
    workers: list[WorkerFootprint] = field(default_factory=list)
    gpus: list[GpuFootprint] = field(default_factory=list)
    totals: EvalTotals | None = None
    error: str = ""

    def to_dict(self) -> dict[str, object]:
        return asdict(self)

    def to_json(self, indent: int = 2) -> str:
        return json.dumps(self.to_dict(), indent=indent)


def render_report(result: EvalResult) -> None:
    """Pretty-print an EvalResult to the terminal via rich."""
    from rich.console import Console
    from rich.table import Table

    console = Console()
    hw = result.hardware

    console.print()
    console.rule(f"[bold]DimOS Eval — {result.blueprint} [{result.run_mode}]")
    gpu_desc = ", ".join(f"{g.name} ({human_bytes(g.memory_total_bytes)})" for g in hw.gpus)
    console.print(
        f"[bold]Hardware[/bold]: {hw.tier}  |  {hw.cpu_model}  "
        f"({hw.cpu_cores_physical}C/{hw.cpu_cores_logical}T, {hw.arch})"
    )
    console.print(
        f"          RAM {human_bytes(hw.ram_total_bytes)}  |  "
        f"GPU: {gpu_desc or 'none'}"
        + (f"  |  {hw.jetson_model} ({hw.jetpack})" if hw.is_jetson else "")
    )

    if result.status != "ok":
        console.print(f"\n[bold red]STATUS: FAILED[/bold red] — {result.error}\n")
        return

    console.print(
        f"          startup {result.startup_seconds:.2f}s  |  "
        f"sampled {result.duration_seconds:.0f}s "
        f"(warmup {result.warmup_seconds:.0f}s, {result.sample_count} samples)"
    )

    table = Table(title="Per-worker footprint")
    table.add_column("Worker", justify="right", style="cyan")
    table.add_column("Modules")
    table.add_column("CPU% mean", justify="right", style="green")
    table.add_column("CPU% peak", justify="right")
    table.add_column("Mem mean", justify="right", style="green")
    table.add_column("Mem peak", justify="right")
    table.add_column("Thr", justify="right")

    rows = list(result.workers)
    if result.coordinator is not None:
        rows = [result.coordinator, *rows]
    for w in rows:
        label = "coord" if w.worker_id < 0 else str(w.worker_id)
        modules = ", ".join(w.modules) if w.modules else "[dim]—[/dim]"
        table.add_row(
            label,
            modules,
            f"{w.cpu_percent_mean:.1f}",
            f"{w.cpu_percent_peak:.1f}",
            human_bytes(w.pss_mean_bytes),
            human_bytes(w.pss_peak_bytes),
            str(w.num_threads),
        )
    console.print()
    console.print(table)

    if result.gpus:
        gtable = Table(title="GPU utilization (system-wide)")
        gtable.add_column("GPU", justify="right", style="cyan")
        gtable.add_column("Name")
        gtable.add_column("Util% mean", justify="right", style="green")
        gtable.add_column("Util% peak", justify="right")
        gtable.add_column("Mem used mean", justify="right")
        gtable.add_column("Mem used peak", justify="right")
        for g in result.gpus:
            gtable.add_row(
                str(g.index),
                g.name,
                f"{g.util_percent_mean:.1f}",
                f"{g.util_percent_peak:.1f}",
                human_bytes(g.mem_used_mean_bytes),
                human_bytes(g.mem_used_peak_bytes),
            )
        console.print()
        console.print(gtable)

    if result.totals is not None:
        t = result.totals
        console.print()
        console.print(
            f"[bold]TOTAL[/bold]: CPU {t.cpu_percent_mean:.1f}%  "
            f"({t.cpu_percent_mean / 100:.2f} cores)  |  "
            f"Mem {human_bytes(t.pss_mean_bytes)} mean / {human_bytes(t.pss_peak_bytes)} peak  "
            f"({t.ram_used_pct_of_host:.1f}% of host RAM)"
        )
    console.print()
