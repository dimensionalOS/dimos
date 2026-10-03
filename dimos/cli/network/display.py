# Copyright 2025-2026 Dimensional Inc.
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

from __future__ import annotations

import os
from typing import Any

from rich import box
from rich.columns import Columns
from rich.console import Console, Group
from rich.panel import Panel
from rich.table import Table
from rich.text import Text

from dimos.cli.network.model import Report


def number(value: float | None, suffix: str = "") -> str:
    return "n/a" if value is None else f"{value:.2f}{suffix}"


def curve(report: Report, quantile: str, width: int = 36, fancy: bool = False) -> Text:
    """Terminal plot; R/L labels preserve the direction without color."""
    height = 8
    maximum = (
        max(
            [1.0, report.baseline.get(quantile) or 0]
            + [
                step["rtt"].get(quantile) or 0
                for steps in report.directions.values()
                for step in steps
            ]
        )
        * 1.1
    )
    pixel_width, pixel_height = (width * 2, height * 4) if fancy else (width, height)
    grid: list[list[tuple[str, str]]] = [
        [(" ", "") for _ in range(pixel_width)] for _ in range(pixel_height)
    ]
    for direction, symbol, color in [
        ("remote_to_local", "R", "cyan"),
        ("local_to_remote", "L", "yellow"),
    ]:
        points = [(0.0, report.baseline.get(quantile))] + [
            (step["target_mbps"], step["rtt"].get(quantile))
            for step in report.directions[direction]
        ]
        previous: tuple[int, int] | None = None
        for load, value in points:
            if value is None:
                continue
            x = min(pixel_width - 1, round(load / report.settings.max_mbps * (pixel_width - 1)))
            y = min(pixel_height - 1, max(0, round((1 - value / maximum) * (pixel_height - 1))))
            coordinates = [(x, y)]
            if previous is not None and x > previous[0]:
                px, py = previous
                coordinates = [
                    (dx, round(py + (y - py) * (dx - px) / (x - px))) for dx in range(px, x + 1)
                ]
            for dx, dy in coordinates:
                old = grid[dy][dx][0]
                grid[dy][dx] = ("+", "white") if old == "R" and symbol == "L" else (symbol, color)
            previous = x, y
    text = Text(f"RTT {quantile.replace('_ms', '')} (ms)\n", style="bold")
    lines = grid
    if fancy:
        dots = ((1, 8), (2, 16), (4, 32), (64, 128))
        lines = []
        for cell_y in range(height):
            line = []
            for cell_x in range(width):
                mask = 0
                colors = set()
                for dy in range(4):
                    for dx in range(2):
                        symbol, color = grid[cell_y * 4 + dy][cell_x * 2 + dx]
                        if symbol != " ":
                            mask |= dots[dy][dx]
                            colors.add(color)
                color = next(iter(colors)) if len(colors) == 1 else "white"
                symbol = chr(0x2800 + mask) if mask else " "
                if cell_x == width - 1 and mask:
                    symbol = "R" if color == "cyan" else "L" if color == "yellow" else "+"
                line.append((symbol, color))
            lines.append(line)
    for y, line in enumerate(lines):
        text.append(f"{maximum * (height - 1 - y) / (height - 1):5.1f} | ", style="dim")
        for symbol, color in line:
            text.append(symbol, style=color)
        text.append("\n")
    text.append("      +" + "-" * (width + 1) + "\n", style="dim")
    text.append(f"      0{' ' * (width - 12)}{report.settings.max_mbps:g} Mbps\n", style="dim")
    return text


def sparkline(values: list[float], width: int = 48, ascii_only: bool = False) -> str:
    if not values:
        return "no replies"
    chars = ".:-=+*#@" if ascii_only else "▁▂▃▄▅▆▇█"
    maximum = max(values) or 1
    # Each cell shows the maximum of its time-ordered sample bucket.
    chunk = max(1, (len(values) + width - 1) // width)
    return "".join(
        chars[min(7, int(max(values[i : i + chunk]) / maximum * 7))]
        for i in range(0, len(values), chunk)
    )


def dashboard(report: Report, width: int = 100, message: str = "", fancy: bool = False) -> Group:
    content: list[Any] = []
    content.append(Text("DimOS / NETWORK QUALITY", style="bold cyan"))
    content.append(
        Text(f"{report.remote}  |  Zenoh {report.local_zenoh} / TCP  |  {report.status}")
    )
    content.append(Text(f"{report.remote_dimos}\n{report.endpoint}", style="dim"))
    content.append(
        Text(
            f"Caps: {report.settings.max_mbps:g} Mbps offered, {report.settings.max_seconds:g} s, "
            f"{report.settings.max_bytes / 1024**2:g} MiB / sender\n"
            f"Payload: {report.settings.payload_bytes} B  |  QoS reliable/drop  |  RTT probes express",
            style="dim",
        )
    )
    if message:
        content.append(Text(message, style="cyan"))
    idle = report.baseline
    content.append(
        Text(
            "\nIDLE RTT  "
            + " / ".join(number(idle.get(p), " ms") for p in ("p50_ms", "p95_ms", "p99_ms"))
            + f"  (p50/p95/p99; {idle.get('replies', 0)} replies, {idle.get('timeouts', 0)} timeouts)",
            style="bold",
        )
    )
    bars = Text("\nGOODPUT / highest completed step\n", style="bold cyan")
    bar_width = max(15, min(48, width - 40))
    for direction, label, color in [
        ("remote_to_local", "Remote -> Local", "cyan"),
        ("local_to_remote", "Local -> Remote", "yellow"),
    ]:
        steps = report.directions[direction]
        if not steps:
            bars.append(f"{label:16} pending\n", style="dim")
            continue
        step = steps[-1]
        rx = step["receiver"]["goodput_mbps"]
        offered = step["sender"]["offered_mbps"]
        filled = min(bar_width, round(rx / report.settings.max_mbps * bar_width))
        bars.append(f"{label:16} ")
        bars.append(("█" if fancy else "#") * filled, style=color)
        bars.append("." * (bar_width - filled), style="dim")
        bars.append(f" {rx:.2f} Mbps\n", style=color)
        bars.append(
            f"  actual offered {offered:.2f}; step target {step['target_mbps']:g} Mbps\n",
            style="dim",
        )
        bars.append(
            "  received ramp: "
            + " -> ".join(f"{item['receiver']['goodput_mbps']:.2f}" for item in steps)
            + " Mbps\n",
            style="dim",
        )
    content.append(bars)
    if any(report.directions.values()):
        plots = [
            curve(report, "p95_ms", 36 if width >= 100 else max(8, min(48, width - 12)), fancy),
            curve(report, "p99_ms", 36 if width >= 100 else max(8, min(48, width - 12)), fancy),
        ]
        content.append(
            Columns(plots, padding=(0, 3), equal=True) if width >= 100 else Group(*plots)
        )
        content.append(
            Text(
                "R = remote -> local; L = local -> remote; + = shared cell. Linear interpolation.",
                style="dim",
            )
        )
        table = Table(box=box.SIMPLE, expand=False, padding=(0, 1))
        table.add_column("Highest step")
        table.add_column("Remote -> Local", style="cyan")
        table.add_column("Local -> Remote", style="yellow")
        last = [steps[-1] if steps else None for steps in report.directions.values()]
        table.add_row(
            "RTT p50/p95/p99 ms",
            *[
                "/".join(number(s["rtt"].get(p)) for p in ("p50_ms", "p95_ms", "p99_ms"))
                if s
                else "pending"
                for s in last
            ],
        )
        table.add_row(
            "Probe timeouts / total",
            *[
                f"{s['rtt']['timeouts']} / {s['rtt']['replies'] + s['rtt']['timeouts']}"
                if s
                else "pending"
                for s in last
            ],
        )
        table.add_row(
            "Missing / sent",
            *[
                f"{s['receiver']['missing']} / {s['sender']['messages']} ({s['receiver']['missing_pct']:.2f}%)"
                if s
                else "pending"
                for s in last
            ],
        )
        table.add_row(
            "Max arrival gap ms",
            *[number(s["receiver"]["max_gap_ms"]) if s else "pending" for s in last],
        )
        content.append(table)
        content.append(
            Text(
                "RTT reply sequence / bucket maxima (each row normalized to its own peak)",
                style="dim",
            )
        )
        for direction, label, color in [
            ("remote_to_local", "Remote", "cyan"),
            ("local_to_remote", "Local", "yellow"),
        ]:
            steps = report.directions[direction]
            if steps:
                vals = steps[-1]["rtt"]["samples_ms"]
                content.append(
                    Text(
                        f"{label:7} {sparkline(vals, ascii_only=not fancy)}  peak {number(max(vals) if vals else None)} ms",
                        style=color,
                    )
                )
    if report.status != "running":
        content.append(
            Text("\nSTOP / " + "; ".join(f"{k}: {v}" for k, v in report.stop_reasons.items()))
        )
        content.append(
            Text(f"Requested thresholds: {report.verdict}  |  Cleanup: {report.cleanup}")
        )
        if report.error:
            content.append(Text(report.error, style="bold red"))
        content.append(
            Text(
                "Bounded goodput, not maximum capacity. RTT is round-trip, not one-way.\n"
                "Missing by deadline is not proven packet loss; no cause is inferred.\n"
                "RTT percentiles exclude timeouts; p99 with few replies is coarse.",
                style="dim",
            )
        )
    return Group(*content)


def terminal_console(*, stderr: bool = False) -> Console:
    return Console(stderr=stderr, no_color=bool(os.environ.get("NO_COLOR")))


def print_report(report: Report, console: Console) -> None:
    console.print(
        Panel(
            dashboard(
                report, console.width - 4, fancy=console.is_terminal and not console.no_color
            ),
            border_style="cyan",
            box=box.ASCII if not console.is_terminal else box.ROUNDED,
        )
    )
