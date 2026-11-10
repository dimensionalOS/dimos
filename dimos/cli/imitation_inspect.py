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


"""Human-readable presentation of learning recording and dataset inspections."""

from collections.abc import Sequence
from pathlib import Path
from typing import Any

from rich.console import Console
from rich.table import Table
from rich.text import Text

from dimos.cli import theme


def _table(console: Console, headers: tuple[str, ...], rows: Sequence[tuple[str, ...]]) -> None:
    table = Table(*headers, header_style=theme.ACCENT, box=None, padding=(0, 2))
    for row in rows:
        table.add_row(*(Text(value, overflow="fold") for value in row))
    console.print(table)


def _fields(console: Console, rows: list[tuple[str, str]]) -> None:
    table = Table.grid(padding=(0, 2))
    table.add_column(style="bold")
    table.add_column(overflow="fold")
    for label, value in rows:
        table.add_row(Text(label), Text(value))
    console.print(table)


def _quality_metrics(console: Console, reports: list[dict[str, Any]]) -> None:
    expected = sum(report["expected_frames"] for report in reports)
    emitted = sum(report["emitted_frames"] for report in reports)
    filled = sum(report["filled_frames"] for report in reports)
    alignment = max(report["max_alignment_error_ms"] for report in reports)
    _fields(
        console,
        [
            ("Reported frames", f"{emitted:,} emitted / {expected:,} expected · {filled:,} filled"),
            ("Alignment", f"{alignment:.2f} ms maximum error"),
        ],
    )
    names = sorted(
        {
            name
            for report in reports
            for field in ("source_rates_hz", "max_gaps_ms")
            for name in report[field]
        }
    )
    rows = []
    for name in names:
        rates = [
            report["source_rates_hz"][name]
            for report in reports
            if name in report["source_rates_hz"]
        ]
        gaps = [report["max_gaps_ms"][name] for report in reports if name in report["max_gaps_ms"]]
        rate = "Not available"
        if rates:
            low, high = f"{min(rates):.2f}", f"{max(rates):.2f}"
            rate = f"{low} Hz" if low == high else f"{low}-{high} Hz"
        gap = f"{max(gaps):.2f} ms" if gaps else "Not available"
        rows.append((name, rate, gap))
    if rows:
        console.print()
        _table(console, ("Feature", "Rate", "Largest gap"), rows)


def _recording(console: Console, info: dict[str, Any], verbose: bool) -> None:
    saved = info["saved_episodes"]
    discarded = info["discarded_episodes"]
    incomplete = info["incomplete_episodes"]
    reports = info.get("quality", [])
    passed = sum(report["valid"] for report in reports)
    quality = "Not assessed"
    if reports:
        status = "PASS" if passed == len(reports) else "FAIL"
        quality = f"{status} · {passed:,}/{len(reports):,} assessed saved episodes passed"
    episodes = f"{saved:,} saved · {discarded:,} discarded · {len(incomplete):,} incomplete"
    if not saved and not discarded and not incomplete:
        episodes = "No episodes"
    _fields(console, [("Episodes", episodes), ("Quality", quality)])
    if info["status_stream"] is None:
        console.print("Episode markers: unavailable")
    console.print()
    if info["streams"]:
        _table(
            console,
            ("Stream", "Messages"),
            [(name, f"{count:,}") for name, count in info["streams"].items()],
        )
    else:
        console.print("No recorded streams")
    if reports:
        console.print()
        _quality_metrics(console, reports)
        for report in reports:
            if verbose:
                console.print()
                result = "PASS" if report["valid"] else "FAIL"
                console.print(
                    Text(f"{report['episode_id']} · {result} · {report['mode']}", style="bold")
                )
                _quality_metrics(console, [report])
            if not report["valid"]:
                reasons = "; ".join(report["rejection_reasons"]) or "Quality checks failed"
                console.print(Text(f"{report['episode_id']}: {reasons}", style=theme.WARNING))
    for episode in incomplete:
        task = episode["task_label"] or "Unlabelled task"
        console.print(
            Text(
                f"Incomplete episode: {task} · start timestamp {episode['start_ts']:.2f} s",
                style=theme.WARNING,
            )
        )


def _dataset(console: Console, info: dict[str, Any]) -> None:
    lengths = info["episode_lengths"]
    rows = [
        ("Robot", str(info["robot"])),
        ("Episodes", f"{info['episodes']:,}"),
        ("Frames", f"{info['frames']:,}"),
        ("Rate", f"{info['fps']:.2f} Hz"),
        (
            "Episode lengths",
            f"{lengths['min']:,}-{lengths['max']:,} frames · mean {lengths['mean']:,.2f}",
        ),
        ("Equal episode lengths", "Yes" if lengths["uniform"] else "No"),
        ("Consistent feature shapes", "Yes" if info["shapes_uniform"] else "No"),
        ("Statistics", "Available" if info["has_stats"] else "Not available"),
    ]
    if "version" in info:
        rows.insert(0, ("Version", str(info["version"])))
    _fields(console, rows)
    features = []
    for group in ("observation", "action"):
        for name, feature in info[group].items():
            shape = feature["shape"]
            dimensions = " x ".join(str(size) for size in shape) if shape else "Scalar"
            features.append((group, name, dimensions, str(feature["dtype"])))
    console.print()
    if features:
        _table(console, ("Group", "Feature", "Shape", "Dtype"), features)
    else:
        console.print("No dataset features")


def print_inspection(info: dict[str, Any], *, console: Console, verbose: bool = False) -> None:
    """Print an inspection result without changing its machine-readable contract."""
    kind = info["format"]
    path = Path(info["path"])
    name = path.parent.name if kind == "recording" else path.name
    title = "Recording" if kind == "recording" else f"{kind.upper()} dataset"
    console.print(Text(f"{title} · {name}", style="bold"))
    console.print(Text(str(path), overflow="fold"))
    console.print()
    if kind == "recording":
        _recording(console, info, verbose)
    else:
        _dataset(console, info)
