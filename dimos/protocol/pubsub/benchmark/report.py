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

"""Self-contained interactive report generation."""

from __future__ import annotations

import html
import json
from pathlib import Path
from typing import Any

import numpy as np

from dimos.protocol.pubsub.benchmark.artifacts import load_summaries
from dimos.protocol.pubsub.benchmark.metrics import aggregate_summaries, compare_to_ros


def _plotly() -> Any:
    try:
        import plotly  # type: ignore[import-untyped]
    except ImportError as error:
        msg = "Plotly is required for reports. Install the benchmark extra: dimos[benchmark]"
        raise ImportError(msg) from error
    import plotly  # type: ignore[import-untyped]

    return plotly


def _format_number(value: Any) -> str:
    if value is None:
        return "—"
    return f"{float(value):+.1f}"


def _comparison_table(comparisons: list[dict[str, Any]]) -> str:
    rows = []
    for row in comparisons:
        cells = (
            row["candidate_stack"],
            row["cohort"],
            row["workload"],
            f"{row['environment']}:{row['profile']}",
            f"1:{row['subscribers']}",
            _format_number(row.get("latency_ms_p99_delta_pct")),
            _format_number(row.get("goodput_bytes_s_delta_pct")),
            _format_number(row.get("cpu_seconds_delta_pct")),
            _format_number(row.get("peak_rss_bytes_delta_pct")),
            _format_number(row.get("loss_pct_delta_pp")),
        )
        rows.append(
            "<tr>" + "".join(f"<td>{html.escape(str(cell))}</td>" for cell in cells) + "</tr>"
        )
    headings = (
        "Candidate",
        "Cohort",
        "Workload",
        "Network",
        "Fanout",
        "p99 latency Δ%",
        "Goodput Δ%",
        "CPU Δ%",
        "RSS Δ%",
        "Loss Δpp",
    )
    header = "".join(
        f"<th onclick='sortTable({index})'>{html.escape(label)}</th>"
        for index, label in enumerate(headings)
    )
    return f"<table id='comparison'><thead><tr>{header}</tr></thead><tbody>{''.join(rows)}</tbody></table>"


def _executive_table(comparisons: list[dict[str, Any]]) -> str:
    groups: dict[tuple[str, str], list[dict[str, Any]]] = {}
    for row in comparisons:
        groups.setdefault((row["candidate_stack"], row["cohort"]), []).append(row)
    rows = []
    metrics = (
        "latency_ms_p99_delta_pct",
        "goodput_bytes_s_delta_pct",
        "cpu_seconds_delta_pct",
        "loss_pct_delta_pp",
    )
    for (candidate, cohort), values in sorted(groups.items()):
        medians = []
        for metric in metrics:
            samples = [float(row[metric]) for row in values if row.get(metric) is not None]
            medians.append(float(np.median(samples)) if samples else None)
        cells = (candidate, cohort, len(values), *(_format_number(value) for value in medians))
        rows.append(
            "<tr>" + "".join(f"<td>{html.escape(str(cell))}</td>" for cell in cells) + "</tr>"
        )
    return (
        "<table><thead><tr><th>Candidate</th><th>Cohort</th><th>Matched cells</th>"
        "<th>Median p99 latency Δ%</th><th>Median goodput Δ%</th>"
        "<th>Median CPU Δ%</th><th>Median loss Δpp</th></tr></thead><tbody>"
        + "".join(rows)
        + "</tbody></table>"
    )


def generate_report(run_dir: Path) -> Path:
    """Generate a self-contained HTML report from one completed campaign."""
    _plotly()
    from plotly import graph_objects as go  # type: ignore[import-untyped] # heavy optional
    from plotly.offline import plot  # type: ignore[import-untyped] # heavy optional

    manifest = json.loads((run_dir / "manifest.json").read_text())
    summaries = load_summaries(run_dir)
    aggregates_path = run_dir / "aggregates.json"
    if aggregates_path.exists():
        aggregates = json.loads(aggregates_path.read_text())
    else:
        aggregates = aggregate_summaries(summaries, seed=int(manifest.get("seed", 7)))
    comparisons_path = run_dir / "comparisons.json"
    if comparisons_path.exists():
        comparisons = json.loads(comparisons_path.read_text())
    else:
        comparisons = compare_to_ros(summaries, seed=int(manifest.get("seed", 7)))
    stacks = sorted({str(row["stack"]) for row in aggregates})
    figures = []
    for metric, title, y_title in (
        ("latency_ms_p99_delta_pct", "p99 latency relative to ROS 2", "difference from ROS 2 (%)"),
        ("goodput_bytes_s_delta_pct", "Goodput relative to ROS 2", "difference from ROS 2 (%)"),
        ("cpu_seconds_delta_pct", "CPU relative to ROS 2", "difference from ROS 2 (%)"),
        (
            "loss_pct_delta_pp",
            "Loss relative to ROS 2",
            "difference from ROS 2 (percentage points)",
        ),
    ):
        figure = go.Figure()
        for candidate in ("lcm", "zenoh"):
            rows = [
                row
                for row in comparisons
                if row["candidate_stack"] == candidate and row.get(metric) is not None
            ]
            figure.add_trace(
                go.Scatter(
                    name=candidate,
                    mode="markers",
                    x=[
                        f"{row['cohort']} / {row['workload']} / "
                        f"{row['environment']}:{row['profile']} / 1:{row['subscribers']}"
                        for row in rows
                    ],
                    y=[row[metric] for row in rows],
                    error_y={
                        "type": "data",
                        "symmetric": False,
                        "array": [row[f"{metric}_ci_high"] - row[metric] for row in rows],
                        "arrayminus": [row[metric] - row[f"{metric}_ci_low"] for row in rows],
                    },
                )
            )
        figure.add_hline(y=0, line_dash="dash", line_color="black")
        figure.update_layout(title=title, yaxis_title=y_title)
        figures.append(figure)

    for metric, title, y_title in (
        ("latency_ms_p99", "p99 application latency", "milliseconds"),
        ("goodput_bytes_s", "Delivered goodput", "bytes per second"),
        ("loss_pct", "Message loss", "percent"),
        ("cpu_seconds", "Process CPU", "CPU seconds per trial"),
    ):
        figure = go.Figure()
        for stack in stacks:
            rows = [
                row for row in aggregates if row["stack"] == stack and row.get(metric) is not None
            ]
            figure.add_trace(
                go.Scatter(
                    name=stack,
                    mode="markers",
                    x=[
                        f"{row['cohort']} / {row['workload']} / "
                        f"{row['environment']}:{row['profile']} / 1:{row['subscribers']}"
                        for row in rows
                    ],
                    y=[row[metric] for row in rows],
                    error_y={
                        "type": "data",
                        "symmetric": False,
                        "array": [row[f"{metric}_ci_high"] - row[metric] for row in rows],
                        "arrayminus": [row[metric] - row[f"{metric}_ci_low"] for row in rows],
                    },
                    legendgroup=stack,
                )
            )
        figure.update_layout(title=title, yaxis_title=y_title)
        figures.append(figure)

    sections = []
    for index, figure in enumerate(figures):
        sections.append(
            plot(
                figure,
                include_plotlyjs=True if index == 0 else False,
                output_type="div",
                config={"responsive": True},
            )
        )

    report_path = run_dir / "report.html"
    report_path.write_text(
        "<!doctype html><html><head><meta charset='utf-8'>"
        "<title>DimOS transport benchmark</title>"
        "<style>body{font-family:system-ui;margin:2rem;max-width:1400px}"
        "pre{background:#f4f4f4;padding:1rem;overflow:auto}"
        "table{border-collapse:collapse;width:100%;font-size:.85rem}"
        "th,td{border:1px solid #ddd;padding:.4rem;text-align:right}"
        "th{cursor:pointer;background:#f4f4f4;position:sticky;top:0}"
        "th:nth-child(-n+5),td:nth-child(-n+5){text-align:left}"
        "</style></head><body>"
        "<h1>DimOS transport benchmark</h1>"
        "<p>Results compare complete application stacks within matched delivery cohorts. "
        "They do not isolate middleware overhead or establish a universal winner.</p>"
        "<p>Relative metrics use ROS 2 over Zenoh as zero. Negative latency, CPU, RSS, and "
        "loss values favor the candidate; positive goodput favors the candidate.</p>"
        "<h2>Executive comparison</h2><p>Median delta across matched cells; use the "
        "per-cell table for workload-specific decisions.</p>"
        + _executive_table(comparisons)
        + "".join(sections)
        + "<h2>Matched cells relative to ROS 2</h2>"
        + _comparison_table(comparisons)
        + "<h2>Run manifest</h2><pre>"
        + html.escape(json.dumps(manifest, indent=2, sort_keys=True))
        + "</pre><script>function sortTable(n){const t=document.getElementById('comparison');"
        "let rows=Array.from(t.tBodies[0].rows),asc=t.dataset.sort!=n||t.dataset.asc!='1';"
        "rows.sort((a,b)=>{let x=a.cells[n].innerText,y=b.cells[n].innerText;"
        "let nx=parseFloat(x),ny=parseFloat(y);if(!isNaN(nx)&&!isNaN(ny)){return asc?nx-ny:ny-nx;}"
        "return asc?x.localeCompare(y):y.localeCompare(x);});"
        "rows.forEach(r=>t.tBodies[0].appendChild(r));t.dataset.sort=n;t.dataset.asc=asc?'1':'0';}"
        "</script></body></html>"
    )
    return report_path
