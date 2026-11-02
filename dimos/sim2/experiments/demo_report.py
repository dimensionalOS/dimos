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

"""Render a standalone, source-linked HTML report from completed experiment data."""

from __future__ import annotations

import argparse
import base64
from collections import defaultdict
from datetime import datetime, timezone
import hashlib
import html
import io
import json
from pathlib import Path
import re
import statistics
from string import Template
from typing import Any
import xml.etree.ElementTree as ET
import zipfile

import matplotlib
import matplotlib.pyplot as plt

from dimos.sim2.experiments.demo_state_scaling import (
    JEFF_SHA,
    OUTPUT,
    PIM_SHA,
    ROOT,
    STRATEGIES,
    command,
)

matplotlib.use("Agg")
COLORS = {
    "pim_full": "#246b9a",
    "compact_full": "#087f70",
    "jeff_full": "#b27818",
    "jeff_delta": "#8751b0",
}
LABELS = {
    "pim_full": "A · Pim full",
    "compact_full": "B · Compact full",
    "jeff_full": "C · Jeff always-send",
    "jeff_delta": "D · Jeff delta",
}
plt.rcParams.update(
    {
        "font.family": "DejaVu Sans",
        "font.size": 10,
        "axes.spines.top": False,
        "axes.spines.right": False,
        "axes.labelcolor": "#334155",
        "text.color": "#26364a",
        "axes.edgecolor": "#cbd5e1",
        "grid.color": "#e6ebf0",
        "figure.facecolor": "white",
    }
)


def svg(figure: Any) -> str:
    out = io.StringIO()
    figure.savefig(out, format="svg", bbox_inches="tight")
    plt.close(figure)
    return "<svg" + out.getvalue().split("<svg", 1)[1]


def med(rows: list[dict[str, Any]], field: str, metric: str | None = None) -> float:
    return float(
        statistics.median(
            r["metrics"][field]["median"] if metric == "metrics" else r[field] for r in rows
        )
    )


def replay_chart(rows: list[dict[str, Any]], fraction: float) -> str:
    fig, axes = plt.subplots(1, 2, figsize=(11.5, 3.7), layout="constrained")
    for kind in STRATEGIES:
        selected = [r for r in rows if r["kind"] == kind and r["fraction"] == fraction]
        sizes = sorted({r["props"] for r in selected})
        for ax, field, scale in ((axes[0], "total_us", 1 / 1000), (axes[1], "bytes", 1 / 1024)):
            vals = [
                [r["metrics"][field]["median"] * scale for r in selected if r["props"] == n]
                for n in sizes
            ]
            centers = [statistics.median(v) for v in vals]
            ax.plot(
                range(len(sizes)), centers, "o-", label=LABELS[kind], color=COLORS[kind], lw=2, ms=5
            )
            ax.fill_between(
                range(len(sizes)),
                [min(v) for v in vals],
                [max(v) for v in vals],
                alpha=0.12,
                color=COLORS[kind],
            )
            ax.set_xticks(range(len(sizes)), [str(n) for n in sizes])
            ax.set_yscale("log")
            ax.grid(axis="y", alpha=0.7)
            ax.set_xlabel("Free props + one moving 29-joint chain")
    axes[0].set_ylabel("Capture + encode + usable state (ms, log scale)")
    axes[1].set_ylabel("Steady-state payload (KiB, log scale)")
    axes[0].set_title("Local processing · lower is better", loc="left", weight="bold")
    axes[1].set_title("Payload · lower is better", loc="left", weight="bold")
    handles, labels = axes[0].get_legend_handles_labels()
    fig.legend(handles, labels, loc="outside lower center", ncol=2, frameon=False)
    return svg(fig)


def stages_chart(rows: list[dict[str, Any]], props: int) -> str:
    fig, ax = plt.subplots(figsize=(10, 3.3), layout="constrained")
    stage_colors = ["#243e5b", "#65a7c5", "#a8d5c8"]
    for i, kind in enumerate(STRATEGIES):
        subset = [
            r for r in rows if r["kind"] == kind and r["props"] == props and r["fraction"] == 0.01
        ]
        offset = 0.0
        for stage, color in zip(
            ("capture_us", "encode_us", "consume_us"), stage_colors, strict=True
        ):
            value = med(subset, stage, "metrics") / 1000
            ax.barh(
                i,
                value,
                left=offset,
                color=color,
                label=stage.replace("_us", "") if i == 0 else None,
            )
            offset += value
        ax.text(offset + 0.15, i, f"{offset:.2f} ms", va="center", fontsize=9)
    ax.set_yticks(range(4), [LABELS[k] for k in STRATEGIES])
    ax.invert_yaxis()
    ax.set_xlabel("Sum of stage medians (ms); 1% of props move")
    ax.set_xlim(right=ax.get_xlim()[1] * 1.12)
    handles, labels = ax.get_legend_handles_labels()
    fig.legend(handles, labels, frameon=False, ncol=3, loc="outside lower center")
    ax.grid(axis="x", alpha=0.5)
    ax.set_axisbelow(True)
    return svg(fig)


def physics_chart(rows: list[dict[str, Any]]) -> str:
    fig, axes = plt.subplots(1, 2, figsize=(11.5, 3.6), layout="constrained")
    for kind in (*STRATEGIES, "physics_only"):
        sizes = sorted({r["props"] for r in rows})
        values = [
            med([r for r in rows if r["kind"] == kind and r["props"] == n], "rtf") for n in sizes
        ]
        axes[0].plot(
            range(len(sizes)),
            values,
            "o-",
            color=COLORS.get(kind, "#192434"),
            label=LABELS.get(kind, "Physics only"),
        )
    axes[0].axhline(1, color="#aeb7c3", ls="--")
    axes[0].set_yscale("log")
    axes[0].set_xticks(range(len(sizes)), [str(n) for n in sizes])
    axes[0].set_xlabel("Free props")
    axes[0].set_ylabel("Offline kernel real-time factor (log scale)")
    axes[0].set_title("Same physics trajectory", loc="left", weight="bold")
    axes[0].grid(axis="y", alpha=0.6)
    for kind in STRATEGIES:
        f = []
        for n in sizes:
            selected = [r for r in rows if r["kind"] == kind and r["props"] == n]
            physics = med(selected, "physics_us", "metrics")
            distribution = med(selected, "distribution_us", "metrics")
            f.append(100 * distribution / (physics + distribution))
        axes[1].plot(range(len(sizes)), f, "o-", color=COLORS[kind])
    axes[1].set_xticks(range(len(sizes)), [str(n) for n in sizes])
    axes[1].set_xlabel("Free props")
    axes[1].set_ylabel("Cycle spent distributing state (%)")
    axes[1].set_title("Amdahl: how much is available to optimize?", loc="left", weight="bold")
    axes[1].grid(axis="y", alpha=0.6)
    h, l = axes[0].get_legend_handles_labels()
    fig.legend(h, l, loc="outside lower center", ncol=3, frameon=False)
    return svg(fig)


def articulation_chart(rows: list[dict[str, Any]]) -> str:
    fig, ax = plt.subplots(figsize=(10, 3.5), layout="constrained")
    sizes = sorted({r["robots"] for r in rows})
    for kind in STRATEGIES:
        vals = [
            med([r for r in rows if r["kind"] == kind and r["robots"] == n], "total_us", "metrics")
            / 1000
            for n in sizes
        ]
        ax.plot(sizes, vals, "o-", label=LABELS[kind], color=COLORS[kind])
    ax.set_xlabel("Moving 29-joint chains (+ 32 props, 1% moving)")
    ax.set_ylabel("Capture + encode + usable state (ms)")
    ax.grid(axis="y", alpha=0.6)
    ax.legend(frameon=False, ncol=2)
    return svg(fig)


def delivery_chart(rows: list[dict[str, Any]]) -> str:
    selected = [
        r
        for r in rows
        if r["hz"] == 200
        and r["readers"] == 1
        and r["fraction"] == 0.01
        and not r["synthetic_bandwidth_mbps"]
    ]
    sizes = sorted({r["props"] for r in selected})
    fig, axes = plt.subplots(1, 2, figsize=(11.5, 3.6), layout="constrained")
    for kind in STRATEGIES:
        groups = [[r for r in selected if r["kind"] == kind and r["props"] == n] for n in sizes]
        axes[0].plot(
            range(len(sizes)),
            [med(g, "achieved_hz") for g in groups],
            "o-",
            color=COLORS[kind],
            label=LABELS[kind],
        )
        axes[1].plot(
            range(len(sizes)),
            [statistics.median(r["latency_us"]["p95"] for r in g) / 1000 for g in groups],
            "o-",
            color=COLORS[kind],
        )
    axes[0].axhline(200, color="#9aa9b7", ls="--")
    axes[0].set_ylim(0, 220)
    axes[0].set_ylabel("Achieved publications / second")
    axes[0].set_title("Requested: 200 Hz", loc="left", weight="bold")
    axes[1].set_yscale("log")
    axes[1].set_ylabel("p95 capture-to-usable-state (ms, log scale)")
    axes[1].set_title("One real subscriber process", loc="left", weight="bold")
    for ax in axes:
        ax.set_xticks(range(len(sizes)), [str(n) for n in sizes])
        ax.set_xlabel("Free props · 1% moving · local TCP")
        ax.grid(axis="y", alpha=0.6)
    h, l = axes[0].get_legend_handles_labels()
    fig.legend(h, l, loc="outside lower center", ncol=2, frameon=False)
    return svg(fig)


def delivery_table(rows: list[dict[str, Any]]) -> str:
    groups: dict[tuple[Any, ...], list[dict[str, Any]]] = defaultdict(list)
    for row in rows:
        groups[
            (
                row["synthetic_bandwidth_mbps"],
                row["props"],
                row["fraction"],
                row["hz"],
                row["readers"],
                row["kind"],
            )
        ].append(row)
    out = []
    for (cap, props, fraction, hz, readers, kind), items in sorted(groups.items()):
        latency = statistics.median(r["latency_us"]["p95"] for r in items) / 1000
        achieved = med(items, "achieved_hz")
        missing = sum(c["missing"] for r in items for c in r["consumers"])
        cpu = statistics.median(
            (r["producer_cpu_seconds"] + r["consumer_cpu_seconds"]) / r["publication_seconds"]
            for r in items
        )
        out.append(
            f'<tr data-props="{props}" data-hz="{hz}" data-readers="{readers}" data-cap="{cap}" data-fraction="{fraction}"><td>{LABELS[kind]}</td><td>{props:,}</td><td>{100 * fraction:g}%</td><td>{readers}</td><td>{hz}</td><td>{achieved:.1f}</td><td>{latency:.2f}</td><td>{cpu:.2f}</td><td>{missing}</td><td>{"Local TCP" if not cap else f"Synthetic {cap:g} Mbps"}</td></tr>'
        )
    return (
        '<div class="tablewrap"><table id="delivery-table"><thead><tr><th>Path</th><th>Props</th><th>Moving</th><th>Readers</th><th>Target Hz</th><th>Actual Hz</th><th>p95 ready (ms)</th><th>CPU cores*</th><th>Missing</th><th>Link condition</th></tr></thead><tbody>'
        + "".join(out)
        + "</tbody></table></div>"
    )


def result_table(headers: list[str], rows: list[list[str]]) -> str:
    headings = "".join(f"<th>{html.escape(cell)}</th>" for cell in headers)
    body = "".join(
        "<tr>" + "".join(f"<td>{html.escape(cell)}</td>" for cell in row) + "</tr>" for row in rows
    )
    return f'<div class="tablewrap"><table class="numeric"><thead><tr>{headings}</tr></thead><tbody>{body}</tbody></table></div>'


def source_excerpt(
    revision: str, path: str, first: int, last: int, highlights: dict[int, str] | None = None
) -> tuple[str, dict[str, Any]]:
    """Read proof excerpts from immutable Git objects, preserving source text and line numbers."""
    source = command("git", "-C", str(ROOT), "show", f"{revision}:{path}")
    lines = source.splitlines()
    if not 1 <= first <= last <= len(lines):
        raise ValueError(f"Invalid source excerpt: {path}:{first}-{last}")
    url = f"https://github.com/dimensionalOS/dimos/blob/{revision}/{path}"
    spans = []
    for number in range(first, last + 1):
        style = (highlights or {}).get(number, "")
        spans.append(
            f'<span class="source-line {style}" data-line="{number}"><a href="{url}#L{number}">{number}</a>'
            f"{html.escape(lines[number - 1])}</span>"
        )
    rendered = (
        f'<div class="source"><div class="source-head"><a href="{url}#L{first}-L{last}">'
        f"{html.escape(path)} · lines {first}&ndash;{last}</a> · {revision[:12]} · verbatim source excerpt</div>"
        "<pre><code>" + "".join(spans) + "</code></pre></div>"
    )
    return rendered, {
        "revision": revision,
        "path": path,
        "first_line": first,
        "last_line": last,
        "text": "\n".join(lines[first - 1 : last]),
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("runs", nargs="+")
    parser.add_argument("--output", type=Path, default=OUTPUT / "report.html")
    args = parser.parse_args()
    paths = [OUTPUT / run / "results.json" for run in args.runs]
    runs = [json.loads(path.read_text()) for path in paths]
    primary = next(r for r in runs if len(r.get("replay", [])) and len(r["protocol"]["props"]) > 1)
    replay = primary["replay"]
    physics = primary["physics"]
    delivery = [row for r in runs for row in r.get("delivery", [])]
    articulation = [
        row
        for r in runs
        if r["protocol"]["name"] != "validation"
        for row in r.get("replay", [])
        if row["props"] == 32 and row["fraction"] == 0.01
    ]
    maximum = max(r["props"] for r in replay)
    bare_rtf = med(
        [r for r in physics if r["props"] == maximum and r["kind"] == "physics_only"], "rtf"
    )
    compact_rtf = med(
        [r for r in physics if r["props"] == maximum and r["kind"] == "compact_full"], "rtf"
    )

    def subset(kind: str) -> list[dict[str, Any]]:
        return [
            r
            for r in replay
            if r["props"] == maximum and r["fraction"] == 0.01 and r["kind"] == kind
        ]

    compact_us = med(subset("compact_full"), "total_us", "metrics")
    delta_us = med(subset("jeff_delta"), "total_us", "metrics")
    full_us = med(subset("jeff_full"), "total_us", "metrics")
    compact_bytes = med(subset("compact_full"), "bytes", "metrics")
    delta_bytes = med(subset("jeff_delta"), "bytes", "metrics")
    reduction = 100 * (1 - delta_bytes / compact_bytes)
    replay_error = max(r["error"] for r in replay)
    physical_error = max(r["state_error"] for r in physics)
    checks = defaultdict(list)
    for r in physics:
        checks[r["props"]].append(r["trajectory_checksum"])
    trajectory_equal = all(max(values) == min(values) for values in checks.values())
    validation = next(r for r in runs if r["protocol"]["name"] == "validation")
    hashes: dict[int, set[str]] = defaultdict(set)
    for row in validation["physics"]:
        hashes[row["props"]].add(row["trajectory_sha256"])
    if set(hashes) != set(checks) or any(len(values) != 1 for values in hashes.values()):
        raise ValueError("Native qpos/qvel trajectory hash validation failed")
    for run in runs:
        if any(row["error"] > 1.00001e-6 for row in run.get("replay", [])):
            raise ValueError("Supplemental state-equivalence check failed")
    if any(
        c["errors"] or c["max_probe_error"] > 1.00001e-6 for r in delivery for c in r["consumers"]
    ):
        raise ValueError("Cross-process decode/probe check failed")
    if not trajectory_equal or replay_error > 1.00001e-6 or physical_error > 1.00001e-6:
        raise ValueError(
            "Refuse to publish a performance report with failed state/trajectory equivalence"
        )
    correctness = ET.parse(OUTPUT / "correctness.xml").getroot()
    suite = correctness.find("testsuite")
    assert suite is not None
    tests = int(suite.attrib["tests"])
    skips = int(suite.attrib["skipped"])
    assert suite.attrib["failures"] == suite.attrib["errors"] == "0"
    assert skips == 3
    totals = sum(
        len(r.get(field, []))
        for r in runs
        for field in ("replay", "physics", "delivery", "representation")
    )
    profiles = []
    for kind in STRATEGIES:
        p = paths[runs.index(primary)].parent / f"{kind}.profile.txt"
        profiles.append(
            f"<details><summary>{LABELS[kind]} · cumulative call profile</summary><pre>{html.escape(p.read_text())}</pre></details>"
        )
    harness_revision = command("git", "-C", str(ROOT), "rev-parse", "df76448032")
    backend_path = "dimos/sim2/mujoco_backend.py"
    world_path = "dimos/sim2/world_change.py"
    harness_path = "dimos/sim2/experiments/demo_state_scaling.py"
    root_highlights = {
        **dict.fromkeys(range(395, 403), "read"),
        **dict.fromkeys(range(403, 405), "compare"),
        **dict.fromkeys(range(405, 407), "send"),
    }
    source_specs = {
        "source_roots": (JEFF_SHA, backend_path, 395, 406, root_highlights),
        "source_close": (JEFF_SHA, backend_path, 468, 469, None),
        "source_pack": (JEFF_SHA, world_path, 85, 90, None),
        "source_joints": (JEFF_SHA, backend_path, 407, 419, None),
        "source_joint_bytes": (JEFF_SHA, world_path, 123, 125, None),
        "source_apply": (JEFF_SHA, world_path, 345, 351, None),
        "source_dense_read": (JEFF_SHA, world_path, 435, 440, None),
        "source_consumer": (harness_revision, harness_path, 402, 407, None),
        "source_ablation": (harness_revision, harness_path, 354, 357, None),
    }
    source_blocks = {}
    source_evidence = {}
    for key, spec in source_specs.items():
        source_blocks[key], source_evidence[key] = source_excerpt(*spec)

    # Means are linear: the stage differences must add to the total difference.
    stage_means = {
        kind: [
            statistics.mean(
                statistics.mean(s[column] for s in run["samples"]) for run in subset(kind)
            )
            / 1000
            for column in (1, 2, 3)
        ]
        for kind in ("jeff_full", "jeff_delta")
    }
    full_stages, delta_stages = stage_means["jeff_full"], stage_means["jeff_delta"]
    capture_extra = delta_stages[0] - full_stages[0]
    later_saved = sum(full_stages[1:]) - sum(delta_stages[1:])
    net_extra = sum(delta_stages) - sum(full_stages)
    if abs(capture_extra - later_saved - net_extra) > 1e-10:
        raise ValueError("Stage differences do not sum to the total difference")
    cost_table = result_table(
        ["Jeff path", "Capture / build", "Outer encoding", "Receiver", "Total"],
        [
            [LABELS[kind], *[f"{v:.2f}" for v in values], f"{sum(values):.2f}"]
            for kind, values in stage_means.items()
        ],
    )
    call_counts = {}
    for kind in ("jeff_full", "jeff_delta"):
        profile_text = (paths[runs.index(primary)].parent / f"{kind}.profile.txt").read_text()
        match = re.search(
            r"^\s+(\d+)\s+.*mujoco\._functions\.mj_objectVelocity", profile_text, re.MULTILINE
        )
        if match is None:
            raise ValueError(f"Missing velocity-query profile evidence for {kind}")
        call_counts[kind] = int(match[1])
    delta_profile = (paths[runs.index(primary)].parent / "jeff_delta.profile.txt").read_text()
    close_match = re.search(
        r"^\s+(\d+)\s+\S+\s+\S+\s+(\S+)\s+\S+\s+.*\(_close\)", delta_profile, re.MULTILINE
    )
    elapsed_match = re.search(r"function calls in ([\d.]+) seconds", delta_profile)
    if close_match is None or elapsed_match is None:
        raise ValueError("Missing comparison profile evidence")
    close_percent = 100 * float(close_match[2]) / float(elapsed_match[1])
    calls_table = result_table(
        ["Profiled operation", "Jeff always-send", "Jeff delta"],
        [
            [
                "Native body-velocity query",
                f"{call_counts['jeff_full']:,}",
                f"{call_counts['jeff_delta']:,}",
            ],
            [
                "Comparison helper _close",
                "Skipped by the control branch",
                f"{int(close_match[1]):,}",
            ],
        ],
    )
    ledger_evidence = {
        "condition": {
            "props": maximum,
            "moving_fraction": 0.01,
            "trials": 5,
            "frames_per_trial": 16,
        },
        "method": "Mean of per-trial sample means, milliseconds; stages are capture/build, outer encoding, receiver.",
        "stage_means_ms": stage_means,
        "capture_extra_ms": capture_extra,
        "later_saved_ms": later_saved,
        "net_extra_ms": net_extra,
        "velocity_query_counts": call_counts,
        "comparison_calls": int(close_match[1]),
        "comparison_profile_percent": close_percent,
    }
    bundle = io.BytesIO()
    with zipfile.ZipFile(bundle, "w", compression=zipfile.ZIP_DEFLATED) as archive:
        for path in paths:
            archive.write(path, f"{path.parent.name}/results.json")
            for extra in path.parent.glob("*.trace.json"):
                archive.write(extra, f"{path.parent.name}/{extra.name}")
            for extra in path.parent.glob("*.profile.txt"):
                archive.write(extra, f"{path.parent.name}/{extra.name}")
        archive.write(OUTPUT / "correctness.xml", "correctness.xml")
        for source in (ROOT / "dimos/sim2/experiments").glob("*.py"):
            archive.write(source, f"harness/{source.name}")
        archive.write(ROOT / "dimos/sim2/experiments/README.md", "harness/README.md")
        archive.write(
            Path(__file__).with_name("report_template.html"), "harness/report_template.html"
        )
        archive.writestr("source-excerpts.json", json.dumps(source_evidence, indent=2))
        archive.writestr("cost-ledger.json", json.dumps(ledger_evidence, indent=2))
        original = (
            command(
                "git",
                "-C",
                str(ROOT),
                "show",
                "6d989a8476:dimos/sim2/experiments/demo_state_scaling.py",
            )
            + "\n"
        )
        if hashlib.sha256(original.encode()).hexdigest() != primary["provenance"]["harness_sha256"]:
            raise ValueError("Original primary harness does not match its recorded hash")
        archive.writestr("harness/primary-at-6d989a8476.py", original)
        current_scaling = hashlib.sha256(
            (ROOT / "dimos/sim2/experiments/demo_state_scaling.py").read_bytes()
        ).hexdigest()
        for run in runs:
            if run["provenance"]["harness_sha256"] not in (
                primary["provenance"]["harness_sha256"],
                current_scaling,
            ):
                raise ValueError("A run's extraction harness is missing from the archive")
            for filename in (
                ("demo_state_delivery.py",)
                if "delivery" in run
                else ("demo_representation.py",)
                if "representation" in run
                else ()
            ):
                expected = run["provenance"]["harness_files"][filename]
                if (
                    hashlib.sha256(
                        (ROOT / "dimos/sim2/experiments" / filename).read_bytes()
                    ).hexdigest()
                    != expected
                ):
                    raise ValueError(f"Source hash mismatch: {filename}")
        archive.writestr(
            "MANIFEST.txt",
            "Pinned production source SHAs and exact run commands are in each results.json.\n"
            "Primary and articulation extraction runs use primary-at-6d989a8476.py.\n"
            "Later runs use demo_state_scaling.py plus demo_state_delivery.py.\n"
            "Report generation is separate from measured timings.\n",
        )
    bundle_bytes = bundle.getvalue()
    zip64 = base64.b64encode(bundle_bytes).decode()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    (OUTPUT / "report-evidence.zip").write_bytes(bundle_bytes)
    prov = primary["provenance"]
    plots = "".join(
        f'<div class="activity" data-activity="{fraction}" {"hidden" if fraction != 0.01 else ""}>{replay_chart(replay, fraction)}</div>'
        for fraction in sorted({r["fraction"] for r in replay})
    )
    controls = next(r["representation"] for r in runs if "representation" in r)
    control_rows = []
    for robots, props in sorted({(r["robots"], r["props"]) for r in controls}):
        cells = []
        for all_links, roll in [(True, True), (True, False), (False, True), (False, False)]:
            selected = [
                r
                for r in controls
                if r["robots"] == robots
                and r["props"] == props
                and r["all_links"] == all_links
                and r["roll"] == roll
            ]
            if len(selected) != 5 or any(r["error"] > 1e-12 for r in selected):
                raise ValueError("Incomplete or incorrect representation control")
            cells.append(
                f"<td>{statistics.median(r['total_us']['median'] for r in selected) / 1000:.3f}</td>"
            )
        control_rows.append(f"<tr><td>{robots}</td><td>{props:,}</td>{''.join(cells)}</tr>")
    controls_table = (
        '<div class="tablewrap"><table><thead><tr><th>Chains</th><th>Props</th><th>All links + roll</th><th>All links + slices</th><th>Roots + roll</th><th>Roots + slices</th></tr></thead><tbody>'
        + "".join(control_rows)
        + "</tbody></table></div>"
    )
    memory_rows = "".join(
        f"<tr><td>{LABELS[r['kind']]}</td><td>{r['peak_traced_bytes_above_start'] / 1024:.1f}</td><td>{r['retained_bytes'] / 1024:.1f}</td></tr>"
        for r in primary["memory"]
    )
    memory_table = (
        '<div class="tablewrap"><table><thead><tr><th>Path</th><th>Peak above start (KiB)</th><th>Retained (KiB)</th></tr></thead><tbody>'
        + memory_rows
        + "</tbody></table></div>"
    )
    runcommands = "\n".join(
        "python -m dimos.sim2.experiments."
        + (
            "demo_state_delivery"
            if "delivery" in r
            else "demo_representation"
            if "representation" in r
            else "demo_state_scaling"
        )
        + " "
        + " ".join(
            f"--{k.replace('_', '-')} " + (" ".join(map(str, v)) if isinstance(v, list) else str(v))
            for k, v in r["protocol"].items()
            if not isinstance(v, bool)
        )
        + (" --physics" if r["protocol"].get("physics") else "")
        for r in runs
    )
    # Each numerical claim is computed from the saved results, not prose constants.
    primary_table = result_table(
        ["Path", "Local processing (ms)", "Bytes per sample"],
        [
            [
                LABELS[kind],
                f"{med(subset(kind), 'total_us', 'metrics') / 1000:.2f}",
                f"{med(subset(kind), 'bytes', 'metrics'):,.0f}",
            ]
            for kind in STRATEGIES
        ],
    )
    local = [
        r
        for r in delivery
        if r["props"] == maximum
        and r["fraction"] == 0.01
        and r["hz"] == 200
        and r["readers"] == 1
        and not r["synthetic_bandwidth_mbps"]
    ]
    local_delivery_table = result_table(
        ["Path", "Achieved Hz", "p95 usable-state latency (ms)"],
        [
            [
                LABELS[kind],
                f"{med([r for r in local if r['kind'] == kind], 'achieved_hz'):.1f}",
                f"{statistics.median(r['latency_us']['p95'] for r in local if r['kind'] == kind) / 1000:.2f}",
            ]
            for kind in STRATEGIES
        ],
    )
    constrained_rows = []
    for cap in (10, 100):
        for kind in ("compact_full", "jeff_delta"):
            group = [
                r
                for r in delivery
                if r["synthetic_bandwidth_mbps"] == cap and r["props"] == 512 and r["kind"] == kind
            ]
            constrained_rows.append(
                [
                    f"{cap} Mbps",
                    LABELS[kind],
                    f"{med(group, 'achieved_hz'):.1f}",
                    f"{statistics.median(r['latency_us']['p95'] for r in group) / 1000:.1f}",
                ]
            )
    constrained_table = result_table(
        ["Synthetic transfer setting", "Path", "Achieved Hz", "p95 usable-state latency (ms)"],
        constrained_rows,
    )
    delivery_comparison_rows = []
    for props, hz, cap, label in (
        (2048, 200, 0, "Actual local TCP · 2,048 props · target 200 Hz"),
        (512, 50, 10, "Synthetic 10 Mbps delay · 512 props · target 50 Hz"),
    ):
        for kind in ("compact_full", "jeff_delta"):
            group = [
                r
                for r in delivery
                if r["props"] == props
                and r["hz"] == hz
                and r["readers"] == 1
                and r["fraction"] == 0.01
                and r["synthetic_bandwidth_mbps"] == cap
                and r["kind"] == kind
            ]
            delivery_comparison_rows.append(
                [
                    label,
                    "Compact full sample" if kind == "compact_full" else "Jeff delta",
                    f"{med(group, 'achieved_hz'):.1f}",
                    f"{statistics.median(r['latency_us']['p95'] for r in group) / 1000:.2f}",
                ]
            )
    delivery_comparison = result_table(
        ["Condition · 1% moving · one receiver", "Path", "Achieved Hz", "p95 usable state (ms)"],
        delivery_comparison_rows,
    )
    fault_rows = []
    for case, label in [
        ("late_join", "Join after initial declarations"),
        ("missed_final_update", "Omit final movement, then keep reading"),
        ("older_frame_rejected", "Reject an older frame"),
    ]:
        cells = [label]
        for kind in ("pim_full", "compact_full", "jeff_delta"):
            result = next(
                (r for r in primary["failures"] if r["kind"] == kind and r["case"] == case), None
            )
            cells.append("Not tested" if result is None else "Pass" if result["pass"] else "Fail")
        fault_rows.append(cells)
    recovery_table = result_table(
        ["Required behavior", "A · Pim full", "B · Compact full", "D · Jeff delta"], fault_rows
    )

    def control_ms(robots: int, props: int, all_links: bool, roll: bool) -> float:
        return float(
            statistics.median(
                r["total_us"]["median"]
                for r in controls
                if r["robots"] == robots
                and r["props"] == props
                and r["all_links"] == all_links
                and r["roll"] == roll
            )
            / 1000
        )

    jeff_full_bytes = med(subset("jeff_full"), "bytes", "metrics")
    sources_table = f"""<div class="tablewrap"><table><tbody>
<tr><th>Pim implementation</th><td><a href="https://github.com/dimensionalOS/dimos/blob/{PIM_SHA}/dimos/sim2/mujoco.py#L316">{PIM_SHA[:12]} · snapshot()</a></td></tr>
<tr><th>Jeff implementation</th><td><a href="https://github.com/dimensionalOS/dimos/blob/{JEFF_SHA}/dimos/sim2/mujoco_backend.py#L390">{JEFF_SHA[:12]} · frame()</a> · <a href="https://github.com/dimensionalOS/dimos/blob/{JEFF_SHA}/dimos/sim2/world_change.py#L319">WorldMirror</a></td></tr>
<tr><th>Generated message code</th><td><a href="https://github.com/dimensionalOS/dimos-lcm/tree/{prov["world_msgs"]}">{prov["world_msgs"][:12]}</a></td></tr>
<tr><th>Host and runtime</th><td>{html.escape(prov["cpu"])} · {html.escape(prov["platform"])} · {prov["memory_bytes"] / 1073741824:.0f} GiB RAM · Python {html.escape(prov["python"])}</td></tr>
<tr><th>Libraries</th><td>{html.escape(json.dumps(prov["versions"]))}</td></tr>
<tr><th>Power and order</th><td>AC / High Power. Heavy sweeps ran serially. Path order shuffled with seed 20261102. Desktop apps remained active; light editing/report preparation occurred during sweeps.</td></tr>
</tbody></table></div>"""
    values = {
        **source_blocks,
        "jeff_sha": JEFF_SHA,
        "pim_sha": PIM_SHA,
        "cost_table": cost_table,
        "calls_table": calls_table,
        "capture_extra": f"{capture_extra:.2f}",
        "later_saved": f"{later_saved:.2f}",
        "net_extra": f"{net_extra:.2f}",
        "mean_full_ms": f"{sum(full_stages):.2f}",
        "mean_delta_ms": f"{sum(delta_stages):.2f}",
        "close_percent": f"{close_percent:.1f}",
        "delivery_comparison": delivery_comparison,
        "controls_table": controls_table,
        "links_32_ms": f"{control_ms(32, 32, True, False):.3f}",
        "roots_32_ms": f"{control_ms(32, 32, False, False):.3f}",
        "representation_gain": f"{control_ms(32, 32, True, False) / control_ms(32, 32, False, False):.1f}",
        "roll_large_ms": f"{control_ms(1, 2048, True, True):.3f}",
        "slice_large_ms": f"{control_ms(1, 2048, True, False):.3f}",
        "articulation_plot": articulation_chart(articulation),
        "primary_table": primary_table,
        "suppression_reduction": f"{100 * (1 - delta_bytes / jeff_full_bytes):.1f}",
        "jeff_full_bytes": f"{jeff_full_bytes:,.0f}",
        "delta_bytes": f"{delta_bytes:,.0f}",
        "jeff_full_ms": f"{full_us / 1000:.2f}",
        "delta_ms": f"{delta_us / 1000:.2f}",
        "suppression_cost": f"{delta_us / full_us:.1f}",
        "compact_reduction": f"{reduction:.1f}",
        "compact_cost": f"{delta_us / compact_us:.1f}",
        "activity_plots": f'<div class="chart">{plots}</div>',
        "stages_plot": stages_chart(replay, maximum),
        "local_delivery_table": local_delivery_table,
        "compact_local_hz": f"{med([r for r in local if r['kind'] == 'compact_full'], 'achieved_hz'):.1f}",
        "delta_local_hz": f"{med([r for r in local if r['kind'] == 'jeff_delta'], 'achieved_hz'):.1f}",
        "received_samples": f"{sum(c['received'] for r in delivery for c in r['consumers']):,}",
        "missing_samples": f"{sum(c['missing'] for r in delivery for c in r['consumers']):,}",
        "delivery_plot": delivery_chart(delivery),
        "constrained_table": constrained_table,
        "recovery_table": recovery_table,
        "passed_tests": str(tests - skips),
        "physics_plot": physics_chart(physics),
        "bare_rtf": f"{bare_rtf:.3f}",
        "compact_rtf": f"{compact_rtf:.3f}",
        "delta_rtf": f"{med([r for r in physics if r['props'] == maximum and r['kind'] == 'jeff_delta'], 'rtf'):.3f}",
        "max_compact_gain": f"{100 * (bare_rtf / compact_rtf - 1):.0f}",
        "peak_contacts": f"{max(r['peak_contacts'] for r in physics):,}",
        "trial_count": f"{totals:,}",
        "archive_mb": f"{len(bundle_bytes) / 1048576:.2f}",
        "archive_hash": hashlib.sha256(bundle_bytes).hexdigest(),
        "archive_base64": zip64,
        "sources_table": sources_table,
        "prop_options": "".join(
            f"<option>{n}</option>" for n in sorted({r["props"] for r in delivery})
        ),
        "cap_options": "".join(
            f'<option value="{n}">Synthetic {n:g} Mbps</option>'
            for n in sorted(
                {r["synthetic_bandwidth_mbps"] for r in delivery if r["synthetic_bandwidth_mbps"]}
            )
        ),
        "all_delivery_table": delivery_table(delivery),
        "profiles": "".join(profiles),
        "memory_table": memory_table,
        "run_commands": html.escape(runcommands),
        "protocol": html.escape((ROOT / "dimos/sim2/experiments/README.md").read_text()),
        "physical_error": f"{physical_error:.3g}",
        "generated_at": datetime.now(timezone.utc).isoformat(),
    }
    document = Template(Path(__file__).with_name("report_template.html").read_text()).substitute(
        values
    )
    args.output.write_text("\n".join(line.rstrip() for line in document.splitlines()) + "\n")
    print(args.output)


if __name__ == "__main__":
    main()
