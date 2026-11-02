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
import statistics
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
    "pim_full": "Pim · full links",
    "compact_full": "Compact · full roots/joints",
    "jeff_full": "Jeff · always send",
    "jeff_delta": "Jeff · deltas",
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
    axes[0].set_title("Local CPU path · lower is better", loc="left", weight="bold")
    axes[1].set_title("Payload · lower is better", loc="left", weight="bold")
    handles, labels = axes[0].get_legend_handles_labels()
    fig.legend(handles, labels, loc="outside lower center", ncol=2, frameon=False)
    return svg(fig)


def stages_chart(rows: list[dict[str, Any]], props: int) -> str:
    fig, ax = plt.subplots(figsize=(10, 3.0), layout="constrained")
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
    ax.legend(frameon=False, ncol=3, loc="lower right")
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
    axes[1].set_ylabel("State distribution share of the measured cycle (%)")
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
    crossover = (
        (compact_bytes - delta_bytes) / ((delta_us - compact_us) / 1e6) * 8 / 1e6
        if delta_us > compact_us
        else float("inf")
    )
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
    args.output.with_suffix(".zip").write_bytes(bundle_bytes)
    prov = primary["provenance"]
    hypotheses = [
        (
            "H1",
            "Compact state reduces work",
            f"At {maximum:,} props, compact full samples take {compact_us / 1000:.2f} ms. The articulation sweep separates fewer link transforms from prop-count effects.",
        ),
        (
            "H2",
            "Deltas reduce bytes",
            f"At 1% moving props, payload falls {reduction:.1f}% versus compact full samples. Initial declarations are excluded here and retained in the data.",
        ),
        (
            "H3",
            "Fewer bytes does not guarantee lower latency",
            f"Jeff's delta CPU path takes {delta_us / compact_us:.1f}&times; the compact path and {delta_us / full_us:.1f}&times; his always-send path in this condition. Use the delivery table to include real transport.",
        ),
        (
            "H4",
            "Recovery is a separate requirement",
            f"{tests - skips} checks pass; {skips} explicitly expected failures demonstrate late join, missed final movement and accepting older frames in the pinned mirror.",
        ),
    ]
    cards = "".join(
        f'<article class="hyp"><div class="kicker">{h}</div><h3>{title}</h3><p>{text}</p></article>'
        for h, title, text in hypotheses
    )
    plots = "".join(
        f'<div class="activity" data-activity="{fraction}" {"hidden" if fraction != 0.01 else ""}>{replay_chart(replay, fraction)}</div>'
        for fraction in sorted({r["fraction"] for r in replay})
    )
    failures = "".join(
        f'<tr><td>{LABELS[r["kind"]]}</td><td>{r["case"].replace("_", " ")}</td><td class="{"pass" if r["pass"] else "fail"}">{"Recovered" if r["pass"] else "Failed"}</td><td>{html.escape(str(r.get("detail", r.get("error", ""))))}</td></tr>'
        for r in primary["failures"]
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
    # Numeric facts are generated from data; interpretation is explicitly bounded.
    document = f"""<!doctype html><html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width, initial-scale=1"><title>World state under load · sim2 experiments</title>
<style>
:root{{--ink:#182b3c;--muted:#5d6e7e;--teal:#087f70;--line:#dfe7ec;--paper:#f4f7f9}}*{{box-sizing:border-box}}body{{margin:0;background:var(--paper);color:var(--ink);font:15px/1.6 system-ui,-apple-system,sans-serif}}a{{color:#17688a}}main{{max-width:1190px;margin:auto;padding:52px 32px 70px}}header{{padding:20px 0 34px;border-bottom:1px solid var(--line)}}.kicker{{font-size:11px;letter-spacing:.13em;text-transform:uppercase;color:var(--teal);font-weight:750}}h1{{font-size:clamp(30px,4.7vw,54px);line-height:1.12;letter-spacing:-.045em;margin:12px 0 20px;max-width:850px}}h2{{font-size:25px;letter-spacing:-.025em;margin:8px 0 15px}}h3{{font-size:16px;line-height:1.35;margin:6px 0}}p{{margin:10px 0}}.lead{{font-size:19px;line-height:1.55;max-width:930px}}.muted,.caption{{color:var(--muted);font-size:13px}}.meta{{display:flex;gap:12px;flex-wrap:wrap;margin:22px 0}}.meta span{{padding:5px 10px;border:1px solid var(--line);border-radius:20px;background:white;font-size:12px}}nav{{position:sticky;top:0;z-index:2;display:flex;gap:6px;overflow-x:auto;background:#f4f7f9ee;backdrop-filter:blur(12px);padding:12px 0}}nav a{{white-space:nowrap;text-decoration:none;color:var(--ink);font-size:13px;padding:7px 12px;border-radius:7px}}nav a:hover{{background:#e4ecef}}section{{margin:26px 0;padding:30px;background:white;border:1px solid var(--line);border-radius:14px;scroll-margin-top:70px}}.grid{{display:grid;grid-template-columns:repeat(2,1fr);gap:16px}}.hyp{{padding:20px;background:#f5f8fa;border:1px solid var(--line);border-radius:10px}}.hyp p{{font-size:14px;color:#425769}}.callout{{border-left:4px solid var(--teal);background:#edf6f3;padding:16px 20px;margin:22px 0}}.warning{{border-left-color:#b27818;background:#fff8e9}}.flow{{display:grid;grid-template-columns:repeat(4,1fr);gap:10px;margin:20px 0}}.flow div{{background:#f1f5f8;border:1px solid var(--line);border-radius:8px;padding:15px;font-size:13px}}.flow b{{display:block;margin-bottom:4px}}svg{{width:100%;height:auto;display:block}}.plot{{margin:18px 0}}button,select{{font:inherit;background:white;border:1px solid #bdcbd4;border-radius:7px;padding:7px 12px;color:var(--ink)}}button{{cursor:pointer}}button:hover{{background:#edf6f3}}.filters{{display:flex;flex-wrap:wrap;gap:12px;align-items:center;margin:20px 0}}label{{font-size:13px}}.tablewrap{{overflow:auto}}table{{border-collapse:collapse;width:100%;font-size:13px;margin:16px 0}}th{{text-align:left;font-size:11px;text-transform:uppercase;letter-spacing:.035em;background:#eef3f6;color:#506373;white-space:nowrap}}td,th{{padding:10px 12px;border-bottom:1px solid var(--line)}}td{{vertical-align:top}}.pass{{color:#087f70;font-weight:700}}.fail{{color:#ac4545;font-weight:700}}code{{font:12px/1.5 ui-monospace,SFMono-Regular,monospace;background:#eef3f6;border-radius:4px;padding:2px 5px}}pre{{font:12px/1.5 ui-monospace,SFMono-Regular,monospace;overflow:auto;background:#122a3a;color:#e0edf4;border-radius:8px;padding:18px;white-space:pre}}details{{border-top:1px solid var(--line);padding:12px 0}}summary{{cursor:pointer;font-weight:650;font-size:14px}}.formula{{font:14px/1.8 ui-monospace,SFMono-Regular,monospace;background:#f4f7f9;padding:16px;border-radius:8px}}.small{{font-size:12px}}footer{{color:var(--muted);font-size:12px;margin:26px 0}}[hidden]{{display:none!important}}@media(max-width:760px){{main{{padding:22px 12px}}section{{padding:18px}}.grid,.flow{{grid-template-columns:1fr}}nav{{position:static}}}}@media print{{body{{background:white}}main{{max-width:none;padding:0}}nav,button,.filters{{display:none}}section{{break-inside:avoid;border-radius:0}}a{{color:inherit}}}}
</style></head><body><main><header><div class="kicker">DimOS / simulation research / {datetime.now(timezone.utc).date()}</div><h1>World state under load</h1><p class="lead">Deltas save bandwidth. Do they make the simulation faster?</p><p class="muted">Measured comparison of Pim's minimal sim2 state path, Jeff's WorldChange implementation, and two controlled baselines. Increasing scene size, activity, contact load and process fan-out.</p><div class="meta"><span>{prov["cpu"]}</span><span>MuJoCo {prov["versions"]["mujoco"]}</span><span>{maximum:,} free props</span><span>{totals:,} recorded trials</span><span>{tests - skips} passing checks · {skips} expected failures</span></div></header>
<nav><a href="#answer">Findings</a><a href="#state">State scaling</a><a href="#physics">Physics</a><a href="#delivery">Delivery</a><a href="#correctness">Correctness</a><a href="#theory">Why</a><a href="#methods">Reproduce</a></nav>
<section id="answer"><div class="kicker">Measured, not assumed</div><h2>Two optimizations, different results</h2><div class="grid">{cards}</div><div class="callout"><strong>Decision supported by these experiments:</strong> keep the readable state contract independent of its delivery format. Compact full samples are the baseline to beat on this host. Deltas remain useful when the saved communication cost exceeds their comparison and reconstruction cost; they also need a recovery contract.</div><p class="caption">This compares the state path and a controlled common physics loop. It is not a whole-blueprint or robot-policy acceptance result. Articulation and control supplements are included in the downloadable data.</p><div class="flow"><div><b>One native world</b>Same model, same MjData, same sampled values.</div><div><b>Four export paths</b>Current full links · compact full · Jeff always-send · Jeff delta.</div><div><b>Actual codecs</b>DimOS CBOR or Jeff's generated LCM messages.</div><div><b>Same usable output</b>Root poses, root velocities and a dense joint-state matrix.</div></div></section>
<section id="state"><div class="kicker">Experiment 1 · representation &times; activity</div><h2>Bandwidth and CPU do not track each other</h2><p>One moving 29-joint chain, four fixed frames, and more free props. The activity control changes only the fraction of props moving; the robot always moves.</p><div class="filters"><label for="activity">Moving props</label><select id="activity"><option value="0">0% · resting</option><option value="0.01" selected>1% · mostly resting</option><option value="0.1">10%</option><option value="1">100% · all moving</option></select></div>{plots}<p class="caption">Logarithmic vertical axes. Each point is the median of five trial medians; shaded areas span the trial minimum and maximum. Activity comes from deterministic pose assignment and kinematic refresh, not a claim about physically integrated motion. Initial announcement bytes are separate.</p><h3>Where the time goes at {maximum:,} props</h3><div class="plot">{stages_chart(replay, maximum)}</div><p class="caption">Capture includes extraction and, for Jeff, comparison plus WorldChange construction. Encode finishes the wire message. Consume includes decoding, mirror application where applicable, and dense projection. Traces and profiles were collected separately from these timings.</p><h3>More articulated robots</h3>{articulation_chart(articulation)}<p class="caption">Synthetic 29-joint chains; no G1 policy inference, trained behavior or robot-specific connection code. This axis increases links/joints independently of free props.</p><h3>Control: representation versus a costly NumPy call</h3><p>The compact baseline changes two things: fewer link rows and replacing per-body <code>np.roll</code> with two slices. This extra 2 &times; 2 experiment holds the dict schema, codec and consumer fixed and varies each choice independently.</p>{controls_table}<p class="caption">Milliseconds per complete local state path; median of five trial medians. These are four experimental captures, not additional shipped modes. A large scene dominated by free props has few derived link rows to remove; the many-chain case reveals the representation saving. We cannot attribute the primary compact baseline's whole speedup to fewer fields.</p></section>
<section id="physics"><div class="kicker">Experiment 2 · actual integrated dynamics</div><h2>Contact load can dominate the loop</h2><p>Every path drives identical joint targets and pushes 10% of the props across the floor. MuJoCo advances at 2 ms, with coherent derived state refreshed after each step. We request one export per ten steps.</p>{physics_chart(physics)}<div class="callout"><strong>Trajectory equality checked:</strong> a separate validation repeat confirms identical SHA-256 hashes over every sampled native qpos/qvel array for all export choices and the physics-only control at all five scene sizes. The main timing sweep also records matching scalar checksums. Maximum delivered-state error: {physical_error:.3g}. Peak contacts in the largest world: {max(r["peak_contacts"] for r in physics):,}.</div><p class="caption">Real-time factor is simulated time divided by measured physics + export/consume kernel time, run as fast as possible. It excludes validation/setup and is not the wall-clock pacing of either shipped module. Both arms use the same integration phase; Jeff's native scheduler does not refresh derived values this way, so these results must not be presented as unmodified full-stack throughput.</p></section>
<section id="delivery"><div class="kicker">Experiment 3 · actual process boundary</div><h2>Measure time until state is usable</h2><p>Each subscriber is a separate spawned process using DimOS's existing Zenoh implementation over loopback TCP. The measured interval starts before capture and ends after decoding and projection. Readers acknowledge a full baseline before timing begins.</p>{delivery_chart(delivery)}<p class="caption">Three independent trials per condition; points show the median achieved rate and median trial p95. Use the table to inspect four-reader fan-out, all-moving props and synthetic bandwidth constraints.</p><div class="filters"><label>Props <select data-filter="props"><option value="all">All</option>{"".join(f"<option>{n}</option>" for n in sorted({r["props"] for r in delivery}))}</select></label><label>Target rate <select data-filter="hz"><option value="all">All</option><option>50</option><option selected>200</option></select></label><label>Readers <select data-filter="readers"><option value="all">All</option><option selected>1</option><option>4</option></select></label><label>Moving props <select data-filter="fraction"><option value="all">All</option><option value="0.01" selected>1%</option><option value="1">100%</option></select></label><label>Condition <select data-filter="cap"><option value="all">All</option><option value="0" selected>Actual local TCP</option>{"".join(f'<option value="{n}">Synthetic {n:g} Mbps</option>' for n in sorted({r["synthetic_bandwidth_mbps"] for r in delivery if r["synthetic_bandwidth_mbps"]}))}</select></label></div>{delivery_table(delivery)}<p class="caption">p95 is the median of each trial's pooled-reader p95; raw per-reader samples are included. Actual rate must be read alongside latency: a producer that cannot meet its target is publishing less often. *CPU cores = aggregate producer + subscriber CPU seconds per publication wall-second; subscriber drain time can extend beyond that window. Missing counts sum across repetitions and readers.</p><div class="callout warning"><strong>Synthetic bandwidth rows are a controlled model, not WAN evidence.</strong> Before each actual Zenoh send, the harness charges payload serialization time at the stated bitrate. This demonstrates the predicted bandwidth-bound regime under a serial store-and-forward assumption. It does not emulate network RTT, jitter, packet loss, pipelining or a production Desktop connection.</div></section>
<section id="correctness"><div class="kicker">Experiment 4 · fault injection</div><h2>A smaller stream must still produce a correct world</h2><div class="tablewrap"><table><thead><tr><th>Path</th><th>Condition</th><th>Result</th><th>Observed detail</th></tr></thead><tbody>{failures}</tbody></table></div><p>These are deliberate message omissions and reordering using the actual writer, generated encoder and mirror. They do not assume that the local network randomly loses data. The failure tests are strict expected failures for the pinned implementation; they are not passes.</p><p class="caption">Complete samples recover current state after a missing sample. They do not automatically reject old packets, recover historical events, or restore a native solver checkpoint. Maximum no-fault replay error across all four paths: {replay_error:.3g} (Jeff's suppression tolerance is 1e-6).</p></section>
<section id="theory"><div class="kicker">Computer science connection</div><h2>The measurements follow the work performed</h2><div class="grid"><article><h3>1. Less output does not imply less scanning</h3><p>Let N be roots, J joint values and k changed groups. This delta implementation still visits roots/joints before deciding what changed. Its capture work remains proportional to N + J. Encoding can shrink toward k. A consumer requesting a dense world still traverses its reconstructed state.</p><div class="formula">full: extract(N + J) + encode(N + J)<br>delta: extract(N + J) + compare(N + J) + encode(k)</div></article><article><h3>2. The bandwidth crossover is conditional</h3><p>A serial model balances additional CPU time against transfer time saved. At {maximum:,} props and 1% activity, the measured local paths imply a crossover near <strong>{crossover:.1f} Mbps</strong>. This is an estimate from these measurements, not a measured WAN threshold.</p><div class="formula">delta helps when:<br>extra CPU seconds &lt; saved bytes / bytes per second</div></article><article><h3>3. Amdahl limits whole-loop speedup</h3><p>If state distribution occupies fraction f of a loop, accelerating that portion by s gives the bound below. Physics, rendering and policies do not disappear when messages get smaller. The physical sweep measures f for this controlled workload.</p><div class="formula">speedup = 1 / ((1 &minus; f) + f / s)</div><p class="small"><a href="https://docs.nvidia.com/cuda/cuda-c-best-practices-guide/index.html#strong-scaling-and-amdahl-s-law">Amdahl's law · NVIDIA guidance</a></p></article><article><h3>4. CPU representation matters</h3><p>Many per-entity Python operations can cost more than copying one dense native array. NumPy advanced indexing, endian conversion and bytes encoding can all allocate. A dirty-field protocol is not automatically a zero-copy protocol.</p><p class="small"><a href="https://numpy.org/doc/stable/user/basics.copies.html">NumPy copies and views</a> · <a href="https://docs.python.org/3/library/tracemalloc.html">What tracemalloc measures</a></p></article></div><h3>What the profiles found</h3><p>In the separate 40-frame profile at 2,048 props, Jeff's path called <code>allclose</code> 163,123 times; that comparison subtree accounted for about 84% of profiled time. Our current snapshot spends most of its profiled time in per-body <code>np.roll</code>. Both are implementation costs, not necessary properties of either state contract. Vectorized comparison or dirty tracking could change the delta result; neither is measured here.</p><h3>Trace the actual calls</h3>{"".join(profiles)}<p class="caption">Profiles explain call costs; they are not the source of the primary timing numbers. Downloaded traces contain capture, encode and consume spans for Perfetto/Chrome tracing. Tracemalloc results are peak live traced memory from separate instrumented runs, not cumulative copied bytes or all native memory traffic.</p><details><summary>Memory: lower peak live allocation is a different result from lower CPU time</summary>{memory_table}<p class="caption">One separately instrumented 20-frame run per path at 2,048 props / 1% activity, including input pose preparation. Tracking begins after initial baseline allocation. These numbers exclude pre-existing caches and untracked native allocations, so they are neither total process memory nor allocation/copy bandwidth.</p></details></section>
<section id="methods"><div class="kicker">Reproducibility and scope</div><h2>Inspect the evidence</h2><p><button id="download">Download raw measurements + traces + harness</button> <span class="muted">ZIP · {len(bundle_bytes) / 1048576:.2f} MiB · embedded in this HTML</span></p><p class="small">Archive SHA-256: <code>{hashlib.sha256(bundle_bytes).hexdigest()}</code></p><div class="tablewrap"><table><tbody><tr><th>Pim source</th><td><a href="https://github.com/dimensionalOS/dimos/blob/{PIM_SHA}/dimos/sim2/mujoco.py#L316">{PIM_SHA[:12]} · snapshot</a></td></tr><tr><th>Jeff source</th><td><a href="https://github.com/dimensionalOS/dimos/blob/{JEFF_SHA}/dimos/sim2/mujoco_backend.py#L390">{JEFF_SHA[:12]} · frame</a> · <a href="https://github.com/dimensionalOS/dimos/blob/{JEFF_SHA}/dimos/sim2/world_change.py#L319">WorldMirror</a></td></tr><tr><th>Message source</th><td><a href="https://github.com/dimensionalOS/dimos-lcm/tree/{prov["world_msgs"]}">{prov["world_msgs"][:12]}</a></td></tr><tr><th>Host</th><td>{html.escape(prov["platform"])} · {prov["memory_bytes"] / 1073741824:.0f} GiB RAM · Python {prov["python"]}</td></tr><tr><th>Libraries</th><td>{html.escape(json.dumps(prov["versions"]))}</td></tr><tr><th>Power at start</th><td>{html.escape(prov["power"])}</td></tr><tr><th>Trial order</th><td>Deterministically shuffled; seed 20261102. Heavy sweeps run serially. Ordinary desktop processes remain running.</td></tr></tbody></table></div><details><summary>Exact run commands</summary><pre>{html.escape(runcommands)}</pre></details><details><summary>Full protocol, normalization and limits</summary><pre>{html.escape((ROOT / "dimos/sim2/experiments/README.md").read_text())}</pre></details><div class="callout warning"><strong>What this establishes:</strong> scaling of the reviewed state extraction, codecs, mirrors and common physics workload on this Mac. <strong>What remains unmeasured:</strong> a complete G1 policy stack, RGB/lidar acquisition load, GPU training, remote WAN behavior, native module scheduler parity and total memory-copy traffic. Synthetic chains expose scaling; they are not representative of every robot/contact scene.</div></section>
<footer>Generated directly from saved measurements · no external JavaScript, fonts or chart services required · {datetime.now(timezone.utc).isoformat()}.</footer></main>
<script id="archive" type="application/octet-stream">{zip64}</script><script>
document.querySelector('#activity').addEventListener('change',e=>document.querySelectorAll('.activity').forEach(x=>x.hidden=Number(x.dataset.activity)!==Number(e.target.value)));
document.querySelectorAll('[data-filter]').forEach(s=>s.addEventListener('change',()=>{{const filters=[...document.querySelectorAll('[data-filter]')];document.querySelectorAll('#delivery-table tbody tr').forEach(row=>row.hidden=filters.some(f=>f.value!=='all'&&Number(row.dataset[f.dataset.filter])!==Number(f.value)));}}));
document.querySelector('[data-filter]').dispatchEvent(new Event('change'));
document.querySelector('#download').addEventListener('click',()=>{{const raw=atob(document.querySelector('#archive').textContent.trim());const bytes=Uint8Array.from(raw,c=>c.charCodeAt(0));const url=URL.createObjectURL(new Blob([bytes],{{type:'application/zip'}}));const a=document.createElement('a');a.href=url;a.download='sim2-state-scaling-evidence.zip';a.click();setTimeout(()=>URL.revokeObjectURL(url),30000);}});
</script></body></html>"""
    args.output.write_text(document)
    print(args.output)


if __name__ == "__main__":
    main()
