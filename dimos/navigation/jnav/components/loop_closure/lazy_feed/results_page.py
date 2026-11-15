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

"""Build the lazy-reform replay results page from lazy-replay run directories.

usage: results_page.py OUT.html LABEL=RUN_DIR [LABEL=RUN_DIR ...] [--status TEXT] [--target-share 0.97]
                       [--headroom 2.5] [--note TEXT]
Each RUN_DIR holds summary.json, timings.json and scans.csv.
"""

from __future__ import annotations

import argparse
import csv
import html
import json
from pathlib import Path

STYLE = """
:root { --ground:#eef1f0; --panel:#fff; --ink:#15201e; --muted:#5b6965; --rule:#d3dbd8; --ok:#1b9165; --bad:#c9481e;
  --target:#c2367a; --strip-off:#e2e8e6; --excluded:#c9cfcd; }
@media (prefers-color-scheme: dark) { :root:not([data-theme="light"]) { color-scheme:dark; --ground:#111816; --panel:#18211f;
  --ink:#e4ebe8; --muted:#93a39e; --rule:#2b3734; --ok:#3ccb94; --bad:#ff7d55; --target:#f06aa8; --strip-off:#26302d; --excluded:#3a4542; } }
:root[data-theme="dark"] { color-scheme:dark; --ground:#111816; --panel:#18211f; --ink:#e4ebe8; --muted:#93a39e; --rule:#2b3734;
  --ok:#3ccb94; --bad:#ff7d55; --target:#f06aa8; --strip-off:#26302d; --excluded:#3a4542; }
* { box-sizing:border-box; }
body { background:var(--ground); color:var(--ink); font:15px/1.5 "IBM Plex Sans", system-ui, sans-serif; padding:24px 16px 48px; }
.wrap { max-width:1100px; margin:0 auto; display:grid; gap:18px; }
h1 { font-size:24px; margin:0; } h2 { font-size:16px; margin:0 0 6px; }
.status { font:13px "IBM Plex Mono", monospace; color:var(--muted); }
.card { background:var(--panel); border:1px solid var(--rule); border-radius:6px; padding:14px 16px; }
.cap { margin:4px 0 0; font-size:13.5px; color:var(--muted); max-width:80ch; }
svg text { fill:var(--ink); font:12px "IBM Plex Mono", monospace; }
.tbl { overflow-x:auto; } table { border-collapse:collapse; width:100%; font-size:13px; font-variant-numeric:tabular-nums; }
th, td { text-align:right; padding:5px 8px; border-bottom:1px solid var(--rule); white-space:nowrap; }
th:first-child, td:first-child { text-align:left; } th { color:var(--muted); font-weight:500; }
"""


def load(label: str, run: Path) -> dict:
    summary = json.loads((run / "summary.json").read_text())
    timings = json.loads((run / "timings.json").read_text())
    with open(run / "scans.csv") as f:
        rows = list(csv.DictReader(f))
    ts = [float(r["ts"]) for r in rows]
    period = (ts[-1] - ts[0]) / max(len(ts) - 1, 1)
    return {
        "name": label,
        "summary": summary,
        "timings": timings,
        "rows": rows,
        "period_ms": 1e3 * period,
    }


def score(run: dict) -> tuple[float, str]:
    s = run["summary"]
    share = s.get("path_share_excluded", s["path_share"])
    return share, (
        "excluding " + ", ".join(f"{a}-{b}" for a, b in s.get("excluded_scans", []))
    ) if "path_share_excluded" in s else ""


def bars(
    items: list[tuple[str, float, str]], target: float, unit: str, scale_max: float, aria: str
) -> str:
    """Horizontal bars: (label, value, colour var) with a dashed target line."""
    width, row_h, left = 680, 30, 220
    height = row_h * len(items) + 30
    scale = (width - left - 90) / scale_max
    parts = [f'<svg viewBox="0 0 {width} {height}" width="100%" role="img" aria-label="{aria}">']
    for i, (label, value, colour) in enumerate(items):
        y = 8 + i * row_h
        w = min(value, scale_max) * scale
        parts.append(f'<text x="0" y="{y + 15}">{html.escape(label)}</text>')
        parts.append(
            f'<rect x="{left}" y="{y + 3}" width="{w:.1f}" height="17" fill="var({colour})"/>'
        )
        parts.append(f'<text x="{left + w + 6:.1f}" y="{y + 15}">{value:.1f}{unit}</text>')
    tx = left + target * scale
    parts.append(
        f'<line x1="{tx:.1f}" x2="{tx:.1f}" y1="2" y2="{height - 18}" stroke="var(--target)" stroke-dasharray="4 3"/>'
    )
    parts.append(
        f'<text x="{max(tx - 70, left):.1f}" y="{height - 4}">target {target:g}{unit}</text>'
    )
    parts.append("</svg>")
    return "".join(parts)


def strip(run: dict) -> str:
    """Per scan after the first path: had a path (blank), no path (orange), excluded (grey); pink ticks = seam checks."""
    rows = run["rows"]
    first = next((i for i, r in enumerate(rows) if r["path"] == "1"), None)
    if first is None:
        return "<p class='cap'>no path yet</p>"
    rows = rows[first:]
    excluded = run["summary"].get("excluded_scans", [])
    t0, t1 = float(rows[0]["ts"]), float(rows[-1]["ts"])
    width, height = 680, 46
    span = max(t1 - t0, 1e-6)

    def x_of(r):
        return (float(r["ts"]) - t0) / span * width

    parts = [
        f'<svg viewBox="0 0 {width} {height + 16}" width="100%" role="img" aria-label="path over time">'
    ]
    parts.append(f'<rect x="0" y="0" width="{width}" height="{height}" fill="var(--strip-off)"/>')
    by_scan = {int(r["scan"]): r for r in rows}
    for a, b in excluded:
        ra, rb = by_scan.get(a, rows[0]), by_scan.get(b, rows[-1])
        parts.append(
            f'<rect x="{x_of(ra):.1f}" y="0" width="{max(x_of(rb) - x_of(ra), 1):.1f}" height="{height}" fill="var(--excluded)"/>'
        )
    for r in rows:
        if r["path"] == "0":
            parts.append(
                f'<rect x="{x_of(r):.1f}" y="0" width="1.2" height="{height}" fill="var(--bad)"/>'
            )
        if r.get("seam_check") == "1":
            parts.append(
                f'<rect x="{x_of(r):.1f}" y="{height}" width="1.5" height="6" fill="var(--target)"/>'
            )
    parts.append(
        f'<text x="0" y="{height + 15}">0 s</text><text x="{width - 60}" y="{height + 15}">{span:.0f} s</text>'
    )
    parts.append("</svg>")
    return "".join(parts)


def stage_rows(timings: dict, bar_ms: float) -> str:
    out = []
    for kind, k in timings["stages"].items():
        mark = "" if k["p99_ms"] <= bar_ms else " ✗"
        out.append(
            f"<tr><td><b>{kind}</b></td><td>{k['calls']}</td><td>{k['p50_ms']:.2f}</td>"
            f"<td>{k['p99_ms']:.2f}{mark}</td><td>{k['max_ms']:.1f}</td></tr>"
        )
        sub = sorted(k.get("stages", {}).items(), key=lambda kv: -kv[1]["p99_ms"])[:6]
        for name, v in sub:
            out.append(
                f"<tr><td>&nbsp;&nbsp;{name}</td><td></td><td>{v['p50_ms']:.2f}</td>"
                f"<td>{v['p99_ms']:.2f}</td><td>{v['max_ms']:.1f}</td></tr>"
            )
    return "".join(out)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("out", type=Path)
    parser.add_argument("runs", nargs="+")
    parser.add_argument("--status", default="")
    parser.add_argument("--note", default="")
    parser.add_argument("--target-share", type=float, default=0.97)
    parser.add_argument("--headroom", type=float, default=2.5)
    parser.add_argument(
        "--det", action="append", default=[], help="RUN_LABEL=DIR_A,DIR_B: a determinism pair"
    )
    args = parser.parse_args()
    runs = []
    for spec in args.runs:
        label, _, path = spec.partition("=")
        if (Path(path) / "summary.json").exists():
            runs.append(load(label, Path(path)))

    share_items = []
    for r in runs:
        share, how = score(r)
        share_items.append(
            (
                r["name"] + (" (scored)" if how else ""),
                100 * share,
                "--ok" if share >= args.target_share else "--bad",
            )
        )
        if how:
            raw = r["summary"]["path_share"]
            share_items.append(
                (
                    r["name"] + " (all scans)",
                    100 * raw,
                    "--ok" if raw >= args.target_share else "--bad",
                )
            )
    timing_items = []
    for r in runs:
        bar_ms = r["period_ms"] / args.headroom
        for kind, k in r["timings"]["stages"].items():
            timing_items.append(
                (
                    f"{r['name']} {kind}",
                    100 * k["p99_ms"] / bar_ms,
                    "--ok" if k["p99_ms"] <= bar_ms else "--bad",
                )
            )

    body = [
        f"<title>Lazy Reform Replays</title><style>{STYLE}</style><div class='wrap'>",
        "<header><h1>Lazy reform replays</h1>",
        f"<p class='status'>{html.escape(args.status)}</p></header>",
        "<section class='card'><h2>Plan passes with a path to the goal</h2>",
        bars(share_items, 100 * args.target_share, "%", 100.0, "share of plan passes with a path"),
        "<p class='cap'>Share of plan passes with a full path, from the first pass that found one. Higher is better.</p>"
        + (f"<p class='cap'>{html.escape(args.note)}</p>" if args.note else "")
        + "</section>",
        "<section class='card'><h2>Per-call p99 against the real-time budget</h2>",
        bars(
            timing_items,
            100.0,
            "%",
            max(150.0, max((v for _, v, _ in timing_items), default=100) * 1.05),
            "p99 per call as a share of the budget",
        ),
        f"<p class='cap'>p99 time per call as a share of the budget: the scan period divided by {args.headroom:g}, "
        "the headroom for a slower Jetson. Lower is better; past the dashed line is over budget.</p></section>",
    ]
    for r in runs:
        s, t = r["summary"], r["timings"]
        bar_ms = r["period_ms"] / args.headroom
        share, how = score(r)
        body.append(
            f"<section class='card'><h2>{html.escape(r['name'])}</h2>{strip(r)}"
            "<p class='cap'>Each orange tick is a scan with no path; grey is an excluded stretch; pink ticks mark "
            "closures that started a layer.</p>"
            f"<p class='cap'>{s['data_s']:.0f} s of data, {t['x_real_time']:.2f}x real time on this machine, "
            f"scan period {r['period_ms']:.0f} ms, budget {bar_ms:.0f} ms. Path share {100 * s['path_share']:.2f}%"
            + (f", {100 * share:.2f}% {html.escape(how)}" if how else "")
            + f". Torn seams {s['torn_seams']} of {s['seam_checks']} checked, {s['new_layers']} layers, "
            f"{s.get('plan_passes', '?')} plan passes, {s['replans']} replans.</p>"
            "<div class='tbl'><table><tr><th>call / stage</th><th>calls</th><th>p50 ms</th><th>p99 ms</th>"
            "<th>max ms</th></tr>" + stage_rows(t, bar_ms) + "</table></div></section>"
        )
    if args.det:
        import hashlib

        def h(f):
            return hashlib.sha1(Path(f).read_bytes()).hexdigest()[:8]

        body.append(
            "<section class='card'><h2>Same inputs, same outputs</h2><div class='tbl'><table><tr><th>run pair</th>"
            "<th>scans.csv</th><th>summary.json</th><th>identical</th></tr>"
        )
        for spec in args.det:
            label, _, dirs = spec.partition("=")
            a, b = (Path(d) for d in dirs.split(","))
            ha, hb = h(a / "scans.csv"), h(b / "scans.csv")
            sa, sb = h(a / "summary.json"), h(b / "summary.json")
            body.append(
                f"<tr><td>{html.escape(label)}</td><td>{ha} / {hb}</td><td>{sa} / {sb}</td>"
                f"<td>{'yes' if (ha, sa) == (hb, sb) else 'NO'}</td></tr>"
            )
        body.append(
            "</table></div><p class='cap'>Each pair replays the same feed twice; per-scan routes and the summary "
            "must match byte for byte.</p></section>"
        )
    body.append(
        "<section class='card'><h2>Every run</h2><div class='tbl'><table><tr><th>map</th><th>data s</th>"
        "<th>plan passes</th><th>replans</th><th>path share</th><th>scored share</th><th>torn seams</th>"
        "<th>layers</th><th>x real time</th></tr>"
    )
    for r in runs:
        s, t = r["summary"], r["timings"]
        share, how = score(r)
        body.append(
            f"<tr><td>{html.escape(r['name'])}</td><td>{s['data_s']:.0f}</td><td>{s.get('plan_passes', '')}</td>"
            f"<td>{s['replans']}</td><td>{100 * s['path_share']:.2f}%</td><td>{100 * share:.2f}%</td>"
            f"<td>{s['torn_seams']}</td><td>{s['new_layers']}</td><td>{t['x_real_time']:.2f}</td></tr>"
        )
    body.append("</table></div></section></div>")
    args.out.write_text("".join(body))
    print(f"wrote {args.out} ({len(runs)} runs)")


if __name__ == "__main__":
    main()
