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

usage: results_page.py OUT.html RUN_DIR [RUN_DIR ...] [--status TEXT] [--target-share 0.97]
Each RUN_DIR holds summary.json, timings.json and scans.csv; its name labels the map.
"""

from __future__ import annotations

import argparse
import csv
import html
import json
from pathlib import Path

STYLE = """
:root { --ground:#eef1f0; --panel:#fff; --ink:#15201e; --muted:#5b6965; --rule:#d3dbd8; --ok:#1b9165; --bad:#c9481e;
  --bar:#1f9cc9; --target:#c2367a; --strip-off:#e2e8e6; }
@media (prefers-color-scheme: dark) { :root:not([data-theme="light"]) { color-scheme:dark; --ground:#111816; --panel:#18211f;
  --ink:#e4ebe8; --muted:#93a39e; --rule:#2b3734; --ok:#3ccb94; --bad:#ff7d55; --bar:#6fd3f0; --target:#f06aa8; --strip-off:#26302d; } }
:root[data-theme="dark"] { color-scheme:dark; --ground:#111816; --panel:#18211f; --ink:#e4ebe8; --muted:#93a39e; --rule:#2b3734;
  --ok:#3ccb94; --bad:#ff7d55; --bar:#6fd3f0; --target:#f06aa8; --strip-off:#26302d; }
* { box-sizing:border-box; }
body { background:var(--ground); color:var(--ink); font:15px/1.5 "IBM Plex Sans", system-ui, sans-serif; padding:24px 16px 48px; }
.wrap { max-width:1100px; margin:0 auto; display:grid; gap:18px; }
h1 { font-size:24px; margin:0; } h2 { font-size:16px; margin:0 0 6px; }
.status { font:13px "IBM Plex Mono", monospace; color:var(--muted); }
.card { background:var(--panel); border:1px solid var(--rule); border-radius:6px; padding:14px 16px; }
.cap { margin:4px 0 0; font-size:13.5px; color:var(--muted); }
svg text { fill:var(--ink); font:12px "IBM Plex Mono", monospace; }
.tbl { overflow-x:auto; } table { border-collapse:collapse; width:100%; font-size:13px; font-variant-numeric:tabular-nums; }
th, td { text-align:right; padding:5px 8px; border-bottom:1px solid var(--rule); white-space:nowrap; }
th:first-child, td:first-child { text-align:left; } th { color:var(--muted); font-weight:500; }
"""


def load(run: Path) -> dict:
    summary = json.loads((run / "summary.json").read_text())
    timings = json.loads((run / "timings.json").read_text())
    with open(run / "scans.csv") as f:
        rows = list(csv.DictReader(f))
    return {"name": run.name, "summary": summary, "timings": timings, "rows": rows}


def share_chart(runs: list[dict], target: float) -> str:
    width, row_h, left = 640, 34, 130
    height = row_h * len(runs) + 30
    scale = (width - left - 60) / 1.0
    parts = [
        f'<svg viewBox="0 0 {width} {height}" width="100%" role="img" aria-label="share of replans with a path">'
    ]
    for i, r in enumerate(runs):
        share = r["summary"]["path_share"]
        y = 10 + i * row_h
        share = share if share == share else 0.0
        colour = "var(--ok)" if share >= target else "var(--bad)"
        parts.append(f'<text x="0" y="{y + 17}">{html.escape(r["name"])}</text>')
        parts.append(
            f'<rect x="{left}" y="{y + 4}" width="{share * scale:.1f}" height="18" fill="{colour}"/>'
        )
        parts.append(
            f'<text x="{left + share * scale + 6:.1f}" y="{y + 17}">{100 * share:.1f}%</text>'
        )
    tx = left + target * scale
    parts.append(
        f'<line x1="{tx:.1f}" x2="{tx:.1f}" y1="4" y2="{height - 18}" stroke="var(--target)" stroke-dasharray="4 3"/>'
    )
    parts.append(f'<text x="{tx - 60:.1f}" y="{height - 4}">target {100 * target:.0f}%</text>')
    parts.append("</svg>")
    return "".join(parts)


def strip(run: dict) -> str:
    """Path found per replan over data time: one thin bar per replan."""
    rows = [r for r in run["rows"] if r["replanned"] == "1"]
    if not rows:
        return "<p class='cap'>no replans yet</p>"
    t0, t1 = float(rows[0]["ts"]), float(rows[-1]["ts"])
    width, height = 640, 46
    span = max(t1 - t0, 1e-6)
    parts = [
        f'<svg viewBox="0 0 {width} {height + 16}" width="100%" role="img" aria-label="replans with a path over time">'
    ]
    parts.append(f'<rect x="0" y="0" width="{width}" height="{height}" fill="var(--strip-off)"/>')
    for r in rows:
        if r["path"] == "1":
            continue
        x = (float(r["ts"]) - t0) / span * width
        parts.append(f'<rect x="{x:.1f}" y="0" width="1.2" height="{height}" fill="var(--bad)"/>')
    for r in run["rows"]:
        if r["seam_check"] == "1":
            x = (float(r["ts"]) - t0) / span * width
            parts.append(
                f'<rect x="{x:.1f}" y="{height}" width="1.5" height="6" fill="var(--target)"/>'
            )
    parts.append(
        f'<text x="0" y="{height + 15}">0 s</text><text x="{width - 60}" y="{height + 15}">{span:.0f} s</text>'
    )
    parts.append("</svg>")
    return "".join(parts)


def stage_row(name: str, stats: dict) -> str:
    return (
        f"<tr><td>{name}</td><td>{stats['calls']}</td><td>{stats['p50_ms']:.2f}</td>"
        f"<td>{stats['p99_ms']:.2f}</td><td>{stats['max_ms']:.1f}</td><td>{stats['total_s']:.1f}</td></tr>"
    )


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("out", type=Path)
    parser.add_argument("runs", type=Path, nargs="+")
    parser.add_argument("--status", default="")
    parser.add_argument("--target-share", type=float, default=0.97)
    args = parser.parse_args()
    runs = [load(r) for r in args.runs if (r / "summary.json").exists()]
    body = [
        f"<title>Lazy Reform Replays</title><style>{STYLE}</style><div class='wrap'>",
        "<header><h1>Lazy reform replays</h1>",
        f"<p class='status'>{html.escape(args.status)}</p></header>",
        "<section class='card'><h2>Replans that found a path</h2>",
        share_chart(runs, args.target_share),
        "<p class='cap'>Share of replans with a full path to the goal, counted from the first replan that found one. "
        "Higher is better; the dashed line is the target.</p></section>",
    ]
    for r in runs:
        s, t = r["summary"], r["timings"]
        body.append(
            f"<section class='card'><h2>{html.escape(r['name'])}</h2>{strip(r)}"
            "<p class='cap'>Each orange tick is a replan with no path; pink ticks under the strip mark closures "
            "that started a new layer. Fewer orange ticks is better.</p>"
            f"<p class='cap'>{s['data_s']:.0f} s of data at {t['x_real_time']:.2f}x real time on this machine, "
            f"torn seams {s['torn_seams']} of {s['seam_checks']} checked, layers {s['layers']}, events {s['events']}.</p>"
            "<div class='tbl'><table><tr><th>stage</th><th>calls</th><th>p50 ms</th><th>p99 ms</th><th>max ms</th>"
            "<th>total s</th></tr>"
            + "".join(stage_row(k, v) for k, v in t["stages"].items())
            + "</table></div></section>"
        )
    body.append(
        "<section class='card'><h2>Every run</h2><div class='tbl'><table><tr><th>map</th><th>data s</th><th>replans</th>"
        "<th>counted</th><th>with path</th><th>share</th><th>torn seams</th><th>new layers</th><th>events</th>"
        "<th>x real time</th></tr>"
    )
    for r in runs:
        s, t = r["summary"], r["timings"]
        body.append(
            f"<tr><td>{html.escape(r['name'])}</td><td>{s['data_s']:.0f}</td><td>{s['replans']}</td>"
            f"<td>{s['replans_counted']}</td><td>{s['counted_with_path']}</td><td>{100 * s['path_share']:.2f}%</td>"
            f"<td>{s['torn_seams']}</td><td>{s['new_layers']}</td><td>{s['events']}</td><td>{t['x_real_time']:.2f}</td></tr>"
        )
    body.append("</table></div></section></div>")
    args.out.write_text("".join(body))
    print(f"wrote {args.out} ({len(runs)} runs)")


if __name__ == "__main__":
    main()
