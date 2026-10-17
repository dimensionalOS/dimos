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

"""Interactive per-frame cost plot for live_timing.py output -> <name>.live_timing.html"""

import json
import sys

from paths import OUT as HERE

name = sys.argv[1]
rows = json.loads((HERE / f"{name}.live_timing.json").read_text())
t0 = rows[0][0]
minutes = [round((r[0] - t0) / 60, 4) for r in rows]
total_ms = [round(r[1] + r[2], 2) for r in rows]
period_ms = 1000 * (rows[-1][0] - t0) / max(1, len(rows) - 1)
rebuilds = [(m, round(r[1], 1)) for m, r in zip(minutes, rows, strict=True) if r[5]]
emits = [(m, round(r[2], 1)) for m, r in zip(minutes, rows, strict=True) if r[6] >= 0]
data = {
    "minutes": minutes,
    "total": total_ms,
    "period": period_ms,
    "rebuild_x": [x for x, _ in rebuilds],
    "rebuild_y": [y for _, y in rebuilds],
    "emit_x": [x for x, _ in emits],
    "emit_y": [y for _, y in emits],
    "kf": [r[3] for r in rows],
    "loops": [r[4] for r in rows],
}
html = f"""<!doctype html><html><head><meta charset="utf-8"><title>{name} live PGO cost</title>
<script src="https://cdn.jsdelivr.net/npm/plotly.js-dist-min@2.35.2/plotly.min.js"></script>
<style>body{{font-family:system-ui;margin:16px;background:#fff;color:#111}}</style></head><body>
<h3>{name}: Ivan's PGOVoxelMapper per-frame cost (unitree-go2-pgo settings)</h3>
<p>Dashed line = lidar frame period ({period_ms:.0f} ms). Above it, the live module drops frames.</p>
<div id="cost" style="height:520px"></div><div id="graph" style="height:300px"></div>
<script>
const d = {json.dumps(data)};
Plotly.newPlot("cost", [
  {{x: d.minutes, y: d.total, name: "every frame (add + map)", mode: "markers", marker: {{size: 3, color: "#9aa5b1"}}}},
  {{x: d.emit_x, y: d.emit_y, name: "global_map() publish", mode: "markers", marker: {{size: 4, color: "#1f77b4"}}}},
  {{x: d.rebuild_x, y: d.rebuild_y, name: "loop rebuild frame", mode: "markers", marker: {{size: 8, color: "#d62728", symbol: "x"}}}},
  {{x: [0, d.minutes[d.minutes.length - 1]], y: [d.period, d.period], name: "frame budget", mode: "lines", line: {{dash: "dash", color: "#2ca02c"}}}},
], {{xaxis: {{title: "recording time (min)"}}, yaxis: {{title: "ms (log)", type: "log"}}, margin: {{t: 10}}}}, {{responsive: true}});
Plotly.newPlot("graph", [
  {{x: d.minutes, y: d.kf, name: "keyframes"}}, {{x: d.minutes, y: d.loops, name: "loops", yaxis: "y2"}},
], {{xaxis: {{title: "recording time (min)"}}, yaxis: {{title: "keyframes"}}, yaxis2: {{title: "loops", overlaying: "y", side: "right"}}, margin: {{t: 10}}}}, {{responsive: true}});
</script></body></html>"""
out = HERE / f"{name}.live_timing.html"
out.write_text(html)
print(out)
