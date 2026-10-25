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

"""The report's summary and performance sections as HTML, from score_all.py's scores and the Jetson replays.
usage: report_sections.py <manifest.json> <scores.json> <jetson runs dir> <out.html>"""

import html
import json
from pathlib import Path
import re
import sys

manifest = json.loads(Path(sys.argv[1]).expanduser().read_text())
scores = json.loads(Path(sys.argv[2]).expanduser().read_text())
jetson = Path(sys.argv[3]).expanduser()
labels = {e["name"]: e.get("label", e["name"]) for e in manifest["recordings"]}


def cm(x):
    return "&ndash;" if x is None else (f"{x * 100:.1f} cm" if x < 1 else f"{x:.2f} m")


def pair(a, b, lower_is_better=True):
    """Two cells, the clearly better one (by >10%) in bold."""
    if a is None or b is None:
        return f"<td>{cm(a)}</td><td>{cm(b)}</td>"
    better_a = a < b * 0.9 if lower_is_better else a > b * 1.1
    better_b = b < a * 0.9 if lower_is_better else b > a * 1.1
    return f"<td{' class=win' if better_a else ''}>{cm(a)}</td><td{' class=win' if better_b else ''}>{cm(b)}</td>"


def get(d, *keys):
    for k in keys:
        if not isinstance(d, dict) or d.get(k) is None:
            return None
        d = d[k]
    return d


rows = []
for name, row in scores.items():
    g, i = row.get("gsc", {}), row.get("ivan", {})
    rows.append(
        f"<tr><td>{html.escape(labels.get(name, name))}</td>"
        + pair(get(g, "gt", "final", "median_m"), get(i, "gt", "final", "median_m"))
        + pair(get(g, "gt", "live", "median_m"), get(i, "gt", "live", "median_m"))
        + f"<td class=muted>{cm(get(g, 'gt', 'raw', 'median_m'))}</td>"
        + pair(get(g, "tags", "final"), get(i, "tags", "final"))
        + pair(get(g, "tags", "live"), get(i, "tags", "live"))
        + f"<td class=muted>{cm(get(g, 'tags', 'raw'))}</td>"
        + f"<td>{g.get('loops', '&ndash;')}</td><td>{i.get('loops', '&ndash;')}</td></tr>"
    )

summary = f"""<section id="summary" class="appendix">
<h2>Summary</h2>
<p>Error of each keyframe's position against ground truth (median), and how far repeat sightings of one AprilTag land
apart (tag spread, RMS). <i>Final</i> is the finished graph; <i>live</i> is what the method believed at the moment, which is what the
planner navigates with. Bold = more than 10% better. "&ndash;" = no ground truth (park1 and both L1 runs; 555 is scored on its
QR start/end gap, shown as its tag spread) or no repeat tag sightings (Athens). x, y and z all count.</p>
<div class="table-wrap"><table>
<thead><tr><th rowspan=2>recording</th><th colspan=2>GT error, final</th><th colspan=2>GT error, live</th><th rowspan=2>raw odom</th>
<th colspan=2>tag spread, final</th><th colspan=2>tag spread, live</th><th rowspan=2>raw odom</th><th colspan=2>loops</th></tr>
<tr><th>GSC</th><th>Ivan</th><th>GSC</th><th>Ivan</th><th>GSC</th><th>Ivan</th><th>GSC</th><th>Ivan</th><th>GSC</th><th>Ivan</th></tr></thead>
<tbody>{"".join(rows)}</tbody></table></div>
<p class="note">Known problem: on Go2 L1 the huge loop gets no closures from GSC. The L1 cloud is a ~3 m local window, so the
scan-context descriptors aren't distinctive; every match it tries is wrong and the solve rejects all of them. Next on the list.</p>
</section>"""

mac = []
for name, row in scores.items():
    g, i = row.get("gsc", {}), row.get("ivan", {})
    mac.append(
        f"<tr><td>{html.escape(name)}</td><td>{g.get('realtime', '&ndash;')}x</td><td>{g.get('cpu_cores', '&ndash;')}</td>"
        f"<td>{g.get('peak_mb', '&ndash;')} MB</td><td>{g.get('dropped', '&ndash;')}</td>"
        f"<td>{i.get('dropped', '&ndash;')} ({i.get('dropped_pct', '&ndash;')}%)</td><td>{round(100 * i['busy_fraction']) if 'busy_fraction' in i else '&ndash;'}%</td></tr>"
    )

jet = []
for run in sorted(p for p in jetson.iterdir() if (p / "a.log").exists() and p.name in scores):
    log = (run / "a.log").read_text() + (run / "time.log").read_text()
    m = re.search(r"([\d.]+)s data in ([\d.]+)s wall \(([\d.]+)x realtime\)", log)
    rss = re.search(r"Maximum resident set size \(kbytes\): (\d+)", log)
    mac_speed = scores[run.name]["gsc"].get("realtime")
    if m:
        jet.append(
            f"<tr><td>{run.name}</td><td>{m[3]}x</td><td>{round(int(rss[1]) / 1e3) if rss else '&ndash;'} MB</td>"
            f"<td>{round(mac_speed / float(m[3]), 1) if mac_speed else '&ndash;'}x</td></tr>"
        )

perf = f"""<section id="performance" class="appendix">
<h2>Performance</h2>
<h3>Ivan live vs GSC live</h3>
<p>GSC: <code>pgo-eval</code> on a MacBook M5 Max, as fast as it runs (deterministic lockstep, nothing dropped). Ivan: his
<code>PGOMap</code> fed at the recording's real rate; it keeps only the newest scan while busy, so a scan that arrives during work
is lost. Busy = share of wall time Ivan spent processing.</p>
<div class="table-wrap"><table>
<thead><tr><th>recording</th><th>GSC speed (vs real time)</th><th>GSC CPU cores</th><th>GSC peak RAM</th><th>GSC scans dropped</th><th>Ivan scans dropped</th><th>Ivan busy</th></tr></thead>
<tbody>{"".join(mac)}</tbody></table></div>
<p>Live, GSC never stalls the robot: scans wait in a 5-deep queue (oldest dropped if it fills), only one closure search and one
solve run at a time, and a search is skipped while scans are waiting. huge_l1 is the exception above: solving its wrong matches made the replay
0.47x real time on the Mac.</p>
<h3>GSC on a Jetson</h3>
<p>The same replays on AlfredJetson (Orin NX, 40 W mode, 8 cores, other dimos processes running). It is 2-3x slower than the Mac and
still faster than real time everywhere except office555, which is ridden at ~4 m/s and makes keyframes 4x as fast as a walking
robot. Closure counts differ slightly from the Mac's because floating-point results differ across CPUs.</p>
<div class="table-wrap"><table>
<thead><tr><th>recording</th><th>speed (vs real time)</th><th>peak RAM</th><th>Mac is faster by</th></tr></thead>
<tbody>{"".join(jet)}</tbody></table></div>
<h3>Against the MLS planner</h3>
<p>The MLS planner needs a map in the corrected frame. With Ivan's PGO it rebuilds its whole map from the optimized cloud on
every publish: 17,201 ms p90 for 8.4M points, with a new map every 500 ms, so it can never keep up on a large map. With GSC the planner
patches only the keyframes a closure moved: 355 ms p90 (676 ms max), 197 patches per hour, on a 4.5M-voxel map after an hour of
driving. Measured on huge (Mac, 2026-10-19, the code before this round); the planner side is unchanged since, but closure counts have changed, so treat these as indicative.</p>
<p>The replays above draw the planner's goal and path through each method's biggest loop closure (one minute either side).
On huge, the planner cannot reach the goal (a keyframe from 19 minutes earlier) on GSC's map before the 208 m correction;
within 30 s of it, it plans a ~650 m route there. On Ivan's map it finds no route in that window. On the stairs recordings
(gir1, gir2, Athens) it also finds almost no route on Ivan's map: his map is a column-carving voxel grid, and a scan of an
upper floor erases the floor below it in the same column (at the Athens goal, the floor 9 m down is gone). With carving
turned off the floor is back but the planner still finds no route there, which I have not run down, so treat Ivan's
planner layer on those three as unexplained rather than as a verdict. On park1 (outdoors, flat) both maps plan every time. On office555 the planner finds no route on GSC's map either, to a goal 255 m away along the bike route; not investigated.</p>
</section>"""

Path(sys.argv[4]).expanduser().write_text(summary + "\n" + perf + "\n")
