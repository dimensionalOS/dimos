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
import math
from pathlib import Path
import re
import sys

manifest = json.loads(Path(sys.argv[1]).expanduser().read_text())
scores = json.loads(Path(sys.argv[2]).expanduser().read_text())
jetson = Path(sys.argv[3]).expanduser()
labels = {e["name"]: e.get("label", e["name"]) for e in manifest["recordings"]}


def cm(x):
    return "&ndash;" if x is None else (f"{x * 100:.1f} cm" if x < 1 else f"{x:.2f} m")


WORSE_FULL_RED_M = (
    1.0  # display scale: a method this many metres worse than raw odometry shades fully red
)


def shade(score):
    """Cell background for a score in [-1, 1]: green = good, red = bad, none at 0 (same scale as viewer.html's shade)."""
    if score is None or not math.isfinite(score):
        return ""
    score = max(-1.0, min(1.0, score))
    color = "--edge-ok" if score > 0 else "--edge-bad"
    return f' style="background: color-mix(in srgb, var({color}) {round(abs(score) * 45)}%, transparent)"'


def vs_raw(value, raw):
    """Against raw odometry: better = the share of raw's error removed (all of it = 1); worse = metres added (1 m = -1)."""
    if value is None or not raw:
        return None
    return (raw - value) / raw if value <= raw else -(value - raw) / WORSE_FULL_RED_M


def speed(realtime):
    """Replay speed vs real time on a log scale: 8x faster = 1, real time = 0, 8x slower = -1."""
    return None if not realtime else math.log2(realtime) / 3


def drops(percent):
    """Share of scans dropped: none = 1, 5% = 0, 10% or more = -1."""
    return None if percent is None else 1 - percent / 5


def pair(a, b, raw):
    """Two cells shaded by their improvement over raw odometry, the clearly better one (by >10%) in bold."""
    if a is None or b is None:
        return f"<td{shade(vs_raw(a, raw))}>{cm(a)}</td><td{shade(vs_raw(b, raw))}>{cm(b)}</td>"
    better_a, better_b = a < b * 0.9, b < a * 0.9
    return (
        f"<td{' class=win' if better_a else ''}{shade(vs_raw(a, raw))}>{cm(a)}</td>"
        f"<td{' class=win' if better_b else ''}{shade(vs_raw(b, raw))}>{cm(b)}</td>"
    )


def tip(*keys):
    """A (?) button; viewer.html's TIPS holds the explanation for each key."""
    return f'<button class="help" type="button" data-tip="{" ".join(keys)}" aria-label="explain">?</button>'


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
        + pair(
            get(g, "gt", "final", "median_m"),
            get(i, "gt", "final", "median_m"),
            get(g, "gt", "raw", "median_m"),
        )
        + pair(
            get(g, "gt", "live", "median_m"),
            get(i, "gt", "live", "median_m"),
            get(g, "gt", "raw", "median_m"),
        )
        + f"<td class=muted>{cm(get(g, 'gt', 'raw', 'median_m'))}</td>"
        + pair(get(g, "tags", "final"), get(i, "tags", "final"), get(g, "tags", "raw"))
        + pair(get(g, "tags", "live"), get(i, "tags", "live"), get(g, "tags", "raw"))
        + f"<td class=muted>{cm(get(g, 'tags', 'raw'))}</td>"
        + "</tr>"
    )

summary = f"""<section id="summary" class="appendix">
<h2>Summary</h2>
<p>Error of each keyframe's position against ground truth (median), and how far repeat sightings of one AprilTag land
apart (tag spread, RMS). <i>Final</i> is the finished graph; <i>live</i> is what the method believed at the moment, which is what the
planner navigates with. Bold = more than 10% better. Shading is against raw odometry: green = the share of raw's error the method removed (deep green =
all of it); red = how much worse than raw in absolute terms (deep red = 1 m worse), so a 2 cm loss barely shows. "&ndash;" = no ground truth (park1 and the L1 runs; 555 is scored on its
QR start/end gap, shown as its tag spread) or no repeat tag sightings (Athens). x, y and z all count.</p>
<div class="table-wrap"><table>
<thead><tr><th rowspan=2>recording{tip("shade")}</th><th colspan=2>GT error, final{tip("gt", "final")}</th><th colspan=2>GT error, live{tip("gt", "live")}</th><th rowspan=2>raw odom{tip("raw")}</th>
<th colspan=2>tag spread, final{tip("tags", "final")}</th><th colspan=2>tag spread, live{tip("tags", "live")}</th><th rowspan=2>raw odom{tip("raw")}</th></tr>
<tr><th>GSC</th><th>Ivan</th><th>GSC</th><th>Ivan</th><th>GSC</th><th>Ivan</th><th>GSC</th><th>Ivan</th></tr></thead>
<tbody>{"".join(rows)}</tbody></table></div>
<p class="note">Known problem: on Go2 L1 the huge loop gets no closures from GSC. The L1 cloud is a ~3 m local window, so the
scan-context descriptors aren't distinctive; every match it tries is wrong and the solve rejects all of them. Next on the list.
The short HK village L1 runs (hk1-6, no ground truth) look like the same problem: GSC keeps few closures and stays near raw
odometry, while Ivan keeps 10-20 and ends with a larger tag spread than raw odometry on all six.</p>
</section>"""

mac = []
for name, row in scores.items():
    g, i = row.get("gsc", {}), row.get("ivan", {})
    mac.append(
        f"<tr><td>{html.escape(name)}</td><td{shade(speed(g.get('realtime')))}>{g.get('realtime', '&ndash;')}x</td><td>{g.get('cpu_cores', '&ndash;')}</td>"
        f"<td>{g.get('peak_mb', '&ndash;')} MB</td><td{shade(drops(0 if g.get('dropped') == 0 else None))}>{g.get('dropped', '&ndash;')}</td>"
        f"<td{shade(drops(i.get('dropped_pct')))}>{i.get('dropped', '&ndash;')} ({i.get('dropped_pct', '&ndash;')}%)</td><td>{round(100 * i['busy_fraction']) if 'busy_fraction' in i else '&ndash;'}%</td></tr>"
    )

jet = []
for run in sorted(p for p in jetson.iterdir() if (p / "a.log").exists() and p.name in scores):
    log = (run / "a.log").read_text() + (run / "time.log").read_text()
    m = re.search(r"([\d.]+)s data in ([\d.]+)s wall \(([\d.]+)x realtime\)", log)
    rss = re.search(r"Maximum resident set size \(kbytes\): (\d+)", log)
    mac_speed = scores[run.name]["gsc"].get("realtime")
    if m:
        jet.append(
            f"<tr><td>{run.name}</td><td{shade(speed(float(m[3])))}>{m[3]}x</td><td>{round(int(rss[1]) / 1e3) if rss else '&ndash;'} MB</td>"
            f"<td>{round(mac_speed / float(m[3]), 1) if mac_speed else '&ndash;'}x</td></tr>"
        )

perf = f"""<section id="performance" class="appendix">
<h2>Performance</h2>
<h3>Ivan live vs GSC live</h3>
<p>GSC: <code>pgo-eval</code> on a MacBook M5 Max, as fast as it runs (deterministic lockstep, nothing dropped). Ivan: his
<code>PGOMap</code> fed at the recording's real rate; it keeps only the newest scan while busy, so a scan that arrives during work
is lost. Busy = share of wall time Ivan spent processing. Shading: speed against real time on a log scale (deep green = 8x faster,
red = slower than real time); scans dropped from none (green) through 5% (no shade) to 10% or more (deep red).</p>
<div class="table-wrap"><table>
<thead><tr><th>recording</th><th>GSC speed (vs real time){tip("speed")}</th><th>GSC CPU cores</th><th>GSC peak RAM</th><th>GSC scans dropped{tip("dropped")}</th><th>Ivan scans dropped{tip("dropped")}</th><th>Ivan busy{tip("busy")}</th></tr></thead>
<tbody>{"".join(mac)}</tbody></table></div>
<p>Live, GSC never stalls the robot: scans wait in a 5-deep queue (oldest dropped if it fills), only one closure search and one
solve run at a time, and a search is skipped while scans are waiting. huge_l1 is the exception above: solving its wrong matches made the replay
0.47x real time on the Mac.</p>
<h3>GSC on a Jetson</h3>
<p>The same replays on AlfredJetson (Orin NX, 40 W mode, 8 cores, other dimos processes running). It is 2-3x slower than the Mac and
still faster than real time everywhere except office555, which is ridden at ~4 m/s and makes keyframes 4x as fast as a walking
robot. Closure counts differ slightly from the Mac's because floating-point results differ across CPUs.</p>
<div class="table-wrap"><table>
<thead><tr><th>recording</th><th>speed (vs real time){tip("speed")}</th><th>peak RAM</th><th>Mac is faster by</th></tr></thead>
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
