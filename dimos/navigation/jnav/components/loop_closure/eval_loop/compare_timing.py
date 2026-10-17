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

"""Per-frame cost of Ivan's live module vs gsc, per minute of recording -> <name>.timing_compare.{json,html}

usage: compare_timing.py <name>
reads $EVAL_LOOP_OUT/<name>.live_timing.json (live_timing.py) and $EVAL_LOOP_OUT/timing/<name>.gsc_<cfg>.json
(`pgo-eval --output`, which records `frame_timing`) at gsc's defaults.
"""

import json
import sys

import numpy as np
from paths import OUT

BUDGET_MS = 100.0  # 10 Hz lidar
# the every-frame chart holds ~70k points per series, so it shows the two live modules only
GSC_PIPELINE = "gsc live module (pipeline thread; GNC solve and iSAM2 rebuild on its worker)"
EVERY_FRAME = (
    "Ivan LIVE module (PGOMap: PGO + voxel map, per frame)",
    GSC_PIPELINE,
)


def per_minute(seconds: np.ndarray, ms: np.ndarray) -> dict:
    minute = (seconds // 60).astype(int)
    stats: dict = {"minute": [], "mean": [], "p99": [], "max": []}
    for m in np.unique(minute):
        values = ms[minute == m]
        stats["minute"].append(int(m))
        stats["mean"].append(round(float(values.mean()), 2))
        stats["p99"].append(round(float(np.percentile(values, 99)), 1))
        stats["max"].append(round(float(values.max()), 1))
    return stats


def plateaus(seconds: np.ndarray, ms: np.ndarray) -> tuple[list, list]:
    """Step points for each frame as a flat segment as wide as it took, on a live timeline (minutes on
    x): one worker takes frames in order, so a frame starts when it arrives or when the previous one
    finishes, whichever is later. Idle time is drawn at 0. Plot with line shape "hv"."""
    x: list[float] = []
    y: list[float] = []
    free_at = -np.inf
    for arrival, cost in zip(seconds.tolist(), ms.tolist(), strict=True):
        start = max(arrival, free_at)
        if x and start > free_at:
            x.append(free_at)
            y.append(0.0)
        x.append(start)
        y.append(cost)
        free_at = start + cost / 1e3
    x.append(free_at)
    y.append(0.0)
    return np.round(np.array(x) / 60, 5).tolist(), np.round(y, 1).tolist()


def output_latency(seconds: np.ndarray, ms: np.ndarray, emits: np.ndarray, cap: int) -> tuple:
    """Seconds from a scan's arrival until the output that includes it is published, for the emitting
    frames, and the share of scans dropped. One worker takes waiting scans oldest-first; a scan that
    arrives while it is busy waits, and when more than `cap` wait the oldest is dropped (its cost is
    not paid). cap=1 is "keep only the newest", Ivan's module's `backpressure`."""
    from collections import deque

    latencies: list[float] = []
    waiting: deque = deque()
    free_at = -np.inf
    dropped = 0

    def run_waiting_until(now: float) -> None:
        nonlocal free_at
        while waiting and free_at <= now:
            index = waiting.popleft()
            free_at = free_at + ms[index] / 1e3
            if emits[index]:
                latencies.append(free_at - seconds[index])

    for i, arrival in enumerate(seconds.tolist()):
        run_waiting_until(arrival)
        if free_at <= arrival and not waiting:
            free_at = arrival + ms[i] / 1e3
            if emits[i]:
                latencies.append(free_at - arrival)
            continue
        waiting.append(i)
        while len(waiting) > cap:
            waiting.popleft()
            dropped += 1
    run_waiting_until(np.inf)
    return np.array(latencies), dropped / len(seconds)


def latency_stats(latency_s: np.ndarray) -> dict:
    ms = latency_s * 1e3
    return {
        "outputs": len(ms),
        "latency_median_ms": round(float(np.median(ms)), 1),
        "latency_p99_ms": round(float(np.percentile(ms, 99)), 1),
        "latency_max_ms": round(float(ms.max()), 1),
        "outputs_over_100ms_pct": round(float((ms > BUDGET_MS).mean() * 100), 2),
        "outputs_over_1s": int((ms > 1000).sum()),
    }


def summarize(seconds: np.ndarray, ms: np.ndarray, keyframes: np.ndarray) -> dict:
    last10 = seconds >= seconds[-1] - 600
    first10 = seconds <= 600
    return {
        "frames": len(ms),
        "keyframes": int(keyframes[-1]),
        "wall_s": round(float(ms.sum() / 1e3), 1),
        "realtime_factor": round(float(seconds[-1] / (ms.sum() / 1e3)), 2),
        "mean_ms_first_10min": round(float(ms[first10].mean()), 2),
        "mean_ms_last_10min": round(float(ms[last10].mean()), 2),
        "worst_10min_mean_ms": round(
            max(
                float(ms[(seconds >= s) & (seconds < s + 600)].mean())
                for s in range(0, int(seconds[-1]) - 599, 60)
            ),
            2,
        ),
        "max_ms": round(float(ms.max()), 1),
        "frames_over_budget_pct": round(float((ms > BUDGET_MS).mean() * 100), 2),
        "frames_over_1s": int((ms > 1000).sum()),
    }


def main() -> None:
    name = sys.argv[1]
    live = np.array(json.loads((OUT / f"{name}.live_timing.json").read_text()))
    seconds = live[:, 0] - live[0, 0]
    series = {
        "Ivan LIVE module (PGOMap: PGO + voxel map, per frame)": (
            seconds,
            live[:, 1] + live[:, 2],
            live[:, 3],
        ),
    }
    # the one gsc case: its live module at its defaults. pgo-eval runs the GNC solve and iSAM2 rebuild
    # inline after each closure and records their ms (column 4); live they run on the GNC worker, so the
    # pipeline thread pays the frame minus that
    frames = np.array(
        json.loads((OUT / "timing" / f"{name}.gsc_default.json").read_text())["frame_timing"]
    )
    series[GSC_PIPELINE] = (frames[:, 0] - live[0, 0], frames[:, 1] - frames[:, 3], frames[:, 2])
    summary = {label: summarize(*values) for label, values in series.items()}
    # what each live module outputs, through its own queue: Ivan's the global map (every 5th frame and
    # each rebuild), keeping only the newest waiting scan (`backpressure`, module.py:83); gsc's live
    # module its corrected odometry every scan from its pipeline thread (GNC solve on a worker), from a
    # FIFO that drops the oldest past `max_scan_queue` = 100 (main.rs:395)
    queues = {
        "Ivan LIVE module: global map out (keeps newest scan only)": (
            seconds,
            live[:, 1] + live[:, 2],
            live[:, 6] >= 0,
            1,
        ),
    }
    if GSC_PIPELINE in series:
        gsc_s, gsc_ms, _ = series[GSC_PIPELINE]
        queues["gsc live module: corrected odometry out (FIFO, 100 deep)"] = (
            gsc_s,
            gsc_ms,
            np.ones(len(gsc_ms), bool),
            100,
        )
    latency_summary = {}
    for label, (values_s, values_ms, emits, cap) in queues.items():
        values, dropped = output_latency(values_s, values_ms, emits, cap)
        latency_summary[label] = {
            **latency_stats(values),
            "scans_dropped_pct": round(dropped * 100, 1),
        }
    summary["Ivan LIVE module (PGOMap: PGO + voxel map, per frame)"]["max_rss_mb"] = round(
        float(live[:, 8].max())
    )
    curves = {label: per_minute(s, ms) for label, (s, ms, _) in series.items()}
    rss = per_minute(seconds, live[:, 8])
    # gsc's resident memory, sampled by pgo-eval while it streams the recording (running max, like
    # Ivan's ru_maxrss)
    gsc_rss = np.array(
        json.loads((OUT / "timing" / f"{name}.gsc_default.json").read_text()).get("rss_samples")
        or json.loads((OUT / "timing" / f"{name}.gsc_mem.json").read_text())["rss_samples"]
    )
    gsc_rss_x = np.round((gsc_rss[:, 0] - live[0, 0]) / 60, 3).tolist()
    gsc_rss_y = np.round(np.maximum.accumulate(gsc_rss[:, 1])).tolist()
    summary[GSC_PIPELINE]["max_rss_mb"] = round(float(gsc_rss[:, 1].max()))
    (OUT / f"{name}.timing_compare.json").write_text(
        json.dumps(
            {"output_latency": latency_summary, "summary": summary, "per_minute": curves}, indent=1
        )
    )
    colors = ["#c0392b", "#e67e22", "#2471a3", "#17a589", "#8e44ad"][: len(series)]
    last_minute = curves[next(iter(curves))]["minute"][-1]
    budget = {
        "x": [0, last_minute],
        "y": [BUDGET_MS, BUDGET_MS],
        "name": "10 Hz frame budget",
        "mode": "lines",
        "line": {"color": "#7f8c8d", "dash": "dash"},
    }
    traces = {
        stat: [
            {
                "x": curve["minute"],
                "y": curve[stat],
                "name": label,
                "mode": "lines",
                "line": {"color": color, "width": 2},
            }
            for (label, curve), color in zip(curves.items(), colors, strict=True)
        ]
        + [budget]
        for stat in ("mean", "max")
    }
    traces["frame"] = [
        {
            "x": plateau_x,
            "y": plateau_y,
            "name": label,
            "type": "scattergl",
            "mode": "lines",
            "line": {"color": color, "width": 1, "shape": "hv"},
        }
        for (label, (seconds_, ms, _)), color in zip(series.items(), colors, strict=True)
        if label in EVERY_FRAME
        for plateau_x, plateau_y in [plateaus(seconds_, ms)]
    ] + [budget]
    rows = "".join(
        f"<tr><td>{label}</td><td>{s['mean_ms_first_10min']}</td><td>{s['mean_ms_last_10min']}</td><td>{s['worst_10min_mean_ms']}</td>"
        f"<td>{s['max_ms']}</td><td>{s['frames_over_budget_pct']}%</td><td>{s['frames_over_1s']}</td><td>{s['wall_s']}</td></tr>"
        for label, s in summary.items()
    )
    lat_rows = "".join(
        f"<tr><td>{label}</td><td>{s['outputs']}</td><td>{s['latency_median_ms']}</td><td>{s['latency_p99_ms']}</td>"
        f"<td>{s['latency_max_ms']}</td><td>{s['outputs_over_100ms_pct']}%</td><td>{s['outputs_over_1s']}</td>"
        f"<td>{str(s['scans_dropped_pct']) + '%' if 'scans_dropped_pct' in s else '0%'}</td></tr>"
        for label, s in latency_summary.items()
    )
    html = f"""<!doctype html><html><head><meta charset="utf-8"><title>PGO cost compare</title>
<script src="https://cdn.jsdelivr.net/npm/plotly.js-dist-min@2.35.2/plotly.min.js"></script>
<style>body{{font-family:system-ui;margin:16px;background:#fff;color:#111}}table{{border-collapse:collapse;font-size:14px}}
td,th{{border:1px solid #ccc;padding:4px 8px;text-align:right}}td:first-child,th:first-child{{text-align:left}}</style></head><body>
<h3>{name}: output latency, Ivan's live module vs gsc's (Apple M5 Max, one replay each)</h3>
<p>Latency = time from a scan's arrival (at its recorded time) until the output that includes it is published,
with one worker taking frames in order. Ivan's module outputs its global map (every 5th frame and on each
rebuild); gsc's outputs its corrected odometry every scan, from its pipeline thread (its GNC solve runs on a
background worker; adopting it and the iSAM2 rebuild run on the pipeline thread and are counted).</p>
<p>Each through its own queue as shipped: Ivan's keeps only the newest waiting scan and drops the rest; gsc's
is a FIFO that drops the oldest beyond 100 waiting scans. A dropped scan's cost is not paid.</p>
<table><tr><th>output</th><th>outputs</th><th>median ms</th><th>p99 ms</th><th>worst ms</th><th>over 100 ms</th><th>over 1 s</th><th>scans dropped</th></tr>{lat_rows}</table>
<h4>Per-frame compute</h4>
<table><tr><th>method</th><th>mean ms, first 10 min</th><th>mean ms, last 10 min</th><th>worst 10-min mean</th><th>worst frame ms</th>
<th>frames over 100 ms</th><th>frames over 1 s</th><th>total s</th></tr>{rows}</table>
<h4>Every frame, on a live timeline (one worker, frames queue while it is busy; idle drawn at 0)</h4><div id="frame" style="height:560px"></div>
<h4>Average frame, per minute</h4><div id="mean" style="height:460px"></div>
<h4>Worst frame, per minute</h4><div id="max" style="height:460px"></div>
<h4>Memory (peak resident so far)</h4><div id="rss" style="height:260px"></div>
<script>
const layout = (title) => ({{xaxis: {{title: "recording time (min)"}}, yaxis: {{title}}, margin: {{t: 10}}, showlegend: true, legend: {{x: 0.01, y: 0.99, xanchor: "left", yanchor: "top", bgcolor: "rgba(255,255,255,0.85)", bordercolor: "#ccc", borderwidth: 1}}}});
Plotly.newPlot("frame", {json.dumps(traces["frame"], separators=(",", ":"))}, layout("ms for that frame"), {{responsive: true}});
Plotly.newPlot("mean", {json.dumps(traces["mean"])}, layout("mean ms per frame"), {{responsive: true}});
Plotly.newPlot("max", {json.dumps(traces["max"])}, layout("worst frame in the minute (ms)"), {{responsive: true}});
Plotly.newPlot("rss", [
  {{x: {json.dumps(rss["minute"])}, y: {json.dumps(rss["max"])}, name: "Ivan LIVE module", mode: "lines", line: {{color: "#c0392b"}}}},
  {{x: {json.dumps(gsc_rss_x)}, y: {json.dumps(gsc_rss_y)}, name: "gsc live module", mode: "lines", line: {{color: "#2471a3"}}}},
], layout("process memory (MB)"), {{responsive: true}});
</script></body></html>"""
    out = OUT / f"{name}.timing_compare.html"
    out.write_text(html)
    print(json.dumps(summary, indent=1))
    print(out)


if __name__ == "__main__":
    main()
