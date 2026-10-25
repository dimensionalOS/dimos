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

"""Top-down video of a reform replay db (export_replay.py): one frame per `step` seconds of data, showing the newest
reformed_map, GSC's keyframe path, the scans of the last `step` seconds where they were placed then, and (when the
db has loop_edges) every loop candidate GSC weighed, between its two keyframes where they are now: yellow while
proposed / waiting for GNC, green dashed once GNC accepts it, green solid once applied to the map, red when a gate
or GNC rejects it (fading out over `fade` frames). Width and opacity follow the edge's confidence.
usage: render_replay.py <replay.db> <out.mp4> [step_s] [title] [fade_frames]"""

from pathlib import Path
import subprocess
import sys

import matplotlib
import numpy as np

matplotlib.use("Agg")
import matplotlib.pyplot as plt

from dimos.memory.store.sqlite import SqliteStore

db, out = Path(sys.argv[1]), Path(sys.argv[2])
step = float(sys.argv[3]) if len(sys.argv) > 3 else 10.0
title = sys.argv[4] if len(sys.argv) > 4 else db.stem
fade = float(sys.argv[5]) if len(sys.argv) > 5 else 3.0
PROPOSED, GATE_REJECTED, COMMITTED, VOTE, ACCEPTED, REJECTED, APPLIED = range(7)
with SqliteStore(path=str(db)) as store:
    maps = [
        (o.ts, np.asarray(o.data.points_f32())) for o in store.streams.reformed_map.order_by("ts")
    ]
    paths = [
        (o.ts, np.asarray(o.data.points_f32())) for o in store.streams.gsc_keyframes.order_by("ts")
    ]
    odom = np.array(
        [
            (o.ts, o.data.pose.position.x, o.data.pose.position.y)
            for o in store.streams.gsc_odometry.order_by("ts")
        ]
    )
    scans = [
        (o.ts, np.asarray(o.data.points_f32())[:, :2])
        for o in store.streams.lidar_live.order_by("ts")
    ]
    edge_events = []
    if "loop_edges" in store.list_streams():
        for o in store.streams.loop_edges.order_by("ts"):
            points = np.asarray(o.data.points_f32()).reshape(-1, 2, 3)
            ids = np.asarray(o.data.offset_times_u32()).reshape(-1, 2)
            states = np.asarray(o.data.tags_u8())[::2]
            intensities = (
                o.data.intensities_f32()
            )  # None when every confidence is 0 (the field is dropped)
            confidence = np.zeros(len(ids)) if intensities is None else np.asarray(intensities)[::2]
            for p, (a, b), state, c in zip(points, ids, states, confidence, strict=False):
                edge_events.append((o.ts, int(a), int(b), int(state), float(c), p))
edge_events.sort(key=lambda e: e[0])
print(f"{len(edge_events)} loop-edge events", flush=True)
scan_ts = np.array([t for t, _ in scans])
everything = np.concatenate([maps[-1][1][:, :2], odom[:, 1:]])
lo, hi = everything.min(0) - 10, everything.max(0) + 10
frames = out.with_suffix("")
frames.mkdir(parents=True, exist_ok=True)
t0 = scan_ts[0]
edges: dict[tuple[int, int], dict] = {}
next_edge = 0
deferred: list[tuple] = []
counts = dict.fromkeys(("proposed", "accepted", "rejected", "applied"), 0)
for n, t in enumerate(np.arange(t0 + step, scan_ts[-1] + step, step)):
    due, deferred = deferred, []
    while next_edge < len(edge_events) and edge_events[next_edge][0] <= t:
        due.append(edge_events[next_edge])
        next_edge += 1
    # a proposal stays yellow for at least one frame: its later events wait for the next frame
    proposed_now: set[tuple[int, int]] = set()
    for ts, a, b, state, c, p in due:
        if (a, b) in proposed_now and state != PROPOSED:
            deferred.append((ts, a, b, state, c, p))
            continue
        if state == PROPOSED:
            proposed_now.add((a, b))
        edge = edges.setdefault(
            (a, b), {"look": "proposed", "since": ts, "confidence": c, "ends": p}
        )
        edge["ends"], edge["confidence"] = p, c
        look = {
            PROPOSED: "proposed",
            COMMITTED: "proposed",
            GATE_REJECTED: "rejected",
            ACCEPTED: "accepted",
            REJECTED: "rejected",
            APPLIED: "applied",
        }.get(state)
        if state == PROPOSED:
            counts["proposed"] += 1
        if look is not None and (look != edge["look"] or state == PROPOSED):
            if look in ("accepted", "rejected", "applied") and look != edge["look"]:
                counts[look] += 1
            edge["look"], edge["since"] = look, ts
    fig, ax = plt.subplots(figsize=(11, 7), dpi=100, facecolor="black")
    ax.set_facecolor("black")
    current = [m for ts, m in maps if ts <= t]
    if current:
        m = current[-1]
        ax.scatter(
            m[:, 0],
            m[:, 1],
            s=0.1,
            c=np.clip(m[:, 2], -1, 3),
            cmap="bone",
            vmin=-2,
            vmax=3,
            linewidths=0,
        )
    path = [p for ts, p in paths if ts <= t]
    if path:
        ax.plot(path[-1][:, 0], path[-1][:, 1], color="white", lw=0.4, alpha=0.6)
    now_path = path[-1] if path else np.zeros((0, 3))
    for (a, b), edge in edges.items():
        age = (t - edge["since"]) / step
        if edge["look"] == "rejected" and age > fade:
            continue
        ends = np.array(
            [now_path[k] if k < len(now_path) else edge["ends"][i] for i, k in enumerate((a, b))]
        )
        c = min(max(edge["confidence"], 0.0), 1.0)
        alpha = 0.2 + 0.8 * c
        if edge["look"] == "rejected":
            alpha *= max(0.0, 1.0 - age / fade)
        color = {"proposed": "yellow", "accepted": "lime", "applied": "lime", "rejected": "red"}[
            edge["look"]
        ]
        style = "--" if edge["look"] == "accepted" else "-"
        # dots at both keyframes: most closures span < 5 m, a few pixels at full-map zoom
        ax.plot(
            ends[:, 0],
            ends[:, 1],
            color=color,
            ls=style,
            lw=1.0 + 2.0 * c,
            alpha=alpha,
            marker="o",
            ms=3 + 3 * c,
        )
    recent = [s for ts, s in scans if t - step < ts <= t]
    if recent:
        r = np.concatenate(recent)
        ax.scatter(r[:, 0], r[:, 1], s=0.3, c="deepskyblue" if edge_events else "red", linewidths=0)
    seen = odom[odom[:, 0] <= t]
    if len(seen):
        ax.plot(seen[-1, 1], seen[-1, 2], "o", color="orange", ms=6)
    ax.set_xlim(lo[0], hi[0])
    ax.set_ylim(lo[1], hi[1])
    ax.set_aspect("equal")
    ax.axis("off")
    legend = (
        "   edges: yellow proposed, green-dashed accepted, green applied, red rejected"
        if edge_events
        else ""
    )
    ax.set_title(
        f"{title}   t = {t - t0:5.0f} s   (map: reformed ray tracer, white: GSC path, {'blue' if edge_events else 'red'} dots: last {step:.0f} s of scans){legend}",
        color="white",
        fontsize=8,
    )
    fig.tight_layout()
    fig.savefig(frames / f"{n:05d}.png", facecolor="black")
    plt.close(fig)
subprocess.run(
    [
        "ffmpeg",
        "-y",
        "-loglevel",
        "error",
        "-framerate",
        "12",
        "-i",
        str(frames / "%05d.png"),
        "-c:v",
        "libx264",
        "-pix_fmt",
        "yuv420p",
        "-vf",
        "pad=ceil(iw/2)*2:ceil(ih/2)*2",
        str(out),
    ],
    check=True,
)
subprocess.run(["rm", "-rf", str(frames)], check=True)
print("wrote", out, flush=True)
if edge_events:
    print("edge state changes:", counts, flush=True)
