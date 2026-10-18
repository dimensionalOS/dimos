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

"""Ivan's live module (PGOVoxelMapper's PGOMap at its defaults) fed a recording at its own rate.

Each scan arrives at its recorded time. The module works one frame at a time and, like its
`backpressure` subscription, keeps only the newest scan that arrived meanwhile; the rest are dropped.
Per frame it does what PGOVoxelMapper._on_lidar does: add the frame, then build the global map on a
rebuild or every emit_every=5 frames. Scored by bench.score into summary["ivan_live"].

usage: ivan_live.py <name> [<name> ...]   (reads $EVAL_LOOP_OUT/<name>.src.db)
"""

import json
import os
import queue
import subprocess
import sys
import threading
import time

import bench
import numpy as np
from paths import OUT, import_ivan

from dimos.memory.store.sqlite import SqliteStore

_module, _, _pgo_map = import_ivan()
RSS_SAMPLE_EVERY = 300  # frames, as pgo-eval samples its own


def resident_megabytes() -> float:
    """This process's resident memory in MB, via `ps` like pgo-eval's."""
    output = subprocess.run(["ps", "-o", "rss=", "-p", str(os.getpid())], capture_output=True)
    return float(output.stdout.decode().strip()) / 1024.0


def replay(db) -> dict:
    cfg = _module.PGOVoxelMapperConfig()
    gmap = _pgo_map.PGOMap(
        voxel_size=cfg.voxel_size,
        frame_id=cfg.frame_id,
        rebuild_cooldown_s=cfg.rebuild_cooldown_s,
        pgo=cfg.pgo,
    )
    latency: list[list[float]] = []
    rss_samples: list[list[float]] = []
    dropped = frames = 0

    def work(obs, arrival: float) -> None:
        nonlocal frames
        gmap.add(obs.data, obs.pose_stamped)
        frames += 1
        if frames % cfg.emit_every == 0:
            gmap.global_map()
        latency.append([obs.ts, (time.perf_counter() - arrival) * 1e3])
        if frames % RSS_SAMPLE_EVERY == 1:
            rss_samples.append([obs.ts, resident_megabytes()])

    # read and decode on another thread, as the live module receives scans on its transport's
    scans: queue.Queue = queue.Queue(maxsize=256)

    def read() -> None:
        with SqliteStore(path=str(db)) as store:
            for obs in store.streams.lidar.order_by("ts"):
                obs.data.points_f32()  # decode here, not on the module's thread
                scans.put(obs)
        scans.put(None)

    threading.Thread(target=read, daemon=True).start()
    started = time.perf_counter()
    wall0 = ts0 = None
    newest = None  # (scan, arrival): the newest scan that arrived while the module was busy
    for obs in iter(scans.get, None):
        if wall0 is None:
            wall0, ts0 = time.perf_counter(), obs.ts
        arrival = wall0 + (obs.ts - ts0)
        if arrival > time.perf_counter() and newest is not None:
            work(*newest)  # the module was free again before this scan arrived
            newest = None
        if arrival <= time.perf_counter():
            if newest is not None:
                dropped += 1  # backpressure keeps only the newest waiting scan
            newest = (obs, arrival)
            continue
        # sleep in slices: macOS stretches a background process's sleep up to ~7x, so one long sleep
        # would deliver the scan late
        while (remaining := arrival - time.perf_counter()) > 0:
            time.sleep(max(remaining / 8, 0.0005))
        work(obs, arrival)
    if newest is not None:
        work(*newest)
    runtime = time.perf_counter() - started
    graph = gmap.graph()
    rows_raw, rows_opt = [], []
    for kf in graph.keyframes:
        for rows, tf in ((rows_raw, kf.local), (rows_opt, kf.optimized)):
            t, r = tf.translation, tf.rotation
            rows.append([kf.ts, t.x, t.y, t.z, r.x, r.y, r.z, r.w])
    result = {
        "k_raw": np.asarray(rows_raw),
        "k_opt": np.asarray(rows_opt),
        "loops": len(graph.loops),
        "runtime": runtime,
        "dropped": dropped,
        "latency": latency,
        "rss_samples": rss_samples,
    }
    gmap.dispose()
    return result


def main() -> None:
    for name in sys.argv[1:]:
        summary_path = OUT / f"{name}.summary.json"
        row = json.loads(summary_path.read_text())
        source = bench.source_for(name)
        times, poses, clouds = bench.load(source)
        markers = (
            bench.markers_for(name, times, poses)[0]
            if row.get("tag_spread_raw_m") is not None
            else None
        )
        result = replay(source)
        maps: dict = {}
        row["ivan_live"] = bench.score(
            result["k_raw"],
            result["k_opt"],
            result["loops"],
            result["runtime"],
            times,
            poses,
            clouds,
            markers,
            maps,
            "ivan_live",
        )
        latency_ms = np.asarray(result["latency"])[:, 1]
        row["ivan_live"].update(
            scans_dropped=result["dropped"],
            latency_ms_median=float(np.median(latency_ms)),
            latency_ms_p99=float(np.percentile(latency_ms, 99)),
            replay="live",
        )
        (OUT / f"{name}.ivan_live.latency.json").write_text(
            json.dumps({"latency": result["latency"], "rss_samples": result["rss_samples"]})
        )
        np.save(OUT / f"{name}.ivan_live_path.npy", maps["ivan_live_path"])
        summary_path.write_text(json.dumps(row, indent=2))
        print(
            name,
            "ivan_live",
            {k: row["ivan_live"][k] for k in ("loops", "voxels", "end_gap_m", "scans_dropped")},
            flush=True,
        )


if __name__ == "__main__":
    main()
