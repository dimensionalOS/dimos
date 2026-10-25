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

"""Pick loop edges whose state changes across frames of a render_replay.py video and save those frames.
usage: edge_evidence.py <replay.db> <video.mp4> <step_s> <out_dir>"""

from collections import Counter
import json
from pathlib import Path
import subprocess
import sys

import numpy as np

from dimos.memory.store.sqlite import SqliteStore

db, video, step, out = Path(sys.argv[1]), Path(sys.argv[2]), float(sys.argv[3]), Path(sys.argv[4])
out.mkdir(parents=True, exist_ok=True)
NAMES = ["proposed", "gate_rejected", "committed", "vote", "accepted", "rejected", "applied"]
life: dict[tuple[int, int], list] = {}
with SqliteStore(path=str(db)) as store:
    t0 = next(iter(store.streams.lidar_live.order_by("ts"))).ts
    for o in store.streams.loop_edges.order_by("ts"):
        ids = np.asarray(o.data.offset_times_u32()).reshape(-1, 2)
        states = np.asarray(o.data.tags_u8())[::2]
        for (a, b), s in zip(ids, states, strict=False):
            life.setdefault((int(a), int(b)), []).append((o.ts, NAMES[s]))
counts = Counter(s for events in life.values() for _, s in events)


def frame(t):
    return (
        int(np.ceil((t - t0) / step)) - 1
    )  # first frame whose time >= t (render_replay frames at t0 + step*(n+1))


picks = {}
for want in ("accepted", "rejected"):
    best = None
    for edge, events in life.items():
        committed = [t for t, s in events if s == "committed"]
        verdict = [t for t, s in events if s == want]
        if committed and verdict and frame(verdict[0]) - frame(committed[0]) >= 2:
            gap = verdict[0] - committed[0]
            if best is None or gap > best[0]:
                best = (gap, edge, committed[0], verdict[0])
    if best:
        gap, edge, t_proposed, t_verdict = best
        last = (
            int(
                subprocess.run(
                    [
                        "ffprobe",
                        "-v",
                        "error",
                        "-count_frames",
                        "-select_streams",
                        "v:0",
                        "-show_entries",
                        "stream=nb_read_frames",
                        "-of",
                        "csv=p=0",
                        str(video),
                    ],
                    capture_output=True,
                    text=True,
                ).stdout.strip()
                or 1
            )
            - 1
        )
        frames = {
            "proposed": min(frame(t_proposed) + 1, last),
            want: min(frame(t_verdict) + 1, last),
        }
        for name, n in frames.items():
            subprocess.run(
                [
                    "ffmpeg",
                    "-y",
                    "-loglevel",
                    "error",
                    "-i",
                    str(video),
                    "-vf",
                    f"select=eq(n\\,{n})",
                    "-vframes",
                    "1",
                    str(out / f"edge_{edge[0]}_{edge[1]}_{want}_{name}_frame{n}.png"),
                ],
                check=True,
            )
        picks[want] = {
            "edge": edge,
            "proposed_at_s": t_proposed - t0,
            f"{want}_at_s": t_verdict - t0,
            "frames": frames,
            "states": [s for _, s in life[edge] if s != "vote"],
        }
summary = {"events": dict(counts), "edges": len(life), "picks": picks}
(out / "evidence.json").write_text(json.dumps(summary, indent=1, default=str))
print(json.dumps(summary, indent=1, default=str))
