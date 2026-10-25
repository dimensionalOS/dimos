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

"""Sanity check of gsc_plan.py / ivan_plan.py JSONs: per file, how many replans found a path, how far paths start
from the robot and end from the goal; with --png, the last plan before and the first after each method's closure
(GSC: its lap warp; Ivan: the rebuild in the window that moved the map most) side by side.
usage: plan_check.py <plans dir>/<rec> [--png out.png]"""

import argparse
import json
from pathlib import Path

import numpy as np

parser = argparse.ArgumentParser()
parser.add_argument("rec_dir", type=Path)
parser.add_argument("--png", type=Path)
args = parser.parse_args()

pairs = {}
for path in sorted(args.rec_dir.glob("*_plan.json")):
    plan = json.loads(path.read_text())
    states = plan["states"]
    with_path = [s for s in states if s["path"]]
    start_gap = [np.hypot(*np.subtract(s["path"][0], s["robot"])) for s in with_path]
    end_gap = [np.hypot(*np.subtract(s["path"][-1], s["goal"])) for s in with_path]
    print(
        f"{path.name}: {len(states)} states {states[0]['ts']:.1f}..{states[-1]['ts']:.1f} s, "
        f"{len(with_path)} with a path; start-robot gap median {np.median(start_gap or [np.nan]):.2f} max "
        f"{np.max(start_gap or [np.nan]):.2f} m; end-goal gap median {np.median(end_gap or [np.nan]):.2f} max "
        f"{np.max(end_gap or [np.nan]):.2f} m"
    )
    if plan["method"] == "ivan" and plan.get("rebuilds"):
        event = max(plan["rebuilds"], key=lambda r: r["moved_m"])["ts"]
    else:
        warps = [s["ts"] for s in states if s["why"] == "warp" and s["ts"] >= plan["lap_ts"] - 0.5]
        event = warps[0] if warps else plan["lap_ts"]
    before = [s for s in states if s["ts"] < event]
    after = [s for s in states if s["ts"] >= event]
    pairs[plan["method"]] = (
        event,
        before[-1] if before else None,
        after[0] if after else None,
        states,
    )

if args.png:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, axes = plt.subplots(1, len(pairs), figsize=(8 * len(pairs), 7), squeeze=False)
    for ax, (method, (event, before, after, states)) in zip(axes[0], pairs.items(), strict=True):
        robot = np.array([s["robot"] for s in states])
        ax.plot(robot[:, 0], robot[:, 1], ".", color="0.75", ms=2, label="robot (window)")
        for state, color, label in ((before, "tab:blue", "before"), (after, "tab:orange", "after")):
            if state is None:
                continue
            p = np.array(state["path"]).reshape(-1, 2)
            if len(p):
                ax.plot(
                    p[:, 0],
                    p[:, 1],
                    color=color,
                    lw=2,
                    label=f"{label} t={state['ts']:.1f} s ({len(p)} pts)",
                )
            else:
                ax.plot([], [], color=color, label=f"{label} t={state['ts']:.1f} s: no path")
            ax.plot(*state["goal"], "*", color=color, ms=16, mec="k")
            ax.plot(*state["robot"], "o", color=color, ms=8, mec="k")
        ax.set_aspect("equal")
        ax.set_title(f"{args.rec_dir.name} {method}: closure at {event:.1f} s")
        ax.legend(fontsize=8)
    fig.tight_layout()
    args.png.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(args.png, dpi=90)
    print("wrote", args.png)
