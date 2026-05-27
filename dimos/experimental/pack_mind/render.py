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

"""PACK MIND sim — G3 render: side-by-side A/B coverage race to mp4 (or GIF).

Left = PACK (shared coverage memory), right = INDEPENDENT (private memory). Same
seed, same single deploy point, same survivors — the only difference is shared vs
private memory. The HUD shows coverage %, survivors found, and tick so the gap is
self-evident on screen.

    python -m dimos.experimental.pack_mind.render --out pack_mind_ab.mp4 --seed 0
"""

from __future__ import annotations

import argparse
from typing import Any

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from matplotlib import animation  # noqa: E402

from dimos.experimental.pack_mind.sim import PackSim, build_sim  # noqa: E402
from dimos.msgs.nav_msgs.OccupancyGrid import CostValues  # noqa: E402

_SNAP_EVERY = 15  # capture a frame every N ticks


def _union_visited(sim: PackSim) -> np.ndarray:
    free = sim.world.grid == CostValues.FREE
    union: np.ndarray | None = None
    for rb in sim.robots:
        v = rb.router._visited
        if v is None:
            continue
        union = v.copy() if union is None else (union | v)
    if union is None:
        union = np.zeros_like(free)
    return union & free


def _run_capture(shared: bool, seed: int, max_ticks: int) -> tuple[PackSim, list[dict[str, Any]]]:
    sim = build_sim(shared=shared, seed=seed)
    free_total = sim._free_total
    frames: list[dict[str, Any]] = []

    def snap() -> None:
        vis = _union_visited(sim)
        frames.append(
            {
                "tick": sim.tick_n,
                "visited": vis,
                "robots": [(rb.x, rb.y, rb.done) for rb in sim.robots],
                "found": set(sim._found),
                "cov": float(np.count_nonzero(vis)) / max(1, free_total),
            }
        )

    snap()
    for _ in range(max_ticks):
        sim.step()
        if sim.tick_n % _SNAP_EVERY == 0:
            snap()
        if all(rb.done for rb in sim.robots):
            break
    snap()
    return sim, frames


def render_ab(seed: int = 0, out_path: str = "pack_mind_ab.mp4", max_ticks: int = 4000) -> str:
    psim, pframes = _run_capture(True, seed, max_ticks)
    _, iframes = _run_capture(False, seed, max_ticks)
    n = max(len(pframes), len(iframes))

    world = psim.world
    grid = world.grid
    h, w = grid.shape
    survivors = psim.survivors

    base = np.ones((h, w, 3))
    base[grid == CostValues.OCCUPIED] = [0.05, 0.05, 0.08]

    fig, (axp, axi) = plt.subplots(1, 2, figsize=(12, 6.2))
    fig.suptitle("PACK MIND — same robots, same start, only memory differs", fontsize=13)

    def at(frames: list[dict[str, Any]], i: int) -> dict[str, Any]:
        return frames[min(i, len(frames) - 1)]

    def draw(ax: Any, frame: dict[str, Any], title: str) -> None:
        ax.clear()
        img = base.copy()
        img[frame["visited"]] = [0.55, 0.78, 1.0]
        ax.imshow(img, origin="lower")
        for si, (sx, sy) in enumerate(survivors):
            gc = world.world_to_grid((sx, sy))
            color = "limegreen" if si in frame["found"] else "red"
            ax.plot(gc.x, gc.y, marker="*", color=color, markersize=15, markeredgecolor="k")
        for rx, ry, done in frame["robots"]:
            gc = world.world_to_grid((rx, ry))
            ax.plot(
                gc.x,
                gc.y,
                marker="o",
                color="gray" if done else "orange",
                markersize=9,
                markeredgecolor="k",
            )
        ax.set_title(
            f"{title}\ncoverage {frame['cov']:.0%}   "
            f"found {len(frame['found'])}/{len(survivors)}   t={frame['tick']}",
            fontsize=11,
        )
        ax.set_xticks([])
        ax.set_yticks([])

    def update(i: int) -> list[Any]:
        draw(axp, at(pframes, i), "PACK · shared memory")
        draw(axi, at(iframes, i), "INDEPENDENT · private")
        return []

    anim = animation.FuncAnimation(fig, update, frames=n, interval=80)
    saved = out_path
    try:
        anim.save(out_path, writer="ffmpeg", fps=12, dpi=100)
    except Exception as exc:  # ffmpeg missing → GIF fallback
        saved = out_path.rsplit(".", 1)[0] + ".gif"
        print(f"[render] ffmpeg unavailable ({exc}); writing GIF -> {saved}")
        anim.save(saved, writer="pillow", fps=12)
    plt.close(fig)
    return saved


def query_report(result: Any) -> str:
    """Scripted situational report from the findings pool (the 'memory' end card)."""
    lines = [f"PACK MIND situational report — {result.found}/{result.total_survivors} survivors located:"]
    for f in result.findings:
        lines.append(f"  • survivor at ({f.survivor_xy[0]:.1f}, {f.survivor_xy[1]:.1f}) — found by {f.by} at t={f.tick}")
    lines.append(f"area coverage: {result.coverage:.0%} (mode={result.mode})")
    return "\n".join(lines)


def _parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="PACK MIND A/B render")
    p.add_argument("--out", default="pack_mind_ab.mp4")
    p.add_argument("--seed", type=int, default=0)
    p.add_argument("--max-ticks", type=int, default=4000)
    return p.parse_args()


if __name__ == "__main__":
    args = _parse_args()
    path = render_ab(seed=args.seed, out_path=args.out, max_ticks=args.max_ticks)
    print(f"rendered -> {path}")
