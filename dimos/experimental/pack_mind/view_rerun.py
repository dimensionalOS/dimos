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

"""View the PACK MIND A/B coverage race in the DimOS Viewer (Rerun).

Streams both runs into one Rerun recording on a shared "tick" timeline so you can
scrub/playback and watch PACK (shared memory) pull ahead of INDEPENDENT (private).

    python -m dimos.experimental.pack_mind.view_rerun --seed 0
    python -m dimos.experimental.pack_mind.view_rerun --seed 0 --save run.rrd   # no GUI
"""

from __future__ import annotations

import argparse
from typing import Any

import numpy as np
import rerun as rr

from dimos.experimental.pack_mind.render import _run_capture
from dimos.msgs.nav_msgs.OccupancyGrid import CostValues
from dimos.visualization.rerun.init import rerun_init

_WALL = np.array([13, 13, 20], dtype=np.uint8)
_FREE = np.array([245, 245, 245], dtype=np.uint8)
_VISITED = np.array([140, 200, 255], dtype=np.uint8)


def _set_tick(tick: int) -> None:
    # rerun's time API name shifts across versions; support both.
    try:
        rr.set_time_sequence("tick", tick)  # type: ignore[attr-defined]
    except AttributeError:
        rr.set_time("tick", sequence=tick)  # type: ignore[attr-defined]


def _frame_rgb(grid: np.ndarray, visited: np.ndarray) -> np.ndarray:
    h, w = grid.shape
    img = np.empty((h, w, 3), dtype=np.uint8)
    img[:] = _FREE
    img[grid == CostValues.OCCUPIED] = _WALL
    img[visited] = _VISITED
    return np.flipud(img)  # flip so +y points up in the viewer


def _log_run(root: str, sim: Any, frames: list[dict[str, Any]]) -> None:
    grid = sim.world.grid
    h = grid.shape[0]
    world = sim.world
    survivors = sim.survivors

    def to_img_xy(wx: float, wy: float) -> tuple[float, float]:
        gc = world.world_to_grid((wx, wy))
        return (gc.x, (h - 1) - gc.y)  # match the vertical flip

    for fr in frames:
        _set_tick(fr["tick"])
        rr.log(f"{root}/map", rr.Image(_frame_rgb(grid, fr["visited"])))

        spos = [to_img_xy(sx, sy) for sx, sy in survivors]
        scol = [
            [60, 220, 60] if si in fr["found"] else [230, 40, 40] for si in range(len(survivors))
        ]
        rr.log(
            f"{root}/map/survivors",
            rr.Points2D(spos, colors=scol, radii=1.6),
        )

        rpos = [to_img_xy(rx, ry) for rx, ry, _ in fr["robots"]]
        rcol = [[150, 150, 150] if done else [255, 150, 0] for _, _, done in fr["robots"]]
        rr.log(f"{root}/map/robots", rr.Points2D(rpos, colors=rcol, radii=1.3))

        rr.log(f"{root}/stats", rr.TextLog(f"coverage {fr['cov']:.0%}  found {len(fr['found'])}/{len(survivors)}"))


def view(seed: int = 0, max_ticks: int = 4000, save: str | None = None) -> None:
    rerun_init(app_id="pack_mind")
    if save:
        rr.save(save)
    else:
        rr.spawn()  # opens the Rerun (DimOS) viewer window

    psim, pframes = _run_capture(True, seed, max_ticks)
    isim, iframes = _run_capture(False, seed, max_ticks)
    _log_run("pack", psim, pframes)
    _log_run("independent", isim, iframes)

    if save:
        print(f"saved Rerun recording -> {save} (open with: rerun {save})")
    else:
        print("Rerun viewer launched. Scrub the 'tick' timeline; compare pack/ vs independent/.")


def _parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="PACK MIND — view A/B race in Rerun")
    p.add_argument("--seed", type=int, default=0)
    p.add_argument("--max-ticks", type=int, default=4000)
    p.add_argument("--save", default=None, metavar="PATH", help="write .rrd instead of opening GUI")
    return p.parse_args()


if __name__ == "__main__":
    args = _parse_args()
    view(seed=args.seed, max_ticks=args.max_ticks, save=args.save)
