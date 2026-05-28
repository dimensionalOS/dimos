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

"""Watch a pack of dogs search a maze with PACK MIND in the DimOS Viewer (Rerun).

One maze, one shared discovered map. Every dog reveals into it via a raycast
sensor (walls block line of sight) and plans toward frontiers; the shared map
de-conflicts them so the pack fans out instead of re-walking each other's ground.
Scrub the "tick" timeline to watch the fog peel back, and drop ``maze/revealed``
into a time-series view to see coverage climb to 100%.

    python -m dimos.experimental.pack_mind.view_explore_rerun
    python -m dimos.experimental.pack_mind.view_explore_rerun --dogs 3 --seed 0
    python -m dimos.experimental.pack_mind.view_explore_rerun --kill 2 120   # offline dog2 at tick 120
    python -m dimos.experimental.pack_mind.view_explore_rerun --save run.rrd # headless, no GUI
"""

from __future__ import annotations

import argparse
from typing import Any

import numpy as np
import rerun as rr

from dimos.experimental.pack_mind.explore_sim import (
    ExploreSim,
    build_explore,
    build_explore_building,
)
from dimos.visualization.rerun.init import rerun_init

# Per-cell fog palette, indexed by state() code:
# 0 unknown · 1 remembered-free · 2 visible-free · 3 remembered-wall · 4 visible-wall
_PALETTE = np.array(
    [
        [12, 12, 20],  # 0 unknown — the fog
        [64, 80, 110],  # 1 remembered free (out of current line of sight)
        [205, 228, 255],  # 2 visible free (currently sensed)
        [38, 40, 58],  # 3 remembered wall
        [150, 130, 95],  # 4 visible wall
    ],
    dtype=np.uint8,
)
_OFFLINE = [110, 110, 110]
_SNAP_EVERY = 4  # capture a frame every N ticks (keeps playback smooth + .rrd small)


def _set_tick(tick: int) -> None:
    # rerun's time API name shifts across versions; support both.
    try:
        rr.set_time_sequence("tick", tick)  # type: ignore[attr-defined]
    except AttributeError:
        rr.set_time("tick", sequence=tick)  # type: ignore[attr-defined]


def _hex_rgb(hex_color: str) -> list[int]:
    h = hex_color.lstrip("#")
    return [int(h[0:2], 16), int(h[2:4], 16), int(h[4:6], 16)]


def _codes_rgb(codes: np.ndarray) -> np.ndarray:
    """Map an (h, w) array of fog codes to an RGB image, flipped so +y points up."""
    return np.flipud(_PALETTE[codes])


def _build(map_name: str, n_dogs: int, seed: int, target_label: str, shared: bool) -> ExploreSim:
    """Pack on the chosen arena, converging once the object is seen.

    shared=True is PACK MIND (one shared discovered map); shared=False gives each
    dog a private map (the A/B baseline that re-walks each other's ground)."""
    builder = build_explore_building if map_name == "office" else build_explore
    return builder(
        shared=shared, seed=seed, n_dogs=n_dogs, target_label=target_label, converge_on_found=True
    )


def _capture(sim: ExploreSim, max_ticks: int, kill: tuple[int, int] | None) -> list[dict[str, Any]]:
    frames: list[dict[str, Any]] = []

    def snap() -> None:
        st = sim.state()
        codes = np.array(st["cells"], dtype=np.uint8).reshape(st["h"], st["w"])
        frames.append(
            {
                "tick": sim.tick_n,
                "codes": codes,
                "revealed": st["revealed"],
                "dogs": [(d.x, d.y, d.color, d.online) for d in sim.dogs],
                "trails": [list(d.trail) for d in sim.dogs],
                "target": st["target"],
            }
        )

    snap()
    for _ in range(max_ticks):
        if kill is not None and sim.tick_n == kill[1]:
            sim.set_online(kill[0], False)
        sim.step()
        if sim.tick_n % _SNAP_EVERY == 0:
            snap()
        if sim.all_done():
            break
    snap()
    return frames


def _log_run(root: str, metric: str, sim: ExploreSim, frames: list[dict[str, Any]]) -> None:
    h = frames[0]["codes"].shape[0]
    world = sim.world
    label = sim.target_label

    def to_img_xy(wx: float, wy: float) -> tuple[float, float]:
        g = world.world_to_grid((wx, wy))
        return (float(g.x), float((h - 1) - g.y))  # match the vertical flip

    for fr in frames:
        _set_tick(fr["tick"])
        rr.log(f"{root}/fog", rr.Image(_codes_rgb(fr["codes"])))

        pts = [to_img_xy(x, y) for x, y, _, _ in fr["dogs"]]
        cols = [_hex_rgb(color) if online else _OFFLINE for _, _, color, online in fr["dogs"]]
        rr.log(f"{root}/fog/dogs", rr.Points2D(pts, colors=cols, radii=1.6))

        strips, scols = [], []
        for (_, _, color, _), trail in zip(fr["dogs"], fr["trails"]):
            if len(trail) > 1:
                strips.append([to_img_xy(px, py) for px, py in trail])
                scols.append(_hex_rgb(color))
        if strips:
            rr.log(f"{root}/fog/trails", rr.LineStrips2D(strips, colors=scols, radii=0.35))

        tgt = fr["target"]
        if tgt is not None:
            tcol = [[60, 220, 90]] if tgt["found"] else [[235, 40, 40]]
            rr.log(f"{root}/fog/target", rr.Points2D([to_img_xy(tgt["x"], tgt["y"])], colors=tcol, radii=2.4))

        rr.log(metric, rr.Scalars(fr["revealed"]))
        if tgt is not None and tgt["found"]:
            msg = f"FOUND {label} — {tgt['found_by']} @ tick {tgt['found_tick']} · searched {fr['revealed']:.0%}"
        else:
            msg = f"searching for {label} … searched {fr['revealed']:.0%} · tick {fr['tick']}"
        rr.log(f"{root}/stats", rr.TextLog(msg))


def view(
    map_name: str = "maze",
    n_dogs: int = 3,
    seed: int = 0,
    target_label: str = "red object",
    max_ticks: int = 4000,
    kill: tuple[int, int] | None = None,
    save: str | None = None,
    shared: bool = True,
) -> None:
    mode = "shared" if shared else "independent"
    rerun_init(app_id=f"pack_mind_explore_{mode}")
    if save:
        rr.save(save)
    else:
        rr.spawn()  # opens the DimOS Viewer (Rerun) window

    sim = _build(map_name, n_dogs, seed, target_label, shared)
    frames = _capture(sim, max_ticks, kill)
    _log_run("maze", "maze/revealed", sim, frames)

    tgt = sim.state()["target"]
    if tgt is not None and tgt["found"]:
        print(
            f"{n_dogs} dogs searched the {map_name}; {tgt['found_by']} saw the {target_label} "
            f"at tick {tgt['found_tick']} (searched {sim.revealed_frac():.0%})."
        )
    else:
        print(
            f"{n_dogs} dogs searched {sim.revealed_frac():.0%} of the {map_name} in "
            f"{sim.tick_n} ticks; {target_label} not seen."
        )
    if save:
        print(f"saved Rerun recording -> {save} (open with: rerun {save})")
    else:
        print("DimOS Viewer launched. Scrub the 'tick' timeline to watch the pack search.")


def _parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="PACK MIND — a pack searches for an object, viewed in Rerun")
    p.add_argument("--map", choices=("maze", "office"), default="maze", help="search arena")
    p.add_argument("--dogs", type=int, default=3, help="number of dogs in the pack")
    p.add_argument("--target", default="red object", help="object descriptor to search for")
    p.add_argument("--seed", type=int, default=0)
    p.add_argument("--max-ticks", type=int, default=4000)
    p.add_argument(
        "--kill",
        nargs=2,
        type=int,
        default=None,
        metavar=("DOG", "TICK"),
        help="take DOG offline at TICK (resilience beat); shared keeps its searched area",
    )
    p.add_argument("--save", default=None, metavar="PATH", help="write .rrd instead of opening GUI")
    p.add_argument(
        "--independent",
        action="store_true",
        help="give each dog a private map (A/B baseline); default is PACK MIND shared memory",
    )
    return p.parse_args()


if __name__ == "__main__":
    args = _parse_args()
    kill = (args.kill[0], args.kill[1]) if args.kill else None
    view(
        map_name=args.map,
        n_dogs=args.dogs,
        seed=args.seed,
        target_label=args.target,
        max_ticks=args.max_ticks,
        kill=kill,
        save=args.save,
        shared=not args.independent,
    )
