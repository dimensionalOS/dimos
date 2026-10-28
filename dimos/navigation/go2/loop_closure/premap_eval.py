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

"""Replay a recording onto a premap at full speed, no pipeline, and score how well they agree.

The relocalizer runs inline, paced as on the robot: the next attempt waits for as much
recording time as the last one took to run::

    uv run python -m dimos.navigation.go2.loop_closure.premap_eval \\
        go2_hongkong_office --premap go2_hongkong_office_first_half --seek 280
"""

from __future__ import annotations

from dataclasses import dataclass
import math
import time

import numpy as np
from scipy.spatial import cKDTree
import typer

from dimos.memory.cli.dataset import open_dataset
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.navigation.go2.loop_closure.pgo_map import PGOMap
from dimos.navigation.go2.loop_closure.premap import Premap
from dimos.navigation.go2.loop_closure.relocalizer import PremapMatcher, attempt_fix

# what counts as the same surface, and how far out the premap still overlaps the live map
SURFACE_M = 0.10
OVERLAP_M = 1.0


@dataclass
class Report:
    frames: int
    keyframes: int
    premap_keyframes: int
    attempts: int
    fixes: int
    attached_at_s: float | None
    attaches: int
    premap_loops: int
    world_to_map: Transform | None
    agree: float  # live points on a premap surface, of those that overlap it
    median_m: float
    wall_s: float


def agreement(live: PointCloud2, premap: PointCloud2) -> tuple[float, float]:
    """Of the live points within OVERLAP_M of the premap, the share within SURFACE_M, and the median."""
    d, _ = cKDTree(premap.points_f32()).query(live.points_f32(), distance_upper_bound=OVERLAP_M)
    d = d[np.isfinite(d)]
    if not len(d):
        return 0.0, math.inf
    return float(np.mean(d <= SURFACE_M)), float(np.median(d))


def run(
    recording: str,
    premap_name: str,
    *,
    seek: float = 0.0,
    duration: float | None = None,
    wrong_first: bool = False,
    reloc_every_s: float = 2.0,
    recheck_s: float = 30.0,
    save: str | None = None,
) -> Report:
    premap = Premap.load(premap_name)
    premap_cloud = premap.cloud(0.05)
    world_map = PGOMap(premap=premap)
    matcher = PremapMatcher(premap_cloud)
    lidar = open_dataset(recording).stream("lidar", PointCloud2).from_time(seek or None)
    lidar = lidar.to_time(duration)

    wrong = Transform.from_matrix(
        np.array(
            [[0.0, -1.0, 0.0, 5.0], [1.0, 0.0, 0.0, -3.0], [0.0, 0.0, 1.0, 0.0], [0, 0, 0, 1]]
        ),
        frame_id="world",
        child_frame_id="map",
    )
    t_start = time.perf_counter()
    first_ts: float | None = None
    next_attempt = -math.inf
    frames = attempts = fixes = 0
    attached_at: float | None = None
    for obs in lidar:
        cloud = obs.data
        cloud.ts = obs.ts  # old recordings carry Unitree's stale lidar stamps
        first_ts = obs.ts if first_ts is None else first_ts
        world_map.add(cloud, obs.pose)
        frames += 1
        if attached_at is None and world_map.pgo.attached:
            attached_at = obs.ts - first_ts
        if obs.ts < next_attempt:
            continue
        window = world_map.recent_window(30.0)
        if window is None:
            continue
        attempts += 1
        t0 = time.perf_counter()
        found: tuple[Transform, float, float | None] | None
        if wrong_first and attempts <= 3:
            # scored as a match would be, so only a better real fix overturns it
            wrong_fix = Transform(
                translation=wrong.translation, rotation=wrong.rotation, ts=window.ts
            )
            found = (wrong_fix, 0.9, None)
        else:
            found = attempt_fix(matcher, window, world_map.pgo.world_to_map())
        if found is not None:
            fixes += 1
            world_map.pgo.relocalize(*found)
        took = time.perf_counter() - t0
        next_attempt = obs.ts + max(took, recheck_s if world_map.pgo.settled else reloc_every_s)

    placement = world_map.pgo.world_to_map()
    agree, median = (0.0, math.inf)
    if placement is not None:
        live = world_map.live_map()
        agree, median = agreement(live, premap_cloud.transform(placement))
    report = Report(
        frames=frames,
        keyframes=world_map.pgo.n_keyframes,
        premap_keyframes=len(premap),
        attempts=attempts,
        fixes=fixes,
        attached_at_s=attached_at,
        attaches=world_map.pgo.n_attaches,
        premap_loops=world_map.pgo.n_premap_loops,
        world_to_map=placement,
        agree=agree,
        median_m=median,
        wall_s=time.perf_counter() - t_start,
    )
    joint = world_map.joint_premap() if save else None
    if save and joint is not None:
        joint.save(save)
        print(f"saved {save} keyframes={len(joint)} edges={len(joint.edges)}")
    elif save:
        print(f"not saved: relocalization never settled, {save} left alone")
    world_map.dispose()
    return report


def main(
    recording: str = typer.Argument(..., help="Recording .db: stem, path, or LFS name"),
    premap: str = typer.Option(..., "--premap", help="Premap .map.db: stem, path, or LFS name"),
    seek: float = typer.Option(0.0, "--seek", help="Skip the first N seconds"),
    duration: float | None = typer.Option(None, "--duration", help="Use only N seconds"),
    wrong_first: bool = typer.Option(
        False, "--wrong-first", help="Feed three wrong fixes first, to watch it change its mind"
    ),
    save: str | None = typer.Option(
        None, "--save", help="Write the run joined to the premap as a new .map.db"
    ),
) -> None:
    r = run(recording, premap, seek=seek, duration=duration, wrong_first=wrong_first, save=save)
    print(
        f"frames={r.frames} keyframes={r.keyframes} premap_keyframes={r.premap_keyframes} "
        f"attempts={r.attempts} fixes={r.fixes} attaches={r.attaches} "
        f"premap_loops={r.premap_loops} attached_at={r.attached_at_s} wall={r.wall_s:.0f}s"
    )
    if r.world_to_map is None:
        print("never attached")
        raise typer.Exit(1)
    t = r.world_to_map.translation
    yaw = math.degrees(r.world_to_map.rotation.euler.z)
    print(f"world -> map t=({t.x:.2f}, {t.y:.2f}, {t.z:.2f}) yaw={yaw:.1f}deg")
    print(
        f"live map on a premap surface: {r.agree:.1%} within {SURFACE_M} m, "
        f"median {r.median_m * 100:.1f} cm"
    )


if __name__ == "__main__":
    typer.run(main)
