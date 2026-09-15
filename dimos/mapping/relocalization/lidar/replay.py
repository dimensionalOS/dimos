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


"""Replay a recording through the lidar relocalizer's decision path, with no planner.

python -m dimos.mapping.relocalization.lidar.tune replay <recording.db> --premap <map>
"""

from __future__ import annotations

import math
from pathlib import Path
from typing import TYPE_CHECKING, NamedTuple

import numpy as np
import typer

from dimos.mapping.ray_tracing.transformer import RayTraceMap, pose_from_tf
from dimos.mapping.relocalization.lidar.module import LidarConfig
from dimos.mapping.relocalization.lidar.relocalize import PRESETS, LidarRelocalizer
from dimos.memory.store.sqlite import SqliteStore
from dimos.memory.tf import StreamTF
from dimos.memory.vis.utils import log_loaded_map, voxel_map_points
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2, register_colormap_annotation
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.utils.data import resolve_named_path

if TYPE_CHECKING:
    from numpy.typing import NDArray

    from dimos.msgs.geometry_msgs.Transform import Transform

TIMELINE = "ts"
MAP_FRAME = "map"
LOADED_MAP_STREAM = "loaded_map"
RECORDED_MAP_COLOR = [255, 120, 120]

_FIELDS = LidarConfig.model_fields


class Attempt(NamedTuple):
    t: float
    fitness: float
    tf: Transform | None


class Replay(NamedTuple):
    attempts: list[Attempt]
    fix: Transform | None
    fix_ts: float
    recorded: Transform | None


def yaw_deg(tf: Transform) -> float:
    return math.degrees(tf.rotation.euler.z)


def fix_error(fix: Transform, recorded: Transform) -> tuple[float, float]:
    """Yaw (deg) and translation (m) between two fixes of the same frame pair."""
    dyaw = (yaw_deg(fix) - yaw_deg(recorded) + 180.0) % 360.0 - 180.0
    return dyaw, (fix.translation - recorded.translation).length()


def place_premap(premap: NDArray[np.float32], fix: Transform) -> NDArray[np.float32]:
    """Premap points moved into the world frame by a ``world -> map`` fix."""
    m = fix.to_matrix()
    placed: NDArray[np.float32] = (premap @ m[:3, :3].T + m[:3, 3]).astype(np.float32)
    return placed


def _recorded_fix(store: SqliteStore, world_frame: str, map_frame: str) -> Transform | None:
    """The first ``world -> map`` edge the live run published, if the recording has one."""
    if "tf" not in store.list_streams():
        return None
    for obs in store.stream("tf", TFMessage).order_by("ts"):
        for tf in obs.data.transforms:
            if (tf.frame_id, tf.child_frame_id) == (world_frame, map_frame):
                return tf
    return None


def _init_recording(name: str, out: Path | None) -> None:
    import rerun as rr
    import rerun.blueprint as rrb

    rr.init("reloc_replay", recording_id=name)
    if out is not None:
        rr.save(str(out))
    else:
        rr.spawn()
    rr.send_blueprint(
        rrb.Blueprint(
            rrb.Horizontal(
                rrb.Spatial3DView(origin="world", name="world"),
                rrb.TimeSeriesView(origin="metrics/reloc", name="reloc"),
                column_shares=[3, 1],
            ),
            collapse_panels=True,
        )
    )
    register_colormap_annotation("turbo")
    rr.log("metrics/reloc/fitness", rr.SeriesLines(names=["fitness"]), static=True)


def replay(
    store: SqliteStore,
    lidar_stream: str,
    premap: PointCloud2,
    preset: str,
    world_frame: str,
    reloc_interval: float,
    min_local_points: int,
    voxel_size: float,
    after_s: float,
    from_time: float | None,
    to_time: float | None,
) -> Replay:
    """Drive the recording through the relocalizer and log the run to rerun."""
    import rerun as rr

    tf = StreamTF.from_store(store)
    if tf is None:
        raise typer.BadParameter("the recording has no tf stream to register clouds from")
    lidar = store.stream(lidar_stream, PointCloud2).order_by("ts")
    if from_time is not None:
        lidar = lidar.from_time(from_time)
    if to_time is not None:
        lidar = lidar.to_time(to_time)
    ray = RayTraceMap(voxel_size=voxel_size, min_health=-1, max_health=5, support_min=4)
    frames = lidar.transform(pose_from_tf(tf, world_frame)).transform(ray)

    relocalizer = LidarRelocalizer(premap.pointcloud, PRESETS[preset])
    recorded = _recorded_fix(store, world_frame, MAP_FRAME)
    premap_pts = premap.points_f32()

    attempts: list[Attempt] = []
    fix: Transform | None = None
    fix_ts = 0.0
    next_attempt = 0.0
    stop_at = math.inf
    t0: float | None = None
    for obs in frames:
        if obs.pose_tuple is None:
            continue
        t0 = obs.ts if t0 is None else t0
        if obs.ts >= stop_at:
            break
        rr.set_time(TIMELINE, timestamp=obs.ts)
        rr.log("world/local_map", voxel_map_points(obs.data.points_f32(), voxel_size))
        if fix is not None or obs.ts < next_attempt or len(obs.data) < min_local_points:
            continue
        next_attempt = obs.ts + reloc_interval
        fix_attempt = relocalizer.attempt(obs.data.pointcloud, world_frame, MAP_FRAME)
        attempts.append(Attempt(obs.ts - t0, fix_attempt.result.fitness, fix_attempt.fix))
        rr.log("metrics/reloc/fitness", rr.Scalars(fix_attempt.result.fitness))
        _print_attempt(attempts[-1], recorded)
        if fix_attempt.fix is None:
            continue
        fix, fix_ts = fix_attempt.fix, obs.ts
        stop_at = obs.ts + after_s
        log_loaded_map(place_premap(premap_pts, fix))
        if recorded is not None:
            rr.log(
                "world/recorded_map",
                rr.Points3D(
                    place_premap(premap_pts, recorded), colors=[RECORDED_MAP_COLOR], radii=0.008
                ),
            )
    return Replay(attempts, fix, fix_ts, recorded)


def write_loaded_map(
    store: SqliteStore, premap: PointCloud2, fix: Transform, ts: float, world_frame: str
) -> bool:
    """Append the placed premap to the recording as the stream plan_rrd seeds from."""
    stream = store.stream(LOADED_MAP_STREAM, PointCloud2)
    if stream.count() > 0:
        print(f"{LOADED_MAP_STREAM} already has {stream.count()} messages, leaving it")
        return False
    placed = PointCloud2.from_numpy(
        place_premap(premap.points_f32(), fix), frame_id=world_frame, timestamp=ts
    )
    stream.append(placed, ts=ts)
    print(f"wrote {len(placed)} placed premap points to {LOADED_MAP_STREAM} at {ts:.3f}")
    return True


def _print_attempt(attempt: Attempt, recorded: Transform | None) -> None:
    if attempt.tf is None:
        print(f"{attempt.t:.1f}s refused fitness={attempt.fitness:.3f}")
        return
    t = attempt.tf.translation
    line = (
        f"{attempt.t:.1f}s fitness={attempt.fitness:.3f} "
        f"t=({t.x:.2f}, {t.y:.2f}, {t.z:.2f}) yaw={yaw_deg(attempt.tf):.1f}deg"
    )
    if recorded is not None:
        dyaw, dt = fix_error(attempt.tf, recorded)
        line += f" vs recorded: yaw {dyaw:+.1f}deg t {dt:.2f}m"
    print(f"{line} ACCEPTED")


def main(
    recording: str = typer.Argument(..., help="Recording .db: bare name (cwd or data/) or path"),
    premap: str = typer.Option(..., "--premap", help="Premap .pc2.lcm: bare name or path"),
    lidar: str = typer.Option("lidar", "--lidar", help="Lidar stream in the recording"),
    world_frame: str = typer.Option("odom", "--world-frame", help="Frame the live map is built in"),
    preset: str = typer.Option("go2-nav", "--preset", help=f"One of {sorted(PRESETS)}"),
    reloc_interval: float = typer.Option(
        _FIELDS["reloc_interval"].default,
        "--reloc-interval",
        help="Seconds of recording between attempts",
    ),
    min_local_points: int = typer.Option(
        _FIELDS["min_local_points"].default,
        "--min-local-points",
        help="Local map points below which an attempt is skipped",
    ),
    voxel_size: float = typer.Option(0.08, "--voxel-size", help="Live map voxel size (m)"),
    after: float = typer.Option(
        10.0, "--after", help="Seconds of live map to keep logging after the fix, for the overlay"
    ),
    out: Path | None = typer.Option(None, "--out", help="Write a .rrd instead of opening rerun"),
    write_loaded_map_stream: bool = typer.Option(
        False,
        "--write-loaded-map",
        help="Append the placed premap to the recording as loaded_map, so plan_rrd seeds from it",
    ),
    from_time: float | None = typer.Option(None, "--from-time", help="Start second (relative)"),
    to_time: float | None = typer.Option(None, "--to-time", help="End second (relative)"),
) -> None:
    """Replay a recording through the relocalizer and compare its fix with the robot's."""
    db_path = resolve_named_path(recording, ".db")
    premap_cloud = PointCloud2.lcm_decode(resolve_named_path(premap, ".pc2.lcm").read_bytes())
    _init_recording(db_path.stem, out)
    print(f"relocalizing {db_path.name} against {premap} ({len(premap_cloud)} points, {preset})")
    store = SqliteStore(path=str(db_path))
    with store:
        result = replay(
            store,
            lidar_stream=lidar,
            premap=premap_cloud,
            preset=preset,
            world_frame=world_frame,
            reloc_interval=reloc_interval,
            min_local_points=min_local_points,
            voxel_size=voxel_size,
            after_s=after,
            from_time=from_time,
            to_time=to_time,
        )
        if result.fix is not None and write_loaded_map_stream:
            write_loaded_map(store, premap_cloud, result.fix, result.fix_ts, world_frame)
    if out is not None:
        print(f"wrote {out}")
    if result.fix is None:
        print(f"no fix accepted in {len(result.attempts)} attempts")
        raise typer.Exit(1)
    if result.recorded is None:
        print("no recorded fix in this recording to compare against")
