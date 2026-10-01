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

"""Render a relocalization replay seen from a camera gliding along picked waypoints.

python -m dimos.navigation.animation.flythrough [recording.db] [--premap <map>] --out fly.rrd

The camera follows a spline through ``waypoints.json`` (premap frame, from
``animation-waypoints``) over the replayed span and always looks at the lidar.
"""

from __future__ import annotations

import json
import math
from pathlib import Path

import numpy as np
from numpy.typing import NDArray
import typer

from dimos.mapping.relocalization.lidar.module import LidarConfig
from dimos.mapping.relocalization.lidar.replay import POINT_RADIUS, TIMELINE, replay
from dimos.memory.store.sqlite import SqliteStore
from dimos.memory.tf import StreamTF
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2, register_colormap_annotation
from dimos.navigation.animation.waypoints import WAYPOINTS_FILE, camera_curve
from dimos.robot.unitree.go2 import nav_3d_config
from dimos.utils.data import resolve_named_path

CAMERA = "world/camera"
WIDTH, HEIGHT = 1920, 1080
FPS = 120.0
# Seconds the aim point is averaged over, so lidar pose jitter doesn't shake the camera.
AIM_SMOOTH_S = 0.5
TF_TOLERANCE_S = 0.2


def distance_profile(
    times: NDArray[np.float64], length: float, slow_m: float, slow_s: float, ease: float
) -> NDArray[np.float64]:
    """Metres flown at each time: ``slow_m`` in the first ``slow_s`` s with speed growing as
    t**ease, then a linear ramp over half the remaining time to a cruise that lands on ``length``.
    """
    t = times - times[0]
    rest_s = t[-1] - slow_s
    if rest_s <= 0 or length <= slow_m:
        raise ValueError(f"{slow_m} m in {slow_s} s leaves nothing for the rest of the flight")
    v_slow = (ease + 1) * slow_m / slow_s
    ramp_s = rest_s / 2
    v_cruise = ((length - slow_m) / ramp_s - v_slow / 2) / 1.5
    # Integrate speed: piecewise, all continuous.
    speed = np.where(
        t < slow_s,
        v_slow * (np.clip(t, 0, None) / slow_s) ** ease,
        np.where(
            t < slow_s + ramp_s,
            v_slow + (v_cruise - v_slow) * (t - slow_s) / ramp_s,
            v_cruise,
        ),
    )
    dist = np.r_[0.0, np.cumsum((speed[1:] + speed[:-1]) / 2 * np.diff(t))]
    return np.asarray(dist * length / dist[-1])


def smoothed_targets(
    tf: StreamTF, sensor_frame: str, world_frame: str, times: NDArray[np.float64]
) -> NDArray[np.float64]:
    """The lidar's position at each time, averaged over ``AIM_SMOOTH_S``.

    Past the recording's end it holds where the lidar stopped, so the camera flies on facing it.
    """
    from scipy.ndimage import uniform_filter1d

    found: list[NDArray[np.float64] | None] = []
    for t in times.tolist():
        sensor = tf.get(
            world_frame, sensor_frame, time_point=t, time_tolerance=TF_TOLERANCE_S, warn=False
        )
        found.append(None if sensor is None else np.array(sensor.translation.to_tuple()))
    first = next(p for p in found if p is not None)
    held = []
    for p in found:
        first = p if p is not None else first
        held.append(first)
    return np.asarray(
        uniform_filter1d(np.array(held), int(AIM_SMOOTH_S * FPS), axis=0, mode="nearest")
    )


def look_at(eye: NDArray[np.float64], target: NDArray[np.float64]) -> NDArray[np.float64]:
    """Rotation whose columns are the camera's right, down, forward (rerun's RDF) in world."""
    forward = target - eye
    forward /= np.linalg.norm(forward)
    right = np.cross(forward, [0.0, 0.0, 1.0])
    right /= np.linalg.norm(right)
    return np.column_stack([right, np.cross(forward, right), forward])


def _init_recording(name: str, out: Path) -> None:
    import rerun as rr
    import rerun.blueprint as rrb

    rr.init("reloc_flythrough", recording_id=name)
    rr.save(str(out))
    rr.send_blueprint(
        rrb.Blueprint(
            rrb.Spatial2DView(origin=CAMERA, contents=["world/**"], name="flythrough"),
            collapse_panels=True,
        )
    )
    register_colormap_annotation("turbo")


def log_near_premap(
    premap: PointCloud2, center: NDArray[np.float64], near_m: float, fix_ts: float, reveal_ts: float
) -> None:
    """Only the premap within ``near_m`` of ``center`` until ``reveal_ts``, then all of it.

    Relogged at the fix's own stamp, the later row wins over the full premap the replay logged.
    """
    import rerun as rr

    points = premap.points_f32()
    near = points[np.linalg.norm(points[:, :2] - center[:2], axis=1) < near_m]
    rr.set_time(TIMELINE, timestamp=fix_ts)
    near_cloud = PointCloud2.from_numpy(near, timestamp=fix_ts)
    rr.log("world/loaded_map", near_cloud.to_rerun(mode="points", ui_radius=POINT_RADIUS))
    rr.set_time(TIMELINE, timestamp=reveal_ts)
    rr.log("world/loaded_map", premap.to_rerun(mode="points", ui_radius=POINT_RADIUS))


def log_camera(
    tf: StreamTF,
    sensor_frame: str,
    world_frame: str,
    waypoints: NDArray[np.float64],
    t0: float,
    t1: float,
    fov_deg: float,
    slow: tuple[float, float],
    ease: float,
) -> None:
    import rerun as rr

    times = np.arange(t0, t1, 1.0 / FPS, dtype=np.float64)
    dense, arc = camera_curve(waypoints)
    dist = distance_profile(times, arc[-1], *slow, ease)
    path = np.column_stack([np.interp(dist, arc, dense[:, i]) for i in range(3)])
    focal = WIDTH / 2 / math.tan(math.radians(fov_deg) / 2)
    rr.log(CAMERA, rr.Pinhole(focal_length=focal, width=WIDTH, height=HEIGHT), static=True)
    targets = smoothed_targets(tf, sensor_frame, world_frame, times)
    for t, eye, target in zip(times.tolist(), path, targets, strict=True):
        rr.set_time(TIMELINE, timestamp=t)
        rr.log(CAMERA, rr.Transform3D(translation=eye, mat3x3=look_at(eye, target)))


def main(
    recording: str = typer.Argument(
        "mid360_raycast_door", help="Recording .db: bare name (cwd or data/, LFS) or path"
    ),
    premap: str = typer.Option(
        "recording_go2_mid360_2026-05-29_4-45pm-PST_corrected",
        "--premap",
        help="Premap .pc2.lcm: bare name or path",
    ),
    waypoints_file: Path = typer.Option(WAYPOINTS_FILE, "--waypoints"),
    lidar: str = typer.Option("lidar", "--lidar", help="Lidar stream in the recording"),
    world_frame: str = typer.Option("odom", "--world-frame"),
    from_time: float = typer.Option(0.0, "--from-time", help="Seconds of recording to skip"),
    to_time: float = typer.Option(60.0, "--to-time", help="Seconds of recording to render"),
    duration: float = typer.Option(
        60.0, "--duration", help="Seconds the camera flies; may outlast the recording"
    ),
    fov: float = typer.Option(70.0, "--fov", help="Horizontal field of view, degrees"),
    slow_m: float = typer.Option(30.0, "--slow-m", help="Metres covered by the slow start"),
    slow_s: float = typer.Option(30.0, "--slow-s", help="Seconds the slow start lasts"),
    ease: float = typer.Option(1.5, "--ease", help="Slow-start speed grows as t**ease"),
    scan: bool = typer.Option(False, "--scan/--no-scan", help="Overlay each raw lidar scan"),
    near_m: float = typer.Option(
        8.0, "--near-m", help="Premap shown only this close to the lidar until --reveal-s; 0 off"
    ),
    reveal_s: float = typer.Option(
        25.0, "--reveal-s", help="Seconds into the flight the full premap appears"
    ),
    out: Path = typer.Option(..., "--out", help=".rrd to write"),
) -> None:
    db_path = resolve_named_path(recording, ".db")
    premap_cloud = PointCloud2.lcm_decode(resolve_named_path(premap, ".pc2.lcm").read_bytes())
    _init_recording(db_path.stem, out)
    with SqliteStore(path=str(db_path)) as store:
        result = replay(
            store,
            lidar_stream=lidar,
            premap=premap_cloud,
            preset="go2-nav",
            world_frame=world_frame,
            reloc_interval=LidarConfig.model_fields["reloc_interval"].default,
            min_local_points=LidarConfig.model_fields["min_local_points"].default,
            voxel_size=nav_3d_config.voxel_size,
            fine=True,
            scan=scan,
            after_s=math.inf,
            from_time=from_time,
            to_time=to_time,
        )
        if result.fix is None:
            print("no fix, so the waypoints can't be placed")
            raise typer.Exit(1)
        # Waypoints were picked on the premap; the fix carries them into the live frame.
        picked = np.array(json.loads(waypoints_file.read_text()), dtype=np.float32)
        placed = PointCloud2.from_numpy(picked, timestamp=0.0).transform(result.fix)
        first = next(iter(store.stream(lidar, PointCloud2).order_by("ts")))
        tf = StreamTF.from_store(store)
        assert tf is not None
        reveal_ts = first.ts + from_time + reveal_s
        if near_m > 0 and result.fix_ts < reveal_ts:
            at_fix = tf.get(world_frame, first.data.frame_id, time_point=result.fix_ts)
            assert at_fix is not None
            log_near_premap(
                premap_cloud.transform(result.fix),
                np.array(at_fix.translation.to_tuple()),
                near_m,
                result.fix_ts,
                reveal_ts,
            )
        log_camera(
            tf,
            first.data.frame_id,
            world_frame,
            placed.points_f32().astype(np.float64),
            first.ts + from_time,
            first.ts + from_time + duration,
            fov,
            (slow_m, slow_s),
            ease,
        )
    print(f"wrote {out}")


if __name__ == "__main__":
    typer.run(main)
