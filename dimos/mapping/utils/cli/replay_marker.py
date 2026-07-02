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

"""Temporary: dump apriltag detector replay (.rrd) for reliability investigation.

Walks a recorded SQLite dataset and writes an rrd containing:
- per-image: camera pose + pinhole + image data
- per-lidar:  base_link pose (a moving axis)
- per detection: marker box in world frame, at the detection timestamp

Throwaway script next to ``map.py``; remove once the apriltag reliability work
lands.
"""

from __future__ import annotations

from pathlib import Path
import subprocess
from typing import TYPE_CHECKING, Any

import rerun as rr
import typer

# Heavy dimos imports (memory2/perception → torch, scipy, cv2) are deferred into
# main() so that `dimos map --help` stays fast. See test_cli_startup.py and the
# same pattern in dimos/mapping/utils/cli/map.py.
if TYPE_CHECKING:
    from dimos.memory2.stream import Stream

TIMELINE = "ts"


def main(
    dataset: str = typer.Argument(..., help="Dataset .db: bare name (cwd or data/) or path"),
    out: Path | None = typer.Option(
        None, "--out", help="Output .rrd path (default: ./<dataset>.rrd)"
    ),
    no_gui: bool = typer.Option(False, "--no-gui", help="Don't launch rerun on the result"),
    seek: float = typer.Option(0.0, "--seek", help="Skip the first N seconds of the recording"),
    duration: float | None = typer.Option(
        None, "--duration", help="Use only N seconds from --seek (default: to the end)"
    ),
    marker_size: float = typer.Option(0.1, "--marker-size", help="Marker edge length (m)"),
    marker_max_speed: float = typer.Option(
        0.5, "--marker-max-speed", help="Detection speed gate (m/s); 0 disables"
    ),
    marker_max_rot_rate: float = typer.Option(
        50.0, "--marker-max-rot-rate", help="Detection rot gate (deg/s); 0 disables"
    ),
    quality_window: float = typer.Option(
        0.1, "--quality-window", help="Sharpest-frame window for detection (s)"
    ),
    smoothing_window: float = typer.Option(
        7.5, "--smoothing-window", help="Buffer window for averaged-track pass (s); 0 disables"
    ),
) -> None:
    """Dump an AprilTag detection replay to .rrd and open it in rerun."""
    from dimos.memory2.store.sqlite import SqliteStore
    from dimos.memory2.transform import QualityWindow, SpeedLimit
    from dimos.memory2.vis.color import Color
    from dimos.msgs.sensor_msgs.Image import Image
    from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
    from dimos.perception.fiducial.marker_transformer import DetectMarkers
    from dimos.robot.unitree.go2.config import camera_info_static
    from dimos.utils.data import resolve_named_path

    db_path = resolve_named_path(dataset, ".db")
    if out is None:
        out = Path.cwd() / f"{db_path.stem}.rrd"
    cam_info = camera_info_static()

    rr.init("dimos markers", recording_id=db_path.stem)
    rr.save(str(out))

    # Static pinhole on the camera entity; per-frame Transform3D goes on the
    # same entity. Image is the child so it projects through the pinhole.
    pinhole = cam_info.to_rerun()
    assert not isinstance(pinhole, list)
    rr.log("world/camera", pinhole, static=True)

    store = SqliteStore(path=str(db_path))
    with store:
        color_image = store.stream("color_image", Image).from_time(seek or None).to_time(duration)
        lidar = store.stream("lidar", PointCloud2).from_time(seek or None).to_time(duration)

        # Pass 1: robot base pose over time (from lidar.pose).
        for lidar_obs in lidar:
            if lidar_obs.pose_tuple is None:
                continue
            rr.set_time(TIMELINE, timestamp=lidar_obs.ts)
            x, y, z, qx, qy, qz, qw = lidar_obs.pose_tuple
            rr.log(
                "world/robot",
                rr.Transform3D(
                    translation=[x, y, z], quaternion=rr.Quaternion(xyzw=[qx, qy, qz, qw])
                ),
            )

        # Pass 2: camera pose + image per color_image frame.
        n_img = color_image.count()
        for i, img_obs in enumerate(color_image):
            rr.set_time(TIMELINE, timestamp=img_obs.ts)
            if img_obs.pose_tuple is not None:
                x, y, z, qx, qy, qz, qw = img_obs.pose_tuple
                rr.log(
                    "world/camera",
                    rr.Transform3D(
                        translation=[x, y, z], quaternion=rr.Quaternion(xyzw=[qx, qy, qz, qw])
                    ),
                )
            rr.log("world/camera/image", img_obs.data.to_rerun())
            # Clear any prior detection box at this ts; pass 3 will overwrite
            # this with the actual detection iff one fires for this frame.
            rr.log(
                "world/camera/image/detections",
                rr.Boxes2D(array=[], array_format=rr.Box2DFormat.XYWH),
            )
            if (i + 1) % 50 == 0 or i + 1 == n_img:
                print(f"images: {i + 1}/{n_img}")

        # Pass 3: marker detections (filtered same way as `dimos map`).
        xf = DetectMarkers(camera_info=cam_info, marker_length_m=marker_size)
        pipeline: Stream[Any] = color_image.transform(
            QualityWindow(lambda img: img.sharpness, window=quality_window)
        )
        if marker_max_speed > 0:
            pipeline = pipeline.transform(
                SpeedLimit(
                    max_mps=marker_max_speed,
                    max_dps=marker_max_rot_rate if marker_max_rot_rate > 0 else None,
                )
            )
        # Each 3D detection gets its own entity path so it persists from its
        # ts onward — earlier detections stay visible as new ones appear.
        # Color by ts (turbo) across the recording's full image-time range.
        ts_min = color_image.first().ts
        ts_max = color_image.last().ts
        ts_span = ts_max - ts_min if ts_max > ts_min else 1.0
        n_det = 0
        for det_obs in pipeline.transform(xf):
            d = det_obs.data
            rr.set_time(TIMELINE, timestamp=det_obs.ts)
            color = Color.from_cmap("turbo", (det_obs.ts - ts_min) / ts_span).rgb_u8()
            rr.log(
                f"world/markers/det_{n_det:05d}",
                rr.Boxes3D(
                    centers=[(d.center.x, d.center.y, d.center.z)],
                    half_sizes=[(marker_size / 2, marker_size / 2, 0.005)],
                    quaternions=[
                        rr.Quaternion(
                            xyzw=[
                                d.orientation.x,
                                d.orientation.y,
                                d.orientation.z,
                                d.orientation.w,
                            ]
                        )
                    ],
                    colors=[color],
                    fill_mode=rr.components.FillMode.Solid,
                    labels=[f"id={d.marker_id}"],
                ),
            )
            n_det += 1
            xs = d.corners_px[:, 0]
            ys = d.corners_px[:, 1]
            x1, y1 = float(xs.min()), float(ys.min())
            x2, y2 = float(xs.max()), float(ys.max())
            rr.log(
                "world/camera/image/detections",
                rr.Boxes2D(
                    array=[[x1, y1, x2 - x1, y2 - y1]],
                    array_format=rr.Box2DFormat.XYWH,
                    labels=[f"id={d.marker_id}"],
                ),
            )
        print(f"detections: {n_det}")

        # Pass 4: averaged tracks (smoothing_window > 0 → per-track ids).
        # Re-runs the same filtered pipeline through a smoothing detector;
        # each track yields one entity that updates as the windowed average
        # refines. Color stable per track_id for visual identity.
        if smoothing_window > 0:
            xf_tracked = DetectMarkers(
                camera_info=cam_info,
                marker_length_m=marker_size,
                smoothing_window=smoothing_window,
            )
            pipeline_tracked: Stream[Any] = color_image.transform(
                QualityWindow(lambda img: img.sharpness, window=quality_window)
            )
            if marker_max_speed > 0:
                pipeline_tracked = pipeline_tracked.transform(
                    SpeedLimit(
                        max_mps=marker_max_speed,
                        max_dps=marker_max_rot_rate if marker_max_rot_rate > 0 else None,
                    )
                )
            seen_tracks: set[int] = set()
            n_updates = 0
            for det_obs in pipeline_tracked.transform(xf_tracked):
                d = det_obs.data
                rr.set_time(TIMELINE, timestamp=det_obs.ts)
                color = Color.from_cmap("tab10", (d.track_id % 10) / 10.0).rgb_u8()
                rr.log(
                    f"world/tracks/track_{d.track_id:04d}",
                    rr.Boxes3D(
                        centers=[(d.center.x, d.center.y, d.center.z)],
                        half_sizes=[(marker_size / 2, marker_size / 2, 0.005)],
                        quaternions=[
                            rr.Quaternion(
                                xyzw=[
                                    d.orientation.x,
                                    d.orientation.y,
                                    d.orientation.z,
                                    d.orientation.w,
                                ]
                            )
                        ],
                        colors=[color],
                        fill_mode=rr.components.FillMode.Solid,
                        labels=[f"track={d.track_id} id={d.marker_id}"],
                    ),
                )
                seen_tracks.add(d.track_id)
                n_updates += 1
            print(f"tracks: {len(seen_tracks)} unique, {n_updates} updates")

    print(f"wrote {out}")
    if no_gui:
        print(f"open with: rerun {out}")
    else:
        subprocess.Popen(["rerun", str(out)])


if __name__ == "__main__":
    typer.run(main)
