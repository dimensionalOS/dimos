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

"""Loop-closure tuning eval — runs the dimos map pipeline over all
hk_village recordings and reports two totals to minimize:

- TOTAL_PGO_TIME (s): wall-clock for the PGO loop across all recordings
- TOTAL_SPREAD  (m): per-recording sum of pairwise distances between
  final-smoothed marker positions of the same marker_id (PGO-corrected),
  summed across all marker_ids across all recordings. Smaller = tighter
  loop closures.

Usage:
    uv run python -m dimos.mapping.loop_closure.eval
    uv run python -m dimos.mapping.loop_closure.eval hk_village1
"""

from __future__ import annotations

import time
from typing import Any

import typer

from dimos.mapping.loop_closure.pgo import PGO
from dimos.memory2.store.sqlite import SqliteStore
from dimos.memory2.stream import Stream
from dimos.memory2.transform import QualityWindow, SpeedLimit
from dimos.memory2.type.observation import Observation
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.sensor_msgs.Image import Image
from dimos.perception.fiducial.marker_transformer import DetectMarkers
from dimos.robot.unitree.go2.connection import _camera_info_static
from dimos.utils.data import get_data

DEFAULT_DATASETS = [f"hk_village{i}" for i in range(1, 7)]


def _pairwise_sum(pts: list[tuple[float, float, float]]) -> float:
    """Sum of euclidean distances over all unordered pairs."""
    total = 0.0
    for i, a in enumerate(pts):
        for b in pts[i + 1 :]:
            dx, dy, dz = a[0] - b[0], a[1] - b[1], a[2] - b[2]
            total += (dx * dx + dy * dy + dz * dz) ** 0.5
    return total


def _eval_recording(
    name: str,
    *,
    marker_size: float,
    marker_max_speed: float,
    marker_max_rot_rate: float,
    marker_quality_window: float,
    marker_smoothing: float,
) -> tuple[float, float]:
    """Returns (pgo_time_s, spread_m) for one recording."""
    db_path = get_data(f"{name}.db")
    cam_info = _camera_info_static()

    store = SqliteStore(path=str(db_path))
    with store:
        lidar = store.streams.lidar

        t0 = time.perf_counter()
        graph = lidar.transform(PGO()).last().data
        pgo_time = time.perf_counter() - t0

        # Marker detection pipeline: same shape as dimos map / markers_rrd.
        color_image = store.stream("color_image", Image)
        xf = DetectMarkers(
            camera_info=cam_info,
            marker_length_m=marker_size,
            smoothing_window=marker_smoothing,
        )
        pipeline: Stream[Image] = color_image.transform(
            QualityWindow(lambda img: img.sharpness, window=marker_quality_window)
        )
        if marker_max_speed > 0:
            pipeline = pipeline.transform(
                SpeedLimit(
                    max_mps=marker_max_speed,
                    max_dps=marker_max_rot_rate if marker_max_rot_rate > 0 else None,
                )
            )
        all_dets = pipeline.transform(xf).to_list()

        # Dedup by track_id → final smoothed pose per track.
        by_track: dict[int, Observation[Any]] = {}
        for d in all_dets:
            by_track[d.data.track_id] = d
        tracks = list(by_track.values())

        # PGO-correct each track's pose; group by marker_id.
        by_marker: dict[int, list[tuple[float, float, float]]] = {}
        for d in tracks:
            raw_tf = Transform(
                translation=d.data.center,
                rotation=d.data.orientation,
                frame_id="world",
                child_frame_id=f"marker_{d.data.marker_id}",
                ts=d.ts,
            )
            corrected = graph.correct(raw_tf)
            t = corrected.translation
            by_marker.setdefault(d.data.marker_id, []).append((t.x, t.y, t.z))

        spread = sum(_pairwise_sum(v) for v in by_marker.values())
        return pgo_time, spread


def main(
    datasets: list[str] = typer.Argument(
        None, help="Recordings to eval; defaults to hk_village1..6"
    ),
    marker_size: float = typer.Option(0.1, "--marker-size"),
    marker_max_speed: float = typer.Option(0.5, "--marker-max-speed"),
    marker_max_rot_rate: float = typer.Option(50.0, "--marker-max-rot-rate"),
    marker_quality_window: float = typer.Option(0.1, "--marker-quality-window"),
    marker_smoothing: float = typer.Option(7.5, "--marker-smoothing"),
) -> None:
    names = datasets or DEFAULT_DATASETS

    # Sequential: PGO (Open3D) and marker detection (cv2/numpy) already
    # saturate available cores internally via OpenMP — process-level
    # parallelism only adds contention.
    wall_start = time.perf_counter()
    results: dict[str, tuple[float, float]] = {}
    for name in names:
        print(f"eval {name}...")
        results[name] = _eval_recording(
            name,
            marker_size=marker_size,
            marker_max_speed=marker_max_speed,
            marker_max_rot_rate=marker_max_rot_rate,
            marker_quality_window=marker_quality_window,
            marker_smoothing=marker_smoothing,
        )
    wall = time.perf_counter() - wall_start

    total_pgo = 0.0
    total_spread = 0.0
    print()
    print(f"{'recording':<14}  {'pgo_time_s':>10}  {'spread_m':>10}")
    print("-" * 40)
    for name in names:  # original order
        pgo_time, spread = results[name]
        print(f"{name:<14}  {pgo_time:>10.2f}  {spread:>10.3f}")
        total_pgo += pgo_time
        total_spread += spread
    print("-" * 40)
    print(f"{'TOTAL':<14}  {total_pgo:>10.2f}  {total_spread:>10.3f}")
    print()
    print(f"TOTAL_PGO_TIME={total_pgo:.2f}")
    print(f"TOTAL_SPREAD={total_spread:.3f}")
    print(f"WALL_TIME={wall:.2f}")


if __name__ == "__main__":
    typer.run(main)
