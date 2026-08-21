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

"""The cameracalibrate command; the implementation lives in dimos.cli.cameracalibrate."""

from __future__ import annotations

from pathlib import Path

import typer


def cameracalibrate(
    source: str = typer.Option(..., "--source", help="Frame source: webcam, folder, or topic"),
    device_index: int = typer.Option(0, "--device-index", help="Webcam device index"),
    images: Path | None = typer.Option(
        None, "--images", help="Directory of calibration images for --source folder"
    ),
    topic: str | None = typer.Option(
        None,
        "--topic",
        help=(
            "Pubsub URI for --source topic (proto:channel), "
            "e.g. 'jpeg_lcm:/color_image' or 'pshm:color_image'."
        ),
    ),
    topic_timeout_sec: float = typer.Option(
        60.0,
        "--topic-timeout-sec",
        help="Abort --source topic if no frames arrive within this many seconds.",
    ),
    cols: int = typer.Option(..., "--cols", help="Inner chessboard corner columns"),
    rows: int = typer.Option(..., "--rows", help="Inner chessboard corner rows"),
    square_size_m: float = typer.Option(
        ..., "--square-size-m", help="Chessboard square size in meters"
    ),
    out: Path | None = typer.Option(None, "--out", help="Optional ROS CameraInfo YAML output path"),
    preview_out: Path | None = typer.Argument(
        None, help="Optional preview PNG output path. Requires --out."
    ),
    camera_name: str = typer.Option("webcam", "--camera-name", help="Camera name in YAML"),
    target_count: int = typer.Option(20, "--target-count", help="Accepted webcam frame count"),
    no_display: bool = typer.Option(False, "--no-display", help="Disable OpenCV preview windows"),
    distortion_model: str = typer.Option(
        "plumb_bob",
        "--distortion-model",
        help=(
            "Lens model: 'plumb_bob' (5 coeffs, near-pinhole) or 'fisheye' "
            "(4 coeffs, wide-angle / fisheye; written as ROS 'equidistant')."
        ),
    ),
) -> None:
    """Calibrate camera intrinsics and write ROS CameraInfo YAML."""
    from dimos.cli.cameracalibrate.cameracalibrate import run_calibration

    if preview_out is not None and out is None:
        raise typer.BadParameter("preview output requires --out")

    try:
        result = run_calibration(
            source=source,
            device_index=device_index,
            images=images,
            topic=topic,
            topic_timeout_sec=topic_timeout_sec,
            cols=cols,
            rows=rows,
            square_size_m=square_size_m,
            out=out,
            preview_out=preview_out,
            camera_name=camera_name,
            target_count=target_count,
            no_display=no_display,
            distortion_model=distortion_model,
        )
    except (ValueError, RuntimeError) as exc:
        raise typer.BadParameter(str(exc)) from exc

    typer.echo(f"RMS: {float(result['rms']):.6f} px ({int(result['n_used'])} frame(s) used)")
    typer.echo(
        f"Detected pattern: {tuple(result.get('pattern_size', (cols, rows)))} "
        f"({result.get('pattern_label', 'requested inner corners')})"
    )
    if out is not None:
        typer.echo(f"Wrote camera info YAML to {out}")
    if preview_out is not None:
        typer.echo(f"Wrote preview overlay PNG to {preview_out}")
