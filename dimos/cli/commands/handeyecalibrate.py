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

"""CLI for the reusable eye-in-hand calibration workflow."""

from __future__ import annotations

from functools import partial
from pathlib import Path
from typing import TYPE_CHECKING

from click import ClickException, Context
import typer
from typer.core import TyperCommand

from dimos.calibration import (
    DEFAULT_BASE_FRAME,
    DEFAULT_EE_FRAME,
    DEFAULT_MARKER_SIZE_M,
    MIN_TRAINING_SAMPLES,
    MIN_VALIDATION_SAMPLES,
)

if TYPE_CHECKING:
    from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo

_COLOR_IMAGE_TOPIC = "zenoh:dimos/color_image"
_CAMERA_INFO_TOPIC = "zenoh:dimos/camera_info"
_TF_TOPIC = "zenoh:dimos/tf"

handeyecalibrate_app = typer.Typer(
    help="Eye-in-hand calibration with a fixed tag36h11.", no_args_is_help=True
)


def _read_intrinsics(topic: str, timeout_sec: float, yaml_path: Path | None = None) -> CameraInfo:
    # Keep the camera and transport dependencies off the global CLI import path.
    from dimos.calibration.hand_eye_capture import read_camera_info
    from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo

    if yaml_path is not None:
        return CameraInfo.from_yaml(str(yaml_path))
    typer.echo(f"Reading color intrinsics from {topic}...")
    return read_camera_info(topic, timeout_sec)


class _RunCommand(TyperCommand):
    """Accept a space-separated --home of any length using Click's repeatable float option."""

    def parse_args(self, ctx: Context, args: list[str]) -> list[str]:
        expanded: list[str] = []
        reading_home = False
        for arg in args:
            if arg.startswith("--"):
                reading_home = arg == "--home" or arg.startswith("--home=")
            elif reading_home and expanded[-1] != "--home":
                expanded.append("--home")
            expanded.append(arg)
        return super().parse_args(ctx, expanded)


@handeyecalibrate_app.command(cls=_RunCommand)
def run(
    session: Path = typer.Option(..., help="Directory for this automatic calibration session."),
    home: list[float] = typer.Option(
        ..., metavar="Q1 Q2 ...", help="Centered home in planner joint order, angles in radians."
    ),
    ee_frame: str = typer.Option(..., help="TF link that the camera is rigidly mounted on."),
    count: int = typer.Option(
        MIN_TRAINING_SAMPLES, min=MIN_TRAINING_SAMPLES, help="Training images."
    ),
    validation_count: int = typer.Option(
        MIN_VALIDATION_SAMPLES, min=MIN_VALIDATION_SAMPLES, help="Additional held-out images."
    ),
    camera_info_topic: str = typer.Option(_CAMERA_INFO_TOPIC),
    image_topic: str = typer.Option(_COLOR_IMAGE_TOPIC),
    tf_topic: str = typer.Option(_TF_TOPIC),
    marker_size_m: float = typer.Option(
        DEFAULT_MARKER_SIZE_M, help="Measured black square outer edge in metres."
    ),
    marker_id: int = typer.Option(0),
    timeout_sec: float = typer.Option(10.0, min=0.1, help="Seconds to wait for camera or TF data."),
) -> None:
    """Capture two tilt rings, solve and validate, then move to the arm's return pose."""
    # Load camera, GUI and planner dependencies only for the motion-enabled command.
    from dimos.calibration.hand_eye_capture import TopicCapture, check_color_frame
    from dimos.calibration.hand_eye_motion import connect_calibration_arm
    from dimos.calibration.hand_eye_run import run_hand_eye
    from dimos.cli.hand_eye_preview import hand_eye_preview

    try:
        info = _read_intrinsics(camera_info_topic, timeout_sec)
        with (
            connect_calibration_arm() as arm,
            TopicCapture(image_topic, tf_topic, arm.info.base_frame, ee_frame) as source,
        ):
            group = arm.info
            # Check the camera and measured mount TF before sending the first physical goal.
            capture_pair = partial(source.capture, timeout_sec)
            image, _ = capture_pair()
            check_color_frame(image, info)
            with hand_eye_preview() as review:
                result = run_hand_eye(
                    session,
                    info,
                    home=home,
                    return_joints=arm.return_joints,
                    base_frame=group.base_frame,
                    ee_frame=ee_frame,
                    joint_names=group.joint_names,
                    tip_frame=group.tip_frame,
                    count=count,
                    validation_count=validation_count,
                    marker_size_m=marker_size_m,
                    marker_id=marker_id,
                    move_joints=arm.move_joints,
                    move_pose=arm.move_pose,
                    capture=partial(arm.capture_at_view, capture_pair),
                    review=review,
                    report=typer.echo,
                )
    except (ValueError, RuntimeError, OSError) as exc:
        raise ClickException(str(exc)) from exc
    if result is None:
        return
    typer.echo(f"{'PASS' if result['validated'] else 'FAIL'}  {session / 'hand_eye.json'}")
    if not result["validated"]:
        raise typer.Exit(1)


@handeyecalibrate_app.command()
def capture(
    session: Path = typer.Option(..., help="Session directory to create or resume"),
    intrinsics: Path | None = typer.Option(
        None, help="Override live intrinsics with a color CameraInfo YAML file."
    ),
    count: int | None = typer.Option(
        None,
        min=1,
        help=f"New observations (defaults to {MIN_TRAINING_SAMPLES} training or "
        f"{MIN_VALIDATION_SAMPLES} validation).",
    ),
    validation: bool = typer.Option(False, help="Reserve these observations for validation only"),
    image_topic: str = typer.Option(_COLOR_IMAGE_TOPIC),
    camera_info_topic: str = typer.Option(
        _CAMERA_INFO_TOPIC, help="CameraInfo topic for the color stream."
    ),
    tf_topic: str = typer.Option(_TF_TOPIC),
    base_frame: str = typer.Option(DEFAULT_BASE_FRAME),
    ee_frame: str = typer.Option(DEFAULT_EE_FRAME),
    marker_size_m: float = typer.Option(
        DEFAULT_MARKER_SIZE_M, help="Measured black square outer edge in metres"
    ),
    marker_id: int = typer.Option(0),
    timeout_sec: float = typer.Option(
        10.0, min=0.1, help="Seconds to wait for CameraInfo or a stationary image/pose pair."
    ),
) -> None:
    """Press Enter at each stationary pose; capture images and measured TF poses."""
    from dimos.calibration.hand_eye import MarkerObservationError
    from dimos.calibration.hand_eye_capture import TopicCapture, check_color_frame
    from dimos.calibration.hand_eye_session import open_session, record_sample

    if count is None:
        count = MIN_VALIDATION_SAMPLES if validation else MIN_TRAINING_SAMPLES
    try:
        camera_info = _read_intrinsics(camera_info_topic, timeout_sec, intrinsics)
        open_session(session, camera_info, base_frame, ee_frame, marker_size_m, marker_id)
        with TopicCapture(image_topic, tf_topic, base_frame, ee_frame) as source:
            accepted = 0
            while accepted < count:
                answer = typer.prompt(
                    f"Pose {accepted + 1}/{count}: hold still, Enter to capture (q to quit)",
                    default="",
                    show_default=False,
                )
                if answer.strip().lower() == "q":
                    typer.echo(f"Stopped; {accepted} new observations saved.")
                    return
                try:
                    image, pose = source.capture(timeout_sec)
                except RuntimeError as exc:
                    typer.echo(f"Rejected: {exc}. Reposition and retry.", err=True)
                    continue
                if intrinsics is None:
                    check_color_frame(image, camera_info)
                try:
                    entry = record_sample(session, image, pose, validation=validation)
                except MarkerObservationError as exc:
                    typer.echo(f"Rejected: {exc}. Reposition and retry.", err=True)
                    continue
                accepted += 1
                typer.echo(
                    f"Saved {entry['image']}: reprojection {entry['reprojection_error_px']:.3f} px"
                )
    except (ValueError, RuntimeError, OSError) as exc:
        raise ClickException(str(exc)) from exc


@handeyecalibrate_app.command()
def solve(session: Path = typer.Option(...)) -> None:
    """Fit training observations and save a candidate camera-to-end-effector transform."""
    from dimos.calibration.hand_eye_session import solve_session

    try:
        result = solve_session(session)
    except (ValueError, RuntimeError, OSError) as exc:
        raise ClickException(str(exc)) from exc
    typer.echo(f"Saved candidate: {session / 'hand_eye.json'}")
    typer.echo(f"Training metrics: {result['training_metrics']}")
    if not result["training_metrics"]["passed"]:
        raise typer.Exit(1)
    typer.echo("Run validate with separate observations before using this transform.")


@handeyecalibrate_app.command()
def validate(session: Path = typer.Option(...)) -> None:
    """Evaluate held-out observations, save metrics, and exit 1 on FAIL."""
    from dimos.calibration.hand_eye_session import validate_session

    try:
        result = validate_session(session)
    except (ValueError, RuntimeError, OSError) as exc:
        raise ClickException(str(exc)) from exc
    typer.echo("PASS" if result["validated"] else "FAIL")
    typer.echo(f"Validation metrics: {result['validation_metrics']}")
    if not result["validated"]:
        raise typer.Exit(1)
