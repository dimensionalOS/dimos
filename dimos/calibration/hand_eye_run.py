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

"""Planner-driven eye-in-hand calibration with optional operator review."""

from __future__ import annotations

from collections.abc import Callable, Sequence
from contextlib import AbstractContextManager, nullcontext
from functools import partial
import math
from numbers import Integral
from pathlib import Path
from typing import TYPE_CHECKING, Any, Literal

from scipy.spatial.transform import Rotation

from dimos.calibration import (
    DEFAULT_CAMERA_INFO_TOPIC,
    DEFAULT_IMAGE_TOPIC,
    DEFAULT_MARKER_SIZE_M,
    DEFAULT_TF_TOPIC,
    MIN_TRAINING_SAMPLES,
    MIN_VALIDATION_SAMPLES,
    hand_eye_capture,
)
from dimos.calibration.hand_eye import MarkerObservationError
from dimos.calibration.hand_eye_session import (
    open_session,
    record_sample,
    save_home_tool_pose,
    solve_session,
    validate_session,
)
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion

if TYPE_CHECKING:
    from dimos.msgs.geometry_msgs.Transform import Transform
    from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
    from dimos.msgs.sensor_msgs.Image import Image

Review = Callable[["Image", str], Literal["accept", "retry", "quit"]]


def check_view_counts(count: int, validation_count: int) -> None:
    for role, views, minimum in (
        ("training", count, MIN_TRAINING_SAMPLES),
        ("validation", validation_count, MIN_VALIDATION_SAMPLES),
    ):
        if not isinstance(views, Integral) or views < minimum:
            raise ValueError(f"The {role} view count must be an integer of at least {minimum}.")


def _ring_tilts(count: int, *, validation: bool) -> list[list[float]]:
    """Local tool X/Y rotation-vector components in radians, in capture order."""
    views = [] if validation else [[0.0, 0.0]]
    ring_count = count - len(views)
    radius = math.radians(7.0 if validation else 11.0)
    for index in range(ring_count):
        phase = math.tau * (index + (0.5 if validation else 0.0)) / ring_count
        views.append([radius * math.cos(phase), radius * math.sin(phase)])
    return views


def _tilted_pose(home: PoseStamped, x: float, y: float) -> PoseStamped:
    local_rotation = Quaternion(Rotation.from_rotvec([x, y, 0.0]).as_quat())
    return PoseStamped(
        frame_id=home.frame_id,
        position=home.position,
        orientation=home.orientation * local_rotation,
    )


def run_hand_eye(
    directory: Path,
    info: CameraInfo,
    *,
    home: Sequence[float],
    joint_names: Sequence[str],
    base_frame: str,
    ee_frame: str,
    tip_frame: str | None,
    move_joints: Callable[[Sequence[float]], PoseStamped],
    move_pose: Callable[[PoseStamped], None],
    capture: Callable[[], tuple[Image, Transform]],
    review: Review | None = None,
    return_joints: Sequence[float] | None = None,
    return_move: Callable[[Sequence[float]], PoseStamped] | None = None,
    report: Callable[[str], None] = print,
    check_cancelled: Callable[[], None] = lambda: None,
    count: int = MIN_TRAINING_SAMPLES,
    validation_count: int = MIN_VALIDATION_SAMPLES,
    marker_size_m: float = DEFAULT_MARKER_SIZE_M,
    marker_id: int = 0,
) -> dict[str, Any] | None:
    """Move, fit and independently validate. Return None when the operator quits.

    Without a reviewer, the first rejected observation ends the run at that pose.
    The motion callbacks block until arrival. A joint move returns the measured world
    pose of the planner's tool. Capture returns a fresh image with the mount pose
    measured at image time. A resumed run reuses the original home tool pose.
    After grading a run that moved the arm, use return_move (move_joints by default)
    to reach return_joints (zero by default).
    """
    home = tuple(home)
    if not joint_names or len(home) != len(joint_names) or not all(map(math.isfinite, home)):
        raise ValueError(
            f"Home must contain {len(joint_names)} finite positions in planner joint order "
            f"({', '.join(joint_names)})."
        )
    if not tip_frame:
        raise ValueError("The selected planning group needs a tool frame for pose targets.")
    return_joints = (0.0,) * len(home) if return_joints is None else tuple(return_joints)
    if len(return_joints) != len(home) or not all(map(math.isfinite, return_joints)):
        raise ValueError(f"Return pose must contain {len(home)} finite positions in planner order.")
    check_view_counts(count, validation_count)
    views = {
        "train": _ring_tilts(count, validation=False),
        "validation": _ring_tilts(validation_count, validation=True),
    }
    session = open_session(
        directory,
        info,
        base_frame=base_frame,
        ee_frame=ee_frame,
        marker_size_m=marker_size_m,
        marker_id=marker_id,
        automatic_views={
            "space": "tool_tilt_xy_rad",
            "joint_names": list(joint_names),
            "home_joints": list(home),
            "tip_frame": tip_frame,
            **views,
        },
    )
    saved_pose = session.get("home_tool_pose")
    home_pose = PoseStamped(**saved_pose) if saved_pose is not None else None
    if home_pose is None and session["samples"]:
        raise ValueError("Session is missing its home tool pose. Use a new session directory.")
    moved = False
    for role, targets in views.items():
        saved = sum(entry["role"] == role for entry in session["samples"])
        for index in range(saved, len(targets)):
            label = f"{role.capitalize()} {index + 1}/{len(targets)}"
            x, y = targets[index]
            at_home = role == "train" and index == 0
            if at_home:
                label += (
                    " (home)" if review is None else " (home: center the target on the crosshair)"
                )
            report(f"Moving to {label}")
            check_cancelled()
            if at_home:
                home_pose = move_joints(home)
                save_home_tool_pose(directory, home_pose)
            else:
                assert home_pose is not None
                move_pose(_tilted_pose(home_pose, x, y))
            moved = True
            try_closer = not at_home
            while True:
                check_cancelled()
                image, pose = capture()
                hand_eye_capture.check_color_frame(image, info)
                choice = review(image, label) if review is not None else "accept"
                if choice == "quit":
                    report("Stopped at the current pose. Accepted observations are saved.")
                    return None
                if choice == "retry":
                    continue
                if choice != "accept":
                    raise ValueError(f"Unknown image review choice {choice!r}")
                check_cancelled()
                try:
                    entry = record_sample(directory, image, pose, validation=role == "validation")
                except MarkerObservationError as error:
                    if review is None:
                        raise MarkerObservationError(f"{label}: {error}") from error
                    report(f"Image rejected. {error}")
                    if try_closer:
                        report("Moving halfway back toward home for a fresh view.")
                        check_cancelled()
                        move_pose(_tilted_pose(home_pose, x / 2, y / 2))
                        try_closer = False
                    else:
                        label = f"{role.capitalize()} {index + 1}: {error}"
                    continue
                report(f"Saved {entry['image']} ({entry['reprojection_error_px']:.3f} px)")
                break
        check_cancelled()
        if role == "train":
            result = solve_session(directory)
            report(f"Training metrics: {result['training_metrics']}")
            if not result["training_metrics"]["passed"]:
                break
        else:
            result = validate_session(directory)
            report(f"Validation metrics: {result['validation_metrics']}")
    if moved:
        report(f"Returning to joint positions {list(return_joints)} rad.")
        check_cancelled()
        if return_move is None:
            return_move = move_joints
        try:
            return_move(return_joints)
        except (RuntimeError, OSError, ValueError) as error:
            raise RuntimeError(
                f"Calibration result saved to {directory / 'hand_eye.json'}. "
                f"Could not confirm return to joint positions {list(return_joints)} rad: {error}"
            ) from error
    return result


def run_live_hand_eye(
    session: Path,
    *,
    home: Sequence[float],
    ee_frame: str,
    count: int = MIN_TRAINING_SAMPLES,
    validation_count: int = MIN_VALIDATION_SAMPLES,
    image_topic: str = DEFAULT_IMAGE_TOPIC,
    camera_info_topic: str = DEFAULT_CAMERA_INFO_TOPIC,
    tf_topic: str = DEFAULT_TF_TOPIC,
    marker_size_m: float = DEFAULT_MARKER_SIZE_M,
    marker_id: int = 0,
    timeout_sec: float = 10.0,
    report: Callable[[str], None] = print,
    preview: Callable[[], AbstractContextManager[Review | None]] = nullcontext,
    check_cancelled: Callable[[], None] = lambda: None,
) -> dict[str, Any] | None:
    """Check the camera and measured TF before the first move, then run both rings.

    Supply a preview for operator review and retakes. Without one, each stationary
    view must contain a usable observation on its first capture.
    """
    # The callback-based runner also works offline without loading the planner SDK.
    from dimos.calibration.hand_eye_motion import connect_calibration_arm

    check_cancelled()
    report(f"Reading color intrinsics from {camera_info_topic}...")
    info = hand_eye_capture.read_camera_info(camera_info_topic, timeout_sec)
    with (
        connect_calibration_arm(check_cancelled=check_cancelled) as arm,
        hand_eye_capture.TopicCapture(
            image_topic, tf_topic, arm.info.base_frame, ee_frame
        ) as source,
    ):
        capture_pair = partial(source.capture, timeout_sec)
        check_cancelled()
        image, _ = capture_pair()
        hand_eye_capture.check_color_frame(image, info)
        with preview() as review:
            return run_hand_eye(
                session,
                info,
                home=home,
                return_joints=arm.return_joints,
                return_move=partial(arm.move_joints, returning=True),
                base_frame=arm.info.base_frame,
                ee_frame=ee_frame,
                tip_frame=arm.info.tip_frame,
                joint_names=arm.info.joint_names,
                count=count,
                validation_count=validation_count,
                marker_size_m=marker_size_m,
                marker_id=marker_id,
                move_joints=arm.move_joints,
                move_pose=arm.move_pose,
                capture=partial(arm.capture_at_view, capture_pair),
                review=review,
                report=report,
                check_cancelled=check_cancelled,
            )
