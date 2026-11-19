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

"""Automated eye-in-hand calibration with operator review of each stationary view."""

from __future__ import annotations

from collections.abc import Callable, Sequence
import math
from pathlib import Path
from typing import TYPE_CHECKING, Any, Literal

from scipy.spatial.transform import Rotation

from dimos.calibration import (
    DEFAULT_MARKER_SIZE_M,
    MIN_TRAINING_SAMPLES,
    MIN_VALIDATION_SAMPLES,
)
from dimos.calibration.hand_eye import MarkerObservationError
from dimos.calibration.hand_eye_capture import check_color_frame
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


def _ring_tilts(count: int, *, validation: bool) -> list[list[float]]:
    """Local tool X/Y rotation-vector components in radians, in capture order."""
    minimum = MIN_VALIDATION_SAMPLES if validation else MIN_TRAINING_SAMPLES
    if count < minimum:
        raise ValueError(f"Need at least {minimum} images for this capture set.")
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
    review: Callable[[Image, str], Literal["accept", "retry", "quit"]],
    return_joints: Sequence[float] | None = None,
    report: Callable[[str], None] = print,
    count: int = MIN_TRAINING_SAMPLES,
    validation_count: int = MIN_VALIDATION_SAMPLES,
    marker_size_m: float = DEFAULT_MARKER_SIZE_M,
    marker_id: int = 0,
) -> dict[str, Any] | None:
    """Move, review, fit and independently validate. Return None when the operator quits.

    The motion callbacks block until arrival. A joint move returns the measured world
    pose of the planner's tool. Capture returns a fresh image with the mount pose
    measured at image time. A resumed run reuses the original home tool pose.
    After grading a run that moved the arm, move to return_joints (zero by default).
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
                label += " (home: center the target on the crosshair)"
            report(f"Moving to {label}")
            if at_home:
                home_pose = move_joints(home)
                save_home_tool_pose(directory, home_pose)
            else:
                assert home_pose is not None
                move_pose(_tilted_pose(home_pose, x, y))
            moved = True
            try_closer = not at_home
            while True:
                image, pose = capture()
                check_color_frame(image, info)
                choice = review(image, label)
                if choice == "quit":
                    report("Stopped at the current pose. Accepted observations are saved.")
                    return None
                if choice == "retry":
                    continue
                if choice != "accept":
                    raise ValueError(f"Unknown image review choice {choice!r}")
                try:
                    entry = record_sample(directory, image, pose, validation=role == "validation")
                except MarkerObservationError as error:
                    report(f"Image rejected. {error}")
                    if try_closer:
                        report("Moving halfway back toward home for a fresh view.")
                        move_pose(_tilted_pose(home_pose, x / 2, y / 2))
                        try_closer = False
                    else:
                        label = f"{role.capitalize()} {index + 1}: {error}"
                    continue
                report(f"Saved {entry['image']} ({entry['reprojection_error_px']:.3f} px)")
                break
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
        try:
            move_joints(return_joints)
        except (RuntimeError, OSError, ValueError) as error:
            raise RuntimeError(
                f"Calibration result saved to {directory / 'hand_eye.json'}. "
                f"Could not confirm return to joint positions {list(return_joints)} rad: {error}"
            ) from error
    return result
