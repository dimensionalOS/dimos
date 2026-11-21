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

import numpy as np
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
from dimos.calibration.hand_eye import MIN_ROTATION_DEG, MarkerObservationError, checked_transform
from dimos.calibration.hand_eye_session import (
    estimate_aiming_mount,
    load_session,
    observe_sample,
    open_session,
    save_home_tool_pose,
    save_observation,
    save_rejected_frame,
    solve_session,
    validate_session,
)
from dimos.calibration.hand_eye_views import (
    HOME_SWEEP_SPACES,
    home_sweep_views,
    read_orbit_mount,
    tag_view_pose,
    tag_views,
)
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3

if TYPE_CHECKING:
    from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
    from dimos.msgs.sensor_msgs.Image import Image

Review = Callable[["Image", str], Literal["accept", "retry", "quit"]]
ReviewRejected = Callable[["Image", str, bool], Literal["retry", "halfway", "quit"]]
_ORBIT_RADII_M = {"train": 0.055, "validation": 0.040}


def check_orbit_axis(axis: str | None, orbit_calibration: Path | None = None) -> None:
    if axis is not None and orbit_calibration is not None:
        raise ValueError("Choose either orbit-axis or orbit-calibration for a run.")
    if axis is not None and axis not in ("x", "-x", "y", "-y", "z", "-z"):
        raise ValueError("Choose the tool axis facing the target: x, -x, y, -y, z or -z.")


def check_view_counts(count: int, validation_count: int) -> None:
    for role, views, minimum in (
        ("training", count, MIN_TRAINING_SAMPLES),
        ("validation", validation_count, MIN_VALIDATION_SAMPLES),
    ):
        if not isinstance(views, Integral) or views < minimum:
            raise ValueError(f"The {role} view count must be an integer of at least {minimum}.")


def _ring_tilts(count: int, *, angle_deg: float, validation: bool) -> list[list[float]]:
    """Two perpendicular tilt components in radians, in capture order."""
    views = [] if validation else [[0.0, 0.0]]
    ring_count = count - len(views)
    radius = math.radians(angle_deg)
    for index in range(ring_count):
        phase = math.tau * (index + (0.5 if validation else 0.0)) / ring_count
        views.append([radius * math.cos(phase), radius * math.sin(phase)])
    return views


def _home_tag_range(sample: dict[str, Any]) -> float:
    tag = checked_transform(sample["camera_from_target"])[:3, 3]
    if tag[2] <= 0:
        raise ValueError("Orbit needs a first tag observation in front of the camera.")
    distance = math.hypot(*tag)
    if distance * math.sin(math.radians(MIN_ROTATION_DEG / 2)) >= _ORBIT_RADII_M["train"]:
        raise ValueError(
            "The tag is too far from home for the training orbit to give "
            "15° rotation diversity. Use a closer centered home in a new session."
        )
    return distance


def _tilted_pose(
    home: PoseStamped,
    x: float,
    y: float,
    fraction: float,
    *,
    forward_axis: str,
    distance_m: float,
    max_radius_m: float,
) -> PoseStamped:
    """Rotate the tool about a point on its forward axis. Zero distance tilts in place."""
    axis = "xyz".index(forward_axis[-1])
    tilt = np.zeros(3)
    tilt[(axis + 1) % 3], tilt[(axis + 2) % 3] = x, y
    if distance_m:
        max_angle = math.asin(min(1.0, max_radius_m / distance_m))
        tilt *= min(1.0, max_angle / math.hypot(x, y))
    rotation = Rotation.from_rotvec(tilt * fraction)
    pivot = np.zeros(3)
    pivot[axis] = -distance_m if forward_axis.startswith("-") else distance_m
    # Translate the TCP so the pivot stays fixed as the tool rotates.
    offset = Vector3(pivot - rotation.apply(pivot))
    return PoseStamped(
        frame_id=home.frame_id,
        position=home.position + home.orientation.rotate_vector(offset),
        orientation=home.orientation * Quaternion(rotation.as_quat()),
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
    move_pose: Callable[[PoseStamped], PoseStamped | None],
    capture: Callable[[], tuple[Image, Transform]],
    review: Review | None = None,
    review_rejected: ReviewRejected | None = None,
    orbit_axis: str | None = None,
    orbit_camera: Transform | None = None,
    tip_from_mount: Transform | None = None,
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

    Without a reviewer, a marker rejection after the first sighting gets one halfway retry.
    With the rigid tip-to-mount TF, five initial training views estimate the camera
    mount for aiming the wide and return sweeps. A supplied orbit_camera skips the four
    mount-estimation tilts.
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
    check_orbit_axis(orbit_axis)
    if tip_from_mount is not None and (
        orbit_camera is not None
        or orbit_axis is not None
        or (tip_from_mount.frame_id, tip_from_mount.child_frame_id) != (tip_frame, ee_frame)
    ):
        raise ValueError(
            "Target-centered views require the camera mounting link in the planner tip."
        )
    if orbit_camera is not None and (
        orbit_axis is not None
        or orbit_camera.frame_id != tip_frame
        or not orbit_camera.child_frame_id
    ):
        raise ValueError(
            "A calibrated orbit requires the optical frame mapped into the planner tip and no orbit-axis."
        )
    # Four opposing tilts make the camera mount observable after the first sighting.
    initial_views = (
        _ring_tilts(5, angle_deg=11.0, validation=False)
        if tip_from_mount is not None
        else [[0.0, 0.0, 0.0]]
    )
    tag_sweep = tip_from_mount is not None or orbit_camera is not None
    saved_schedule = (
        load_session(directory)["settings"].get("automatic_views", {})
        if (directory / "session.json").is_file()
        else {}
    )
    saved_space = saved_schedule.get("space")
    angled_start = saved_schedule.get("angled_start", not saved_schedule)
    home_relative = saved_space not in (
        "tag_azimuth_deg_radius_height_m",
        "tag_arc",
        "tool_tilts_then_tag_arc",
    )
    if tag_sweep:
        home_space = saved_space if saved_space in HOME_SWEEP_SPACES else HOME_SWEEP_SPACES[-1]
        sweep = (
            partial(home_sweep_views, space=home_space)
            if home_relative
            else partial(tag_views, flat_outer_arc=saved_space == "tag_azimuth_deg_radius_height_m")
        )
        views = {
            "train": initial_views + sweep(count - len(initial_views), validation=False),
            "validation": sweep(validation_count, validation=True),
        }
    else:
        views = {
            "train": _ring_tilts(count, angle_deg=11.0, validation=False),
            "validation": _ring_tilts(
                validation_count, angle_deg=7.0 if orbit_axis is None else 8.0, validation=True
            ),
        }
    if angled_start:
        # The wide sweep starts right of home, yawed inward toward the centered tag.
        # Axis-based rings use an in-place tilt until the tag range is known.
        views["train"][0] = [-0.03, math.radians(8)] if tag_sweep else [0.0, math.radians(-8)]
    schedule: dict[str, Any] = {
        "space": "tool_tilt_xy_rad",
        "joint_names": list(joint_names),
        "home_joints": list(home),
        "tip_frame": tip_frame,
        **views,
    }
    if angled_start:
        schedule["angled_start"] = True
    if orbit_axis is not None:
        schedule.update(space="tool_orbit_rad", orbit_axis=orbit_axis, max_radius_m=_ORBIT_RADII_M)
    if tag_sweep:
        schedule["space"] = home_space if home_relative else saved_space
    if orbit_camera is not None:
        schedule["camera_frame"] = orbit_camera.child_frame_id
    if tip_from_mount is not None:
        schedule["initial_tilt_views"] = len(initial_views)
    session = open_session(
        directory,
        info,
        base_frame=base_frame,
        ee_frame=ee_frame,
        marker_size_m=marker_size_m,
        marker_id=marker_id,
        automatic_views=schedule,
    )
    saved_pose = session.get("home_tool_pose")
    sighting_world_from_tip = (
        checked_transform(session["sighting_world_from_tip"])
        if angled_start and tag_sweep and session["samples"]
        else None
    )
    home_pose = PoseStamped(**saved_pose) if saved_pose is not None else None
    if home_pose is None and session["samples"]:
        raise ValueError("Session is missing its home tool pose. Use a new session directory.")
    tip_from_camera = None
    camera_from_tag = (
        checked_transform(session["samples"][0]["camera_from_target"])
        if tag_sweep and session["samples"]
        else None
    )
    if orbit_camera is not None:
        tip_from_camera = checked_transform(orbit_camera.to_matrix())
        if saved_pose is not None:
            saved_mount = checked_transform(np.asarray(session.get("orbit_tip_from_camera")))
            if not np.allclose(saved_mount, tip_from_camera, rtol=0, atol=1e-6):
                raise ValueError("Orbit camera mount changed; use a new session directory.")
            tip_from_camera = saved_mount
    elif tip_from_mount is not None and "orbit_tip_from_camera" in session:
        tip_from_camera = checked_transform(session["orbit_tip_from_camera"])
    distance_m = (
        _home_tag_range(session["samples"][0]) if orbit_axis and session["samples"] else 0.0
    )
    moved = False
    for role, targets in views.items():
        saved = sum(entry["role"] == role for entry in session["samples"])
        for index in range(saved, len(targets)):
            label = f"{role.capitalize()} {index + 1}/{len(targets)}"
            first_view = role == "train" and index == 0
            if first_view and not angled_start:
                label += (
                    " (home)" if review is None else " (home: center the target on the crosshair)"
                )
            report(f"Moving to {label}")
            check_cancelled()
            view_pose: Callable[[float], PoseStamped] | None = None
            if first_view:
                home_pose = move_joints(home)
                save_home_tool_pose(directory, home_pose, tip_from_camera)
            if first_view and angled_start and tag_sweep:
                assert home_pose is not None
                lateral, yaw = targets[index]
                goal = PoseStamped(
                    frame_id=home_pose.frame_id,
                    position=home_pose.position + Vector3(0, lateral, 0),
                    orientation=Quaternion(Rotation.from_euler("z", yaw).as_quat())
                    * home_pose.orientation,
                )
                check_cancelled()
                reached = move_pose(goal)
                sighting_world_from_tip = Transform.from_pose(
                    tip_frame, reached if reached is not None else goal
                ).to_matrix()
            elif not first_view or angled_start:
                assert home_pose is not None
                if tag_sweep and (role == "validation" or index >= len(initial_views)):
                    if tip_from_camera is None:
                        assert tip_from_mount is not None
                        mount = estimate_aiming_mount(directory, len(initial_views))
                        tip_from_camera = checked_transform(tip_from_mount.to_matrix() @ mount)
                        save_home_tool_pose(directory, home_pose, tip_from_camera)
                    assert camera_from_tag is not None
                    view_pose = partial(
                        tag_view_pose,
                        home_pose,
                        tip_from_camera,
                        camera_from_tag,
                        targets[index],
                        home_relative=home_relative,
                        sighting_world_from_tip=sighting_world_from_tip,
                    )
                else:
                    x, y = targets[index]
                    view_pose = partial(
                        _tilted_pose,
                        home_pose,
                        x,
                        y,
                        forward_axis=orbit_axis or "z",
                        distance_m=distance_m,
                        max_radius_m=_ORBIT_RADII_M[role],
                    )
                check_cancelled()
                move_pose(view_pose(1.0))
                if first_view:
                    view_pose = None  # Require a usable first sighting before orbiting.
            moved = True
            while True:
                check_cancelled()
                image, pose = capture()
                hand_eye_capture.check_color_frame(image, info)
                if orbit_camera is not None and image.frame_id != orbit_camera.child_frame_id:
                    raise ValueError("Image optical frame differs from the orbit calibration.")
                capture_session = load_session(directory)
                try:
                    sample = observe_sample(capture_session, image, pose)
                except MarkerObservationError as error:
                    reason = f"{label}: {error}"
                    path = save_rejected_frame(
                        directory,
                        capture_session,
                        image,
                        pose,
                        reason,
                        validation=role == "validation",
                    )
                    diagnostic = f"{reason}. Saved rejected frame to {path}"
                    report(f"Image rejected. {diagnostic}")
                    if review is None:
                        if view_pose is None:
                            raise MarkerObservationError(diagnostic) from error
                        rejected = "halfway"
                    elif review_rejected is None:
                        rejected = "quit" if review(image, reason) == "quit" else "retry"
                    else:
                        rejected = review_rejected(image, reason, view_pose is not None)
                    if rejected == "quit":
                        report("Stopped at the current pose. Accepted observations are saved.")
                        return None
                    if rejected == "halfway" and view_pose is not None:
                        label += " (halfway retry)"
                        report(f"Moving to {label} after rejection: {error}")
                        check_cancelled()
                        move_pose(view_pose(0.5))
                        view_pose = None
                    continue
                choice = review(image, label) if review is not None else "accept"
                if choice == "quit":
                    report("Stopped at the current pose. Accepted observations are saved.")
                    return None
                if choice == "retry":
                    path = save_rejected_frame(
                        directory,
                        capture_session,
                        image,
                        pose,
                        f"{label}: operator requested a retake",
                        validation=role == "validation",
                    )
                    report(f"Saved rejected frame to {path}")
                    continue
                if choice != "accept":
                    raise ValueError(f"Unknown image review choice {choice!r}")
                check_cancelled()
                if first_view and sighting_world_from_tip is not None:
                    capture_session["sighting_world_from_tip"] = sighting_world_from_tip.tolist()
                entry = save_observation(
                    directory, capture_session, image, sample, validation=role == "validation"
                )
                report(f"Saved {entry['image']} ({entry['reprojection_error_px']:.3f} px)")
                if first_view and tag_sweep:
                    camera_from_tag = checked_transform(entry["camera_from_target"])
                    if orbit_camera is not None:
                        report("Located the tag. Starting the outer camera sweep.")
                if first_view and orbit_axis is not None:
                    distance_m = _home_tag_range(entry)
                    report(
                        f"Orbit center estimated {distance_m:.3f} m along TCP {orbit_axis} from home."
                    )
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
    orbit_axis: str | None = None,
    orbit_calibration: Path | None = None,
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
    review_rejected: ReviewRejected | None = None,
    check_cancelled: Callable[[], None] = lambda: None,
) -> dict[str, Any] | None:
    """Check the camera and measured TF before the first move, then run both sweeps.

    Supply a preview for operator review and retakes. Without one, retry a rejected
    observation after the first sighting once from the halfway pose.
    """
    # The callback-based runner also works offline without loading the planner SDK.
    from dimos.calibration.hand_eye_motion import connect_calibration_arm

    check_orbit_axis(orbit_axis, orbit_calibration)
    if (
        orbit_calibration is not None
        and orbit_calibration.resolve() == (session / "hand_eye.json").resolve()
    ):
        raise ValueError("Use a separate session for the new calibration and its orbit reference.")
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
        existing = load_session(session) if (session / "session.json").is_file() else {}
        optical_frame = existing.get("camera_frame")
        if optical_frame and optical_frame != image.frame_id:
            raise ValueError("Image optical frame differs from the saved session.")
        existing_space = existing.get("settings", {}).get("automatic_views", {}).get("space")
        orbit_camera = tip_from_mount = None
        if orbit_calibration is not None or (
            orbit_axis is None and existing_space != "tool_tilt_xy_rad"
        ):
            if not arm.info.tip_frame:
                raise ValueError("Orbit planning requires a planner tip frame.")
            tip_from_mount = source.tip_from_mount(arm.info.tip_frame, image.ts)
            if orbit_calibration is not None:
                mount = read_orbit_mount(orbit_calibration, ee_frame, image.frame_id)
                orbit_camera = tip_from_mount + mount
                tip_from_mount = None
        with preview() as review:
            return run_hand_eye(
                session,
                info,
                home=home,
                orbit_axis=orbit_axis,
                orbit_camera=orbit_camera,
                tip_from_mount=tip_from_mount,
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
                review_rejected=review_rejected,
                report=report,
                check_cancelled=check_cancelled,
            )
