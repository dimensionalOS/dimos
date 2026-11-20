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

"""Small on-disk hand-eye workflow, shared by the CLI and future agent tools."""

from __future__ import annotations

from itertools import islice
import json
from pathlib import Path
from typing import Any
from uuid import uuid4

import numpy as np
from numpy.typing import NDArray

from dimos.calibration import DEFAULT_BASE_FRAME, DEFAULT_MARKER_SIZE_M
from dimos.calibration.hand_eye import (
    TAG_DICTIONARY,
    HandEyeSample,
    check_marker,
    checked_camera_matrices,
    detect_sample,
    fit_hand_eye,
    solve_hand_eye,
    validate_hand_eye,
)
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image


def _write_json(path: Path, data: dict[str, Any]) -> None:
    temporary = path.with_suffix(".tmp")
    temporary.write_text(json.dumps(data, indent=2, allow_nan=False) + "\n")
    temporary.replace(path)


def _clear_validation(path: Path, result: dict[str, Any]) -> None:
    result["validated"] = False
    result.pop("validation_metrics", None)
    result.pop("validation_sample_ids", None)
    _write_json(path, result)


def load_session(directory: Path) -> dict[str, Any]:
    data: dict[str, Any] = json.loads((directory / "session.json").read_text())
    if data.get("version") != 1:
        raise ValueError("Unsupported hand-eye session version")
    return data


def open_session(
    directory: Path,
    camera_info: CameraInfo,
    base_frame: str = DEFAULT_BASE_FRAME,
    *,
    ee_frame: str,
    marker_size_m: float = DEFAULT_MARKER_SIZE_M,
    marker_id: int = 0,
    automatic_views: dict[str, Any] | None = None,
) -> dict[str, Any]:
    """Create or resume a session with identical intrinsics and frame settings.

    One operator writes a session at a time. Keep the camera mount, robot base,
    and target fixed for the entire session, including held-out validation.
    """
    if not base_frame or not ee_frame or base_frame == ee_frame:
        raise ValueError("Choose distinct, nonempty base and end-effector frames")
    check_marker(marker_size_m, marker_id)
    checked_camera_matrices(camera_info)
    settings = {
        "base_frame": base_frame,
        "ee_frame": ee_frame,
        "marker_size_m": marker_size_m,
        "marker_id": marker_id,
        "dictionary": TAG_DICTIONARY,
        "intrinsics": camera_info.to_dict(),
    }
    if automatic_views is not None:
        settings["automatic_views"] = automatic_views
    if (directory / "session.json").exists():
        data = load_session(directory)
        if data["settings"] != settings:
            raise ValueError("Session settings changed; use a new session directory")
        return data
    directory.mkdir(parents=True, exist_ok=True)
    data = {
        "version": 1,
        "session_id": uuid4().hex,
        "settings": settings,
        "camera_frame": None,
        "samples": [],
    }
    _write_json(directory / "session.json", data)
    return data


def save_home_tool_pose(
    directory: Path, pose: PoseStamped, tip_from_camera: NDArray[np.float64] | None = None
) -> None:
    """Save the measured home pose and optional orbit mount for repeatable views on resume."""
    position, orientation = pose.position.to_list(), pose.orientation.to_list()
    if (
        pose.frame_id != "world"
        or not np.isfinite(position).all()
        or not np.isclose(np.linalg.norm(orientation), 1.0)
    ):
        raise ValueError("The home tool pose must be a finite world pose with a unit quaternion.")
    data = load_session(directory)
    data["home_tool_pose"] = {
        "frame_id": pose.frame_id,
        "position": position,
        "orientation": orientation,
    }
    if tip_from_camera is not None:
        data["orbit_tip_from_camera"] = tip_from_camera.tolist()
    _write_json(directory / "session.json", data)


def record_sample(
    directory: Path, image: Image, base_from_ee: Transform, *, validation: bool = False
) -> dict[str, Any]:
    """Save a paired image and measured pose after successful marker detection.

    Live callers should obtain the pair through TopicCapture.capture(). Other
    callers are responsible for stationary, timestamp-matched observations.
    """
    data = load_session(directory)
    sample = observe_sample(data, image, base_from_ee)
    return save_observation(directory, data, image, sample, validation=validation)


def observe_sample(data: dict[str, Any], image: Image, base_from_ee: Transform) -> HandEyeSample:
    """Check the stationary image/pose pair and measure the tag before operator review."""
    settings = data["settings"]
    if (base_from_ee.frame_id, base_from_ee.child_frame_id) != (
        settings["base_frame"],
        settings["ee_frame"],
    ):
        raise ValueError("Robot pose frames do not match the session")
    if not image.frame_id or data["camera_frame"] not in (None, image.frame_id):
        raise ValueError("Image optical frame is missing or changed")
    if not np.isfinite(image.ts) or any(s["image_timestamp"] == image.ts for s in data["samples"]):
        raise ValueError("Each observation must have a new, finite image timestamp")
    return detect_sample(
        image,
        CameraInfo(**settings["intrinsics"]),
        base_from_ee.to_matrix(),
        settings["marker_size_m"],
        settings["marker_id"],
    )


def save_observation(
    directory: Path, data: dict[str, Any], image: Image, sample: HandEyeSample, *, validation: bool
) -> dict[str, Any]:
    """Save the exact frame and measurement accepted for this single-writer session."""
    number = int(data["samples"][-1]["id"]) + 1 if data["samples"] else 1
    # An interrupted save can leave a PNG ahead of the manifest. Preserve it on resume.
    while (image_path := directory / f"{number:04d}.png").exists():
        number += 1
    if not image.save(str(image_path)):
        raise RuntimeError(f"Could not save {image_path}")
    entry = {
        "id": image_path.stem,
        "role": "validation" if validation else "train",
        "image": image_path.name,
        "image_timestamp": float(image.ts),
        "pose_query_timestamp": float(image.ts),
        "base_from_ee": sample.base_from_ee.tolist(),
        "camera_from_target": sample.camera_from_target.tolist(),
        "reprojection_error_px": sample.reprojection_error_px,
    }
    data["camera_frame"] = image.frame_id
    data["samples"].append(entry)
    result_path = directory / "hand_eye.json"
    if result_path.exists():
        _clear_validation(result_path, json.loads(result_path.read_text()))
    _write_json(directory / "session.json", data)
    return entry


def _samples(data: dict[str, Any], role: str, limit: int | None = None) -> list[HandEyeSample]:
    entries = (s for s in data["samples"] if s["role"] == role)
    return [
        HandEyeSample(
            np.array(s["base_from_ee"]),
            np.array(s["camera_from_target"]),
            s["reprojection_error_px"],
        )
        for s in islice(entries, limit)
    ]


def _sample_ids(data: dict[str, Any], role: str) -> list[str]:
    return [s["id"] for s in data["samples"] if s["role"] == role]


def estimate_aiming_mount(directory: Path, sample_count: int) -> NDArray[np.float64]:
    """Estimate mount <- camera from the initial training views for aiming subsequent moves."""
    samples = _samples(load_session(directory), "train", limit=sample_count)
    if len(samples) != sample_count:
        raise ValueError(f"Need {sample_count} initial training views to aim the camera")
    try:
        result = fit_hand_eye(samples)
        metrics = result["training_metrics"]
        if not metrics["passed"]:
            raise ValueError(
                f"Initial views disagree on the target pose or reprojection: {metrics}"
            )
    except ValueError as error:
        raise ValueError(
            f"Could not aim the camera from the first {sample_count} training views: {error}. "
            "Check that the tag stays fixed and clearly visible through the tilts."
        ) from error
    return np.asarray(result["camera_to_ee"], dtype=np.float64)


def solve_session(directory: Path) -> dict[str, Any]:
    """Save a candidate result; independent validation is still required."""
    data = load_session(directory)
    result = solve_hand_eye(_samples(data, "train"))
    result.update(
        {
            "version": 1,
            "session_id": data["session_id"],
            "method": "PARK",
            "parent_frame": data["settings"]["ee_frame"],
            "child_frame": data["camera_frame"],
            "convention": "p_ee = camera_to_ee @ p_camera_optical (homogeneous column vectors)",
            "translation_unit": "metre",
            "settings": data["settings"],
            "training_sample_ids": _sample_ids(data, "train"),
            "validated": False,
        }
    )
    _write_json(directory / "hand_eye.json", result)
    return result


def validate_session(directory: Path) -> dict[str, Any]:
    """Check held-out samples without refitting, and save PASS/FAIL in the result."""
    data = load_session(directory)
    path = directory / "hand_eye.json"
    result: dict[str, Any] = json.loads(path.read_text())
    if (
        result["session_id"] != data["session_id"]
        or result["settings"] != data["settings"]
        or result["child_frame"] != data["camera_frame"]
    ):
        raise ValueError("Result does not belong to this session")
    # Clear an earlier PASS even if the new validation fails before producing metrics.
    _clear_validation(path, result)
    validation_ids = _sample_ids(data, "validation")
    if set(validation_ids) & set(result["training_sample_ids"]):
        raise ValueError("Validation samples must not have been used to solve")
    if _sample_ids(data, "train") != result["training_sample_ids"]:
        raise ValueError("Training samples changed. Run solve again before validation.")
    metrics = validate_hand_eye(
        _samples(data, "validation"),
        np.array(result["camera_to_ee"]),
        np.array(result["base_from_target"]),
    )
    result["validation_metrics"] = metrics
    result["validation_sample_ids"] = validation_ids
    result["validated"] = bool(result["training_metrics"]["passed"] and metrics["passed"])
    _write_json(path, result)
    return result
