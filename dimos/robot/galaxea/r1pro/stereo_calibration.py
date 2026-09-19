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

"""The R1 Pro head's stereo calibration: what ``calibration.json`` holds and how it is read.

The head is two monocular cameras that Galaxea never calibrated *as a pair*.
Each eye publishes its own factory intrinsics on the wire
(``/calib/head_{left,right}/camera_info``, forwarded by the connection as
``head_left_info``/``head_right_info``), but nothing on the robot says how the
right eye is aimed relative to the left, and that is the number stereo matching
lives or dies on: a relative yaw shifts every disparity by the same amount, a
relative pitch puts the two pictures on different rows. ``calibrate_stereo.py``
fits those angles and the baseline from a recording; this module is the file it
writes and the loader everything else reads it through.

The numbers belong to a rig, not to the model. They change when the head is
unbolted and not otherwise, so the committed :data:`R1PRO_HEAD_CALIBRATION` is
the fallback for a robot that has never been fitted, and the file at
:data:`DEFAULT_PATH` (or wherever ``DIMOS_R1_STEREO_CALIBRATION`` points) wins
when it exists.
"""

from __future__ import annotations

import json
import os
from pathlib import Path
from typing import Any

from pydantic import BaseModel, ConfigDict, Field, ValidationError

from dimos.utils.logging_config import setup_logger

logger = setup_logger()

ENV_VAR = "DIMOS_R1_STEREO_CALIBRATION"
DEFAULT_PATH = Path("~/.dimos/r1pro/calibration.json")

# The relative rotation is a mount error. Anything past ~11 degrees is not a
# misaligned eye, it is a wrong file, and StereoCloudConfig refuses it too.
_MAX_RELATIVE_ANGLE_RAD = 0.2


class StereoCalibrationError(ValueError):
    """A calibration file that exists but cannot be used, naming the file and the field."""


class EyeIntrinsics(BaseModel):
    """One eye's pinhole model, as a CameraInfo carries it.

    The R1 publishes these itself, so the stereo module never reads them from
    here. They are written down so the file records *which* intrinsics the
    angles were fitted against: a fit is only as good as the intrinsics it
    assumed, and a factory re-flash would change them under it silently.
    """

    model_config = ConfigDict(extra="forbid")

    width: int = Field(gt=0)
    height: int = Field(gt=0)
    fx: float = Field(gt=0.0)
    fy: float = Field(gt=0.0)
    cx: float
    cy: float
    distortion_model: str = "plumb_bob"
    distortion: list[float] = Field(default_factory=list)


class R1StereoCalibration(BaseModel):
    """On-disk schema of ``calibration.json``.

    ``schema`` is bumped when a field changes meaning, so an old file is
    refused rather than silently read as something it is not.
    """

    model_config = ConfigDict(extra="forbid", populate_by_name=True)

    # ``schema`` on disk; a field literally named ``schema`` would shadow
    # pydantic's own ``BaseModel.schema``.
    schema_version: int = Field(default=1, alias="schema", ge=1, le=1)

    # Distance between the two eyes. Depth scales linearly with it.
    baseline_m: float = Field(gt=0.0)
    # How the right eye is aimed relative to the left, in the left eye's frame.
    right_roll_rad: float = Field(ge=-_MAX_RELATIVE_ANGLE_RAD, le=_MAX_RELATIVE_ANGLE_RAD)
    right_pitch_rad: float = Field(ge=-_MAX_RELATIVE_ANGLE_RAD, le=_MAX_RELATIVE_ANGLE_RAD)
    right_yaw_rad: float = Field(ge=-_MAX_RELATIVE_ANGLE_RAD, le=_MAX_RELATIVE_ANGLE_RAD)

    # Optional: the robot self-reports intrinsics on the wire, so a file may
    # pin them for the record without the loader needing them.
    left: EyeIntrinsics | None = None
    right: EyeIntrinsics | None = None

    # Provenance. Where the numbers came from, so a bad cloud can be traced to
    # the capture it was fitted on rather than argued about.
    source_recording: str | None = None
    # ISO 8601.
    fitted_at: str | None = None
    pairs_used: int | None = None
    # Whatever the fitter measured about its own fit, by name -- e.g. the
    # residual row misalignment in pixels, or the fraction of lidar points the
    # stereo cloud agreed with. Free-form because the fitter's metrics will
    # change faster than this schema; the fitter documents its own keys.
    score: dict[str, float] | None = None

    notes: str | None = None

    def to_json(self) -> str:
        return json.dumps(self.model_dump(by_alias=True), indent=2) + "\n"


R1PRO_HEAD_CALIBRATION = R1StereoCalibration(
    # The URDF's head camera joints: y +0.059919 and y -0.060276.
    baseline_m=0.120195,
    right_roll_rad=-0.002,
    right_pitch_rad=0.0035,
    right_yaw_rad=-0.01275,
    source_recording="20260915-103437-r1pro-kronknav",
    notes=(
        "Fitted against the chassis lidar on 20260915-103437-r1pro-kronknav. These "
        "are the properties of that one rig's head mount and travel with the robot, "
        "not the model: refit after the head is unbolted, and do not copy to another "
        "R1 Pro."
    ),
)


def resolve_calibration_path(path: Path | str | None = None) -> tuple[Path, bool]:
    """Where to read from, and whether the caller named it.

    The second value decides what a missing file means: a path the caller
    passed is an assertion that the file exists, so its absence is an error;
    the environment and the default are only *places to look*, so their absence
    falls back to the committed numbers.
    """
    if path is not None:
        return Path(path).expanduser(), True
    from_env = os.environ.get(ENV_VAR)
    if from_env:
        return Path(from_env).expanduser(), False
    return DEFAULT_PATH.expanduser(), False


def write_stereo_calibration(path: Path | str, calibration: R1StereoCalibration) -> Path:
    """Write pretty JSON, creating the directory. Returns the path written."""
    target = Path(path).expanduser()
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_text(calibration.to_json())
    return target


def _field_path(error: dict[str, Any]) -> str:
    return ".".join(str(part) for part in error["loc"]) or "<root>"


def parse_stereo_calibration(text: str, *, source: str) -> R1StereoCalibration:
    """Parse the JSON text of a calibration file.

    Pydantic's own error is verbose and never says which file; this rewraps it
    so the message a user sees on the robot is the file and the field.
    """
    try:
        raw = json.loads(text)
    except json.JSONDecodeError as error:
        raise StereoCalibrationError(
            f"{source}: not valid JSON (line {error.lineno}, column {error.colno}: {error.msg})"
        ) from error
    try:
        return R1StereoCalibration.model_validate(raw)
    except ValidationError as error:
        problems = "; ".join(
            f"field {_field_path(item)!r}: {item['msg']}" for item in error.errors()
        )
        raise StereoCalibrationError(f"{source}: {problems}") from error


def load_stereo_calibration(path: Path | str | None = None) -> R1StereoCalibration:
    """The calibration to use on this machine.

    ``None`` looks at ``DIMOS_R1_STEREO_CALIBRATION``, then the default path;
    if neither names an existing file the committed rig numbers are returned
    and one INFO line says so. A path passed explicitly must exist.
    """
    target, explicit = resolve_calibration_path(path)
    if not target.exists():
        if explicit:
            raise FileNotFoundError(f"stereo calibration file not found: {target}")
        logger.info(
            "no stereo calibration at %s (set %s to point elsewhere); using the committed "
            "R1PRO_HEAD_CALIBRATION",
            target,
            ENV_VAR,
        )
        return R1PRO_HEAD_CALIBRATION
    return parse_stereo_calibration(target.read_text(), source=str(target))


def stereo_cloud_kwargs(calibration: R1StereoCalibration) -> dict[str, float]:
    """Exactly the ``StereoCloudConfig`` fields a calibration decides.

    Intrinsics are deliberately not among them: the module reads those off the
    ``head_*_info`` ports, which are the robot's own and always current.
    """
    return {
        "baseline_m": calibration.baseline_m,
        "right_roll_rad": calibration.right_roll_rad,
        "right_pitch_rad": calibration.right_pitch_rad,
        "right_yaw_rad": calibration.right_yaw_rad,
    }
