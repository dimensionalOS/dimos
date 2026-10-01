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

"""Calibration construction and array access for generated ROS CameraInfo values."""

from copy import deepcopy
import math
from pathlib import Path
import re
from typing import Any, Literal

from dimos_generated.builtin_interfaces.msg import Time
from dimos_generated.sensor_msgs.msg import CameraInfo
from dimos_generated.std_msgs.msg import Header
import numpy as np
from numpy.typing import NDArray
import yaml


def _dimensions(width: int, height: int) -> None:
    if any(
        isinstance(value, bool) or not isinstance(value, int) or value <= 0
        for value in (width, height)
    ):
        raise ValueError("Calibration width and height must be positive integers")


def camera_info_from_intrinsics(
    fx: float, fy: float, cx: float, cy: float, width: int, height: int, *, header: Header
) -> CameraInfo:
    """Construct an undistorted pinhole calibration with an explicit source header."""
    _dimensions(width, height)
    if not all(math.isfinite(value) for value in (fx, fy, cx, cy)) or fx <= 0 or fy <= 0:
        raise ValueError("Intrinsics must be finite with positive focal lengths")
    return CameraInfo(
        header=header,
        width=width,
        height=height,
        distortion_model="plumb_bob",
        d=[0.0] * 5,
        k=[fx, 0, cx, 0, fy, cy, 0, 0, 1],
        r=[1, 0, 0, 0, 1, 0, 0, 0, 1],
        p=[fx, 0, cx, 0, 0, fy, cy, 0, 0, 0, 1, 0],
    )


def camera_info_from_fov(
    fov_degrees: float,
    width: int,
    height: int,
    *,
    header: Header,
    axis: Literal["horizontal", "vertical"] = "vertical",
) -> CameraInfo:
    """Construct a centered pinhole model with square pixels from an axis's FOV."""
    _dimensions(width, height)
    if not math.isfinite(fov_degrees) or not 0 < fov_degrees < 180:
        raise ValueError("Field of view must be finite and between 0 and 180 degrees")
    if axis not in ("horizontal", "vertical"):
        raise ValueError("Field-of-view axis must be horizontal or vertical")
    extent = width if axis == "horizontal" else height
    focal = extent / (2 * math.tan(math.radians(fov_degrees) / 2))
    return camera_info_from_intrinsics(
        focal, focal, width / 2, height / 2, width, height, header=header
    )


def _matrix(data: dict[str, Any], name: str, rows: int, columns: int | None) -> list[float]:
    matrix = data[name]
    if not isinstance(matrix, dict) or matrix.get("rows") != rows:
        raise ValueError(f"{name} must declare {rows} rows")
    actual_columns = matrix.get("cols")
    if (
        not isinstance(actual_columns, int)
        or actual_columns < 0
        or (columns is not None and actual_columns != columns)
    ):
        raise ValueError(f"{name} has invalid columns")
    values = matrix.get("data")
    if not isinstance(values, list) or len(values) != rows * actual_columns:
        raise ValueError(f"{name} data length does not match its dimensions")
    converted = [float(value) for value in values]
    if not all(math.isfinite(value) for value in converted):
        raise ValueError(f"{name} must contain finite values")
    return converted


def camera_info_from_yaml(path: str | Path, *, header: Header) -> CameraInfo:
    """Read a ROS calibration YAML file, validating declared matrix dimensions."""
    with Path(path).open() as source:
        data = yaml.safe_load(source)
    if not isinstance(data, dict):
        raise ValueError("Calibration YAML must contain a mapping")
    width, height = data["image_width"], data["image_height"]
    _dimensions(width, height)
    return CameraInfo(
        header=header,
        width=width,
        height=height,
        distortion_model=data["distortion_model"],
        k=_matrix(data, "camera_matrix", 3, 3),
        d=_matrix(data, "distortion_coefficients", 1, None),
        r=_matrix(data, "rectification_matrix", 3, 3),
        p=_matrix(data, "projection_matrix", 3, 4),
    )


def intrinsic_matrix(message: CameraInfo) -> NDArray[np.float64]:
    """Copy the 3x3 intrinsic matrix into independently mutable NumPy storage."""
    return np.array(message.k, dtype=np.float64, copy=True).reshape(3, 3)


def camera_info_with_stamp(message: CameraInfo, stamp: Time) -> CameraInfo:
    """Copy a calibration with an exact source stamp, without mutating prior publishes."""
    result = deepcopy(message)
    result.header.stamp = deepcopy(stamp)
    return result


class CalibrationProvider:
    """Lazily load named YAML calibrations as plain generated CameraInfo values.

    Both SingleWebcam and single_webcam resolve single_webcam.yaml. This
    helper lives outside generated messages; calibration files stay local.
    """

    def __init__(self, calibration_dir: str | Path, *, frame_id: str = "camera_optical") -> None:
        self._calibration_dir = Path(calibration_dir)
        self._frame_id = frame_id
        self._cache: dict[str, CameraInfo] = {}

    @staticmethod
    def _to_snake_case(name: str) -> str:
        first = re.sub("(.)([A-Z][a-z]+)", r"\1_\2", name)
        return re.sub("([a-z0-9])([A-Z])", r"\1_\2", first).lower()

    def __getattr__(self, name: str) -> CameraInfo:
        if not re.fullmatch(r"[A-Za-z][A-Za-z0-9_]*", name):
            raise AttributeError(f"No calibration file found for: {name}")
        snake = self._to_snake_case(name)
        for key in (name, snake):
            if key in self._cache:
                return self._cache[key]
        for key in (name, snake):
            path = self._calibration_dir / f"{key}.yaml"
            if path.is_file():
                value = camera_info_from_yaml(path, header=Header(frame_id=self._frame_id))
                self._cache[name] = value
                self._cache[snake] = value
                return value
        raise AttributeError(f"No calibration file found for: {name}")

    def __dir__(self) -> list[str]:
        names: set[str] = set()
        for path in self._calibration_dir.glob("*.yaml"):
            names.add(path.stem)
            names.add("".join(word.capitalize() for word in path.stem.split("_")))
        return sorted(names)
