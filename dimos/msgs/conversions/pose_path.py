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

"""Agent-facing encoding for a complete timed robot path."""

from __future__ import annotations

import base64
from itertools import pairwise
import json
from typing import TYPE_CHECKING, Any

import cv2
import numpy as np

if TYPE_CHECKING:
    from dimos.msgs.nav_msgs.Path import Path


_IMAGE_SIZE = 768
_IMAGE_MARGIN = 72


def _path_arrays(path: Path) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    positions = np.asarray([[pose.x, pose.y, pose.z] for pose in path.poses], dtype=np.float64)
    quaternions = np.asarray(
        [pose.orientation.to_list() for pose in path.poses], dtype=np.float64
    )
    timestamps = np.asarray([float(pose.ts) for pose in path.poses], dtype=np.float64)
    if not np.all(np.isfinite(positions)) or not np.all(np.isfinite(quaternions)):
        raise ValueError("Path poses must contain finite positions and orientations")
    if not np.all(np.isfinite(timestamps)) or np.any(np.diff(timestamps) < 0):
        raise ValueError("Path pose timestamps must be finite and nondecreasing")
    norms = np.linalg.norm(quaternions, axis=1)
    if np.any(norms == 0):
        raise ValueError("Path poses must contain nonzero quaternions")
    return positions, quaternions / norms[:, None], timestamps


def _euler_deg(quaternions: np.ndarray) -> np.ndarray:
    x, y, z, w = quaternions.T
    roll = np.arctan2(2 * (w * x + y * z), 1 - 2 * (x**2 + y**2))
    pitch = np.arcsin(np.clip(2 * (w * y - z * x), -1.0, 1.0))
    yaw = np.arctan2(2 * (w * z + x * y), 1 - 2 * (y**2 + z**2))
    return np.degrees(np.column_stack((roll, pitch, yaw)))


def _plot_geometry(positions: np.ndarray) -> tuple[np.ndarray, float]:
    xy = positions[:, :2]
    low = xy.min(axis=0)
    high = xy.max(axis=0)
    center = (low + high) / 2
    span = max(float(np.max(high - low)), 0.5) * 1.12
    return center, span


def _path_image(positions: np.ndarray, center: np.ndarray, span: float) -> bytes:
    canvas = np.full((_IMAGE_SIZE, _IMAGE_SIZE, 3), 20, dtype=np.uint8)
    xy = positions[:, :2]
    low = xy.min(axis=0)
    high = xy.max(axis=0)
    usable = _IMAGE_SIZE - 2 * _IMAGE_MARGIN
    scale = usable / span

    def pixel(point: np.ndarray) -> tuple[int, int]:
        px = round(_IMAGE_SIZE / 2 + (point[0] - center[0]) * scale)
        py = round(_IMAGE_SIZE / 2 - (point[1] - center[1]) * scale)
        return px, py

    for fraction in np.linspace(-0.5, 0.5, 11):
        offset = round(fraction * usable)
        cv2.line(
            canvas,
            (_IMAGE_SIZE // 2 + offset, _IMAGE_MARGIN),
            (_IMAGE_SIZE // 2 + offset, _IMAGE_SIZE - _IMAGE_MARGIN),
            (43, 43, 43),
            1,
        )
        cv2.line(
            canvas,
            (_IMAGE_MARGIN, _IMAGE_SIZE // 2 + offset),
            (_IMAGE_SIZE - _IMAGE_MARGIN, _IMAGE_SIZE // 2 + offset),
            (43, 43, 43),
            1,
        )

    pixels = [pixel(point) for point in xy]
    denominator = max(len(pixels) - 1, 1)
    for index, (start, end) in enumerate(pairwise(pixels)):
        fraction = index / denominator
        color = (int(255 * (1 - fraction)), int(170 + 70 * fraction), int(255 * fraction))
        cv2.line(canvas, start, end, color, 3, cv2.LINE_AA)

    cv2.circle(canvas, pixels[0], 9, (70, 220, 110), -1, cv2.LINE_AA)
    cv2.circle(canvas, pixels[-1], 9, (70, 90, 255), -1, cv2.LINE_AA)
    cv2.putText(canvas, "START", (pixels[0][0] + 12, pixels[0][1] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (160, 255, 185), 2, cv2.LINE_AA)
    cv2.putText(canvas, "END", (pixels[-1][0] + 12, pixels[-1][1] + 22), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (160, 180, 255), 2, cv2.LINE_AA)
    cv2.putText(canvas, "+Y", (_IMAGE_MARGIN, 38), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (210, 210, 210), 2, cv2.LINE_AA)
    cv2.putText(canvas, "+X", (_IMAGE_SIZE - 55, _IMAGE_SIZE - 28), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (210, 210, 210), 2, cv2.LINE_AA)
    cv2.putText(
        canvas,
        f"X [{low[0]:.2f}, {high[0]:.2f}] m   Y [{low[1]:.2f}, {high[1]:.2f}] m",
        (24, _IMAGE_SIZE - 28),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.48,
        (180, 180, 180),
        1,
        cv2.LINE_AA,
    )
    success, encoded = cv2.imencode(".png", canvas)
    if not success:
        raise ValueError("Failed to encode path visualization as PNG")
    return encoded.tobytes()


def pose_path_agent_encode(path: Path) -> list[dict[str, Any]]:
    """Encode complete path evidence without generic stream downsampling."""
    if not path.poses:
        return [{"type": "text", "text": f"Robot trajectory frame={path.frame_id!r} is empty."}]

    positions, quaternions, timestamps = _path_arrays(path)
    plot_center, plot_span = _plot_geometry(positions)
    t0 = timestamps[0]
    euler = _euler_deg(quaternions)
    rows = np.column_stack((timestamps - t0, positions, quaternions, euler))
    summary = {
        "schema": "dimos.pose_path.v1",
        "frame_id": path.frame_id,
        "pose_count": len(path.poses),
        "visualization": {
            "image_size_px": _IMAGE_SIZE,
            "margin_px": _IMAGE_MARGIN,
            "center_m": np.round(plot_center, 6).tolist(),
            "span_m": round(plot_span, 6),
        },
        "columns": [
            "t_s",
            "x_m",
            "y_m",
            "z_m",
            "qx",
            "qy",
            "qz",
            "qw",
            "roll_deg",
            "pitch_deg",
            "yaw_deg",
        ],
        "poses": np.round(rows, 6).tolist(),
    }
    description = (
        "Complete ordered robot trajectory. The top-down image has equal X/Y scale, +X right, "
        "+Y up, green START, red END, and line color progressing from cyan to yellow with time. "
        "The JSON contains every pose in order without downsampling. Position is in meters. "
        "Quaternion components use [x, y, z, w]; roll, pitch, and yaw are intrinsic XYZ angles "
        "in degrees. Robot forward/left/up are local +X/+Y/+Z. Compute traveled distance as the "
        "sum of consecutive 3D position distances. Tilt is the angle between robot local +Z and "
        "world +Z. One-second windowed speed is distance traveled over a centered 1.0-second "
        "window. A self-intersection means non-adjacent XY path segments cross; ordinary shared "
        "endpoints of consecutive segments do not count. Trajectory JSON: "
        f"{json.dumps(summary, separators=(',', ':'))}"
    )
    image = base64.b64encode(_path_image(positions, plot_center, plot_span)).decode("ascii")
    return [
        {"type": "text", "text": description},
        {"type": "image_url", "image_url": {"url": f"data:image/png;base64,{image}"}},
    ]
