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

"""Saves images as JPEG and point clouds as XYZ JSON for notify attachments."""

from __future__ import annotations

from dataclasses import dataclass
import hashlib
import json
from pathlib import Path
from typing import Literal, TypeAlias, cast

import numpy as np

from dimos.agents.harness.runtime.constants import (
    EXPORTS_DIR,
    JPEG_QUALITY,
    JSON,
    MAX_EXPORT_FILES,
    MAX_EXPORT_POINTS,
)
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2

Exportable: TypeAlias = Image | PointCloud2


@dataclass(frozen=True)
class ImageExport:
    path: str
    sha256: str
    kind: Literal["image"] = "image"

    def to_json(self) -> dict[str, JSON]:
        return {"path": self.path, "kind": self.kind, "sha256": self.sha256}


@dataclass(frozen=True)
class PointsExport:
    path: str
    sha256: str
    points: int
    """Points written, after dropping non-finite points and applying the stride."""
    source_points: int
    """Points in the message."""
    kind: Literal["points"] = "points"

    def to_json(self) -> dict[str, JSON]:
        return {
            "path": self.path,
            "kind": self.kind,
            "points": self.points,
            "source_points": self.source_points,
            "sha256": self.sha256,
        }


Export: TypeAlias = ImageExport | PointsExport


def jpeg_bytes(image: Image, quality: int) -> bytes:
    """The image as JPEG. A DEPTH image becomes grayscale, its finite range spread over 0-255."""
    if image.format != ImageFormat.DEPTH:
        return image.to_jpeg_bytes(quality=quality)
    depth = np.asarray(image.data, dtype=np.float64)
    finite = np.isfinite(depth)
    gray = np.zeros(depth.shape, dtype=np.uint8)
    if finite.any():
        low = depth[finite].min()
        span = depth[finite].max() - low
        if span > 0:
            gray[finite] = np.round((depth[finite] - low) * (255 / span)).astype(np.uint8)
    return Image(
        data=gray, format=ImageFormat.GRAY, frame_id=image.frame_id, ts=image.ts
    ).to_jpeg_bytes(quality=quality)


def export(message: Exportable, stem: str, directory: Path = EXPORTS_DIR) -> Export:
    """Write message as directory/stem.jpg or .json and drop the oldest files past the cap."""
    directory.mkdir(parents=True, exist_ok=True)
    if isinstance(message, Image):
        path = directory / f"{stem}.jpg"
        data = jpeg_bytes(message, JPEG_QUALITY)
        path.write_bytes(data)
        result: Export = ImageExport(str(path), hashlib.sha256(data).hexdigest())
    else:
        points, colors = message.as_numpy()
        finite = np.isfinite(points[:, :3]).all(axis=1)
        kept = points[finite]
        stride = max(1, -(-len(kept) // MAX_EXPORT_POINTS))
        selected = kept[::stride, :3]
        cloud: dict[str, JSON] = {
            "points": selected.astype(float).round(4).tolist(),
            "frame": message.frame_id,
        }
        if message.ts is not None:
            cloud["timestamp"] = float(message.ts)
        if colors is not None and len(colors) == len(points):
            # PointCloud2 colors are in [0, 1]; the XYZ JSON export uses 0-255.
            cloud["colors"] = (
                (colors[finite][::stride, :3] * 255).round().clip(0, 255).astype(int).tolist()
            )
        path = directory / f"{stem}.json"
        text = json.dumps(cloud, allow_nan=False)
        path.write_text(text)
        result = PointsExport(
            str(path), hashlib.sha256(text.encode()).hexdigest(), len(selected), len(points)
        )
    dated: list[tuple[float, Path]] = []
    for item in directory.iterdir():
        try:
            dated.append((item.stat().st_mtime, item))
        except FileNotFoundError:
            # Another process exporting into the directory pruned it.
            continue
    for _, old in sorted(dated)[:-MAX_EXPORT_FILES]:
        old.unlink(missing_ok=True)
    return result


def replace_exports(
    data: dict[str, object], stem: str, directory: Path = EXPORTS_DIR
) -> tuple[dict[str, JSON], list[Export]]:
    """data with every Image and PointCloud2 (at any depth) replaced by its export's path.

    Raises TypeError, naming the key, for a value that is neither exportable nor JSON, such
    as an infinite or NaN number.
    """
    exports: list[Export] = []

    def replace(value: object) -> JSON:
        if isinstance(value, Image | PointCloud2):
            exports.append(export(value, f"{stem}.{len(exports)}", directory))
            return exports[-1].path
        if isinstance(value, dict):
            return {str(key): replace(item) for key, item in value.items()}
        if isinstance(value, list | tuple):
            return [replace(item) for item in value]
        json.dumps(value, allow_nan=False)
        return cast("JSON", value)

    result: dict[str, JSON] = {}
    for key, value in data.items():
        try:
            result[key] = replace(value)
        except (TypeError, ValueError) as error:
            raise TypeError(
                f"notify() data {key!r} must be JSON, an Image or a PointCloud2, "
                f"or lists and dicts of those: {error}"
            ) from error
    return result, exports
