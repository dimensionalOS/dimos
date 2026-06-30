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

from __future__ import annotations

from io import BytesIO
import json
import time
from typing import BinaryIO, cast

import numpy as np

from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.types.timestamped import Timestamped

DEFAULT_RERUN_SURFACE_LEVEL = 0.1


class TSDFGrid(Timestamped):
    """Dense TSDF grid with min-corner origin semantics.

    ``distances`` is always shaped ``(1, X, Y, Z)``. Voxel ``[0, 0, 0]`` is at
    ``origin`` in ``frame_id``. The metric center of voxel ``[i, j, k]`` is
    ``origin + [i, j, k] * voxel_size``.
    """

    msg_name = "reconstruction_msgs.TSDFGrid"

    def __init__(
        self,
        distances: np.ndarray,
        voxel_size: float,
        truncation_distance: float,
        origin: Vector3 | None = None,
        weights: np.ndarray | None = None,
        frame_id: str = "world",
        ts: float | None = None,
    ) -> None:
        self.distances = _validate_distances(distances)
        self.voxel_size = float(voxel_size)
        self.truncation_distance = float(truncation_distance)
        self.origin = origin if origin is not None else Vector3()
        self.weights = (
            _validate_weights(weights, self.distances.shape) if weights is not None else None
        )
        self.frame_id = frame_id
        self.ts = ts if ts is not None else time.time()

    @property
    def resolution(self) -> tuple[int, int, int]:
        shape = self.distances.shape
        return (shape[1], shape[2], shape[3])

    @property
    def size(self) -> tuple[float, float, float]:
        x, y, z = self.resolution
        return (x * self.voxel_size, y * self.voxel_size, z * self.voxel_size)

    def voxel_position(self, i: int, j: int, k: int) -> Vector3:
        """Return the metric min-corner position for a voxel index."""
        return Vector3(
            self.origin.x + i * self.voxel_size,
            self.origin.y + j * self.voxel_size,
            self.origin.z + k * self.voxel_size,
        )

    def lcm_encode(self) -> bytes:
        """Encode as a compact NumPy container for LCM transport."""
        metadata = {
            "voxel_size": self.voxel_size,
            "truncation_distance": self.truncation_distance,
            "origin": [self.origin.x, self.origin.y, self.origin.z],
            "frame_id": self.frame_id,
            "ts": self.ts,
        }
        out = BytesIO()
        if self.weights is not None:
            np.savez_compressed(
                out,
                distances=self.distances,
                metadata=np.asarray(json.dumps(metadata), dtype=np.bytes_),
                weights=self.weights,
            )
        else:
            np.savez_compressed(
                out,
                distances=self.distances,
                metadata=np.asarray(json.dumps(metadata), dtype=np.bytes_),
            )
        return out.getvalue()

    encode = lcm_encode

    @classmethod
    def lcm_decode(cls, data: bytes | BinaryIO) -> TSDFGrid:
        raw = _read_bytes(data)
        with np.load(BytesIO(raw), allow_pickle=False) as archive:
            metadata_raw = archive["metadata"].item()
            metadata_text = (
                metadata_raw.decode("utf-8")
                if isinstance(metadata_raw, bytes)
                else str(metadata_raw)
            )
            metadata = _as_metadata(json.loads(metadata_text))
            weights = archive["weights"] if "weights" in archive.files else None
            origin_values = _as_float_list(metadata["origin"], expected_len=3)
            return cls(
                distances=archive["distances"],
                voxel_size=_as_float(metadata["voxel_size"]),
                truncation_distance=_as_float(metadata["truncation_distance"]),
                origin=Vector3(origin_values),
                weights=weights,
                frame_id=str(metadata["frame_id"]),
                ts=_as_float(metadata["ts"]),
            )

    decode = lcm_decode

    def to_rerun(self, max_points: int = 25_000) -> object:
        """Visualize near-surface TSDF voxels as Rerun points."""
        import rerun as rr  # type: ignore[import-not-found]

        field = self.distances[0]
        surface_mask = np.abs(field) <= DEFAULT_RERUN_SURFACE_LEVEL
        if self.weights is not None:
            weights = self.weights[0] if self.weights.ndim == 4 else self.weights
            surface_mask = np.logical_and(surface_mask, weights > 0.0)
        surface = np.argwhere(surface_mask)
        if len(surface) == 0:
            return rr.Points3D([])
        if len(surface) > max_points:
            stride = int(np.ceil(len(surface) / max_points))
            surface = surface[::stride]

        origin = np.array([self.origin.x, self.origin.y, self.origin.z], dtype=np.float32)
        points = origin + surface.astype(np.float32) * np.float32(self.voxel_size)
        values = np.clip(np.abs(field[tuple(surface.T)]), 0.0, 1.0)
        colors = np.stack(
            [
                (255.0 * (1.0 - values)).astype(np.uint8),
                (180.0 * values).astype(np.uint8),
                np.full_like(values, 255, dtype=np.uint8),
            ],
            axis=1,
        )
        return rr.Points3D(points, colors=colors, radii=self.voxel_size * 0.5)


def _validate_distances(distances: np.ndarray) -> np.ndarray:
    arr = np.asarray(distances, dtype=np.float32)
    if arr.ndim != 4 or arr.shape[0] != 1:
        raise ValueError(f"TSDF distances must have shape (1, X, Y, Z), got {arr.shape}")
    return np.ascontiguousarray(arr)


def _validate_weights(weights: np.ndarray, distances_shape: tuple[int, ...]) -> np.ndarray:
    arr = np.asarray(weights, dtype=np.float32)
    if arr.shape == distances_shape[1:]:
        return np.ascontiguousarray(arr)
    if arr.shape == distances_shape:
        return np.ascontiguousarray(arr)
    raise ValueError(
        f"TSDF weights must have shape {distances_shape[1:]} or {distances_shape}, got {arr.shape}"
    )


def _read_bytes(data: bytes | BinaryIO) -> bytes:
    if isinstance(data, bytes):
        return data
    return data.read()


def _as_metadata(value: object) -> dict[str, object]:
    if not isinstance(value, dict):
        raise ValueError("TSDF metadata must decode to a JSON object")
    return cast("dict[str, object]", value)


def _as_float(value: object) -> float:
    if not isinstance(value, int | float | str):
        raise ValueError(f"Expected a float-compatible value, got {type(value).__name__}")
    return float(value)


def _as_float_list(value: object, *, expected_len: int) -> list[float]:
    if not isinstance(value, list) or len(value) != expected_len:
        raise ValueError(f"Expected a list of {expected_len} floats")
    return [float(item) for item in value]
