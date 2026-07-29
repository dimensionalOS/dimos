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

"""Deterministic YCB scene input loaded through the repository data system."""

from __future__ import annotations

import hashlib
import json
from pathlib import Path
import re
from typing import TypedDict

import numpy as np

from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.utils.data import get_data

DATASET = "graspgenx_ycb_banana_scene"
SCENE_FILE = "scene.npz"
METADATA_FILE = "scene.json"
BANANA_LABEL = 0


class SceneMetadata(TypedDict, total=False):
    counts: dict[str, int]
    labels: dict[str, dict[str, str]]
    frame: str
    timestamp: float
    final_sha256: str
    source: dict[str, object]


def _dataset_dir() -> Path:
    return get_data(DATASET)


def _validate_record(
    path: Path,
    points: np.ndarray,
    labels: np.ndarray,
    timestamp: float,
    metadata: SceneMetadata,
) -> None:
    expected_sha = metadata.get("final_sha256")
    if not isinstance(expected_sha, str) or not re.fullmatch(r"[0-9a-f]{64}", expected_sha):
        raise ValueError("scene fixture metadata must contain a valid final SHA-256")
    if hashlib.sha256(path.read_bytes()).hexdigest() != expected_sha:
        raise ValueError("scene fixture bytes do not match provenance SHA-256")

    expected_counts = {"banana": 3500, "table": 256, "distractor": 48, "total": 3804}
    if metadata.get("counts") != expected_counts or expected_counts["total"] != len(points):
        raise ValueError("scene fixture counts do not match bytes")
    if (metadata.get("labels") or {}).get("encoding") != {
        "0": "banana",
        "1": "table",
        "2": "distractor",
    }:
        raise ValueError("scene fixture label encoding metadata is invalid")

    actual_counts = dict(zip(*np.unique(labels, return_counts=True), strict=True))
    if {str(label): int(count) for label, count in actual_counts.items()} != {
        "0": 3500,
        "1": 256,
        "2": 48,
    }:
        raise ValueError("scene fixture label values do not match metadata")
    if metadata.get("frame") != "world" or timestamp != float(
        metadata.get("timestamp", float("nan"))
    ):
        raise ValueError("scene fixture frame/timestamp metadata is invalid")

    source = metadata.get("source")
    if not isinstance(source, dict) or source.get("path") != (
        "assets/sample_data/object_mesh/banana.obj"
    ):
        raise ValueError("scene fixture source OBJ provenance is incomplete")
    source_sha = source.get("source_obj_sha256")
    if not isinstance(source_sha, str) or not re.fullmatch(r"[0-9a-f]{64}", source_sha):
        raise ValueError("scene fixture source OBJ must have its 64-character SHA-256")


def load_scene_record(
    dataset_dir: Path | None = None,
) -> tuple[np.ndarray, np.ndarray, SceneMetadata]:
    """Load stored XYZ points, semantic labels, and provenance metadata."""
    root = dataset_dir if dataset_dir is not None else _dataset_dir()
    scene_path = root / SCENE_FILE
    metadata_path = root / METADATA_FILE
    with np.load(scene_path, allow_pickle=False) as data:
        points = np.asarray(data["points"], dtype=np.float32)
        labels = np.asarray(data["labels"], dtype=np.uint8)
        timestamp = float(np.asarray(data["timestamp"]).item())
    if points.shape != (len(labels), 3) or not np.all(np.isfinite(points)):
        raise ValueError("scene fixture has invalid points or labels")
    metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
    _validate_record(scene_path, points, labels, timestamp, metadata)
    return points, labels, metadata


def load_demo_clouds(
    dataset_dir: Path | None = None,
) -> tuple[PointCloud2, PointCloud2]:
    """Return the complete scene and its labeled banana Object Point Cloud."""
    points, labels, metadata = load_scene_record(dataset_dir)
    timestamp = float(metadata["timestamp"])
    scene = PointCloud2.from_numpy(points, frame_id="world", timestamp=timestamp)
    object_cloud = PointCloud2.from_numpy(
        points[labels == BANANA_LABEL],
        frame_id="world",
        timestamp=timestamp,
    )
    return scene, object_cloud
