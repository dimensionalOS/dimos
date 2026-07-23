# Copyright 2026 Dimensional Inc.
"""Recorded YCB scene input. This module never synthesizes runtime points."""

import hashlib
import json
from pathlib import Path
import re
from typing import TypedDict

import numpy as np

from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2

from .extractor import AxisAlignedROI

FIXTURE = Path(__file__).parent / "fixtures" / "ycb_banana_scene.npz"
METADATA = FIXTURE.with_suffix(".json")


def fixture_roi(path: Path = METADATA) -> AxisAlignedROI:
    """Return the deployment ROI recorded with the fixture provenance."""
    metadata = json.loads(path.read_text(encoding="utf-8"))
    roi = metadata.get("roi")
    if not isinstance(roi, dict):
        raise ValueError("scene fixture ROI metadata is missing")
    return AxisAlignedROI(tuple(roi["min"]), tuple(roi["max"]))


class SceneMetadata(TypedDict, total=False):
    counts: dict[str, int]
    labels: dict[str, dict[str, str]]
    roi: dict[str, object]
    frame: str
    timestamp: float
    final_sha256: str
    source: dict[str, object]


def load_ycb_scene(path: Path = FIXTURE) -> PointCloud2:
    """Load the recorded world-frame scene as a PointCloud2."""
    points, labels, metadata = load_scene_record(path)
    timestamp = float(metadata.get("timestamp", 0.0))
    if metadata.get("frame") != "world":
        raise ValueError("scene fixture metadata frame must be world")
    return PointCloud2.from_numpy(points, frame_id="world", timestamp=timestamp)


def _validate_record(
    path: Path, points: np.ndarray, labels: np.ndarray, timestamp: float, metadata: SceneMetadata
) -> None:
    expected_sha = metadata.get("final_sha256")
    if not isinstance(expected_sha, str) or not re.fullmatch(r"[0-9a-f]{64}", expected_sha):
        raise ValueError("scene fixture metadata must contain a valid final SHA-256")
    if hashlib.sha256(path.read_bytes()).hexdigest() != expected_sha:
        raise ValueError("scene fixture bytes do not match provenance SHA-256")
    counts = metadata.get("counts", {})
    expected_counts = {"banana": 3500, "table": 256, "distractor": 48, "total": 3804}
    if counts != expected_counts or counts.get("total") != len(points):
        raise ValueError("scene fixture counts do not match bytes")
    encoding = (metadata.get("labels") or {}).get("encoding")
    if encoding != {"0": "banana", "1": "table", "2": "distractor"}:
        raise ValueError("scene fixture label encoding metadata is invalid")
    actual_counts = dict(zip(*np.unique(labels, return_counts=True), strict=True))
    if {str(label): int(count) for label, count in actual_counts.items()} != {
        "0": 3500,
        "1": 256,
        "2": 48,
    }:
        raise ValueError("scene fixture label values do not match metadata")
    if metadata.get("frame") != "world" or not isinstance(metadata.get("timestamp"), (int, float)):
        raise ValueError("scene fixture frame/timestamp metadata is invalid")
    if timestamp != float(metadata.get("timestamp", float("nan"))):
        raise ValueError("scene fixture NPZ timestamp does not match metadata")
    bounds = metadata.get("bounds")
    if (
        not isinstance(bounds, dict)
        or not np.allclose(points.min(axis=0), bounds["min"])
        or not np.allclose(points.max(axis=0), bounds["max"])
    ):
        raise ValueError("scene fixture bounds do not match bytes")
    source = metadata.get("source")
    if (
        not isinstance(source, dict)
        or source.get("path") != "assets/sample_data/object_mesh/banana.obj"
    ):
        raise ValueError("scene fixture source OBJ provenance is incomplete")
    source_sha = source.get("source_obj_sha256")
    if not isinstance(source_sha, str) or not re.fullmatch(r"[0-9a-f]{64}", source_sha):
        raise ValueError("scene fixture source OBJ must have its 64-character SHA-256")
    if (
        source.get("license") != "Apache-2.0"
        or "license_evidence" not in source
        or "repository" not in source
        or "commit" not in source
    ):
        raise ValueError("scene fixture source license provenance is incomplete")
    roi = metadata.get("roi")
    if not isinstance(roi, dict) or roi.get("inclusive") is not True:
        raise ValueError("scene fixture ROI metadata is incomplete")


def _load_arrays(path: Path) -> tuple[np.ndarray, np.ndarray, float]:
    with np.load(path, allow_pickle=False) as data:
        points = np.asarray(data["points"], dtype=np.float32)
        labels = np.asarray(data["labels"], dtype=np.uint8)
        if points.shape != (len(data["labels"]), 3):
            raise ValueError("scene fixture points and labels do not agree")
        if not np.all(np.isfinite(points)):
            raise ValueError("scene fixture contains non-finite points")
        timestamp = float(np.asarray(data["timestamp"]).item())
    return points, labels, timestamp


def load_scene_record(path: Path = FIXTURE) -> tuple[np.ndarray, np.ndarray, SceneMetadata]:
    """Return stored points, semantic labels, and the adjacent provenance record."""
    points, labels, timestamp = _load_arrays(path)
    if points.shape != (len(labels), 3) or not np.all(np.isfinite(points)):
        raise ValueError("scene fixture has invalid points or labels")
    metadata = json.loads(path.with_suffix(".json").read_text(encoding="utf-8"))
    _validate_record(path, points, labels, timestamp, metadata)
    return points, labels, metadata
