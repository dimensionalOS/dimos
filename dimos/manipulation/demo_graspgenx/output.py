# Copyright 2026 Dimensional Inc.
"""Stable, human-readable grasp result serialization."""

import os
from pathlib import Path
import tempfile
from typing import Any

import yaml

from dimos.msgs.manipulation_msgs.GraspCandidateArray import GraspCandidateArray


def _pose(candidate: Any) -> dict[str, list[float]]:
    p, q = candidate.pose.position, candidate.pose.orientation
    return {
        "position": [float(p.x), float(p.y), float(p.z)],
        "orientation": [float(q.x), float(q.y), float(q.z), float(q.w)],
    }


def write_grasps(path: Path, result: GraspCandidateArray, calibration: object) -> None:
    if result.header.frame_id != "world":
        raise ValueError("grasp result must be in world")
    payload = {
        "format": "dimos.graspgenx.ycb.v1",
        "frame": result.header.frame_id,
        "timestamp": float(result.header.timestamp),
        "tcp_calibration": calibration,
        "grasps": [{"score": float(c.score), "tcp_pose": _pose(c)} for c in result.candidates],
    }
    path.parent.mkdir(parents=True, exist_ok=True)
    text = yaml.safe_dump(payload, sort_keys=False, default_flow_style=False)
    # Validate before replacing an existing result. A failed write can never
    # truncate the user's last known-good YAML.
    temporary: Path | None = None
    try:
        with tempfile.NamedTemporaryFile(
            mode="w", encoding="utf-8", dir=path.parent, prefix=f".{path.name}.", delete=False
        ) as handle:
            temporary = Path(handle.name)
            handle.write(text)
            handle.flush()
            os.fsync(handle.fileno())
        read_grasps(temporary)
        os.replace(temporary, path)
        temporary = None
    finally:
        if temporary is not None:
            temporary.unlink(missing_ok=True)


def read_grasps(path: Path) -> dict[str, Any]:
    value = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict) or value.get("format") != "dimos.graspgenx.ycb.v1":
        raise ValueError("invalid grasp YAML header")
    if not isinstance(value.get("grasps"), list):
        raise ValueError("grasp YAML grasps must be an array")
    return value
