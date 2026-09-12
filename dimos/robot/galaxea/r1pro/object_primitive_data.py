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

"""Reusable pick/place views of archived RGB demonstrations; originals remain untouched."""

import hashlib
import json
from pathlib import Path
from typing import Any

import numpy as np

from dimos.robot.galaxea.r1pro.object_packing import OBJECT_PACKING_IO

PRIMITIVE_TASKS = {
    "pick": "Grasp the selected object, lift it, and stop while holding it. Do not place or release.",
    "place": "Place the held object at the assigned supported tray goal, release, and retreat.",
}


def episode_frame_slice(episode: dict[str, Any], total: int) -> slice:
    """Validate a full episode or a half-open view before images/actions/statistics are read."""
    start = episode.get("frame_start", 0)
    stop = episode.get("frame_stop", total)
    if not isinstance(start, int) or not isinstance(stop, int):
        raise ValueError("Episode frame bounds must be integers")
    if not 0 <= start < stop <= total or stop - start != episode["frames"]:
        raise ValueError("Episode frame bounds do not match its declared frames")
    return slice(start, stop)


def segment_demonstrations(source: Path, output: Path) -> dict[str, Any]:
    """Create phase-bounded manifests referencing original arrays, without copying their images."""
    source = source.resolve()
    raw = (source / "manifest.json").read_bytes()
    manifest = json.loads(raw)
    if manifest.get("profile") != OBJECT_PACKING_IO.name or not manifest.get("images"):
        raise ValueError("Expected the verified random-object RGB demonstration collection")
    if output.exists():
        raise FileExistsError(output)
    episodes: dict[str, list[dict[str, Any]]] = {"pick": [], "place": []}
    for row in manifest["episodes"]:
        if not row["success"] or "frame_start" in row or "frame_stop" in row:
            raise ValueError("Segment only original successful full demonstrations")
        path = source / row["file"]
        with np.load(path, allow_pickle=False) as data:
            phases = data["phase"]
        if len(phases) != row["frames"] or phases.ndim != 1:
            raise ValueError(f"Invalid phase array in {path}")
        lifts = np.flatnonzero(phases == "lift")
        if not len(lifts):
            raise ValueError(f"Missing grasp/lift boundary in {path}")
        boundary = int(lifts[-1]) + 1
        if (
            set(phases[:boundary]) != {"above", "approach", "grasp", "lift"}
            or boundary == len(phases)
            or phases[boundary] != "clear_sources"
            or not {"release", "retreat", "settle"}.issubset(set(phases[boundary:]))
        ):
            raise ValueError(f"Unexpected primitive phase order in {path}")
        for primitive, start, stop in (("pick", 0, boundary), ("place", boundary, len(phases))):
            view = {
                **row,
                "file": str(path),
                "source_file": row["file"],
                "source_frames": len(phases),
                "frame_start": start,
                "frame_stop": stop,
                "frames": stop - start,
            }
            episode_frame_slice(view, len(phases))
            episodes[primitive].append(view)
    if not episodes["pick"]:
        raise ValueError("No successful demonstrations to segment")
    output.mkdir(parents=True)
    report: dict[str, Any] = {
        "source": str(source),
        "source_manifest_sha256": hashlib.sha256(raw).hexdigest(),
        "source_modified": False,
        "new_demonstrations": 0,
        "primitives": {},
    }
    for primitive, rows in episodes.items():
        target = output / primitive
        target.mkdir()
        derivative = {
            **manifest,
            "episodes": rows,
            "rejected": [],
            "primitive": primitive,
            "task": PRIMITIVE_TASKS[primitive],
            "source_collection": str(source),
            "source_manifest_sha256": report["source_manifest_sha256"],
        }
        (target / "manifest.json").write_text(json.dumps(derivative, indent=2) + "\n")
        report["primitives"][primitive] = {
            "episodes": len(rows),
            "frames": sum(row["frames"] for row in rows),
            "manifest": str(target / "manifest.json"),
        }
    (output / "segmentation.json").write_text(json.dumps(report, indent=2) + "\n")
    return report
