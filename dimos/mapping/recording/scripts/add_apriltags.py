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

"""Detect AprilTags in a recording's camera stream and write them back as a stream.

Replays the camera stream through the incremental AprilTagger (apriltag_tracker)
and appends one finalized TagInfo per tag visit to the output stream (pose =
tag-in-camera from solvePnP, camera_frame_id carried in the observation tags).
Unlike `post_process.py` step 2 (batch detect_apriltags -> `april_tags`
PoseStamped stream feeding the GTSAM solve), this is standalone and incremental.

Intrinsics resolution order: --camera_intrinsics_json, a `camera_intrinsics.json`
sidecar next to the db, a CameraInfo-like stream in the db, then the go2 rig
constants from `utils/camera.py` (with a notice).

Usage:
    uv run python dimos/mapping/recording/scripts/add_apriltags.py
    ... --db_path <path/to/mem2.db> --camera_stream color_image
    ... [--camera_intrinsics_json <json_path>] [--apriltag_stream_name april_tag]
    ... [--marker_length 0.10] [--dictionary DICT_APRILTAG_36h11] [--force]
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from types import SimpleNamespace
from typing import Any

import numpy as np

from dimos.mapping.recording.utils.apriltag_tracker import (
    AprilTagger,
    GroupingOptions,
    TagInfo,
)
from dimos.mapping.recording.utils.camera import CAMERA_DISTORTION, CAMERA_INTRINSICS
from dimos.memory2.store.sqlite import SqliteStore

DEFAULT_APRILTAG_STREAM = "april_tag"
SIDECAR_NAME = "camera_intrinsics.json"
CAMERA_INFO_STREAM_CANDIDATES = (
    "camera_info",
    "color_camera_info",
    "camera_intrinsics",
)


def load_intrinsics_json(path: Path) -> dict[str, Any]:
    raw = json.loads(path.read_text())
    camera_intrinsics: dict[str, Any] = {
        "intrinsics": np.array(raw["intrinsics"], dtype=np.float64).reshape(3, 3)
    }
    if "distortion" in raw:
        camera_intrinsics["distortion"] = np.array(raw["distortion"], dtype=np.float64)
    for key in ("marker_length", "dictionary"):
        if key in raw:
            camera_intrinsics[key] = raw[key]
    return camera_intrinsics


def intrinsics_from_db(store: SqliteStore) -> dict[str, Any] | None:
    """Camera matrices from the first CameraInfo-like observation in the db."""
    stream_names = store.list_streams()
    for name in CAMERA_INFO_STREAM_CANDIDATES:
        if name not in stream_names:
            continue
        for observation in store.stream(name):
            camera_info = observation.data
            if hasattr(camera_info, "K"):
                matrix = np.array(camera_info.K, dtype=np.float64).reshape(3, 3)
                distortion = np.array(camera_info.D if camera_info.D else [], dtype=np.float64)
                return {"intrinsics": matrix, "distortion": distortion}
            break
    return None


def append_tag(tag_stream: Any, tag: TagInfo) -> None:
    tag_stream.append(
        tag,
        ts=tag.ts,
        pose=tuple(tag.pose),
        tags={"marker_id": tag.tag_number, "camera_frame_id": tag.camera_frame_id},
    )


def add_april_tags(
    db_path: Path,
    camera_stream: str,
    camera_intrinsics: dict[str, Any],
    apriltag_stream_name: str = DEFAULT_APRILTAG_STREAM,
    *,
    force: bool = False,
) -> int:
    """Detect tags over the camera stream; write finalized tags. Returns count written."""
    store = SqliteStore(path=str(db_path), must_exist=True)
    store.start()
    try:
        stream_names = store.list_streams()
        if camera_stream not in stream_names:
            raise SystemExit(f"no stream {camera_stream!r} in {db_path} (have: {stream_names})")
        if apriltag_stream_name in stream_names:
            if not force:
                raise SystemExit(
                    f"stream {apriltag_stream_name!r} already exists in {db_path};"
                    " pass --force to overwrite"
                )
            store.delete_stream(apriltag_stream_name)

        options = GroupingOptions()
        tagger = AprilTagger(camera_intrinsics, options, options.tag_ids_to_ignore)
        tag_stream = store.stream(apriltag_stream_name, TagInfo)

        written = 0
        n_images = 0
        state: dict[str, Any] = {}
        for observation in store.stream(camera_stream):
            n_images += 1
            next_observation = SimpleNamespace(color_image=observation.data)
            _, _, finished_tags = tagger.iter_april_tag_groups(next_observation, options, state)
            for tag in finished_tags.values():
                append_tag(tag_stream, tag)
                written += 1

        # Estimates still pending when the stream ends are final too.
        for tag in state.get("tag_estimates", {}).values():
            append_tag(tag_stream, tag)
            written += 1

        print(
            f"{db_path}: {written} tag(s) -> stream {apriltag_stream_name!r}"
            f" (over {n_images} images from {camera_stream!r})"
        )
        return written
    finally:
        store.stop()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--db_path", type=Path, required=True)
    parser.add_argument("--camera_stream", required=True)
    parser.add_argument("--camera_intrinsics_json", type=Path)
    parser.add_argument("--apriltag_stream_name", default=DEFAULT_APRILTAG_STREAM)
    parser.add_argument("--marker_length", type=float, help="tag side length in metres")
    parser.add_argument("--dictionary", help="aruco dictionary name, e.g. DICT_APRILTAG_36h11")
    parser.add_argument("--force", action="store_true", help="overwrite an existing tag stream")
    args = parser.parse_args()

    if not args.db_path.exists():
        raise SystemExit(f"no such db: {args.db_path}")

    sidecar = args.db_path.parent / SIDECAR_NAME
    if args.camera_intrinsics_json is not None:
        camera_intrinsics = load_intrinsics_json(args.camera_intrinsics_json)
    elif sidecar.exists():
        print(f"using intrinsics sidecar: {sidecar}")
        camera_intrinsics = load_intrinsics_json(sidecar)
    else:
        probe_store = SqliteStore(path=str(args.db_path), must_exist=True)
        probe_store.start()
        try:
            found = intrinsics_from_db(probe_store)
        finally:
            probe_store.stop()
        if found is None:
            print(
                f"no {SIDECAR_NAME} or CameraInfo stream for {args.db_path}"
                " — falling back to go2 rig constants (utils/camera.py)"
            )
            found = {"intrinsics": CAMERA_INTRINSICS, "distortion": CAMERA_DISTORTION}
        camera_intrinsics = found

    if args.marker_length is not None:
        camera_intrinsics["marker_length"] = args.marker_length
    if args.dictionary is not None:
        camera_intrinsics["dictionary"] = args.dictionary

    add_april_tags(
        args.db_path,
        args.camera_stream,
        camera_intrinsics,
        args.apriltag_stream_name,
        force=args.force,
    )


if __name__ == "__main__":
    main()
