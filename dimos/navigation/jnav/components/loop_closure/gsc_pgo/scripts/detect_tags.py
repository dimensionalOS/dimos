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

# Untyped analysis script: gtsam/open3d/cv2 lack type stubs.
# mypy: ignore-errors
"""Build the raw AprilTag stream: EVERY detection over the camera image, NO filtering whatsoever
(no blur/reproj/distance/angle/motion/size gate, no time-clustering). One row per per-frame
detection that yields a valid PnP pose. Each row carries its gate diagnostics in tags
(sharpness, reproj_px, tag_px, distance_m, view_angle_deg, lin_speed, ang_speed) so downstream
gate tuning in post_process.py needs no re-detection.

Prints the raw per-marker histogram + visit structure (visit = sightings >30s apart).

Usage: python dimos/navigation/jnav/components/loop_closure/gsc_pgo/scripts/detect_tags.py --rec=PATH
       [--camera=color_image] [--tag-size=0.10]
       [--dict=DICT_APRILTAG_36h11] [--intrinsics=PATH] [--out=raw_april_tags]
"""

import json
from pathlib import Path
import sys

import cv2
import numpy as np
from scipy.spatial.transform import Rotation

from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.sensor_msgs.Image import Image
from dimos.navigation.jnav.utils import recording_db as rdb
from dimos.navigation.jnav.utils.apriltags import (
    _camera_speeds,
    estimate_marker_pose,
    make_detector,
    reprojection_error_px,
    tag_pixel_size,
    tag_sharpness,
    view_quality,
)


def arg(flag, default=None):
    return next(
        (argument.split("=", 1)[1] for argument in sys.argv if argument.startswith(flag + "=")),
        default,
    )


REC_ARG = arg("--rec")
if not REC_ARG:
    sys.exit(
        "usage: python dimos/navigation/jnav/components/loop_closure/gsc_pgo/scripts/detect_tags.py --rec=PATH [--camera=...] [--tag-size=...] "
        "[--dict=...] [--intrinsics=PATH] [--out=...]   (--rec is required)"
    )
REC = Path(REC_ARG).expanduser()
CAMERA = arg("--camera", "color_image")  # camera image stream the tags are detected on
MARKER_LENGTH_M = float(arg("--tag-size", "0.10"))
DICTIONARY = arg("--dict", "DICT_APRILTAG_36h11")
STREAM = arg("--out", "raw_april_tags")
VISIT_GAP_S = 30.0

DB = REC / "mem2.db"
intrinsics_path = Path(arg("--intrinsics", str(REC / "camera_intrinsics.json"))).expanduser()
intrinsics = json.loads(intrinsics_path.read_text())
camera_matrix = np.array(intrinsics["intrinsics"], float).reshape(3, 3)
distortion = np.array(intrinsics.get("distortion", []), float)
store = rdb.store(DB)
if CAMERA not in store.list_streams():
    sys.exit(f"!! camera stream '{CAMERA}' not in db — available: {store.list_streams()}")

detector = make_detector(DICTIONARY)
print(f"loading '{CAMERA}' frames...")
images = store.stream(CAMERA, Image).to_list()
speed_by_ts, speed_available = _camera_speeds(images)
print(
    f"detecting over {len(images)} frames (unfiltered), tag_size={MARKER_LENGTH_M} m, dict={DICTIONARY}..."
)

rows = []
for image_obs in images:
    image = image_obs.data
    bgr = image.numpy() if hasattr(image, "numpy") else np.asarray(image.data)
    gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY) if bgr.ndim == 3 else bgr
    all_corners, marker_ids, _ = detector.detectMarkers(bgr)
    if marker_ids is None:
        continue
    for corners, marker_id in zip(all_corners, marker_ids.flatten(), strict=False):
        pose = estimate_marker_pose(corners, MARKER_LENGTH_M, camera_matrix, distortion)
        if pose is None:
            continue
        rotation_vector, translation_vector = pose
        quaternion = Rotation.from_rotvec(rotation_vector.reshape(3)).as_quat()  # x,y,z,w
        translation = translation_vector.reshape(3)
        tag_pose = [
            float(translation[0]),
            float(translation[1]),
            float(translation[2]),
            float(quaternion[0]),
            float(quaternion[1]),
            float(quaternion[2]),
            float(quaternion[3]),
        ]
        distance, view_angle = view_quality(tag_pose)
        speed = speed_by_ts.get(float(image_obs.ts))
        rows.append(
            {
                "ts": float(image_obs.ts),
                "marker_id": int(marker_id),
                "tcm": tag_pose,
                "sharpness": float(tag_sharpness(gray, corners)),
                "reproj_px": float(
                    reprojection_error_px(
                        corners,
                        rotation_vector,
                        translation_vector,
                        MARKER_LENGTH_M,
                        camera_matrix,
                        distortion,
                    )
                ),
                "tag_px": float(tag_pixel_size(corners)),
                "distance_m": float(distance),
                "view_angle_deg": float(view_angle),
                "lin_speed": float(speed[0]) if speed else -1.0,
                "ang_speed": float(speed[1]) if speed else -1.0,
            }
        )
rows.sort(key=lambda row: row["ts"])

if STREAM in store.list_streams():
    store.delete_stream(STREAM)
out_stream = store.stream(STREAM, PoseStamped)
for row in rows:
    tag_pose = row["tcm"]
    out_stream.append(
        PoseStamped(ts=row["ts"], position=tag_pose[:3], orientation=tag_pose[3:]),
        ts=row["ts"],
        pose=tuple(tag_pose),
        tags={
            tag_key: row[tag_key]
            for tag_key in (
                "marker_id",
                "sharpness",
                "reproj_px",
                "tag_px",
                "distance_m",
                "view_angle_deg",
                "lin_speed",
                "ang_speed",
            )
        },
    )
print(f"\nwrote {STREAM}: {len(rows)} unfiltered detections")

rows_by_marker = {}
for row in rows:
    rows_by_marker.setdefault(row["marker_id"], []).append(row)
print(f"\n=== RAW per-marker (visit = >{VISIT_GAP_S:.0f}s apart) ===")
print(
    f"{'tag':>4} {'det':>4} {'visits':>6} {'dist_m':>12} {'sharp>=60%':>10} {'reproj<=2%':>10} {'span_s':>7}"
)
for marker_id in sorted(rows_by_marker):
    marker_rows = sorted(rows_by_marker[marker_id], key=lambda row: row["ts"])
    times = [row["ts"] for row in marker_rows]
    visits = [[times[0]]]
    for timestamp in times[1:]:
        (
            visits[-1].append(timestamp)
            if timestamp - visits[-1][-1] <= VISIT_GAP_S
            else visits.append([timestamp])
        )
    distances = [row["distance_m"] for row in marker_rows]
    sharp_ok = 100 * np.mean([row["sharpness"] >= 60 for row in marker_rows])
    reproj_ok = 100 * np.mean([row["reproj_px"] <= 2 for row in marker_rows])
    print(
        f"{marker_id:>4} {len(marker_rows):>4} {len(visits):>6} {min(distances):5.2f}-{max(distances):5.2f} "
        f"{sharp_ok:9.0f}% {reproj_ok:9.0f}% {times[-1] - times[0]:7.0f}"
    )
print("\nmarkers present:", sorted(rows_by_marker))
