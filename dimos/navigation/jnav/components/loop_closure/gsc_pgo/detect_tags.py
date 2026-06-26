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

Usage: python dimos/navigation/jnav/components/loop_closure/gsc_pgo/detect_tags.py --rec=PATH
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
    return next((a.split("=", 1)[1] for a in sys.argv if a.startswith(flag + "=")), default)


REC_ARG = arg("--rec")
if not REC_ARG:
    sys.exit(
        "usage: python dimos/navigation/jnav/components/loop_closure/gsc_pgo/detect_tags.py --rec=PATH [--camera=...] [--tag-size=...] "
        "[--dict=...] [--intrinsics=PATH] [--out=...]   (--rec is required)"
    )
REC = Path(REC_ARG).expanduser()
CAMERA = arg("--camera", "color_image")  # camera image stream the tags are detected on
MARKER_LENGTH_M = float(arg("--tag-size", "0.10"))
DICTIONARY = arg("--dict", "DICT_APRILTAG_36h11")
STREAM = arg("--out", "raw_april_tags")
VISIT_GAP_S = 30.0

DB = REC / "mem2.db"
intr_path = Path(arg("--intrinsics", str(REC / "camera_intrinsics.json"))).expanduser()
intr = json.loads(intr_path.read_text())
K = np.array(intr["intrinsics"], float).reshape(3, 3)
dist = np.array(intr.get("distortion", []), float)
st = rdb.store(DB)
if CAMERA not in st.list_streams():
    sys.exit(f"!! camera stream '{CAMERA}' not in db — available: {st.list_streams()}")

detector = make_detector(DICTIONARY)
print(f"loading '{CAMERA}' frames...")
images = st.stream(CAMERA, Image).to_list()
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
        pose = estimate_marker_pose(corners, MARKER_LENGTH_M, K, dist)
        if pose is None:
            continue
        rvec, tvec = pose
        quat = Rotation.from_rotvec(rvec.reshape(3)).as_quat()  # x,y,z,w
        t = tvec.reshape(3)
        tcm = [
            float(t[0]),
            float(t[1]),
            float(t[2]),
            float(quat[0]),
            float(quat[1]),
            float(quat[2]),
            float(quat[3]),
        ]
        distance, view_angle = view_quality(tcm)
        speed = speed_by_ts.get(float(image_obs.ts))
        rows.append(
            {
                "ts": float(image_obs.ts),
                "marker_id": int(marker_id),
                "tcm": tcm,
                "sharpness": float(tag_sharpness(gray, corners)),
                "reproj_px": float(
                    reprojection_error_px(corners, rvec, tvec, MARKER_LENGTH_M, K, dist)
                ),
                "tag_px": float(tag_pixel_size(corners)),
                "distance_m": float(distance),
                "view_angle_deg": float(view_angle),
                "lin_speed": float(speed[0]) if speed else -1.0,
                "ang_speed": float(speed[1]) if speed else -1.0,
            }
        )
rows.sort(key=lambda r: r["ts"])

if STREAM in st.list_streams():
    st.delete_stream(STREAM)
out = st.stream(STREAM, PoseStamped)
for r in rows:
    tcm = r["tcm"]
    out.append(
        PoseStamped(ts=r["ts"], position=tcm[:3], orientation=tcm[3:]),
        ts=r["ts"],
        pose=tuple(tcm),
        tags={
            k: r[k]
            for k in (
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

by = {}
for r in rows:
    by.setdefault(r["marker_id"], []).append(r)
print(f"\n=== RAW per-marker (visit = >{VISIT_GAP_S:.0f}s apart) ===")
print(
    f"{'tag':>4} {'det':>4} {'visits':>6} {'dist_m':>12} {'sharp>=60%':>10} {'reproj<=2%':>10} {'span_s':>7}"
)
for mid in sorted(by):
    rs = sorted(by[mid], key=lambda r: r["ts"])
    times = [r["ts"] for r in rs]
    visits = [[times[0]]]
    for tt in times[1:]:
        (visits[-1].append(tt) if tt - visits[-1][-1] <= VISIT_GAP_S else visits.append([tt]))
    d = [r["distance_m"] for r in rs]
    sharp_ok = 100 * np.mean([r["sharpness"] >= 60 for r in rs])
    reproj_ok = 100 * np.mean([r["reproj_px"] <= 2 for r in rs])
    print(
        f"{mid:>4} {len(rs):>4} {len(visits):>6} {min(d):5.2f}-{max(d):5.2f} "
        f"{sharp_ok:9.0f}% {reproj_ok:9.0f}% {times[-1] - times[0]:7.0f}"
    )
print("\nmarkers present:", sorted(by))
