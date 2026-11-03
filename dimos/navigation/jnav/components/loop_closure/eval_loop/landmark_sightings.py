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

"""Turn a recording's `raw_april_tags` (tag-in-camera_optical PoseStamped, marker_id + quality in tags)
into a pgo-eval `--landmark-sightings` CSV in an odometry's body frame (camera -> body via the db's tf).

Point sightings (14 columns: time, id, body xyz, 3x3 covariance): AprilTag PnP rotation is left out.
Detections are kept through apriltags.py's per-glimpse gates (sharpness, reprojection, tag size,
distance, view angle, camera speed), then thinned to one per tag per `--min-gap-s`.

usage: landmark_sightings.py --db <db> --body-frame <l1_link|mid360_link> --out <csv> [--exclude 17]
"""

import argparse
from pathlib import Path

import numpy as np

from dimos.memory.store.sqlite import SqliteStore
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.navigation.jnav.utils.apriltags import (
    DEFAULT_MAX_ANGULAR_SPEED_DPS,
    DEFAULT_MAX_DISTANCE_M,
    DEFAULT_MAX_LINEAR_SPEED_MPS,
    DEFAULT_MAX_REPROJ_PX,
    DEFAULT_MAX_VIEW_ANGLE_DEG,
    DEFAULT_MIN_SHARPNESS,
    DEFAULT_MIN_TAG_PX,
)
from dimos.navigation.jnav.utils.recording_tf import RecordingTF


def passes_gates(tags: dict) -> bool:
    """apriltags.py's per-glimpse rejection gates (a field missing from the row passes)."""
    checks = (
        ("sharpness", lambda v: v >= DEFAULT_MIN_SHARPNESS),
        ("reproj_px", lambda v: v <= DEFAULT_MAX_REPROJ_PX),
        ("tag_px", lambda v: v >= DEFAULT_MIN_TAG_PX),
        ("distance_m", lambda v: v <= DEFAULT_MAX_DISTANCE_M),
        ("view_angle_deg", lambda v: v <= DEFAULT_MAX_VIEW_ANGLE_DEG),
        ("lin_speed", lambda v: v <= DEFAULT_MAX_LINEAR_SPEED_MPS),
        ("ang_speed", lambda v: v <= DEFAULT_MAX_ANGULAR_SPEED_DPS),
    )
    return all(name not in tags or ok(float(tags[name])) for name, ok in checks)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--db", type=Path, required=True)
    parser.add_argument("--body-frame", required=True, help="the odometry's child frame (l1_link, mid360_link)")
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--stream", default="raw_april_tags")
    parser.add_argument("--exclude", type=int, nargs="*", default=[17], help="marker ids not fixed in the world")
    parser.add_argument("--position-sd-m", type=float, default=0.05, help="per-axis sd of a sighting's position")
    parser.add_argument("--min-gap-s", type=float, default=0.5, help="one sighting per tag per this many seconds")
    args = parser.parse_args()

    rows, last_kept, counts = [], {}, {"read": 0, "excluded": 0, "gated": 0, "thinned": 0, "no_tf": 0}
    covariance = (np.eye(3) * args.position_sd_m**2).ravel()
    with SqliteStore(path=args.db.expanduser(), must_exist=True) as store:
        tf = RecordingTF.from_store(store)
        for observation in store.stream(args.stream, PoseStamped).order_by("ts"):
            counts["read"] += 1
            tags = observation.tags or {}
            marker = int(tags.get("marker_id", -1))
            ts = float(observation.ts)
            if marker < 0 or marker in args.exclude:
                counts["excluded"] += 1
                continue
            if not passes_gates(tags):
                counts["gated"] += 1
                continue
            if ts - last_kept.get(marker, -np.inf) < args.min_gap_s:
                counts["thinned"] += 1
                continue
            pose = observation.data
            frame = pose.frame_id or "camera_optical"
            try:
                body_from_camera = tf.get(args.body_frame, frame, ts).to_matrix()
            except LookupError:
                counts["no_tf"] += 1
                continue
            in_camera = np.array([pose.position.x, pose.position.y, pose.position.z, 1.0])
            in_body = (body_from_camera @ in_camera)[:3]
            last_kept[marker] = ts
            rows.append([ts, marker, *in_body, *covariance])
    args.out.parent.mkdir(parents=True, exist_ok=True)
    np.savetxt(args.out, np.asarray(rows), delimiter=",", fmt="%.9f")
    per_tag = {m: sum(1 for r in rows if r[1] == m) for m in sorted({int(r[1]) for r in rows})}
    print(f"{args.out}: {len(rows)} sightings {per_tag}; {counts}")


if __name__ == "__main__":
    main()
