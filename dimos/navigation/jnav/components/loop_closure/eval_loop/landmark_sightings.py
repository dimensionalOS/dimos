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

The detections go through the live publisher's `SightingClusterer` (apriltags.py): loose per-glimpse gates, a tag's
detections within the cooldown are one sighting, published once enough agree with its medoid (Huber outlier test in
the tf's odom frame) and republished under its id whenever the medoid moves. Each row is one publish, at the time the
publisher would have sent it, so pgo-eval feeds the replacements as the module would receive them.

Rows (16 columns): publish time, sighting id, sighting time, tag id, body xyz, 3x3 covariance. AprilTag PnP rotation
is left out (a point sighting).

usage: landmark_sightings.py --db <db> --body-frame <base_link|l1_link> --out <csv> [--exclude 17]
"""

import argparse
from collections import Counter
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

from dimos.memory.store.sqlite import SqliteStore
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.navigation.jnav.utils.apriltags import (
    STABLE_POSE_KEY,
    Detection,
    SightingClusterer,
    SightingClustering,
)
from dimos.navigation.jnav.utils.recording_tf import RecordingTF


def to_matrix(pose: list[float]) -> np.ndarray:
    """4x4 from xyz + xyzw quaternion."""
    matrix = np.eye(4)
    matrix[:3, :3] = Rotation.from_quat(pose[3:7]).as_matrix()
    matrix[:3, 3] = pose[:3]
    return matrix


def from_matrix(matrix: np.ndarray) -> list[float]:
    """xyz + xyzw quaternion from a 4x4."""
    return [*matrix[:3, 3].tolist(), *Rotation.from_matrix(matrix[:3, :3]).as_quat().tolist()]


def main() -> None:
    defaults = SightingClustering()
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--db", type=Path, required=True)
    parser.add_argument(
        "--body-frame", required=True, help="the odometry's body frame (base_link, l1_link)"
    )
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--stream", default="raw_april_tags")
    parser.add_argument(
        "--odom-frame", default="odom", help="tf frame the detections of a sighting are compared in"
    )
    parser.add_argument(
        "--exclude", type=int, nargs="*", default=[17], help="marker ids not fixed in the world"
    )
    parser.add_argument(
        "--position-sd-m", type=float, default=0.05, help="per-axis sd of a sighting's position"
    )
    for name, value in vars(defaults).items():
        parser.add_argument(f"--{name.replace('_', '-')}", type=type(value), default=value)
    args = parser.parse_args()
    clustering = SightingClustering(**{name: getattr(args, name) for name in vars(defaults)})

    clusterer = SightingClusterer(clustering)
    rows: list[list[float]] = []
    counts: Counter[str] = Counter()
    sightings_per_tag: Counter[int] = Counter()
    covariance = (np.eye(3) * args.position_sd_m**2).ravel()
    with SqliteStore(path=args.db.expanduser(), must_exist=True) as store:
        tf = RecordingTF.from_store(store)
        for observation in store.stream(args.stream, PoseStamped).order_by("ts"):
            counts["raw"] += 1
            tags = observation.tags or {}
            marker = int(tags.get("marker_id", -1))
            ts = float(observation.ts)
            if marker < 0 or marker in args.exclude:
                counts["excluded"] += 1
                continue
            pose = observation.data
            frame = pose.frame_id or "camera_optical"
            try:
                body_from_camera = tf.get(args.body_frame, frame, ts).to_matrix()
                odom_from_camera = tf.get(args.odom_frame, frame, ts).to_matrix()
            except LookupError:
                counts["no_tf"] += 1
                continue
            camera_marker = [
                pose.position.x,
                pose.position.y,
                pose.position.z,
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w,
            ]
            speed = (
                (float(tags["lin_speed"]), float(tags["ang_speed"]))
                if "lin_speed" in tags and "ang_speed" in tags
                else None
            )
            detection: Detection = {
                "ts": ts,
                "marker_id": marker,
                "t_cam_marker": camera_marker,
                STABLE_POSE_KEY: from_matrix(odom_from_camera @ to_matrix(camera_marker)),
                "body_position": (body_from_camera @ [*camera_marker[:3], 1.0])[:3],
                "speed": speed,
                **{
                    key: float(tags[key])
                    for key in ("sharpness", "reproj_px", "tag_px")
                    if key in tags
                },
            }
            update = clusterer.add(detection)
            if update is None:
                continue
            counts["supersedes" if update.supersedes else "sightings"] += 1
            sightings_per_tag[marker] += not update.supersedes
            medoid = update.medoid
            rows.append(
                [
                    update.publish_ts,
                    update.sighting_id,
                    medoid["ts"],
                    marker,
                    *medoid["body_position"],
                    *covariance,
                ]
            )
    args.out.parent.mkdir(parents=True, exist_ok=True)
    header = "publish_time,sighting_id,time,landmark,x,y,z,cov_xx,cov_xy,cov_xz,cov_yx,cov_yy,cov_yz,cov_zx,cov_zy,cov_zz"
    np.savetxt(args.out, np.asarray(rows), delimiter=",", fmt="%.9f", header=header)
    print(
        f"{args.out}: {dict(counts)}, gated {dict(clusterer.rejected)}; "
        f"sightings per tag {dict(sorted(sightings_per_tag.items()))}"
    )


if __name__ == "__main__":
    main()
