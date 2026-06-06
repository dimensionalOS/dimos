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

"""One-off: reframe a go2_mid360 recording into the FAST-LIO (mid360) world.

Given a recording dir, destructively:

  1. truncates everything before the first `fastlio_odometry` message,
  2. rebases the Go2 onboard `odom` + `lidar` (which live in the Go2's own odom
     frame) into the mid360 world with a single rigid transform: the *initial*
     base_link is placed at `fastlio(t0) . static(mid360_link -> base_link)`
     (the static mid360<->base offset comes from the URDF), and that one
     transform is applied to every odom pose and lidar cloud,
  3. forces every `color_image` pose to be the static URDF offset off mid360:
     `fastlio(t) . static(mid360_link -> camera_optical)`,

then rebuilds the recording's `main.rrd`. World frame = the FAST-LIO odom frame
(mid360 at t0); FAST-LIO reports `world -> mid360` directly as the odom value.

    uv run python dimos/mapping/recording/utils/reframe_go2.py REC_DIR
"""

from __future__ import annotations

import argparse
import math
from pathlib import Path
import sqlite3

import numpy as np
from scipy.spatial.transform import Rotation

from dimos.mapping.recording.utils.db_reform import _nearest, parse_urdf_graph, transform_between
from dimos.mapping.recording.utils.short import rename_go2_streams
from dimos.mapping.recording.utils.trunc import first_fastlio_ts, rebuild_rrd, truncate_db
from dimos.memory2.store.sqlite import SqliteStore
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2

DB_NAME = "mem2.db"
RRD_NAME = "main.rrd"
ODOM = "odom"
LIDAR = "lidar"
FASTLIO_ODOM = "fastlio_odometry"
COLOR_IMAGE = "color_image"
WORLD_FRAME = "mid360_link"
BASE_FRAME = "base_link"
CAMERA_FRAME = "camera_optical"
DEFAULT_URDF = Path(__file__).resolve().parent.parent / "go2_mid360" / "static_transforms.urdf"


def _to_transform(message: object) -> Transform:
    """A Transform from any msg exposing `.position` (Vector3) + `.orientation`."""
    position = message.position  # type: ignore[attr-defined]
    orientation = message.orientation  # type: ignore[attr-defined]
    return Transform(
        translation=Vector3(position.x, position.y, position.z),
        rotation=Quaternion(orientation.x, orientation.y, orientation.z, orientation.w),
    )


def _pose7(transform: Transform) -> tuple[float, float, float, float, float, float, float]:
    translation = transform.translation
    rotation = transform.rotation
    return (
        translation.x,
        translation.y,
        translation.z,
        rotation.x,
        rotation.y,
        rotation.z,
        rotation.w,
    )


def _mat(transform: Transform) -> np.ndarray:
    matrix = np.eye(4)
    rot = transform.rotation
    matrix[:3, :3] = Rotation.from_quat([rot.x, rot.y, rot.z, rot.w]).as_matrix()
    matrix[:3, 3] = [transform.translation.x, transform.translation.y, transform.translation.z]
    return matrix


def _transform_from_mat(matrix: np.ndarray) -> Transform:
    quat = Rotation.from_matrix(matrix[:3, :3]).as_quat()
    return Transform(
        translation=Vector3(matrix[0, 3], matrix[1, 3], matrix[2, 3]),
        rotation=Quaternion(quat[0], quat[1], quat[2], quat[3]),
    )


def _level_in_place(align: Transform, start: Transform) -> Transform:
    """Compose an in-place rotation onto `align` so the go2 `start` pose is flat
    (zero roll/pitch, yaw kept), pivoting about its own position so it doesn't move.
    Levels the whole go2 trajectory + lidar even if FAST-LIO's frame is tilted."""
    position = np.array([start.translation.x, start.translation.y, start.translation.z])
    rotation = Rotation.from_quat(
        [start.rotation.x, start.rotation.y, start.rotation.z, start.rotation.w]
    ).as_matrix()
    forward = rotation @ np.array([1.0, 0.0, 0.0])  # body x-axis in world
    yaw = math.atan2(forward[1], forward[0])
    level = Rotation.from_euler("z", yaw).as_matrix()
    correction = level @ rotation.T  # world rotation that levels the start orientation
    pivot = np.eye(4)
    pivot[:3, :3] = correction
    pivot[:3, 3] = position - correction @ position  # rotate about `position`, no translation
    return _transform_from_mat(pivot @ _mat(align))


def _update_point_pose(
    conn: sqlite3.Connection, stream: str, row_id: int, pose: tuple, has_rtree: bool
) -> None:
    conn.execute(
        f'UPDATE "{stream}" SET pose_x=?,pose_y=?,pose_z=?,'
        f"pose_qx=?,pose_qy=?,pose_qz=?,pose_qw=? WHERE id=?",
        (*pose, row_id),
    )
    if has_rtree:
        x, y, z = pose[0], pose[1], pose[2]
        conn.execute(
            f'INSERT OR REPLACE INTO "{stream}_rtree"(id,x_min,x_max,y_min,y_max,z_min,z_max) '
            f"VALUES (?,?,?,?,?,?,?)",
            (row_id, x, x, y, y, z, z),
        )


def reframe(db_path: str, urdf_path: str, rrd_path: str) -> None:
    # --- 1. truncate to the first fastlio_odometry message ---
    t0 = first_fastlio_ts(db_path)
    if t0 is None:
        raise SystemExit(f"no '{FASTLIO_ODOM}' messages in {db_path}")
    removed = truncate_db(db_path, t0)
    print(
        f"   truncate: removed {sum(removed.values())} pre-fastlio rows from {len(removed)} streams"
    )

    # legacy recordings store the Go2 onboard streams as go2_odom/go2_lidar; rename
    # to odom/lidar (only when those are missing) so the steps below find them.
    renamed = rename_go2_streams(db_path)
    if renamed:
        print(f"   renamed {', '.join(f'{old}->{new}' for old, new in renamed)}")

    graph = parse_urdf_graph(urdf_path)
    mid360_to_base = transform_between(graph, WORLD_FRAME, BASE_FRAME)
    mid360_to_camera = transform_between(graph, WORLD_FRAME, CAMERA_FRAME)

    # `.data` is lazy, so materialize everything while the store is open. World =
    # FAST-LIO's world (it reports world->mid360 as the odom value); reframe leaves
    # the fastlio streams untouched and only expresses camera/odom/lidar in it.
    with SqliteStore(path=db_path) as store:
        fastlio_ts, fastlio_pose = [], []  # world -> mid360
        for obs in store.stream(FASTLIO_ODOM, Odometry):
            fastlio_ts.append(obs.ts)
            fastlio_pose.append(_to_transform(obs.data))
        odom_ts, odom_pose = [], []  # go2_odom_frame -> base_link
        for obs in store.stream(ODOM, PoseStamped):
            odom_ts.append(obs.ts)
            odom_pose.append(_to_transform(obs.data))
        lidar_data = [  # (ts, points Nx3, intensities)
            (obs.ts, obs.data.points_f32(), obs.data.intensities_f32())
            for obs in store.stream(LIDAR, PointCloud2)
        ]
    if not fastlio_ts:
        raise SystemExit(f"no '{FASTLIO_ODOM}' rows after truncation")
    if not odom_ts:
        raise SystemExit(f"no '{ODOM}' rows after truncation")
    fastlio_ts = np.array(fastlio_ts)
    odom_ts = np.array(odom_ts)

    # --- 3. camera: pose := fastlio(t) . (mid360 -> camera_optical) ---
    conn = sqlite3.connect(db_path)
    try:
        tables = {r[0] for r in conn.execute("SELECT name FROM sqlite_master WHERE type='table'")}
        color_rows = conn.execute(f'SELECT id, ts FROM "{COLOR_IMAGE}"').fetchall()
        conn.execute("BEGIN")
        for row_id, ts in color_rows:
            world_to_cam = fastlio_pose[_nearest(fastlio_ts, ts)] + mid360_to_camera
            _update_point_pose(
                conn, COLOR_IMAGE, row_id, _pose7(world_to_cam), f"{COLOR_IMAGE}_rtree" in tables
            )
        # normalize fastlio_odometry's pose columns to the raw value (world->mid360):
        # recorders disagree (some store base_link there), so overwrite from the value
        # so downstream (build_rrd's mid360->base box) always reads a mid360 frame.
        fastlio_has_rtree = f"{FASTLIO_ODOM}_rtree" in tables
        fastlio_ids = [r[0] for r in conn.execute(f'SELECT id FROM "{FASTLIO_ODOM}" ORDER BY ts')]
        for row_id, value in zip(fastlio_ids, fastlio_pose, strict=True):
            _update_point_pose(conn, FASTLIO_ODOM, row_id, _pose7(value), fastlio_has_rtree)
        conn.execute("COMMIT")
    finally:
        conn.close()
    print(f"   camera: re-posed {len(color_rows)} '{COLOR_IMAGE}' off {WORLD_FRAME}")
    print(f"   fastlio: normalized {len(fastlio_ids)} '{FASTLIO_ODOM}' poses to mid360")

    # --- 2. align Go2 odom + lidar with one rigid transform A ---
    target0 = fastlio_pose[_nearest(fastlio_ts, odom_ts[0])] + mid360_to_base
    align = target0 + odom_pose[0].inverse()  # go2 odom frame -> mid360 world
    align = _level_in_place(align, target0)  # force the go2 lidar flat with the ground
    print("   level: go2 frame leveled in place (roll/pitch zeroed at start)")

    with SqliteStore(path=db_path) as store:
        store.delete_stream(ODOM)
        odom_out = store.stream(ODOM, PoseStamped)
        for ts, raw in zip(odom_ts, odom_pose, strict=True):
            pose7 = _pose7(align + raw)
            odom_out.append(
                PoseStamped(ts=float(ts), position=list(pose7[:3]), orientation=list(pose7[3:])),
                ts=float(ts),
                pose=pose7,
            )
        print(f"   odom: rebased {len(odom_pose)} '{ODOM}' poses into {WORLD_FRAME} world")

        store.delete_stream(LIDAR)
        lidar_out = store.stream(LIDAR, PointCloud2)
        for ts, points, intensities in lidar_data:
            transformed = PointCloud2.from_numpy(
                points, timestamp=ts, intensities=intensities
            ).transform(align)
            new_cloud = PointCloud2.from_numpy(
                transformed.points_f32(), timestamp=ts, intensities=intensities
            )
            anchor = align + odom_pose[_nearest(odom_ts, ts)]
            lidar_out.append(new_cloud, ts=ts, pose=_pose7(anchor))
        print(f"   lidar: rebased {len(lidar_data)} '{LIDAR}' clouds into {WORLD_FRAME} world")

    print(f"   rrd: rebuilding -> {rrd_path}")
    rebuild_rrd(db_path, Path(rrd_path))


def main() -> None:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("recording", help="recording dir (or its mem2.db)")
    parser.add_argument("--urdf", default=str(DEFAULT_URDF), help="URDF frame tree")
    args = parser.parse_args()

    target = Path(args.recording)
    db_path = target if target.suffix == ".db" else target / DB_NAME
    if not db_path.exists():
        raise SystemExit(f"no db at {target}")
    # mem2.db -> main.rrd; any other db (e.g. short.db) -> matching <stem>.rrd
    rrd_path = db_path.parent / (RRD_NAME if db_path.name == DB_NAME else f"{db_path.stem}.rrd")

    print(f">> reframing {db_path.parent.name} into the {WORLD_FRAME} world")
    reframe(str(db_path), args.urdf, str(rrd_path))
    print("done")


if __name__ == "__main__":
    main()
