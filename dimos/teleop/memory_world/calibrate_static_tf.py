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

"""Measure the camera's real mount from the data, and write it back as a corrected tf.

The cart recordings' ``tf_static`` puts the camera 90 degrees out in yaw: pictures
hang askew against a level world even though the pictures themselves are level.
Nothing about the recording says so, and the only ground truth is the lidar -- the
same walls, seen twice. So this converts each depth image into a point cloud, pools
correspondences to the simultaneous lidar scan over every sampled frame, and solves
the ONE rigid transform that explains them all. Per-frame registration was tried
first and is far too unstable to trust: a rigid mount only shows up when the frames
are solved together.

With ``--write`` the answer goes into the recording's own ``tf_static``, replacing
whatever that edge said. Nothing reads it specially afterwards: a recording whose
static tf is wrong is fixed by writing the right static tf.

    python -m dimos.teleop.memory_world.calibrate_static_tf <recording.db> [--samples 20]
        [--write]
"""

from __future__ import annotations

import argparse
import logging
from typing import Any

import numpy as np

from dimos.teleop.memory_world.tf_tree import quaternion_from_matrix

logger = logging.getLogger(__name__)

MIN_DEPTH_M, MAX_DEPTH_M = 1.0, 4.0  # outside this a d455 returns noise, not geometry
MAX_LIDAR_M = 12.0
VOXEL_M = 0.06
# Below this share of camera points within 10 cm of a lidar point the clouds never met.
MIN_INLIER_FRACTION = 0.25
GRID_DEG = np.arange(-40, 41, 8.0)  # the seed rotations, around the nominal axis swap
SEEDS_REFINED = 24
# camera optical (x right, y down, z forward) -> a livox body frame (x fwd, y left, z up)
NOMINAL = np.array([[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]])


def _rotation(roll: float, pitch: float, yaw: float) -> np.ndarray:
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    return np.array(
        [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ]
    )


def _nearest(stream: Any, ts: float, tolerance: float) -> Any:
    """The observation closest to *ts*, not merely the first inside the window."""
    near = list(stream.at(ts, tolerance=tolerance))
    if not near:
        raise LookupError(f"nothing within {tolerance}s of {ts}")
    return min(near, key=lambda obs: abs(float(obs.ts) - ts))


def _thinned(points: np.ndarray, size: float, cap: int) -> np.ndarray:
    """One point per *size* cube, then at most *cap* of them."""
    _, keep = np.unique(np.floor(points / size).astype(np.int64), axis=0, return_index=True)
    points = points[keep]
    if len(points) > cap:
        points = points[np.random.default_rng(0).choice(len(points), cap, replace=False)]
    return points


def _pairs(store: Any, streams: dict[str, Any], samples: int) -> list[tuple[np.ndarray, Any]]:
    """(depth cloud in the camera's optical frame, kd-tree of the lidar scan) per sample."""
    from scipy.spatial import cKDTree

    from dimos.teleop.memory_world.recording import depth_info_stream_for

    info_name = depth_info_stream_for(
        set(store.list_streams()), streams["depth"], streams["camera_info"]
    )
    K = np.asarray(store.streams[info_name].first().data.K, dtype=np.float64)
    fx, fy, cx, cy = K[0], K[4], K[2], K[5]
    depth, lidar = store.streams[streams["depth"]], store.streams[streams["lidar"]]
    first, last = depth.first(), depth.last()

    out = []
    for k in range(samples):
        ts = float(first.ts) + (k + 0.5) / samples * (float(last.ts) - float(first.ts))
        try:
            near = _nearest(depth, ts, 0.2)
            scan = _nearest(lidar, float(near.ts), 0.12)
        except LookupError:
            continue
        raw = np.asarray(near.data.data)
        if raw.ndim != 2:
            continue
        step = 4  # a 1280x720 depth image is far more than the fit needs
        rows, cols = np.mgrid[0 : raw.shape[0] : step, 0 : raw.shape[1] : step]
        z = raw[::step, ::step].astype(np.float64)
        if not np.issubdtype(raw.dtype, np.floating):
            z /= 1000.0  # an integer depth image is millimetres; a float one is metres
        usable = (z > MIN_DEPTH_M) & (z < MAX_DEPTH_M) & np.isfinite(z)
        if usable.sum() < 2000:
            continue
        zz = z[usable]
        cloud = np.stack([(cols[usable] - cx) * zz / fx, (rows[usable] - cy) * zz / fy, zz], axis=1)
        points = np.asarray(scan.data.points_f32(), dtype=np.float64)
        points = points[np.isfinite(points).all(axis=1)]
        points = points[np.linalg.norm(points, axis=1) < MAX_LIDAR_M]
        if len(points) < 3000:
            continue
        out.append((_thinned(cloud, VOXEL_M, 2500), cKDTree(_thinned(points, VOXEL_M, 60000))))
    return out


def _residual(matrix: np.ndarray, pairs: list[Any], trim: float) -> float:
    """Mean distance from the camera cloud to the lidar cloud, capped at *trim*."""
    total, count = 0.0, 0
    for cloud, kd in pairs:
        distance, _ = kd.query(cloud @ matrix[:3, :3].T + matrix[:3, 3], workers=-1)
        total += float(np.minimum(distance, trim).sum())
        count += len(distance)
    return total / max(count, 1)


def _joint_icp(matrix: np.ndarray, pairs: list[Any], rounds: int = 40) -> np.ndarray:
    """Pool every frame's correspondences and solve one rigid transform from the pool."""
    matrix = matrix.copy()
    for step in range(rounds):
        trim = max(0.12, 0.6 * (0.9**step))  # a wide gate first, tightening as it settles
        source, target = [], []
        for cloud, kd in pairs:
            distance, nearest = kd.query(cloud @ matrix[:3, :3].T + matrix[:3, 3], workers=-1)
            keep = distance < trim
            if keep.sum() < 50:
                continue
            source.append(cloud[keep])
            target.append(kd.data[nearest[keep]])
        if not source:
            return matrix
        source, target = np.vstack(source), np.vstack(target)
        source_mid, target_mid = source.mean(axis=0), target.mean(axis=0)
        u, _, vt = np.linalg.svd((source - source_mid).T @ (target - target_mid))
        rotation = (u @ np.diag([1, 1, np.sign(np.linalg.det(u @ vt))]) @ vt).T
        matrix = np.eye(4)
        matrix[:3, :3], matrix[:3, 3] = rotation, target_mid - rotation @ source_mid
    return matrix


def measure_camera_from_lidar(
    store: Any, streams: dict[str, Any], samples: int = 20, pairs: list[Any] | None = None
) -> Any:
    """Solve ``lidar_T_depth_optical`` from *samples* depth/lidar pairs.

    None when too few frames are usable, and None again when enough were usable but
    never agreed; the caller says which, since they mean different things to whoever
    is running it.
    """
    if pairs is None:
        pairs = _pairs(store, streams, samples)
    logger.info("%d usable depth/lidar pairs", len(pairs))
    if len(pairs) < 4:
        return None
    scout = pairs[:: max(1, len(pairs) // 5)][:5]  # a cheap score picks the seeds to refine
    seeds = []
    for roll in np.radians(GRID_DEG):
        for pitch in np.radians(GRID_DEG):
            for yaw in np.radians(GRID_DEG):
                seed = np.eye(4)
                seed[:3, :3] = _rotation(roll, pitch, yaw) @ NOMINAL
                seeds.append((_residual(seed, scout, 0.6), seed))
    seeds.sort(key=lambda pair: pair[0])

    best, best_cost = None, np.inf
    for _, seed in seeds[:SEEDS_REFINED]:
        fitted = _joint_icp(seed, pairs)
        cost = _residual(fitted, pairs, 0.3)
        if cost < best_cost:
            best, best_cost = fitted, cost
    if best is None:
        return None
    inliers = [
        float((kd.query(cloud @ best[:3, :3].T + best[:3, 3], workers=-1)[0] < 0.1).mean())
        for cloud, kd in pairs
    ]
    logger.info(
        "residual %.4f m, median inlier fraction %.2f", best_cost, float(np.median(inliers))
    )
    # The truncated residual stays finite when nothing matched at all -- every point
    # simply costs the cap -- so a seed that never moved would otherwise be published
    # as a measurement. Support is what says the two clouds actually found each other.
    supported = sum(1 for fraction in inliers if fraction >= MIN_INLIER_FRACTION)
    if float(np.median(inliers)) < MIN_INLIER_FRACTION or supported < len(pairs) // 2:
        logger.warning(
            "no agreement: %d of %d frames reach %.0f%% inliers; this is not a mount",
            supported,
            len(pairs),
            MIN_INLIER_FRACTION * 100,
        )
        return None
    return best, best_cost, inliers


def rigidly_joined(tree: Any, frame: str, other: str) -> bool:
    """Whether one fixed transform relates *frame* to *other* for the whole recording.

    That is the property a calibration needs, and it is the property to test: a map
    frame moves against the camera however it is named or wherever it hangs, and a
    sensor frame does not however far above the body it sits.
    """
    hops = tree._path(frame, other)
    if hops is None:
        return False
    for parent, child, _ in hops:
        edge = tree._edges.get((parent, child))
        if edge is None:
            return False
        if edge.static:
            continue
        _, positions, orientations = edge._arrays()
        if not len(positions):
            continue
        moved = float(np.ptp(positions, axis=0).max())
        # q and -q are the same rotation, and a republisher is free to flip the sign,
        # so compare by how far apart the rotations are, not by the numbers.
        aligned = orientations @ orientations[0]
        turned = float(np.abs(np.abs(aligned) - 1.0).max())
        if max(moved, turned) > 1e-6:  # a republished static edge never moves
            return False
    return True


def sensor_lidar_stream(
    store: Any, tree: Any, streams: dict[str, Any], camera_frame: str
) -> str | None:
    """The lidar stream whose points are still fixed with respect to the camera.

    A stitched recording also carries its clouds in the map frame (``*_corrected``,
    stamped ``corrected_odom``). Those are the same walls seen from a moving robot,
    so no single transform relates them to the camera; only a rigidly joined frame
    calibrates.
    """
    for name in [streams.get("lidar"), *streams.get("lidar_candidates", [])]:
        if not name:
            continue
        try:
            sample = store.streams[name].first()
        except LookupError:  # declared but empty: try the next candidate, not exit
            continue
        frame = str(getattr(sample.data, "frame_id", "") or "").lstrip("/")
        if frame and rigidly_joined(tree, frame, camera_frame):
            return name
    return None


def camera_mount_edge(
    tree: Any, camera_frame: str, lidar_frame: str, body: str = "base_link"
) -> tuple[str | None, str]:
    """The edge to put the whole correction on: (what the camera hangs off, the camera root).

    Walk the tf path from the lidar to the camera. It climbs to the frame the two
    sensors share and then descends; the outermost descending edge that is not the
    robot's own body is the camera's, so correcting it carries colour, depth and infra
    together and leaves the trajectory alone. Writing it onto the body edge instead
    would rotate every base_link pose by the camera's error, which is how you end up
    with a right camera and a wrong path.

    A relative measurement cannot say WHICH joint along the path is wrong -- a yaw at
    the camera's joint and the opposite yaw at the lidar's give the same answer -- so
    this puts the whole correction on one edge by choice, not by deduction. Separating
    them needs something outside both chains, such as gravity on the body.
    """
    hops = tree._path(lidar_frame, camera_frame)
    if not hops:
        return None, camera_frame
    for parent, child, forward in hops:
        if forward and child != body:  # descending, and not the robot itself
            return parent, child
    return None, camera_frame


def corrected_mount(
    tree: Any, lidar_T_camera: np.ndarray, camera_frame: str, lidar_frame: str, ts: float
) -> Any:
    """What the camera's mount edge should have been, given the measured extrinsic."""
    mount, camera_root = camera_mount_edge(tree, camera_frame, lidar_frame)
    if mount is None:
        return None
    mount_T_lidar = tree.lookup(mount, lidar_frame, ts, 0.5)
    root_T_camera = tree.lookup(camera_root, camera_frame, ts, 0.5)
    if mount_T_lidar is None or root_T_camera is None:
        return None
    matrix = np.asarray(mount_T_lidar) @ lidar_T_camera @ np.linalg.inv(np.asarray(root_T_camera))
    return mount, camera_root, matrix


def write_static_mount(store: Any, mount: str, child: str, matrix: np.ndarray, ts: float) -> str:
    """Put the measured mount into the recording's own ``tf_static``, and say where.

    Not a correction layer the reader has to know about: a recording whose static tf
    is wrong is fixed by writing the right static tf, and everything downstream then
    reads an ordinary recording. A recording with no ``tf_static`` gains one holding
    this edge alone; one that has it keeps every other edge and loses only this one.
    """
    from dimos.msgs.geometry_msgs.Quaternion import Quaternion
    from dimos.msgs.geometry_msgs.Transform import Transform
    from dimos.msgs.geometry_msgs.Vector3 import Vector3
    from dimos.msgs.tf2_msgs.TFMessage import TFMessage
    from dimos.teleop.memory_world.recording import detect_streams

    name = detect_streams(store).get("tf_static") or "tf_static"
    if name in getattr(getattr(store, "recording", None), "list_streams", list)():
        raise SystemExit(
            f"{name!r} belongs to the recording itself, which is read only. Fix the mount"
            " where the recording is written, or convert it to a .db first."
        )
    # One entry per edge, first sample winning, which is what every reader resolves to
    # anyway. Flattening every sample instead would rewrite a stream that latches its
    # static tf once a second as one message holding N copies of every edge.
    by_edge: dict[tuple[str, str], Any] = {}
    for obs in store.streams[name] if name in store.list_streams() else []:
        for t in obs.data.transforms:
            by_edge.setdefault((str(t.frame_id), str(t.child_frame_id)), t)
    original = list(by_edge.values())
    kept = [t for edge, t in by_edge.items() if edge != (mount, child)]
    x, y, z, w = quaternion_from_matrix(matrix[:3, :3])
    corrected = Transform(
        translation=Vector3(*(float(v) for v in matrix[:3, 3])),
        rotation=Quaternion(float(x), float(y), float(z), float(w)),
        frame_id=mount,
        child_frame_id=child,
        ts=ts,
    )
    if name in store.list_streams():
        store.delete_stream(name)  # rewritten whole: one sample of an edge, held for all time
    try:
        store.stream(name, TFMessage).append(TFMessage(*kept, corrected), ts=ts)
    except BaseException:
        # Between the delete and the append the recording has no static tf at all, and the
        # only copy of it is in memory. A full disk or a Ctrl-C there would take the lidar
        # mount, the imu and everything else with it, permanently and silently. What goes
        # back is what was THERE -- `kept` is missing the very edge being replaced, so
        # restoring that would drop the old camera mount and, if it was the only edge,
        # would write nothing at all.
        if original:
            store.stream(name, TFMessage).append(TFMessage(*original), ts=ts)
        raise
    return name


def drop_what_the_mount_invalidates(store: Any, recording: str) -> list[str]:
    """Remove the caches that hold camera poses computed with the mount just replaced.

    A search index and a Hyperspace memory db store poses AS COMPUTED and never re-place
    them on read, so moving the mount leaves both a whole correction away from the map
    while every surface still says ready. Rather than teach the readers to notice, the
    command that invalidates them clears them: the next start rebuilds. The ray-traced
    map, the path and the lidar are untouched, because the mount edge carries only the
    camera.
    """
    from dimos.teleop.memory_world.hyperspace_search import memory_db_for

    dropped = []
    memory_db = memory_db_for(recording)
    for path in (memory_db, *(memory_db.with_name(memory_db.name + s) for s in ("-wal", "-shm"))):
        if path.exists():
            path.unlink()
            if path == memory_db:
                dropped.append(path.name)
    # By payload, not by name: an index built with --index-stream can be called
    # anything, and one missed here keeps poses from the mount that was just replaced.
    for name in list(store.list_streams()):
        try:
            payload = type(store.streams[name].first().data).__name__
        except Exception:  # empty or unreadable: nothing of the old mount in it
            continue
        if payload != "PatchGrid":
            continue
        try:
            store.delete_stream(name)
        except ValueError:  # part of the recording itself: not ours to remove
            continue
        dropped.append(name)
    return dropped


def _report(name: str, matrix: np.ndarray) -> str:
    r = matrix[:3, :3]
    pitch = float(np.arcsin(np.clip(-r[2, 0], -1, 1)))
    if abs(r[2, 0]) > 0.999999:
        roll, yaw = float(np.arctan2(-r[1, 2], r[1, 1])), 0.0
    else:
        roll, yaw = float(np.arctan2(r[2, 1], r[2, 2])), float(np.arctan2(r[1, 0], r[0, 0]))
    xyz = np.round(matrix[:3, 3], 4)
    return f"{name}: xyz {xyz}  rpy {np.round(np.degrees([roll, pitch, yaw]), 2)} deg"


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("recording")
    parser.add_argument("--samples", type=int, default=20)
    parser.add_argument(
        "--write",
        action="store_true",
        help="put the measured mount into the recording's tf_static (default: print only)",
    )
    args = parser.parse_args()
    logging.basicConfig(level=logging.INFO, format="%(message)s")

    from dimos.teleop.memory_world.recording import (
        build_tf_tree,
        depth_info_stream_for,
        detect_streams,
        open_recording,
    )

    store = open_recording(args.recording)
    store.start()
    try:
        streams = detect_streams(store)
        for role in ("depth", "lidar", "tf"):
            if not streams.get(role):
                raise SystemExit(f"{args.recording} has no {role} stream; cannot calibrate")
        # The depth camera's own intrinsics are what unprojects its images; a rig with no
        # colour camera has no colour camera_info and does not need one.
        if not depth_info_stream_for(
            set(store.list_streams()), streams["depth"], streams.get("camera_info")
        ):
            raise SystemExit(f"{args.recording} has no camera_info for {streams['depth']!r}")
        tree = build_tf_tree(store, streams["tf"])
        camera_frame = str(
            getattr(store.streams[streams["depth"]].first().data, "frame_id", "") or ""
        ).lstrip("/")
        sensor_lidar = sensor_lidar_stream(store, tree, streams, camera_frame)
        if sensor_lidar is None:
            raise SystemExit(
                f"no lidar stream of {args.recording} is rigidly joined to {camera_frame!r};"
                " only scans that hold still against the camera can calibrate it"
            )
        streams["lidar"] = sensor_lidar
        lidar_frame = str(
            getattr(store.streams[streams["lidar"]].first().data, "frame_id", "") or ""
        ).lstrip("/")
        ts = float(store.streams[streams["depth"]].first().ts)

        if args.write:  # before six minutes of fitting, not after
            static = detect_streams(store).get("tf_static")
            if (
                static
                and static in getattr(getattr(store, "recording", None), "list_streams", list)()
            ):
                raise SystemExit(
                    f"{static!r} belongs to {args.recording} itself, which is read only."
                    " Fix the mount where the recording is written, or convert it to a .db."
                )
        pairs = _pairs(store, streams, args.samples)
        if len(pairs) < 4:
            raise SystemExit(
                f"only {len(pairs)} depth/lidar pairs are usable; cannot measure the mount"
            )
        measured = measure_camera_from_lidar(store, streams, args.samples, pairs=pairs)
        if measured is None:
            raise SystemExit(
                f"{len(pairs)} frames were usable but never agreed on one transform; the"
                " scene may be too empty or too far to calibrate against"
            )
        lidar_T_camera, residual, inliers = measured
        print(_report(f"measured {lidar_frame} <- {camera_frame}", lidar_T_camera))
        print(f"  residual {residual:.4f} m, inlier fraction {np.round(inliers, 2)}")

        as_recorded = tree.lookup(lidar_frame, camera_frame, ts, 0.5)
        if as_recorded is not None:  # the same frames, so the two numbers compare
            print(_report("the recording's own tf ", np.asarray(as_recorded)))
            print(f"  residual {_residual(np.asarray(as_recorded), pairs, 0.3):.4f} m")

        fixed = corrected_mount(tree, lidar_T_camera, camera_frame, lidar_frame, ts)
        if fixed is None:
            raise SystemExit(f"no mount edge above {camera_frame}; nothing to correct")
        mount, child, matrix = fixed
        print(_report(f"corrected {mount} -> {child}", matrix))
        if not args.write:
            print("measured only; pass --write to put it in the recording's tf_static")
            return
        name = write_static_mount(store, mount, child, matrix, ts)
        print(f"wrote {mount} -> {child} into {name!r} of {args.recording}")
        for gone in drop_what_the_mount_invalidates(store, args.recording):
            print(f"dropped {gone}, which holds poses from the old mount")
    finally:
        store.stop()


if __name__ == "__main__":
    main()
