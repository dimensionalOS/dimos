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

"""End-to-end visual stress test for ApplyClosure.

A robot walks a closed rectangular loop inside a known room. Per-step odometry
adds a systematic yaw bias plus translational noise, so by the time it returns
to the start its drifted trajectory has accumulated several meters of position
error and ~10 degrees of yaw error. At each keyframe a synthetic lidar
"observes" visible ground-truth landmark points; observations are projected
into world space using the *drifted* pose (so the resulting global map is
smeared along the drift).

The whole sequence runs inside a streaming ``while`` loop so each keyframe's
pose, observations, and accumulating voxels are rr.log'd as they're computed
— you can scrub the rerun timeline to verify the math step by step.

After the loop, the demo synthesizes a pose-graph correction by linearly
blending each drifted pose toward its known ground-truth pose (translation
lerp + rotation slerp; alpha = i / (N-1)). This mimics the redistribution
that GTSAM iSAM2 would produce when a loop-closing edge nails the endpoint
back to the start. Then ApplyClosure warps the accumulated DynamicCloud and
the corrected map is logged.

Run:
    uv run python -m dimos.navigation.nav_stack.modules.apply_closure.demo_closure_scene --step-ms 200

Flags:
    --no-spawn   Do not auto-launch the rerun viewer.
    --step-ms N  Sleep N milliseconds between keyframes (default 80).
"""

from __future__ import annotations

import argparse
import math
import time

import numpy as np
import rerun as rr
from scipy.spatial.transform import Rotation, Slerp

from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.nav_msgs.DynamicCloud import DynamicCloud
from dimos.msgs.nav_msgs.Graph3D import Graph3D
from dimos.msgs.nav_msgs.GraphDelta3D import GraphDelta3D
from dimos.navigation.nav_stack.modules.apply_closure.apply_closure import (
    apply_closure_to_cloud,
)

ROOM_SIZE = 20.0
WALL_HEIGHT = 3.0
COLUMN_RADIUS = 0.4
COLUMN_HEIGHT = 3.0
COLUMN_CENTERS = [
    (5.0, 5.0),
    (15.0, 5.0),
    (5.0, 15.0),
    (15.0, 15.0),
]


def sample_wall(p0: tuple[float, float], p1: tuple[float, float]) -> np.ndarray:
    n_along = max(2, int(np.linalg.norm(np.array(p1) - np.array(p0)) * 8))
    n_vert = 12
    us = np.linspace(0.0, 1.0, n_along)
    vs = np.linspace(0.0, WALL_HEIGHT, n_vert)
    p0a = np.array(p0, dtype=np.float64)
    p1a = np.array(p1, dtype=np.float64)
    seg = p0a[None, :] + (p1a - p0a)[None, :] * us[:, None]
    pts = np.zeros((n_along, n_vert, 3))
    pts[..., 0] = seg[:, 0:1]
    pts[..., 1] = seg[:, 1:2]
    pts[..., 2] = vs[None, :]
    return pts.reshape(-1, 3)


def sample_cylinder(center: tuple[float, float]) -> np.ndarray:
    n_around = 32
    n_vert = 12
    angles = np.linspace(0.0, 2.0 * math.pi, n_around, endpoint=False)
    zs = np.linspace(0.0, COLUMN_HEIGHT, n_vert)
    xs = center[0] + COLUMN_RADIUS * np.cos(angles)
    ys = center[1] + COLUMN_RADIUS * np.sin(angles)
    pts = np.zeros((n_around, n_vert, 3))
    pts[..., 0] = xs[:, None]
    pts[..., 1] = ys[:, None]
    pts[..., 2] = zs[None, :]
    return pts.reshape(-1, 3)


def build_ground_truth() -> np.ndarray:
    walls = np.concatenate(
        [
            sample_wall((0.0, 0.0), (ROOM_SIZE, 0.0)),
            sample_wall((ROOM_SIZE, 0.0), (ROOM_SIZE, ROOM_SIZE)),
            sample_wall((ROOM_SIZE, ROOM_SIZE), (0.0, ROOM_SIZE)),
            sample_wall((0.0, ROOM_SIZE), (0.0, 0.0)),
        ],
        axis=0,
    )
    columns = np.concatenate([sample_cylinder(c) for c in COLUMN_CENTERS], axis=0)
    return np.concatenate([walls, columns], axis=0)


PATH_INSET = 2.0
KEYFRAMES_PER_SIDE = 12  # 48 keyframes total around the perimeter


def build_true_poses() -> tuple[np.ndarray, np.ndarray]:
    """Return (times[N], poses[N, 4, 4]) tracing a closed rectangular loop."""
    corners = np.array(
        [
            [PATH_INSET, PATH_INSET],
            [ROOM_SIZE - PATH_INSET, PATH_INSET],
            [ROOM_SIZE - PATH_INSET, ROOM_SIZE - PATH_INSET],
            [PATH_INSET, ROOM_SIZE - PATH_INSET],
            [PATH_INSET, PATH_INSET],
        ],
        dtype=np.float64,
    )

    positions: list[np.ndarray] = []
    yaws: list[float] = []
    for i in range(4):
        a, b = corners[i], corners[i + 1]
        for k in range(KEYFRAMES_PER_SIDE):
            t = k / KEYFRAMES_PER_SIDE
            positions.append(a + (b - a) * t)
            yaws.append(math.atan2(b[1] - a[1], b[0] - a[0]))
    n = len(positions)
    poses = np.zeros((n, 4, 4))
    times = np.linspace(1.0, 1.0 + n * 0.5, n)  # 0.5s per keyframe
    for i, (xy, yaw) in enumerate(zip(positions, yaws, strict=True)):
        T = np.eye(4)
        T[:3, :3] = Rotation.from_euler("z", yaw).as_matrix()
        T[:2, 3] = xy
        T[2, 3] = 0.5  # robot sensor at 0.5m above the floor
        poses[i] = T
    return times, poses


YAW_BIAS_PER_STEP_DEG = 0.35
TRANSLATION_NOISE_STD = 0.02  # m per step
LIDAR_MAX_RANGE = 6.0
VOXEL_SIZE = 0.25


def step_drift(
    prev_drifted: np.ndarray, body_step: np.ndarray, rng: np.random.Generator
) -> np.ndarray:
    """Compose the prev drifted pose with a noisy version of the true body step."""
    yaw_bias = math.radians(YAW_BIAS_PER_STEP_DEG)
    R_bias = Rotation.from_euler("z", yaw_bias).as_matrix()
    noisy = body_step.copy()
    noisy[:3, :3] = R_bias @ noisy[:3, :3]
    noisy[:3, 3] += rng.normal(0.0, TRANSLATION_NOISE_STD, 3)
    return prev_drifted @ noisy  # type: ignore[no-any-return]


def visible_points(true_pose: np.ndarray, gt_points: np.ndarray) -> np.ndarray:
    """Return GT points within ``LIDAR_MAX_RANGE`` of the pose origin."""
    origin = true_pose[:3, 3]
    d = np.linalg.norm(gt_points - origin, axis=1)
    return gt_points[d <= LIDAR_MAX_RANGE]  # type: ignore[no-any-return]


def apply_delta(delta: np.ndarray, points: np.ndarray) -> np.ndarray:
    """Apply a 4x4 transform to (M, 3) points."""
    homog = np.concatenate([points, np.ones((points.shape[0], 1))], axis=1)
    return (homog @ delta.T)[:, :3]  # type: ignore[no-any-return]


def synthesize_closure_correction(drifted_poses: np.ndarray, true_poses: np.ndarray) -> np.ndarray:
    """Blend drifted toward true linearly along the trajectory.

    Mimics what GTSAM + ICP produces after the closing edge nails the endpoint
    back to the start: alpha = i / (N-1), slerp on rotation, lerp on translation.
    """
    n = drifted_poses.shape[0]
    corrected = np.empty_like(drifted_poses)
    alphas = np.linspace(0.0, 1.0, n)

    drifted_R = Rotation.from_matrix(drifted_poses[:, :3, :3])
    true_R = Rotation.from_matrix(true_poses[:, :3, :3])
    drifted_t = drifted_poses[:, :3, 3]
    true_t = true_poses[:, :3, 3]

    for i in range(n):
        a = alphas[i]
        key_R = Rotation.concatenate([drifted_R[i], true_R[i]])
        slerp = Slerp([0.0, 1.0], key_R)
        R_blend = slerp([a])[0]
        t_blend = (1.0 - a) * drifted_t[i] + a * true_t[i]
        T = np.eye(4)
        T[:3, :3] = R_blend.as_matrix()
        T[:3, 3] = t_blend
        corrected[i] = T
    return corrected


def lerp_pose_arrays(A: np.ndarray, B: np.ndarray, alpha: float) -> np.ndarray:
    """Per-node lerp/slerp between two pose arrays at fraction ``alpha``.

    Translations are linearly interpolated; rotations use scipy's Slerp on
    each pair of quaternions independently. Used to animate the closure
    correction so we can watch the cloud snap from drifted to corrected.
    """
    n = A.shape[0]
    R_A = Rotation.from_matrix(A[:, :3, :3])
    R_B = Rotation.from_matrix(B[:, :3, :3])
    out = np.empty_like(A)
    for i in range(n):
        key_R = Rotation.concatenate([R_A[i], R_B[i]])
        slerp = Slerp([0.0, 1.0], key_R)
        R_blend = slerp([alpha])[0]
        t_blend = (1.0 - alpha) * A[i, :3, 3] + alpha * B[i, :3, 3]
        T = np.eye(4)
        T[:3, :3] = R_blend.as_matrix()
        T[:3, 3] = t_blend
        out[i] = T
    return out


def _matrix_to_translation_quaternion(mat: np.ndarray) -> tuple[Vector3, Quaternion]:
    quat = Rotation.from_matrix(mat[:3, :3]).as_quat()  # [x, y, z, w]
    return (
        Vector3(float(mat[0, 3]), float(mat[1, 3]), float(mat[2, 3])),
        Quaternion(float(quat[0]), float(quat[1]), float(quat[2]), float(quat[3])),
    )


def make_graph_delta(
    times: np.ndarray, prev_poses: np.ndarray, target_poses: np.ndarray
) -> GraphDelta3D:
    """Build a GraphDelta3D carrying the per-node correction from prev → target.

    ``nodes[i].pose`` snapshots ``prev_poses[i]``; ``transforms[i]`` is the
    world-frame delta s.t. ``transforms[i] @ prev_poses[i] = target_poses[i]``.
    This is the message PGO would publish on a real loop-closure event.
    """
    nodes: list[Graph3D.Node3D] = []
    transforms: list[GraphDelta3D.Transform] = []
    for i, ts in enumerate(times):
        prev_mat = prev_poses[i]
        delta_mat = target_poses[i] @ np.linalg.inv(prev_mat)

        prev_t, prev_q = _matrix_to_translation_quaternion(prev_mat)
        delta_t, delta_q = _matrix_to_translation_quaternion(delta_mat)

        pose = PoseStamped(
            ts=float(ts),
            frame_id="map",
            position=[prev_t.x, prev_t.y, prev_t.z],
            orientation=[prev_q.x, prev_q.y, prev_q.z, prev_q.w],
        )
        nodes.append(Graph3D.Node3D(pose=pose, id=i, metadata_id=0))
        transforms.append(GraphDelta3D.Transform(translation=delta_t, rotation=delta_q))
    return GraphDelta3D(ts=float(times[-1]), nodes=nodes, transforms=transforms)


def log_pose_arrow(name: str, T: np.ndarray, color: tuple[int, int, int]) -> None:
    origin = T[:3, 3]
    forward = T[:3, :3] @ np.array([0.6, 0.0, 0.0])
    rr.log(name, rr.Arrows3D(origins=[origin], vectors=[forward], colors=[color]))


def log_voxels(
    name: str,
    cloud: DynamicCloud,
    color: tuple[int, int, int],
    radii: float | None = None,
) -> None:
    pts = cloud.world_positions()
    if pts.shape[0] == 0:
        return
    r = cloud.voxel_size / 2 if radii is None else radii
    rr.log(name, rr.Points3D(pts, colors=[color], radii=r))


def mean_nearest_distance(cloud_points: np.ndarray, target_points: np.ndarray) -> float:
    """Mean nearest-neighbor distance from cloud_points to target_points."""
    if cloud_points.shape[0] == 0 or target_points.shape[0] == 0:
        return float("nan")
    chunk = 2048
    total = 0.0
    for i in range(0, cloud_points.shape[0], chunk):
        block = cloud_points[i : i + chunk]
        d2 = ((block[:, None, :] - target_points[None, :, :]) ** 2).sum(axis=2)
        total += float(np.sqrt(d2.min(axis=1)).sum())
    return total / cloud_points.shape[0]  # type: ignore[no-any-return]


def run_demo(spawn: bool, step_ms: int) -> None:
    rr.init("apply_closure_demo", spawn=spawn)

    gt_points = build_ground_truth()
    rr.log(
        "world/ground_truth",
        rr.Points3D(gt_points, colors=[150, 150, 150], radii=0.04),
        static=True,
    )

    times, true_poses = build_true_poses()
    n = len(times)
    rr.log(
        "world/trajectory/true",
        rr.LineStrips3D([true_poses[:, :3, 3]], colors=[(60, 200, 80)], radii=0.06),
        static=True,
    )

    # Streaming state built up step by step inside the while loop below.
    rng = np.random.default_rng(7)
    drifted_poses = np.empty_like(true_poses)
    drifted_poses[0] = true_poses[0]

    voxel_to_idx: dict[tuple[int, int, int], int] = {}
    quantity: list[int] = []
    event_idx: list[int] = []
    event_ts: list[int] = []
    accumulating: list[np.ndarray] = []

    i = 0
    while i < n:
        rr.set_time("step", sequence=i)
        rr.set_time("sim_time", duration=float(times[i] - times[0]))

        # Drifted pose: identity at i=0, accumulate noisy body steps otherwise.
        if i > 0:
            body_step = np.linalg.inv(true_poses[i - 1]) @ true_poses[i]
            drifted_poses[i] = step_drift(drifted_poses[i - 1], body_step, rng)

        true_T = true_poses[i]
        drifted_T = drifted_poses[i]

        # Visible GT points from the TRUE pose (what the robot actually sees);
        # project into world using the DRIFTED pose (what the robot thinks);
        # voxelize and accumulate.
        seen = visible_points(true_T, gt_points)
        log_pose_arrow("world/pose/true", true_T, (60, 200, 80))
        log_pose_arrow("world/pose/drifted", drifted_T, (220, 70, 70))
        rr.log(
            "world/trajectory/drifted_so_far",
            rr.LineStrips3D([drifted_poses[: i + 1, :3, 3]], colors=[(220, 70, 70)], radii=0.06),
        )

        if seen.shape[0] > 0:
            delta = drifted_T @ np.linalg.inv(true_T)
            observed = apply_delta(delta, seen)
            accumulating.append(observed)

            voxels = np.rint(observed / VOXEL_SIZE).astype(np.int32)
            ts_ns = int(times[i] * 1_000_000_000)
            for v in voxels:
                key = (int(v[0]), int(v[1]), int(v[2]))
                idx = voxel_to_idx.get(key)
                if idx is None:
                    idx = len(voxel_to_idx)
                    voxel_to_idx[key] = idx
                    quantity.append(0)
                quantity[idx] += 1
                event_idx.append(idx)
                event_ts.append(ts_ns)

            rr.log(
                "world/observations/this_frame",
                rr.Points3D(observed, colors=[255, 200, 60], radii=0.06),
            )
            cumulative = np.concatenate(accumulating, axis=0)
            rr.log(
                "world/observations/drifted_accum",
                rr.Points3D(cumulative, colors=[220, 70, 70], radii=0.05),
            )

        if step_ms > 0:
            time.sleep(step_ms / 1000.0)
        i += 1

    # Closure event: synthesize the target correction and apply it.
    rr.set_time("step", sequence=n)
    rr.set_time("sim_time", duration=float(times[-1] - times[0] + 1.0))

    # The per-frame yellow points were temporary; clear them so the voxel
    # global map is the dominant thing visible after the loop.
    rr.log("world/observations/this_frame", rr.Clear(recursive=False))

    corrected_poses = synthesize_closure_correction(drifted_poses, true_poses)
    rr.log(
        "world/closure/correction_arrows",
        rr.Arrows3D(
            origins=drifted_poses[:, :3, 3],
            vectors=corrected_poses[:, :3, 3] - drifted_poses[:, :3, 3],
            colors=[90, 140, 255],
        ),
        static=True,
    )

    # Materialize the accumulated DynamicCloud — this is what the running
    # system has produced just before the closure event fires.
    unique = np.array(sorted(voxel_to_idx, key=lambda k: voxel_to_idx[k]), dtype=np.int32)
    drifted_cloud = DynamicCloud(
        voxels=unique,
        quantity=np.array(quantity, dtype=np.uint32),
        event_indices=np.array(event_idx, dtype=np.uint32),
        event_timestamps=np.array(event_ts, dtype=np.uint64),
        voxel_size=VOXEL_SIZE,
        frame_id="map",
        ts=float(times[-1]),
    )
    # Snapshot the "before" state on the closure step so it's still visible
    # if you scrub back here.
    log_voxels("world/global_map/drifted", drifted_cloud, (220, 70, 70), radii=0.10)

    # Animate the closure: ramp alpha 0→1 across n_anim frames, applying
    # ApplyClosure each frame so the voxel map visibly snaps into place. Each
    # frame builds a fresh GraphDelta3D whose transforms[i] is the partial
    # correction needed at fraction alpha.
    n_anim = 24
    for j in range(n_anim + 1):
        alpha = j / n_anim
        rr.set_time("step", sequence=n + 1 + j)
        rr.set_time("sim_time", duration=float(times[-1] - times[0] + 1.0 + alpha))

        interp_poses = lerp_pose_arrays(drifted_poses, corrected_poses, alpha)
        closure_event = make_graph_delta(times, drifted_poses, interp_poses)
        corrected_at_alpha = apply_closure_to_cloud(drifted_cloud, closure_event)

        log_voxels("world/global_map/corrected", corrected_at_alpha, (60, 200, 80), radii=0.10)
        rr.log(
            "world/trajectory/corrected",
            rr.LineStrips3D([interp_poses[:, :3, 3]], colors=[(90, 140, 255)], radii=0.06),
        )
        if step_ms > 0:
            time.sleep(step_ms / 1000.0)

    # Final corrected cloud is whatever the full correction produces.
    final_closure_event = make_graph_delta(times, drifted_poses, corrected_poses)
    corrected_cloud = apply_closure_to_cloud(drifted_cloud, final_closure_event)

    err_before = mean_nearest_distance(drifted_cloud.world_positions(), gt_points)
    err_after = mean_nearest_distance(corrected_cloud.world_positions(), gt_points)
    endpoint_drift = float(np.linalg.norm(drifted_poses[-1, :3, 3] - true_poses[0, :3, 3]))
    print(
        f"keyframes               : {n}\n"
        f"endpoint position error : {endpoint_drift:.2f} m\n"
        f"drifted cloud  → GT mean nn dist: {err_before:.3f} m\n"
        f"corrected cloud → GT mean nn dist: {err_after:.3f} m\n"
        f"voxels in cloud: {len(drifted_cloud)} drifted, {len(corrected_cloud)} corrected"
    )


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--no-spawn", action="store_true", help="Do not auto-launch the rerun viewer."
    )
    parser.add_argument(
        "--step-ms",
        type=int,
        default=80,
        help="Sleep this many ms between keyframes so you can watch generation live.",
    )
    args = parser.parse_args()
    run_demo(spawn=not args.no_spawn, step_ms=args.step_ms)


if __name__ == "__main__":
    main()
