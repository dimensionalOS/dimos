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

"""Cross-recording ground-truth tag eval (todo item 4).

Run the best PGO on huge_loop_realsense to get corrected April-tag world
positions (treated as ground truth, since cmu reaches ~0.14 m tag spread there),
then measure how close each PGO moves huge_loop_go2's tags toward those GT
locations — a CROSS-recording metric, unlike the within-recording agreement.

Both recordings see the same physical tags. The pipeline per recording:
  1. run a PGO -> corrected keyframe trajectory (graph).
  2. read each tag sighting's pose-in-optical from the recorded april_tags
     stream (PoseStamped: t_optical_tag).
  3. place each sighting in the world:
       tag_world = T_world_base(corrected, t) · T_base_optical · t_optical_tag
     where T_base_optical = the recording's `optical_in_base` extrinsic.
  4. average each tag's world positions -> one centroid per tag (the estimate).
Then Umeyama-align go2's tag constellation onto the realsense GT constellation
over shared tag ids; residual RMSE = how well the PGO recovered the true tag
geometry (lower = better; an uncorrected / wrong PGO leaves higher residual).

Tag 17 is dynamic on huge_loop_realsense; it is dropped via --ignore-tags.

Results are written to eval_results/<test_recording>__ground_truth_tag/summary.json
(per-module residual + closures, plus the GT tag ids), and the frame-composition
core is covered by --self-test.
"""

from __future__ import annotations

import argparse
from collections import defaultdict
from collections.abc import Callable
import json
from pathlib import Path
from typing import Any

import numpy as np
from scipy.spatial.transform import Rotation

from dimos.navigation.jnav.components.loop_closure.eval import (
    run_module_graph,
    tf_pose_samples,
)
from dimos.navigation.jnav.utils.module_loading import (
    filter_config_for_module,
    load_module_class,
)
from dimos.navigation.jnav.utils.recording_db import ODOM_MATCH_TOLERANCE_S, store
from dimos.navigation.jnav.utils.trajectory_metrics import (
    drift_delta_lookup,
    pose7_lookup,
    rigid_align_rmse,
)


def pose7_to_matrix(pose7: np.ndarray | list[float]) -> np.ndarray:
    """[x,y,z, qx,qy,qz,qw] -> 4x4 homogeneous transform."""
    pose = np.asarray(pose7, dtype=np.float64)
    transform = np.eye(4)
    transform[:3, :3] = Rotation.from_quat(pose[3:7]).as_matrix()
    transform[:3, 3] = pose[:3]
    return transform


def tag_world_position(
    robot_pose7: np.ndarray | list[float],
    optical_in_base7: np.ndarray | list[float],
    tag_in_optical7: np.ndarray | list[float],
) -> np.ndarray:
    """World xyz of a tag from the corrected robot pose, the camera->base
    extrinsic, and the tag-in-optical detection. Composes
    T_world_base · T_base_optical · T_optical_tag and returns the translation."""
    transform = (
        pose7_to_matrix(robot_pose7)
        @ pose7_to_matrix(optical_in_base7)
        @ pose7_to_matrix(tag_in_optical7)
    )
    return np.asarray(transform[:3, 3])


def read_optical_in_base(intrinsics_json: Path) -> list[float]:
    raw = json.loads(Path(intrinsics_json).read_text())
    extrinsic = raw.get("optical_in_base")
    if extrinsic is None:
        raise SystemExit(f"no optical_in_base (camera->base extrinsic) in {intrinsics_json}")
    return list(extrinsic)


def tag_in_camera_sightings(db_path: Path) -> list[tuple[float, int, list[float]]]:
    """(ts, marker_id, tag-in-optical pose7) per April-tag observation, read
    straight from the recorded april_tags PoseStamped stream (no re-detection)."""
    sightings: list[tuple[float, int, list[float]]] = []
    tag_stream: Any = store(db_path).stream("april_tags")
    for observation in tag_stream:
        pose_tuple = observation.pose_tuple
        if pose_tuple is None:
            continue
        sightings.append(
            (float(observation.ts), int(observation.tags["marker_id"]), list(pose_tuple))
        )
    return sightings


def corrected_pose7_at(
    timestamp: float,
    raw_pose7_lookup: Callable[[float], np.ndarray | None],
    delta_lookup: Callable[[float], tuple[np.ndarray, np.ndarray] | None],
) -> np.ndarray | None:
    """Corrected robot pose7 at a time: apply the nearest keyframe's drift
    correction (R_delta, t_delta) to the raw odom pose."""
    raw = raw_pose7_lookup(timestamp)
    delta = delta_lookup(timestamp)
    if raw is None or delta is None:
        return None
    rotation_delta, translation_delta = delta
    raw = np.asarray(raw, dtype=np.float64)
    rotation_corrected = rotation_delta @ Rotation.from_quat(raw[3:7]).as_matrix()
    translation_corrected = rotation_delta @ raw[:3] + translation_delta
    return np.array([*translation_corrected, *Rotation.from_matrix(rotation_corrected).as_quat()])


def tag_constellation(
    db_path: Path,
    module_path: Path,
    module_name: str,
    config: dict[str, Any],
    *,
    lidar_stream: str,
    odom_stream: str,
    optical_in_base: list[float],
    ignore_tags: set[int],
    world_frame: str = "world",
    body_frame: str = "base_link",
) -> tuple[dict[int, np.ndarray], int]:
    """Run the PGO, then place each tag sighting in the world via the corrected
    trajectory + extrinsic, averaging per tag. Returns {marker_id: world centroid}
    and the closure count."""
    module_class = load_module_class(module_path, module_name)
    config = filter_config_for_module(module_class, config)
    graph, closures, _ = run_module_graph(
        db_path,
        module_class,
        config,
        lidar_stream=lidar_stream,
        odom_stream=odom_stream,
        lockstep=True,
    )
    raw_times, raw_poses7 = tf_pose_samples(
        db_path, odom_stream, world_frame=world_frame, body_frame=body_frame
    )
    raw_pose7_lookup = pose7_lookup(raw_times, raw_poses7, ODOM_MATCH_TOLERANCE_S)
    delta_lookup = drift_delta_lookup(graph, raw_pose7_lookup)
    by_tag: dict[int, list[np.ndarray]] = defaultdict(list)
    for timestamp, marker_id, tag_pose7 in tag_in_camera_sightings(db_path):
        if marker_id in ignore_tags:
            continue
        robot_pose7 = corrected_pose7_at(timestamp, raw_pose7_lookup, delta_lookup)
        if robot_pose7 is None:
            continue
        by_tag[marker_id].append(tag_world_position(robot_pose7, optical_in_base, tag_pose7))
    constellation = {tag: np.mean(positions, axis=0) for tag, positions in by_tag.items()}
    return constellation, closures


def constellation_residual(
    gt: dict[int, np.ndarray], test: dict[int, np.ndarray]
) -> tuple[float, list[int]]:
    """Umeyama-align the test tag constellation onto the GT; residual RMSE over
    shared tags (lower = the PGO recovered the true tag geometry better)."""
    shared = sorted(set(gt) & set(test))
    if len(shared) < 3:
        return float("nan"), shared
    gt_points = np.asarray([gt[tag] for tag in shared])
    test_points = np.asarray([test[tag] for tag in shared])
    return rigid_align_rmse(test_points, gt_points), shared


def _self_test() -> None:
    identity = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]
    # 1) everything identity, tag 2m ahead in optical -> world = 2m ahead.
    world = tag_world_position(identity, identity, [0.0, 0.0, 2.0, 0.0, 0.0, 0.0, 1.0])
    assert np.allclose(world, [0.0, 0.0, 2.0]), world

    # 2) robot translated by (10,5,0), no rotation: tag world shifts by that.
    world = tag_world_position(
        [10.0, 5.0, 0.0, 0.0, 0.0, 0.0, 1.0], identity, [1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]
    )
    assert np.allclose(world, [11.0, 5.0, 0.0]), world

    # 3) robot yawed 90deg (+z): a tag 1m ahead in base (x) lands 1m to +y of robot.
    yaw90 = [0.0, 0.0, 0.0, *Rotation.from_euler("z", 90, degrees=True).as_quat()]
    world = tag_world_position(yaw90, identity, [1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0])
    assert np.allclose(world, [0.0, 1.0, 0.0], atol=1e-9), world

    # 4) optical->base rotation (-0.5,0.5,-0.5,0.5) maps optical +z (forward) to
    #    base +x (forward). Tag 3m ahead in optical -> 3m ahead in base/world.
    optical_in_base = [0.0, 0.0, 0.0, -0.5, 0.5, -0.5, 0.5]
    world = tag_world_position(identity, optical_in_base, [0.0, 0.0, 3.0, 0.0, 0.0, 0.0, 1.0])
    assert np.allclose(world, [3.0, 0.0, 0.0], atol=1e-9), world

    print("frame-composition self-test PASSED (4 cases)")


LOOP_CLOSURE_DIR = Path(__file__).resolve().parent
DEFAULT_GT_CONFIG = {"use_scan_context": True, "global_map_publish_rate": 0}


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--gt-db", type=Path, required=True, help="recording for ground truth (realsense)"
    )
    parser.add_argument("--gt-intrinsics", type=Path, required=True)
    parser.add_argument("--gt-odom-stream", default="fastlio_odometry")
    parser.add_argument("--gt-lidar-stream", default="fastlio_lidar")
    parser.add_argument("--test-db", type=Path, required=True, help="recording to score (go2)")
    parser.add_argument("--test-intrinsics", type=Path, required=True)
    parser.add_argument("--test-odom-stream", default="fastlio_odometry")
    parser.add_argument("--test-lidar-stream", default="fastlio_lidar")
    parser.add_argument("--gt-module", default="gsc_pgo", help="PGO dir used to build GT")
    parser.add_argument("--modules", default="gsc_pgo,ivan_pgo,ivan_pgo_transformer,unrefined_pgo")
    parser.add_argument("--ignore-tags", default="17", help="dynamic tags to drop")
    args = parser.parse_args()

    ignore_tags = {int(t) for t in args.ignore_tags.split(",")} if args.ignore_tags else set()
    gt_extrinsic = read_optical_in_base(args.gt_intrinsics)
    test_extrinsic = read_optical_in_base(args.test_intrinsics)

    def module_py(module_dir: str) -> Path:
        return LOOP_CLOSURE_DIR / module_dir / "module.py"

    print(f"building GT constellation: {args.gt_module} on {args.gt_db.parent.name}")
    gt_constellation, _ = tag_constellation(
        args.gt_db,
        module_py(args.gt_module),
        "PGO",
        dict(DEFAULT_GT_CONFIG),
        lidar_stream=args.gt_lidar_stream,
        odom_stream=args.gt_odom_stream,
        optical_in_base=gt_extrinsic,
        ignore_tags=ignore_tags,
    )
    print(f"  GT tags: {sorted(gt_constellation)}")

    print(f"\n{'module':24s} {'tag-to-GT residual (m)':>22s} {'shared tags':>12s} {'closures':>9s}")
    print("-" * 70)
    rows: list[dict[str, Any]] = []
    for module_dir in [m.strip() for m in args.modules.split(",")]:
        try:
            test_constellation, closures = tag_constellation(
                args.test_db,
                module_py(module_dir),
                "PGO",
                dict(DEFAULT_GT_CONFIG),
                lidar_stream=args.test_lidar_stream,
                odom_stream=args.test_odom_stream,
                optical_in_base=test_extrinsic,
                ignore_tags=ignore_tags,
            )
            residual, shared = constellation_residual(gt_constellation, test_constellation)
            rows.append(
                {
                    "module": module_dir,
                    "residual_m": residual,
                    "shared_tags": len(shared),
                    "closures": closures,
                }
            )
            print(f"{module_dir:24s} {residual:>22.3f} {len(shared):>12d} {closures:>9d}")
        except Exception as error:
            print(f"{module_dir:24s} FAILED: {type(error).__name__}: {error}")
            rows.append({"module": module_dir, "residual_m": None, "error": str(error)})

    scored = [row for row in rows if row.get("residual_m") is not None]
    best_module = min(scored, key=lambda row: row["residual_m"])["module"] if scored else None
    if best_module is not None:
        best = next(row for row in scored if row["module"] == best_module)
        print(
            f"\nbest (lowest residual to GT tag geometry): {best_module} ({best['residual_m']:.3f} m)"
        )

    out_dir = LOOP_CLOSURE_DIR / "eval_results" / f"{args.test_db.parent.name}__ground_truth_tag"
    out_dir.mkdir(parents=True, exist_ok=True)
    summary = {
        "gt_db": str(args.gt_db),
        "gt_module": args.gt_module,
        "test_db": str(args.test_db),
        "ignore_tags": sorted(ignore_tags),
        "gt_tags": sorted(int(tag) for tag in gt_constellation),
        "best_module": best_module,
        "rows": rows,
    }
    (out_dir / "summary.json").write_text(json.dumps(summary, indent=2) + "\n")
    print(f"\nresults -> {out_dir / 'summary.json'}")


if __name__ == "__main__":
    import sys

    if "--self-test" in sys.argv:
        _self_test()
    else:
        main()
