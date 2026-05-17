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

"""READ-ONLY evaluation harness for the autoresearch relocalization experiment.

DO NOT MODIFY THIS FILE. It contains the fixed evaluation, data loading,
and time-budget enforcement. The agent edits `relocalize.py` only.

This is the **entry point** for the experiment. Run it as:

    uv run dimos/mapping/relocalization/run.py

It imports `relocalize` from `relocalize.py` (sibling file), runs it
against the 20 cached test frames under a 5-minute wall-clock budget,
and prints a grep-friendly summary block.

The agent's `relocalize` must have the signature:

    def relocalize(
        global_map: open3d.geometry.PointCloud,
        local_map: open3d.geometry.PointCloud,
    ) -> numpy.ndarray              # 4x4 homogeneous transform

The transform should map points in `local_map`'s (body) frame into
`global_map`'s (world) frame, such that `T @ [body; 1] ≈ [world; 1]`.
"""

from __future__ import annotations

import pickle
import random
import time
from pathlib import Path
from typing import Callable

import numpy as np
import open3d as o3d
from scipy.spatial.transform import Rotation

DATA_DIR = Path(__file__).parent / "data"
TIME_BUDGET_SEC = 300.0  # 5 minutes wall-clock for the entire run
SUCCESS_T_M = 1.0        # success threshold: translation error < 1m
SUCCESS_R_DEG = 15.0     # success threshold: rotation error < 15°

RelocalizeFn = Callable[
    [o3d.geometry.PointCloud, o3d.geometry.PointCloud], np.ndarray
]


def _to_o3d_pcd(pts: np.ndarray) -> o3d.geometry.PointCloud:
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(pts.astype(np.float64))
    return pcd


def _load_data() -> tuple[o3d.geometry.PointCloud, list[dict]]:
    if not (DATA_DIR / "global_map.npy").exists():
        raise FileNotFoundError(
            f"Missing {DATA_DIR / 'global_map.npy'}. "
            "Run `uv run python -m dimos.mapping.prepare` to generate the data files."
        )
    if not (DATA_DIR / "test_frames.pkl").exists():
        raise FileNotFoundError(
            f"Missing {DATA_DIR / 'test_frames.pkl'}. "
            "Run `uv run python -m dimos.mapping.prepare` to generate the data files."
        )
    global_map_pts = np.load(DATA_DIR / "global_map.npy")
    global_map = _to_o3d_pcd(global_map_pts)
    test_frames = pickle.loads((DATA_DIR / "test_frames.pkl").read_bytes())
    return global_map, test_frames


def evaluate(relocalize_fn: RelocalizeFn) -> dict:
    """Run ``relocalize_fn`` on the test set under a fixed 5-minute budget.

    Prints a summary block and returns the same metrics as a dict.
    The text label rows printed are stable and grep-friendly:
        ``^average_distance:``  ``^median_distance:``  ``^success_rate:``
        ``^total_seconds:``  ``^num_frames_done:``  ``^num_frames_total:``
        ``^all_distances:``  ``^all_rotations:``
    """
    global_map, test_frames = _load_data()
    # Deterministic eval order: same prefix evaluated even when the time
    # budget cuts a run short.
    test_frames = sorted(test_frames, key=lambda f: f["frame_idx"])
    print(
        f"[run] global_map={len(global_map.points)} pts, "
        f"test_frames={len(test_frames)}, time_budget={TIME_BUDGET_SEC:.0f}s"
    )

    t_start = time.perf_counter()
    distances: list[float] = []
    rotations: list[float] = []
    per_call_times: list[float] = []
    crashed: list[int] = []

    for i, frame in enumerate(test_frames):
        elapsed = time.perf_counter() - t_start
        if elapsed > TIME_BUDGET_SEC:
            print(
                f"[run] time budget exceeded after {i}/{len(test_frames)} frames "
                f"({elapsed:.1f}s)"
            )
            break

        local_map = _to_o3d_pcd(frame["body_pts"])
        gt_R = np.asarray(frame["gt_R"], dtype=np.float64)
        gt_t = np.asarray(frame["gt_t"], dtype=np.float64)

        # Reseed every stochastic source the agent could possibly touch,
        # before every frame, so two runs of the same relocalize.py
        # produce identical metrics. The agent's signal is then pure
        # algorithm delta, not RNG luck.
        seed = int(frame["frame_idx"])
        o3d.utility.random.seed(seed)
        np.random.seed(seed)
        random.seed(seed)

        t_call = time.perf_counter()
        try:
            T = relocalize_fn(global_map, local_map)
        except Exception as e:  # noqa: BLE001
            print(f"[run] frame {frame['frame_idx']}: relocalize() raised {e!r}, skipping")
            crashed.append(frame["frame_idx"])
            continue
        dt = time.perf_counter() - t_call
        per_call_times.append(dt)

        T = np.asarray(T, dtype=np.float64)
        if T.shape != (4, 4):
            print(
                f"[run] frame {frame['frame_idx']}: relocalize() returned shape "
                f"{T.shape}, expected (4, 4) — skipping"
            )
            crashed.append(frame["frame_idx"])
            continue

        reg_R = T[:3, :3]
        reg_t = T[:3, 3]
        err_t = float(np.linalg.norm(reg_t - gt_t))
        err_r = float(Rotation.from_matrix(reg_R @ gt_R.T).magnitude() * 180.0 / np.pi)
        distances.append(err_t)
        rotations.append(err_r)
        print(
            f"[run] frame {frame['frame_idx']:>5} ({i+1}/{len(test_frames)}): "
            f"err_t={err_t:7.3f}m  err_r={err_r:6.1f}°  ({dt:.2f}s)"
        )

    t_end = time.perf_counter()
    distances_arr = np.array(distances) if distances else np.array([])
    rotations_arr = np.array(rotations) if rotations else np.array([])
    ok = (
        (distances_arr < SUCCESS_T_M) & (rotations_arr < SUCCESS_R_DEG)
        if distances else np.array([], dtype=bool)
    )

    avg_d = float(distances_arr.mean()) if distances_arr.size else float("nan")
    med_d = float(np.median(distances_arr)) if distances_arr.size else float("nan")
    avg_r = float(rotations_arr.mean()) if rotations_arr.size else float("nan")
    succ = float(ok.mean()) if ok.size else 0.0
    avg_call = float(np.mean(per_call_times)) if per_call_times else 0.0

    print("---")
    print(f"average_distance:    {avg_d:.6f}")
    print(f"median_distance:     {med_d:.6f}")
    print(f"average_rotation:    {avg_r:.2f}")
    print(f"success_rate:        {succ:.4f}")
    print(f"total_seconds:       {t_end - t_start:.1f}")
    print(f"avg_call_seconds:    {avg_call:.2f}")
    print(f"num_frames_done:     {len(distances)}")
    print(f"num_frames_total:    {len(test_frames)}")
    print(f"num_crashed:         {len(crashed)}")
    print(f"all_distances:       {[round(d, 3) for d in distances]}")
    print(f"all_rotations:       {[round(r, 1) for r in rotations]}")

    return {
        "average_distance": avg_d,
        "median_distance": med_d,
        "average_rotation": avg_r,
        "success_rate": succ,
        "total_seconds": t_end - t_start,
        "num_frames_done": len(distances),
        "num_frames_total": len(test_frames),
        "num_crashed": len(crashed),
    }


if __name__ == "__main__":
    # Force line-buffered stdout so progress shows up live in redirected logs.
    import sys
    sys.stdout.reconfigure(line_buffering=True)

    from relocalize import relocalize
    evaluate(relocalize)
