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

import os

# Force single-threaded OpenMP BEFORE importing open3d, so RANSAC's
# parallel sampling becomes deterministic. `o3d.utility.random.seed()`
# alone is not enough — thread scheduling order is itself a source of
# non-determinism even at a fixed RNG seed. Must be set before import.
os.environ.setdefault("OMP_NUM_THREADS", "1")

from collections.abc import Callable
from concurrent.futures import ProcessPoolExecutor, as_completed
import multiprocessing as mp
from pathlib import Path
import pickle
import random
import time

import numpy as np
import open3d as o3d
from scipy.spatial.transform import Rotation

DATA_DIR = Path(__file__).parent / "data"
TIME_BUDGET_SEC = 300.0  # 5 minutes wall-clock for the entire run
SUCCESS_T_M = 1.0  # success threshold: translation error < 1m
SUCCESS_R_DEG = 15.0  # success threshold: rotation error < 15°
NUM_WORKERS = os.cpu_count() or 1  # eval frames in parallel — uses all cores

RelocalizeFn = Callable[[o3d.geometry.PointCloud, o3d.geometry.PointCloud], np.ndarray]

# Inherited by fork-child workers (set in evaluate() before pool launch).
_GLOBAL_MAP_PTS: np.ndarray | None = None
_RELOCALIZE_FN: RelocalizeFn | None = None


def _to_o3d_pcd(pts: np.ndarray) -> o3d.geometry.PointCloud:
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(pts.astype(np.float64))
    return pcd


def _eval_one_frame(frame: dict) -> dict:
    """Worker: evaluate a single frame. Runs in a fork-child process.

    Inherits ``_GLOBAL_MAP_PTS`` and ``_RELOCALIZE_FN`` from the parent
    via fork. Returns a flat dict (picklable) — no Open3D objects cross
    the process boundary.
    """
    seed = int(frame["frame_idx"])
    o3d.utility.random.seed(seed)
    np.random.seed(seed)
    random.seed(seed)

    global_map = _to_o3d_pcd(_GLOBAL_MAP_PTS)
    local_map = _to_o3d_pcd(frame["body_pts"])

    t_call = time.perf_counter()
    try:
        T = _RELOCALIZE_FN(global_map, local_map)
    except Exception as e:
        return {
            "frame_idx": int(frame["frame_idx"]),
            "status": "crashed",
            "error": repr(e),
            "dt": 0.0,
        }
    dt = time.perf_counter() - t_call

    T = np.asarray(T, dtype=np.float64)
    if T.shape != (4, 4):
        return {
            "frame_idx": int(frame["frame_idx"]),
            "status": "bad_shape",
            "shape": tuple(T.shape),
            "dt": dt,
        }

    gt_R = np.asarray(frame["gt_R"], dtype=np.float64)
    gt_t = np.asarray(frame["gt_t"], dtype=np.float64)
    err_t = float(np.linalg.norm(T[:3, 3] - gt_t))
    err_r = float(Rotation.from_matrix(T[:3, :3] @ gt_R.T).magnitude() * 180.0 / np.pi)
    return {
        "frame_idx": int(frame["frame_idx"]),
        "status": "ok",
        "err_t": err_t,
        "err_r": err_r,
        "dt": dt,
    }


def evaluate(relocalize_fn: RelocalizeFn) -> dict:
    """Run ``relocalize_fn`` on the test set under a fixed 5-minute budget.

    Evaluates frames in parallel across ``NUM_WORKERS`` fork-child
    processes (each child runs single-threaded RANSAC — see
    ``OMP_NUM_THREADS`` above — and reseeds RNGs from ``frame_idx`` so
    per-frame results are reproducible). Prints a summary block and
    returns the same metrics as a dict. Grep-friendly label rows:
        ``^average_distance:``  ``^median_distance:``  ``^success_rate:``
        ``^total_seconds:``  ``^num_frames_done:``  ``^num_frames_total:``
        ``^all_distances:``  ``^all_rotations:``
    """
    global _GLOBAL_MAP_PTS, _RELOCALIZE_FN
    for fname in ("global_map.npy", "test_frames.pkl"):
        if not (DATA_DIR / fname).exists():
            raise FileNotFoundError(
                f"Missing {DATA_DIR / fname}. "
                "Run `uv run python -m dimos.mapping.prepare` to generate the data files."
            )
    global_map_pts = np.load(DATA_DIR / "global_map.npy")
    test_frames = pickle.loads((DATA_DIR / "test_frames.pkl").read_bytes())
    # Deterministic eval order: same frame_idx → same seed → same result,
    # regardless of which worker happens to pick it up.
    test_frames = sorted(test_frames, key=lambda f: f["frame_idx"])

    _GLOBAL_MAP_PTS = global_map_pts
    _RELOCALIZE_FN = relocalize_fn

    print(
        f"[run] global_map={len(global_map_pts)} pts, "
        f"test_frames={len(test_frames)}, workers={NUM_WORKERS}, "
        f"time_budget={TIME_BUDGET_SEC:.0f}s"
    )

    t_start = time.perf_counter()
    results: dict[int, dict] = {}

    # fork keeps `_GLOBAL_MAP_PTS` and `_RELOCALIZE_FN` available in the
    # children without IPC. Required: parent must not have touched any
    # Open3D thread pool that doesn't survive fork (we set OMP=1, so OK).
    ctx = mp.get_context("fork")
    pool = ProcessPoolExecutor(max_workers=NUM_WORKERS, mp_context=ctx)
    try:
        futures = {pool.submit(_eval_one_frame, f): int(f["frame_idx"]) for f in test_frames}
        try:
            for future in as_completed(futures, timeout=TIME_BUDGET_SEC):
                r = future.result()
                results[r["frame_idx"]] = r
                if r["status"] == "ok":
                    print(
                        f"[run] frame {r['frame_idx']:>5}: "
                        f"err_t={r['err_t']:7.3f}m  err_r={r['err_r']:6.1f}°  "
                        f"({r['dt']:.2f}s)"
                    )
                elif r["status"] == "crashed":
                    print(
                        f"[run] frame {r['frame_idx']}: relocalize() raised {r['error']}, skipping"
                    )
                else:  # bad_shape
                    print(
                        f"[run] frame {r['frame_idx']}: relocalize() returned shape "
                        f"{r['shape']}, expected (4, 4) — skipping"
                    )
        except TimeoutError:
            done = len(results)
            print(
                f"[run] time budget exceeded after {done}/{len(test_frames)} frames "
                f"({time.perf_counter() - t_start:.1f}s)"
            )
    finally:
        pool.shutdown(wait=False, cancel_futures=True)

    t_end = time.perf_counter()

    # Reassemble in frame_idx order so all_distances / all_rotations are
    # stable across runs even though completion order isn't.
    distances: list[float] = []
    rotations: list[float] = []
    per_call_times: list[float] = []
    crashed: list[int] = []
    for f in test_frames:
        r = results.get(int(f["frame_idx"]))
        if r is None:
            continue  # never finished (budget killed)
        if r["status"] == "ok":
            distances.append(r["err_t"])
            rotations.append(r["err_r"])
            per_call_times.append(r["dt"])
        else:
            crashed.append(r["frame_idx"])

    distances_arr = np.array(distances) if distances else np.array([])
    rotations_arr = np.array(rotations) if rotations else np.array([])
    ok = (
        (distances_arr < SUCCESS_T_M) & (rotations_arr < SUCCESS_R_DEG)
        if distances
        else np.array([], dtype=bool)
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
