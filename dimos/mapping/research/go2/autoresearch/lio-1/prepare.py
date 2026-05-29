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

"""Fixed constants, data paths, and the evaluation harness for the LIO
autoresearch experiment. This is the analog of nanochat-autoresearch's
prepare.py: it is READ-ONLY. It defines where the input data and ground truth
live, how to build the Point-LIO substrate, and — most importantly — the
ground-truth metric `evaluate()`. Do not modify this file; tuning happens in
train.py.

The experiment: run Point-LIO (offline, on a recorded Go2 L1 lidar+IMU stream)
to produce an odometry trajectory, and score how well it agrees with the
robot's onboard leg-inertial odometry (`robot_odom`), which loop-closes to
~0.47 m over a 16.5 m path and is our rough ground truth.

METRIC (current): 2D (xy) absolute trajectory error vs ground truth, after a
rigid (rotation+translation, no scale) Umeyama alignment of the LIO frame onto
the odom frame. Lower is better. z is intentionally ignored for now: on this
flat single-story recording robot_odom's z is near-constant and uninformative.
When the stairs ("ankle killer") recording lands we will add 3D criteria
(measured height/range change at specific timesteps) — see evaluate_3d() below.
"""

import os

import numpy as np

from dimos.utils.data import get_data

HERE = os.path.dirname(os.path.abspath(__file__))

# --- fixed data inputs, pulled from the dimos LFS data store. get_data resolves
# <repo>/data/go2dds_data1/, downloading + decompressing the LFS archive on first
# use. The bin + GT are pre-made; see human-debug/ for how. ---
BIN_PATH = str(get_data("go2dds_data1/go2-185959.bin"))  # PLNR1 lidar+imu input
GT_PATH = str(get_data("go2dds_data1/gt_robot_odom.tsv"))  # robot_odom ground truth

# --- Point-LIO substrate (fixed; built once by setup.sh) ---
POINTLIO_DIR = os.path.join(HERE, "point_lio")
POINTLIO_BIN = os.path.join(POINTLIO_DIR, "build", "pointlio_mapping")
TRAJ_PATH = os.path.join(POINTLIO_DIR, "Log", "mat_out.txt")  # written by each run
ACTIVE_YAML = os.path.join(POINTLIO_DIR, "config", "_active.yaml")  # train.py writes this

# --- run limits ---
RUN_TIMEOUT = 600  # seconds; a healthy run is ~1-3 min, kill at 10 min

# mat_out.txt columns: t_rel  euler(r p y)  pos(x y z)  vel(x y z)  ...
TRAJ_T, TRAJ_XYZ = 0, slice(4, 7)


def check_data():
    """Setup-step sanity check: input bin, GT, and built binary all present."""
    ok = True
    for label, p in [
        ("input bin", BIN_PATH),
        ("ground truth", GT_PATH),
        ("pointlio binary", POINTLIO_BIN),
    ]:
        present = os.path.exists(p)
        print(f"  [{'ok' if present else 'MISSING'}] {label}: {p}")
        ok = ok and present
    if not os.path.exists(POINTLIO_BIN):
        print("  -> pointlio binary missing; run ./setup.sh to build it.")
    return ok


def load_gt():
    """Ground truth robot_odom. Returns (t_rel[N], xyz[N,3])."""
    a = np.loadtxt(GT_PATH)
    return a[:, 0], a[:, 1:4]


def load_traj(path=TRAJ_PATH):
    """LIO trajectory from a Point-LIO mat_out.txt. Returns (t_rel[N], xyz[N,3])."""
    a = np.loadtxt(path)
    if a.ndim != 2 or a.shape[0] < 5:
        raise ValueError(f"trajectory too short / malformed: {path} shape={a.shape}")
    return a[:, TRAJ_T], a[:, TRAJ_XYZ]


def _umeyama_2d(src, dst):
    """Rigid (R,t), no scale, minimizing |R src + t - dst| for Nx2 point sets."""
    ms, md = src.mean(0), dst.mean(0)
    H = (src - ms).T @ (dst - md)
    U, _, Vt = np.linalg.svd(H)
    d = np.sign(np.linalg.det(Vt.T @ U.T))
    R = Vt.T @ np.diag([1.0, d]) @ U.T
    return R, md - R @ ms


def _path_len(P):
    return float(np.sum(np.linalg.norm(np.diff(P, axis=0), axis=1)))


def evaluate(traj_path=TRAJ_PATH):
    """THE METRIC. Score a LIO trajectory against robot_odom ground truth in 2D.

    Returns a dict; the primary number to minimize is `val_ate_xy` (meters):
    RMSE of the rigid-aligned LIO xy trajectory vs GT, sampled at the LIO times
    over the overlapping interval.
    """
    gt_t, gt_p = load_gt()
    t, P = load_traj(traj_path)
    gt_xy = gt_p[:, :2]
    xy = P[:, :2]

    # GT interpolated onto LIO timestamps over the overlap
    lo, hi = max(t[0], gt_t[0]), min(t[-1], gt_t[-1])
    m = (t >= lo) & (t <= hi)
    tv, xyv = t[m], xy[m]
    gti = np.column_stack([np.interp(tv, gt_t, gt_xy[:, k]) for k in range(2)])

    R, tr = _umeyama_2d(xyv, gti)
    aligned = (R @ xyv.T).T + tr
    ate = float(np.sqrt(np.mean(np.sum((aligned - gti) ** 2, axis=1))))
    final_err = float(np.linalg.norm(aligned[-1] - gti[-1]))

    return {
        "val_ate_xy": ate,  # PRIMARY (minimize), meters
        "final_err_xy": final_err,  # aligned end-pose error, m
        "loop_close_xy": float(np.linalg.norm(xy[-1] - xy[0])),  # LIO |end-start|, m
        "gt_loop_xy": float(np.linalg.norm(gt_xy[-1] - gt_xy[0])),  # GT |end-start| (~0.47)
        "path_len": _path_len(xy),  # LIO xy path length, m
        "gt_path_len": _path_len(gt_xy),  # GT xy path length (~16.5)
        "num_poses": int(P.shape[0]),
        "overlap_s": float(hi - lo),
    }


def evaluate_3d(traj_path=TRAJ_PATH):
    """PLACEHOLDER for future 3D criteria. Not part of the metric yet.

    Once the stairs ("ankle killer") recording is captured with tape-measured
    step heights, this will check actual range / z-height change at specific
    timesteps (e.g. total descent over the ~5 steps) against the LIO z. Today's
    flat recording has near-constant robot_odom z, so there is nothing to score
    in 3D — the metric is 2D only (see evaluate()).
    """
    return {}


if __name__ == "__main__":
    print("LIO autoresearch — data + harness check")
    print(f"  HERE: {HERE}")
    ok = check_data()
    if ok:
        gt_t, gt_p = load_gt()
        print(
            f"  GT: {len(gt_t)} poses, t {gt_t[0]:.1f}..{gt_t[-1]:.1f}s, "
            f"xy path {_path_len(gt_p[:, :2]):.2f}m, "
            f"xy loop {np.linalg.norm(gt_p[-1, :2] - gt_p[0, :2]):.3f}m"
        )
        print("Done! Ready to run train.py.")
    else:
        print("Setup incomplete — run ./setup.sh.")
