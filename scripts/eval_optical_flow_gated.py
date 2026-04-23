"""
Odom-gated Optical Flow vs LiDAR Correlation
Issue #1775 — Real-time obstacle avoidance from monocular optical flow

Question
--------
On frames where the robot is purely translating forward (no turning),
how well does optical flow τ-estimation correlate with LiDAR obstacle
proximity?

Odom gate
---------
  v_fwd  > 0.15 m/s   (robot moving forward)
  |ω_yaw| < 0.15 rad/s (robot not turning)

This selects wall-approach segments where τ = 1/divergence is
theory-valid: pure forward translation generates a radially expanding
flow field from the focus of expansion, and divergence is proportional
to 1/τ.

Metrics
-------
1. Correlation   — Pearson r and Spearman r between of_tau and
                   lidar_ttc = min_fwd_dist / v_fwd
2. Binary P/R    — τ < threshold vs lidar_dist < 1.5 m
3. PR curve      — sweep τ ∈ [0.3, 6.0] s
4. Latency       — mean per-frame compute time

Backend
-------
  FAST corner detector + Lucas-Kanade sparse flow.
  ORB-SLAM uses the same FAST corners for map-point extraction; the
  additional orientation step ORB computes (for BRIEF descriptor
  matching) is irrelevant here — LK tracks image patches, not
  descriptors.

Run from repo root
------------------
  PYTHONPATH=.venv/lib/python3.12/site-packages/rerun_sdk \\
      uv run python3 scripts/eval_optical_flow_gated.py

Output
------
  results/optical_flow_gated.json
"""

import sys
import os
import json
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, ".")
from dimos.memory.timeseries.legacy import LegacyPickleStore
from dimos.perception.optical_flow.backends.lucas_kanade import LucasKanadeBackend

# ── Paths ─────────────────────────────────────────────────────────────────────
REPO_ROOT = Path(__file__).resolve().parent.parent
DATA      = REPO_ROOT / "data" / "unitree_office_walk"

# ── Odom gate thresholds ──────────────────────────────────────────────────────
V_FWD_MIN  = 0.15    # m/s  — must be moving forward
OMEGA_MAX  = 0.15    # rad/s — must not be turning
MIN_DT     = 0.02    # s    — skip duplicate/near-duplicate odom timestamps

# ── Sensor config ─────────────────────────────────────────────────────────────
FOV_DEG    = 60.0    # forward FOV for lidar obstacle extraction
MAX_RANGE  = 5.0     # m — max lidar range to consider
DANGER_M   = 1.5     # m — lidar distance threshold → "danger" GT label

# ── Optical flow config ────────────────────────────────────────────────────────
GRID_SIZE  = 5
TAU_SWEEP  = np.round(np.linspace(0.3, 6.0, 57), 2)  # PR curve sweep


# ── Odom kinematics ───────────────────────────────────────────────────────────

def odom_kinematics(odom_items):
    """
    Per-frame forward velocity (m/s) and yaw rate (rad/s).
    Frames with dt < MIN_DT carry forward the last valid value.
    Returns (v_fwd, omega, gate) arrays, length = len(odom_items).
    """
    n   = len(odom_items)
    ts  = np.array([t for t, _ in odom_items])
    xs  = np.array([d["data"]["pose"]["position"]["x"] for _, d in odom_items])
    ys  = np.array([d["data"]["pose"]["position"]["y"] for _, d in odom_items])

    raw_yaws = []
    for _, d in odom_items:
        ori = d["data"]["pose"]["orientation"]
        qx, qy, qz, qw = ori["x"], ori["y"], ori["z"], ori["w"]
        raw_yaws.append(np.arctan2(2*(qw*qz + qx*qy), 1 - 2*(qy**2 + qz**2)))
    yaws = np.unwrap(raw_yaws)

    v_fwd = np.zeros(n)
    omega = np.zeros(n)

    last_v, last_w = 0.0, 0.0
    for i in range(1, n):
        dt = ts[i] - ts[i - 1]
        if dt < MIN_DT:
            v_fwd[i] = last_v
            omega[i]  = last_w
            continue
        dx, dy  = xs[i] - xs[i - 1], ys[i] - ys[i - 1]
        heading = np.array([np.cos(yaws[i - 1]), np.sin(yaws[i - 1])])
        last_v  = np.dot([dx, dy], heading) / dt
        last_w  = abs(yaws[i] - yaws[i - 1]) / dt
        v_fwd[i] = last_v
        omega[i]  = last_w

    v_fwd[0] = v_fwd[1]
    omega[0]  = omega[1]

    gate = (v_fwd > V_FWD_MIN) & (omega < OMEGA_MAX)
    return v_fwd, omega, gate


# ── Frame sync ─────────────────────────────────────────────────────────────────

def _nearest(arr, val):
    return int(np.argmin(np.abs(arr - val)))


def sync_frames(lidar_s, video_s, odom_s, tol=0.1):
    """
    Anchor on lidar timestamps; find nearest video and odom within tol.
    Yields (ts, {lidar, video, odom, v_fwd, omega, is_gated}).
    """
    video_items = list(video_s._iter_items())
    odom_items  = list(odom_s._iter_items())
    v_fwd, omega, gate = odom_kinematics(odom_items)

    v_ts   = np.array([t for t, _ in video_items])
    o_ts   = np.array([t for t, _ in odom_items])
    v_data = [d for _, d in video_items]
    o_data = [d for _, d in odom_items]

    for ts_l, data_l in lidar_s._iter_items():
        vi = _nearest(v_ts, ts_l)
        oi = _nearest(o_ts, ts_l)
        if abs(v_ts[vi] - ts_l) > tol or abs(o_ts[oi] - ts_l) > tol:
            continue
        yield ts_l, {
            "lidar":    data_l,
            "video":    v_data[vi],
            "odom":     o_data[oi],
            "v_fwd":    float(v_fwd[oi]),
            "omega":    float(omega[oi]),
            "is_gated": bool(gate[oi]),
        }


# ── Coordinate transform ───────────────────────────────────────────────────────

def world_to_body(pts_world: np.ndarray, odom_msg: dict) -> np.ndarray:
    pose = odom_msg["data"]["pose"]
    pos, ori = pose["position"], pose["orientation"]
    rx, ry, rz = pos["x"], pos["y"], pos["z"]
    qx, qy, qz, qw = ori["x"], ori["y"], ori["z"], ori["w"]
    R = np.array([
        [1 - 2*(qy**2 + qz**2),  2*(qx*qy - qw*qz),   2*(qx*qz + qw*qy)],
        [2*(qx*qy + qw*qz),      1 - 2*(qx**2 + qz**2), 2*(qy*qz - qw*qx)],
        [2*(qx*qz - qw*qy),      2*(qy*qz + qw*qx),   1 - 2*(qx**2 + qy**2)],
    ])
    return (pts_world - np.array([rx, ry, rz])) @ R


def min_fwd_dist(lidar_msg, odom_msg):
    """Minimum distance to any forward obstacle in body frame. inf if none."""
    try:
        pts = world_to_body(np.asarray(lidar_msg["data"]["data"]["points"]), odom_msg)
    except Exception:
        return np.inf
    x, y, z = pts[:, 0], pts[:, 1], pts[:, 2]
    dist = np.sqrt(x**2 + y**2)
    mask = (x > 0.2) & (dist < MAX_RANGE) & \
           (np.abs(np.arctan2(y, x)) < np.radians(FOV_DEG / 2)) & \
           (z > -0.3) & (z < 1.5)
    return float(dist[mask].min()) if mask.any() else np.inf


# ── Correlation helpers ────────────────────────────────────────────────────────

def pearson_r(x, y):
    if len(x) < 3:
        return float("nan")
    xm, ym = x - x.mean(), y - y.mean()
    denom = np.sqrt((xm**2).sum() * (ym**2).sum())
    return float((xm * ym).sum() / denom) if denom > 0 else float("nan")


def spearman_r(x, y):
    if len(x) < 3:
        return float("nan")
    rx = np.argsort(np.argsort(x)).astype(float)
    ry = np.argsort(np.argsort(y)).astype(float)
    return pearson_r(rx, ry)


def auc_trapz(recalls, precisions):
    pairs  = sorted(zip(recalls, precisions))
    r_arr  = np.array([p[0] for p in pairs])
    p_arr  = np.array([p[1] for p in pairs])
    return float(np.trapezoid(p_arr, r_arr))


# ── Main evaluation loop ───────────────────────────────────────────────────────

def run_backend(synced, BackendClass, bname):
    """
    Process every frame through the OF backend (maintains tracking state),
    but compute metrics only on odom-gated frames.
    """
    backend = BackendClass()

    # Arrays built on gated frames only
    of_tau_list   = []   # optical flow min_tau (may be inf)
    lidar_d_list  = []   # LiDAR forward distance (m)
    lidar_ttc_list = []  # LiDAR TTC = dist / v_fwd (s)
    v_fwd_list    = []   # forward speed on that frame
    lat_list      = []   # latency (ms) for all processed frames

    for _, data in synced:
        frame = data["video"]
        if not isinstance(frame, np.ndarray):
            frame = np.array(frame)

        t0     = time.perf_counter()
        result = backend.compute(frame)
        lat_ms = (time.perf_counter() - t0) * 1000
        lat_list.append(lat_ms)

        if not data["is_gated"] or result is None:
            continue

        of_tau = result["min_tau"]
        lidar_d = min_fwd_dist(data["lidar"], data["odom"])
        v_fwd   = data["v_fwd"]

        lidar_ttc = lidar_d / v_fwd if (np.isfinite(lidar_d) and v_fwd > 0.05) else np.inf

        of_tau_list.append(of_tau)
        lidar_d_list.append(lidar_d)
        lidar_ttc_list.append(lidar_ttc)
        v_fwd_list.append(v_fwd)

    n_gated = len(of_tau_list)
    mean_lat = float(np.mean(lat_list))

    of_tau_arr    = np.array(of_tau_list)
    lidar_d_arr   = np.array(lidar_d_list)
    lidar_ttc_arr = np.array(lidar_ttc_list)
    v_fwd_arr     = np.array(v_fwd_list)

    print(f"\n  Gated frames: {n_gated}  |  mean latency: {mean_lat:.2f} ms/frame")
    if v_fwd_arr.size:
        print(f"  v_fwd on gated: "
              f"p25={np.percentile(v_fwd_arr,25):.2f}  "
              f"p50={np.percentile(v_fwd_arr,50):.2f}  "
              f"p75={np.percentile(v_fwd_arr,75):.2f} m/s")

    # ── Metric 1: τ correlation ───────────────────────────────────────────────
    # Keep only frames where BOTH of_tau and lidar_ttc are finite and sane
    corr_mask = (np.isfinite(of_tau_arr) & np.isfinite(lidar_ttc_arr) &
                 (lidar_ttc_arr < 15.0) & (of_tau_arr < 15.0))
    of_c  = of_tau_arr[corr_mask]
    ltc_c = lidar_ttc_arr[corr_mask]

    pr = pearson_r(of_c, ltc_c) if len(of_c) >= 3 else float("nan")
    sr = spearman_r(of_c, ltc_c) if len(of_c) >= 3 else float("nan")
    rmse = float(np.sqrt(np.mean((of_c - ltc_c)**2))) if len(of_c) >= 3 else float("nan")
    mae  = float(np.mean(np.abs(of_c - ltc_c))) if len(of_c) >= 3 else float("nan")

    print(f"\n  Correlation  (n={len(of_c)} paired frames)")
    print(f"    Pearson r  : {pr:+.3f}")
    print(f"    Spearman r : {sr:+.3f}")
    print(f"    RMSE       : {rmse:.3f} s")
    print(f"    MAE        : {mae:.3f} s")

    # ── Metric 2 & 3: PR curve over τ threshold ───────────────────────────────
    # GT: lidar_dist < DANGER_M
    gt_danger = lidar_d_arr < DANGER_M

    pr_curve = []
    f1_best = 0.0; tau_best = TAU_SWEEP[0]; prec_best = 0.0; rec_best = 0.0

    for tau in TAU_SWEEP:
        pred = of_tau_arr < tau
        TP = int(np.sum(pred & gt_danger))
        FP = int(np.sum(pred & ~gt_danger))
        FN = int(np.sum(~pred & gt_danger))
        p  = TP / (TP + FP + 1e-8)
        r  = TP / (TP + FN + 1e-8)
        f1 = 2*p*r / (p + r + 1e-8)
        pr_curve.append({"tau": float(tau), "precision": round(p,4),
                          "recall": round(r,4), "f1": round(f1,4)})
        if f1 > f1_best:
            f1_best = f1; tau_best = float(tau)
            prec_best = p; rec_best = r

    auc = auc_trapz([pt["recall"] for pt in pr_curve],
                    [pt["precision"] for pt in pr_curve])

    n_gt_pos = int(gt_danger.sum())
    print(f"\n  Binary detection  (GT: lidar_dist < {DANGER_M} m)")
    print(f"    GT positive frames: {n_gt_pos}/{n_gated} ({n_gt_pos/max(n_gated,1)*100:.0f}%)")
    print(f"    AUC-PR      : {auc:.4f}")
    print(f"    Optimal τ   : {tau_best:.2f} s  →  "
          f"P={prec_best:.3f}  R={rec_best:.3f}  F1={f1_best:.3f}")

    return {
        "backend":         bname,
        "dataset":         "unitree_office_walk",
        "odom_gate":       {"v_fwd_min": V_FWD_MIN, "omega_max": OMEGA_MAX},
        "n_gated_frames":  n_gated,
        "mean_latency_ms": round(mean_lat, 2),
        "correlation": {
            "n_paired":  int(len(of_c)),
            "pearson_r": round(pr,  4),
            "spearman_r":round(sr,  4),
            "rmse_s":    round(rmse, 4),
            "mae_s":     round(mae,  4),
        },
        "binary_detection": {
            "gt_danger_m":    DANGER_M,
            "n_gt_positive":  n_gt_pos,
            "auc_pr":         round(auc, 4),
            "optimal_tau_s":  tau_best,
            "optimal_f1":     round(f1_best, 4),
            "optimal_prec":   round(prec_best, 4),
            "optimal_rec":    round(rec_best, 4),
        },
        "pr_curve": pr_curve,
        "v_fwd_percentiles": {
            "p25": round(float(np.percentile(v_fwd_arr, 25)), 3) if v_fwd_arr.size else None,
            "p50": round(float(np.percentile(v_fwd_arr, 50)), 3) if v_fwd_arr.size else None,
            "p75": round(float(np.percentile(v_fwd_arr, 75)), 3) if v_fwd_arr.size else None,
        },
    }


# ── Entry point ────────────────────────────────────────────────────────────────

def main():
    print("=" * 68)
    print("Odom-gated Optical Flow vs LiDAR Correlation")
    print(f"Dataset : {DATA}")
    print(f"Gate    : v_fwd > {V_FWD_MIN} m/s  AND  |ω| < {OMEGA_MAX} rad/s")
    print(f"GT      : lidar_dist < {DANGER_M} m (forward FOV={FOV_DEG}°, max={MAX_RANGE}m)")
    print("=" * 68)

    lidar_store = LegacyPickleStore(DATA / "lidar")
    video_store = LegacyPickleStore(DATA / "video")
    odom_store  = LegacyPickleStore(DATA / "odom")

    print("\nSyncing frames (anchored on lidar)...", end=" ", flush=True)
    synced = list(sync_frames(lidar_store, video_store, odom_store))
    n_gated = sum(1 for _, d in synced if d["is_gated"])
    print(f"done.  Total: {len(synced)}  Gated: {n_gated} "
          f"({n_gated/max(len(synced),1)*100:.0f}%)")

    print(f"\n{'─'*68}")
    print("Backend: LucasKanade-FAST")
    print(f"{'─'*68}")
    result = run_backend(synced, LucasKanadeBackend, "lucas_kanade_fast")

    # ── Interpretation ─────────────────────────────────────────────────────────
    lk_c = result["correlation"];  lk_b = result["binary_detection"]
    sr = lk_c["spearman_r"];       pr_r = lk_c["pearson_r"]
    auc = lk_b["auc_pr"];          tau_opt = lk_b["optimal_tau_s"]
    f1  = lk_b["optimal_f1"];      prec = lk_b["optimal_prec"];  rec_best = lk_b["optimal_rec"]

    tau3 = next((pt for pt in result["pr_curve"] if abs(pt["tau"] - 3.0) < 0.06), {})

    corr_interp = (
        "τ tracks real TTC: use raw value as a continuous urgency score."
        if sr > 0.4 else
        "τ weakly correlated with TTC: useful for ranking imminence, not precise timing."
        if sr > 0.15 else
        "τ not correlated with TTC on this dataset: use as presence detector only."
    )
    print(f"\n{'='*68}")
    print("Interpretation")
    print(f"{'='*68}")
    print(f"""
  Correlation r={pr_r:+.3f} (Pearson), {sr:+.3f} (Spearman)
    → {corr_interp}

  Production default τ=3.0  (high-precision shoulder of PR curve)
    P={tau3.get('precision','?'):.3f}  R={tau3.get('recall','?'):.3f}  F1={tau3.get('f1','?'):.3f}

  Data-optimal τ={tau_opt:.1f}  (maximum F1 from PR sweep)
    P={prec:.3f}  R={rec_best:.3f}  F1={f1:.3f}
    → Higher recall at cost of earlier (looser) firing; tune to use-case.

  AUC-PR={auc:.3f}  (random baseline ≈ {lk_b['n_gt_positive']/max(result['n_gated_frames'],1):.2f})
  Mean latency: {result['mean_latency_ms']:.2f} ms/frame
""")

    os.makedirs("results", exist_ok=True)
    out = "results/optical_flow_gated.json"
    with open(out, "w") as f:
        json.dump(result, f, indent=2, default=str)
    print(f"Saved → {out}")


if __name__ == "__main__":
    main()
