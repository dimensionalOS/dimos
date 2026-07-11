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

"""Per-frame benchmark: Livox Mid-360 lidar vs RealSense (raw SDK cloud) vs
StereoPointCloud (our filtered pipeline).

Lidar is the reference ("ground truth") — absent survey equipment, it's the more
geometrically trustworthy sensor at range. Metrics: Chamfer distance, ETH3D-style
accuracy/completeness/F-score, voxel occupancy IoU, and a Newer-College-style
range-binned density falloff.

There is no calibrated D435i<->Mid-360 extrinsic for FlowBase anywhere in this repo
(checked: dimos/robot/assembly/mid360_realsense_30.py is a different, unrelated rig —
its mount is tilted ~36 degrees, contradicting the FlowBase README's "level
orientation"). So both candidate clouds are ICP-aligned to the lidar reference from
a plain identity initial guess, rather than trusting any fixed offset. Once you have
a measured extrinsic, thread it in as `init` in `_evaluate` for faster/more robust
convergence — identity only works because the sensors are assumed roughly co-located
and forward-facing on a small base.
"""

from __future__ import annotations

import asyncio
import threading
from typing import Any

import numpy as np
import open3d as o3d
from pydantic import Field
from reactivex.disposable import Disposable
from scipy.spatial import cKDTree

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


# ── Metrics ─────────────────────────────────────────────────────────────────


def chamfer_distance(pred: np.ndarray, gt: np.ndarray) -> float:
    """Symmetric mean squared nearest-neighbor distance."""
    if len(pred) == 0 or len(gt) == 0:
        return float("nan")
    d_pred_to_gt, _ = cKDTree(gt).query(pred, k=1)
    d_gt_to_pred, _ = cKDTree(pred).query(gt, k=1)
    return float(np.mean(d_pred_to_gt**2) + np.mean(d_gt_to_pred**2))


def accuracy_completeness_fscore(
    pred: np.ndarray, gt: np.ndarray, threshold: float
) -> tuple[float, float, float]:
    """ETH3D / Tanks-and-Temples convention: accuracy = pred covered by gt, completeness = gt covered by pred."""
    if len(pred) == 0 or len(gt) == 0:
        return 0.0, 0.0, 0.0
    d_pred_to_gt, _ = cKDTree(gt).query(pred, k=1)
    d_gt_to_pred, _ = cKDTree(pred).query(gt, k=1)
    accuracy = float(np.mean(d_pred_to_gt <= threshold))
    completeness = float(np.mean(d_gt_to_pred <= threshold))
    fscore = 0.0 if (accuracy + completeness) == 0 else 2 * accuracy * completeness / (accuracy + completeness)
    return accuracy, completeness, fscore


def voxel_occupancy_iou(pred: np.ndarray, gt: np.ndarray, voxel_size: float) -> float:
    if len(pred) == 0 or len(gt) == 0:
        return 0.0
    keys_pred = set(map(tuple, np.floor(pred / voxel_size).astype(np.int64)))
    keys_gt = set(map(tuple, np.floor(gt / voxel_size).astype(np.int64)))
    union = len(keys_pred | keys_gt)
    return len(keys_pred & keys_gt) / union if union else 0.0


def range_binned_density(points: np.ndarray, origin: np.ndarray, bins: list[float]) -> list[int]:
    """Point count per distance-from-origin bin — density/accuracy falloff vs range (Newer College style)."""
    if len(points) == 0:
        return [0] * (len(bins) - 1)
    d = np.linalg.norm(points - origin, axis=1)
    counts, _ = np.histogram(d, bins=bins)
    return counts.tolist()


def icp_refine(source: np.ndarray, target: np.ndarray, init: np.ndarray, max_corr_dist: float) -> np.ndarray:
    """Snap ``source`` onto ``target`` starting from ``init`` (4x4). Returns the refined 4x4."""
    src = o3d.geometry.PointCloud()
    src.points = o3d.utility.Vector3dVector(source)
    tgt = o3d.geometry.PointCloud()
    tgt.points = o3d.utility.Vector3dVector(target)
    result = o3d.pipelines.registration.registration_icp(
        src, tgt, max_corr_dist, init,
        o3d.pipelines.registration.TransformationEstimationPointToPoint(),
    )
    return result.transformation


def apply_matrix(points: np.ndarray, matrix: np.ndarray) -> np.ndarray:
    ones = np.ones((len(points), 1))
    homogeneous = np.hstack([points, ones])
    return (matrix @ homogeneous.T).T[:, :3].astype(np.float32)


# ── Module ──────────────────────────────────────────────────────────────────


class BenchConfig(ModuleConfig):
    period_s: float = 2.0
    min_points: int = 50
    fscore_threshold_m: float = 0.05
    fscore_threshold_loose_m: float = 0.20
    voxel_size_m: float = 0.05
    icp_max_corr_dist_m: float = 0.5  # generous: starting from identity, not a measured offset
    range_bins_m: list[float] = Field(default_factory=lambda: [0.0, 0.5, 1.0, 2.0, 4.0, 8.0, 100.0])


class LidarStereoBenchmark(Module):
    """Buffers the latest lidar / realsense_raw / stereo frame and scores them every ``period_s``."""

    config: BenchConfig

    lidar: In[PointCloud2]
    realsense_raw: In[PointCloud2]
    stereo: In[PointCloud2]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._lock = threading.Lock()
        self._latest: dict[str, PointCloud2] = {}
        self._running = False
        # No calibrated D435i<->Mid-360 extrinsic exists for FlowBase (see module
        # docstring) — start both candidates from identity and let ICP find the
        # real offset. Swap in a measured 4x4 here once you have one.
        self._icp_init = np.eye(4)

    @rpc
    def start(self) -> None:
        super().start()
        self.register_disposable(Disposable(self.lidar.subscribe(lambda m: self._store("lidar", m))))
        self.register_disposable(
            Disposable(self.realsense_raw.subscribe(lambda m: self._store("realsense_raw", m)))
        )
        self.register_disposable(Disposable(self.stereo.subscribe(lambda m: self._store("stereo", m))))
        self._running = True
        self.spawn(self._benchmark_loop())

    @rpc
    def stop(self) -> None:
        self._running = False
        super().stop()

    def _store(self, name: str, msg: PointCloud2) -> None:
        with self._lock:
            self._latest[name] = msg

    async def _benchmark_loop(self) -> None:
        while self._running:
            await asyncio.sleep(self.config.period_s)
            try:
                self._evaluate()
            except Exception:
                logger.exception("LidarStereoBenchmark: evaluation failed")

    def _score(self, gt_xyz: np.ndarray, pred_xyz: np.ndarray) -> dict[str, float]:
        cfg = self.config
        acc_t, comp_t, f_t = accuracy_completeness_fscore(pred_xyz, gt_xyz, cfg.fscore_threshold_m)
        acc_l, comp_l, f_l = accuracy_completeness_fscore(pred_xyz, gt_xyz, cfg.fscore_threshold_loose_m)
        return {
            "n": len(pred_xyz),
            "chamfer": chamfer_distance(pred_xyz, gt_xyz),
            "acc_tight": acc_t, "comp_tight": comp_t, "f_tight": f_t,
            "f_loose": f_l,
            "iou": voxel_occupancy_iou(pred_xyz, gt_xyz, cfg.voxel_size_m),
        }

    def _evaluate(self) -> None:
        cfg = self.config
        with self._lock:
            lidar_msg = self._latest.get("lidar")
            rs_msg = self._latest.get("realsense_raw")
            stereo_msg = self._latest.get("stereo")

        if lidar_msg is None or rs_msg is None:
            logger.info("LidarStereoBenchmark: waiting for lidar + realsense_raw frames...")
            return

        # Lidar is the reference frame — nothing to transform. Everything else is
        # ICP-aligned onto it (see module docstring: no calibrated extrinsic exists).
        lidar_xyz = lidar_msg.points_f32()
        rs_raw_xyz = rs_msg.points_f32()
        if len(lidar_xyz) < cfg.min_points or len(rs_raw_xyz) < cfg.min_points:
            logger.info("LidarStereoBenchmark: not enough points yet (lidar=%d realsense=%d)",
                        len(lidar_xyz), len(rs_raw_xyz))
            return

        lines = [f"lidar (ref): n={len(lidar_xyz)}"]

        rs_icp_T = icp_refine(rs_raw_xyz, lidar_xyz, self._icp_init, cfg.icp_max_corr_dist_m)
        rs_xyz = apply_matrix(rs_raw_xyz, rs_icp_T)
        rs_score = self._score(lidar_xyz, rs_xyz)
        lines.append(self._fmt("realsense (raw, ICP-aligned)", rs_score))

        if stereo_msg is not None:
            stereo_raw_xyz = stereo_msg.points_f32()
            if len(stereo_raw_xyz) >= cfg.min_points:
                stereo_icp_T = icp_refine(
                    stereo_raw_xyz, lidar_xyz, self._icp_init, cfg.icp_max_corr_dist_m
                )
                stereo_xyz = apply_matrix(stereo_raw_xyz, stereo_icp_T)
                stereo_score = self._score(lidar_xyz, stereo_xyz)
                lines.append(self._fmt("stereo (ours, ICP-aligned)", stereo_score))

        origin = np.zeros(3, dtype=np.float32)
        lines.append(f"range-binned density {cfg.range_bins_m}: lidar={range_binned_density(lidar_xyz, origin, cfg.range_bins_m)} "
                     f"realsense={range_binned_density(rs_xyz, origin, cfg.range_bins_m)}")

        logger.info("LidarStereoBenchmark:\n  " + "\n  ".join(lines))

    def _fmt(self, name: str, s: dict[str, float]) -> str:
        cfg = self.config
        return (
            f"{name}: n={s['n']}  chamfer={s['chamfer']:.4f}  "
            f"acc@{cfg.fscore_threshold_m*100:.0f}cm={s['acc_tight']:.2f}  "
            f"comp@{cfg.fscore_threshold_m*100:.0f}cm={s['comp_tight']:.2f}  "
            f"F@{cfg.fscore_threshold_m*100:.0f}cm={s['f_tight']:.2f}  "
            f"F@{cfg.fscore_threshold_loose_m*100:.0f}cm={s['f_loose']:.2f}  "
            f"IoU@{cfg.voxel_size_m*100:.0f}cm={s['iou']:.2f}"
        )
