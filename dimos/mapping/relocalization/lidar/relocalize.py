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

"""Placing a live lidar cloud into a prior map.

Coarse FPFH+RANSAC to find the place, then point-to-plane ICP over narrowing
distances to settle it. Pure computation; ``lidar/module.py`` is the runtime
and ``tune.py`` measures it.

    relocalizer = LidarRelocalizer(premap_cloud, PRESETS["mid360"])
    tf = relocalizer.relocalize(live_cloud, "world", "map")  # world -> map, or None

Configs are per-rig :data:`PRESETS`; see tune.md to measure a new one.
"""

from __future__ import annotations

from typing import TYPE_CHECKING, NamedTuple

import numpy as np

from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.protocol.service.spec import BaseConfig
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

if TYPE_CHECKING:
    # open3d is imported lazily (heavy); these names are for signatures only.
    from open3d.geometry import PointCloud
    from open3d.pipelines.registration import Feature, RegistrationResult


class _Prepared(NamedTuple):
    """A cloud's downsampled forms and features, under one config."""

    coarse: PointCloud  # downsampled at voxel_coarse, with normals
    fpfh: Feature  # FPFH descriptors of `coarse`
    fine: PointCloud  # downsampled at voxel_fine, with normals


class RelocAttempt(NamedTuple):
    """One decided attempt: the ``world -> map`` fix if accepted, and what it was scored on."""

    fix: Transform | None
    result: RegistrationResult


class RelocalizeConfig(BaseConfig):
    """The aligner's knobs for one rig; scale fields are required, so there is no default."""

    voxel_coarse: float  # FPFH + RANSAC scale
    voxel_fine: float  # ICP scale
    # Normal and feature neighbourhoods, in multiples of the working voxel.
    normal_radius_factor: float
    fpfh_radius_factor: float
    coarse_dist_factor: float  # RANSAC correspondence distance / voxel_coarse
    ransac_iters: int
    mutual_filter: bool
    edge_length: float
    # ICP correspondence distance / voxel_fine for the last stage; earlier stages double it
    icp_dist_factor: float
    icp_stages: int
    # Orient normals to +z so FPFH is consistent across passes; needs a shared up axis
    orient_normals: bool
    # Best-of-N RANSAC+ICP, by fitness
    ransac_restarts: int
    # ICP fitness a fix must clear to be published
    fitness_threshold: float
    # Scan window: first try at `min_frames`, hold at most `max_frames`
    min_frames: int
    max_frames: int

    # Search budgets, not rig-specific.
    normal_max_nn: int = 30
    fpfh_max_nn: int = 100
    ransac_confidence: float = 0.999
    ransac_n: int = 3  # the minimum for a rigid transform
    icp_max_iter: int = 200


# Livox mid360 on a Go2, outdoors; trial 229 of the go2-sf-area1 study.
MID360 = RelocalizeConfig(
    voxel_coarse=0.59,
    voxel_fine=0.30,
    normal_radius_factor=1.72,
    fpfh_radius_factor=4.13,
    coarse_dist_factor=2.73,
    ransac_iters=1_578_291,
    mutual_filter=True,
    edge_length=0.70,
    icp_dist_factor=0.55,
    icp_stages=2,
    orient_normals=True,
    ransac_restarts=1,
    fitness_threshold=0.6,
    min_frames=3,
    max_frames=7,
)

# The mid360 scales with the nav stack accept policy, from the go2 sf office replays.
MID360_NAV = MID360.model_copy(update={"fitness_threshold": 0.8, "ransac_restarts": 3})

# Rig name to its measured settings (tune.md).
PRESETS: dict[str, RelocalizeConfig] = {"mid360": MID360, "mid360-nav": MID360_NAV}
DEFAULT_PRESET = "mid360"


class LidarRelocalizer:
    """A prior map, preprocessed once under ``config``, that places live clouds into itself."""

    def __init__(self, global_map: PointCloud, config: RelocalizeConfig) -> None:
        self.config = config
        self.map = global_map
        self._target = self._prepare(global_map)

    def _prepare(self, cloud: PointCloud) -> _Prepared:
        """Downsample, estimate normals, compute FPFH under this relocalizer's config."""
        import open3d as o3d  # type: ignore[import-untyped]

        cfg = self.config
        reg = o3d.pipelines.registration

        def normals(pcd: PointCloud, voxel: float) -> PointCloud:
            pcd.estimate_normals(
                o3d.geometry.KDTreeSearchParamHybrid(
                    radius=voxel * cfg.normal_radius_factor, max_nn=cfg.normal_max_nn
                )
            )
            if cfg.orient_normals:
                pcd.orient_normals_to_align_with_direction([0.0, 0.0, 1.0])
            return pcd

        coarse = normals(cloud.voxel_down_sample(cfg.voxel_coarse), cfg.voxel_coarse)
        fpfh = reg.compute_fpfh_feature(
            coarse,
            o3d.geometry.KDTreeSearchParamHybrid(
                radius=cfg.voxel_coarse * cfg.fpfh_radius_factor, max_nn=cfg.fpfh_max_nn
            ),
        )
        fine = normals(cloud.voxel_down_sample(cfg.voxel_fine), cfg.voxel_fine)
        return _Prepared(coarse=coarse, fpfh=fpfh, fine=fine)

    def align(self, local_map: PointCloud) -> RegistrationResult:
        """Best registration of ``local_map`` against the prior map, however poor.

        ``.transformation`` places the live cloud in the map (map <- live).
        """
        import open3d as o3d  # type: ignore[import-untyped]

        cfg = self.config
        reg = o3d.pipelines.registration
        source = self._prepare(local_map)
        target = self._target
        dist = cfg.voxel_coarse * cfg.coarse_dist_factor

        def hypothesis() -> RegistrationResult:
            return reg.registration_ransac_based_on_feature_matching(
                source.coarse,
                target.coarse,
                source.fpfh,
                target.fpfh,
                mutual_filter=cfg.mutual_filter,
                max_correspondence_distance=dist,
                estimation_method=reg.TransformationEstimationPointToPoint(False),
                ransac_n=cfg.ransac_n,
                checkers=[
                    reg.CorrespondenceCheckerBasedOnEdgeLength(cfg.edge_length),
                    reg.CorrespondenceCheckerBasedOnDistance(dist),
                ],
                criteria=reg.RANSACConvergenceCriteria(cfg.ransac_iters, cfg.ransac_confidence),
            )

        def refine(coarse_T: np.ndarray) -> RegistrationResult:
            """Point-to-plane ICP from wide to narrow, so a distant guess is still reachable."""
            result = None
            for stage in reversed(range(max(cfg.icp_stages, 1))):
                result = reg.registration_icp(
                    source.fine,
                    target.fine,
                    cfg.voxel_fine * cfg.icp_dist_factor * (2**stage),
                    coarse_T,
                    reg.TransformationEstimationPointToPlane(),
                    reg.ICPConvergenceCriteria(max_iteration=cfg.icp_max_iter),
                )
                coarse_T = result.transformation
            return result

        scored = [
            refine(np.asarray(hypothesis().transformation))
            for _ in range(max(cfg.ransac_restarts, 1))
        ]
        return max(scored, key=lambda r: r.fitness)

    def relocalize(
        self, local_map: PointCloud, world_frame: str, map_frame: str
    ) -> Transform | None:
        """The ``world_frame -> map_frame`` transform, or ``None`` when nothing was good enough."""
        return self.attempt(local_map, world_frame, map_frame).fix

    def attempt(self, local_map: PointCloud, world_frame: str, map_frame: str) -> RelocAttempt:
        """The fix if accepted, ready to publish, with the registration it was scored on."""
        result = self.align(local_map)
        logger.info(f"align: fitness={result.fitness:.3f} rmse={result.inlier_rmse:.3f}")
        if result.fitness < self.config.fitness_threshold:
            return RelocAttempt(None, result)
        placement = Transform.from_matrix(
            np.asarray(result.transformation), frame_id=map_frame, child_frame_id=world_frame
        )
        return RelocAttempt(placement.inverse(), result)
