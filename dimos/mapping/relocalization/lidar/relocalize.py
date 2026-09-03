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

Coarse FPFH+RANSAC to find the place, then point-to-plane ICP over
widening-to-narrowing distances to settle it. No streams, no modules, no
clock - :mod:`dimos.mapping.relocalization.module` is the runtime around
this, and ``tune.py`` measures it. Strategies + evals began at
https://github.com/leshy/relocalization-test (this was ``align_fast``).

    relocalizer = LidarRelocalizer(premap_cloud, PRESETS["mid360"])
    tf = relocalizer.relocalize(live_cloud)   # world -> map, or None if it refused

The map is preprocessed once when the relocalizer is built - downsampling
it, estimating normals and computing FPFH features is the pipeline's
dominant cost, and it does not change between queries.

Every scale in a :class:`RelocalizeConfig` was measured against one rig on
one recording, so configs are named rather than universal - there is no
default config, only :data:`PRESETS`. See tune.md for how to measure one
for a rig that is not in there yet.
"""

from __future__ import annotations

from typing import TYPE_CHECKING, NamedTuple

import numpy as np

from dimos.mapping.relocalization.module import FRAME_MAP
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.protocol.service.spec import BaseConfig
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

if TYPE_CHECKING:
    # open3d is imported lazily inside the methods that need it - it is a
    # heavy import and a module-scope one would cost every process that
    # merely touches this file. These names are for reading the signatures;
    # open3d ships no stubs, so mypy still widens them to Any.
    from open3d.geometry import PointCloud
    from open3d.pipelines.registration import Feature, RegistrationResult


class _Prepared(NamedTuple):
    """A cloud's downsampled forms and features, under one config."""

    coarse: PointCloud  # downsampled at voxel_coarse, with normals
    fpfh: Feature  # FPFH descriptors of `coarse`
    fine: PointCloud  # downsampled at voxel_fine, with normals


class RelocalizeConfig(BaseConfig):
    """The aligner's knobs for **one rig**. There is no universal setting.

    The required fields are scales, wrong for a different sensor or a
    room-sized map. Required so no bare ``RelocalizeConfig()`` can quietly
    mean somebody else's rig. Measure a new one with a study (tune.md) and
    add a preset; do not nudge an existing one.
    """

    voxel_coarse: float  # FPFH + RANSAC scale
    voxel_fine: float  # ICP scale
    # Normal and feature neighbourhoods, in multiples of the working voxel.
    normal_radius_factor: float
    fpfh_radius_factor: float
    coarse_dist_factor: float  # RANSAC correspondence distance / voxel_coarse
    ransac_iters: int
    mutual_filter: bool
    edge_length: float
    # ICP correspondence distance / voxel_fine, for the *last* stage. Earlier
    # stages double it each step back, so a hypothesis landing further out
    # than the final threshold still has a stage wide enough to reach it.
    icp_dist_factor: float
    icp_stages: int
    # Point every normal into one half-space. Normal sign is arbitrary out of
    # estimate_normals and FPFH is built from angles involving it, so without
    # this two passes over the same corner describe it differently. Needs a
    # shared up axis, which lidar-inertial odometry gives both sides; turn it
    # off for a premap of unknown provenance.
    orient_normals: bool
    # Best-of-N RANSAC, by fitness. Its best hypothesis is sometimes simply
    # wrong and ICP then polishes a wrong answer. One suffices at mid360's
    # iteration budget; restarts rescued the older, sparser search.
    ransac_restarts: int
    # ICP fitness a fix must clear to be published. On the mid360 walk the two
    # populations sit far apart - right place 0.84-0.94, place absent from the
    # premap 0.04-0.24 - so this goes in the empty middle. Strictness is
    # cheap: a rejected fix is a retry, an accepted wrong one is a TF the
    # whole stack believes.
    fitness_threshold: float
    # Retry window, in accumulated scans: first try at `min_frames`, give up
    # past `max_frames`. Rig scales, but hand-picked rather than measured -
    # a study should search them (tune.md).
    min_frames: int
    max_frames: int

    # Search budgets and caps. Not scales, so not per-rig, so defaulted -
    # tune them if a rig is slow, not because it is shaped differently.
    normal_max_nn: int = 30
    fpfh_max_nn: int = 100
    ransac_confidence: float = 0.999
    ransac_n: int = 3  # the minimum for a rigid transform
    icp_max_iter: int = 200


# Livox mid360 on a Go2, outdoors, against a premap of the same block.
# Trial 229 of the go2-sf-area1 study - the candidate that held its hit rate
# on probes it was never tuned against,
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

# A rig's name to its measured settings. Add an entry by running a study for
# that rig (tune.md); do not retune an existing one for a new sensor.
PRESETS: dict[str, RelocalizeConfig] = {"mid360": MID360}
DEFAULT_PRESET = "mid360"


class LidarRelocalizer:
    """A prior map, prepared once, that places live clouds into itself.

    The preprocessing the map needs is the pipeline's dominant cost and does
    not change between queries, so it belongs in the constructor - a live
    module builds one at startup and calls :meth:`relocalize` per cloud.
    Holding the config here too means the map's derived forms and the
    settings that produced them cannot drift apart, which a per-call config
    argument would allow.
    """

    def __init__(self, global_map: PointCloud, config: RelocalizeConfig) -> None:
        self.config = config
        self.map = global_map
        self._target = self._prepare(global_map)

    def _prepare(self, cloud: PointCloud) -> _Prepared:
        """Downsample, estimate normals, compute FPFH - under this relocalizer's config."""
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
        """Open3D's registration result for ``local_map`` against the prior map.

        Always answers, however poor the answer, and answers in open3d's own
        terms: ``.transformation`` places the live cloud into the map,
        ``.fitness`` and ``.inlier_rmse`` say how well. :meth:`relocalize` is
        the one a runtime calls; this is for a caller that wants to see a
        refused fix, such as the eval measuring where a rejected hypothesis
        landed.
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
            """Point-to-plane ICP from wide to narrow, so a distant guess is still reachable.

            A single pass at the final distance cannot pull in a hypothesis
            further out than that distance - its correspondences are already
            wrong - which is why each earlier stage doubles it.
            """
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

    def relocalize(self, local_map: PointCloud, world_frame: str) -> Transform | None:
        """The ``world_frame -> map`` transform, or ``None`` when nothing was good enough.

        Ready to publish: stamped with the frames the TF tree expects, and
        already inverted from the placement open3d computes. Refusing is a
        real answer and the common one for a place the prior map never saw.
        Everything the decision rests on - the aligner's knobs and
        ``fitness_threshold`` - is this object's config, so a caller
        configures it once and checks whether it got a transform.
        """
        result = self.align(local_map)
        logger.info(f"align: fitness={result.fitness:.3f} rmse={result.inlier_rmse:.3f}")
        if result.fitness < self.config.fitness_threshold:
            return None
        placement = Transform.from_matrix(
            np.asarray(result.transformation), frame_id=FRAME_MAP, child_frame_id=world_frame
        )
        return placement.inverse()
