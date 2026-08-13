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

"""Write an evaluation report into a rerun recording, one scene per dataset."""

from __future__ import annotations

from typing import TYPE_CHECKING

import numpy as np

from dimos.navigation.nav_3d.evaluator.metrics import body_box_half_extents
from dimos.navigation.nav_3d.mls_planner.viz import clearance_colors
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from pathlib import Path

    from numpy.typing import NDArray
    import rerun.blueprint as rrb

    from dimos.navigation.nav_3d.evaluator.cases import Suite
    from dimos.navigation.nav_3d.evaluator.config import EvalConfig
    from dimos.navigation.nav_3d.evaluator.runner import (
        CaseResult,
        PlannerArtifacts,
        PlanOutcome,
        Report,
    )

logger = setup_logger()

# Drawn radius of a map voxel. Small enough that a dense map still reads as
# surfaces rather than one solid blob.
VOXEL_RADIUS = 0.007
# Surface cells stay tied to the cell they represent, so the planner's
# standable surface still reads as a grid.
SURFACE_RADIUS_SCALE = 0.25
ENDPOINT_RADIUS = 0.05
EDGE_RADIUS = 0.008
WALKED_RADIUS = 0.019
# The online path is drawn over the final one, so it is the thicker of the two.
ONLINE_PATH_RADIUS = 0.05
FINAL_PATH_RADIUS = 0.025
# A refusal is reviewed on its own, an unreachable goal only against its path.
NEGATIVE_INTENT_RADIUS = 0.0075
FAILED_INTENT_RADIUS = 0.00375
# Gate violations are drawn as points on their path, wide enough to spot.
VIOLATION_RADIUS_SCALE = 3.0
WALKED_PATH_COLOR = [255, 255, 255]
START_COLOR = [0, 255, 255]
GOAL_COLOR = [255, 140, 0]
STEEP_COLOR = [160, 32, 240]
COLLISION_COLOR = [255, 0, 0]
UNSUPPORTED_COLOR = [255, 0, 255]
NEGATIVE_INTENT_COLOR = [255, 255, 0]
NEAR_WALL_COLOR = [120, 120, 120]

VALID_PATH_COLOR = [0, 220, 0]
# Darker than the violation markers drawn on top of it.
INVALID_PATH_COLOR = [150, 0, 0]
UNREACHED_PATH_COLOR = [255, 200, 0]

CLEARANCE_CLAMP_M = 1.0
# Cells colored gray as too close to a wall. Display threshold only.
CLEARANCE_NEAR_WALL_M = 0.1


def turbo_by_height(points: NDArray[np.float32]) -> NDArray[np.uint8]:
    # Lazy: matplotlib is a heavy viz-only dependency.
    import matplotlib.pyplot as plt

    if len(points) == 0:
        return np.zeros((0, 3), dtype=np.uint8)
    z = points[:, 2].astype(np.float64)
    span = float(z.max() - z.min())
    t = (z - z.min()) / max(span, 1e-6)
    return np.asarray(plt.get_cmap("turbo")(t)[:, :3] * 255, dtype=np.uint8)


def _clearance_colors(clearance: NDArray[np.float32], hard_clearance: float) -> NDArray[np.uint8]:
    """The planner's clearance ramp, with cells below the hard limit called out."""
    out: NDArray[np.uint8] = clearance_colors(clearance, CLEARANCE_CLAMP_M)
    out[clearance < hard_clearance] = NEAR_WALL_COLOR
    return out


def _edge_cost_colors(costs: NDArray[np.float32]) -> NDArray[np.uint8]:
    t = np.log1p(np.maximum(costs, 0.0))
    t = t / max(float(t.max()), 1e-6)
    low = np.array([220.0, 220.0, 220.0])
    high = np.array([255.0, 40.0, 40.0])
    return np.asarray(low + t[:, None] * (high - low), dtype=np.uint8)


def _log_planner(entity: str, artifacts: PlannerArtifacts | None, cfg: EvalConfig) -> None:
    import rerun as rr

    if artifacts is None:
        return
    if artifacts.occupied.size:
        rr.log(
            f"{entity}/voxels",
            rr.Points3D(
                artifacts.occupied,
                colors=turbo_by_height(artifacts.occupied),
                radii=VOXEL_RADIUS,
            ),
            static=True,
        )
    surface = artifacts.surface_clearance
    if surface.size:
        rr.log(
            f"{entity}/surface",
            rr.Points3D(
                surface[:, :3],
                colors=_clearance_colors(surface[:, 3], CLEARANCE_NEAR_WALL_M),
                radii=cfg.voxel_size * SURFACE_RADIUS_SCALE,
            ),
            static=True,
        )
    edges = artifacts.edges
    if edges.size:
        rr.log(
            f"{entity}/edges",
            rr.LineStrips3D(
                edges[:, :6].reshape(-1, 2, 3),
                colors=_edge_cost_colors(edges[:, 6]),
                radii=EDGE_RADIUS,
            ),
            static=True,
        )


def _outcome_color(outcome: PlanOutcome) -> list[int]:
    if outcome.success:
        return VALID_PATH_COLOR
    if outcome.planned:
        return INVALID_PATH_COLOR
    return UNREACHED_PATH_COLOR


def _log_collisions(entity: str, outcome: PlanOutcome, cfg: EvalConfig) -> None:
    """The robot body where the path put it into occupancy."""
    import rerun as rr

    if not outcome.collisions:
        return
    half = body_box_half_extents(cfg)
    rr.log(
        f"{entity}/collisions",
        rr.Boxes3D(
            centers=[box.center for box in outcome.collisions],
            half_sizes=[half] * len(outcome.collisions),
            quaternions=[rr.Quaternion(xyzw=box.rotation) for box in outcome.collisions],
            colors=[COLLISION_COLOR],
            fill_mode="majorwireframe",
        ),
        static=True,
    )


def _log_path(entity: str, outcome: PlanOutcome, radius: float, cfg: EvalConfig) -> None:
    import rerun as rr

    if not outcome.waypoints:
        return
    rr.log(
        entity,
        rr.LineStrips3D([outcome.waypoints], colors=[_outcome_color(outcome)], radii=radius),
        static=True,
    )
    _log_collisions(entity, outcome, cfg)
    for name, points, color in (
        ("steep", outcome.steep, STEEP_COLOR),
        ("unsupported", outcome.unsupported, UNSUPPORTED_COLOR),
    ):
        if not points:
            continue
        rr.log(
            f"{entity}/{name}",
            rr.Points3D(points, colors=[color], radii=radius * VIOLATION_RADIUS_SCALE),
            static=True,
        )


def _dataset_view(root: str, case_ids: list[str]) -> rrb.Spatial3DView:
    """One view per dataset, with every case hidden until toggled on."""
    import rerun.blueprint as rrb

    hidden = [f"{root}/planner_final/edges"]
    hidden += [f"{root}/cases/{cid}" for cid in case_ids]
    return rrb.Spatial3DView(
        origin=f"/{root}",
        name=root,
        overrides={path: rrb.EntityBehavior(visible=False) for path in hidden},
    )


def _log_case(base: str, case: CaseResult, cfg: EvalConfig) -> None:
    """Endpoints, intent line, what the pipeline held at plan time, both paths."""
    import rerun as rr

    rr.log(
        f"{base}/start",
        rr.Points3D([case.start], colors=[START_COLOR], radii=ENDPOINT_RADIUS),
        static=True,
    )
    rr.log(
        f"{base}/goal",
        rr.Points3D([case.goal], colors=[GOAL_COLOR], radii=ENDPOINT_RADIUS),
        static=True,
    )
    if case.expect_fail:
        # Always visible, so a correct refusal is reviewable too.
        rr.log(
            f"{base}/intent",
            rr.LineStrips3D(
                [[case.start, case.goal]],
                colors=[NEGATIVE_INTENT_COLOR],
                radii=NEGATIVE_INTENT_RADIUS,
            ),
            static=True,
        )
    elif not case.online.success:
        rr.log(
            f"{base}/intent",
            rr.LineStrips3D(
                [[case.start, case.goal]], colors=[INVALID_PATH_COLOR], radii=FAILED_INTENT_RADIUS
            ),
            static=True,
        )
    # What the pipeline held at plan time, saved for every case.
    _log_planner(f"{base}/known", case.online_artifacts, cfg)
    _log_path(f"{base}/online", case.online, ONLINE_PATH_RADIUS, cfg)
    _log_path(f"{base}/final", case.final, FINAL_PATH_RADIUS, cfg)


def write_rrd(report: Report, suites: list[Suite], cfg: EvalConfig, out: Path) -> None:
    import rerun as rr
    import rerun.blueprint as rrb

    rr.init("nav3d_eval", recording_id="nav3d_eval")
    out.parent.mkdir(parents=True, exist_ok=True)
    rr.save(str(out))

    suites_by_dataset = {suite.dataset: suite for suite in suites}
    for dataset in report.datasets:
        suite = suites_by_dataset[dataset.dataset]
        trajectory = suite.trajectory()
        root = dataset.dataset

        foot = trajectory.foot(cfg.robot_height)
        rr.log(
            f"{root}/walked_path",
            rr.LineStrips3D([foot], colors=[WALKED_PATH_COLOR], radii=WALKED_RADIUS),
            static=True,
        )

        _log_planner(f"{root}/planner_final", dataset.final_artifacts, cfg)

        for case in dataset.cases:
            _log_case(f"{root}/cases/{case.id}", case, cfg)

    views = [_dataset_view(d.dataset, [c.id for c in d.cases]) for d in report.datasets]
    rr.send_blueprint(rrb.Blueprint(rrb.Tabs(*views) if len(views) > 1 else views[0]))

    logger.info("wrote rerun recording", path=out)
