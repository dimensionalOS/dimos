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

"""Write an evaluation report into a rerun recording.

One static scene per dataset:
- map/obstacles: final voxels, turbo colormap by height
- walked_path: the recorded foot path (white)
- planner_final: the planner graph the full aggregated map produced.
  Surface cells colored by wall clearance (red inside the hard clearance),
  edges colored white to red by log traversal cost.
- cases/<id>: start (cyan), goal (orange), online and final planned paths
  colored by verdict (green valid, red gate-invalid, yellow unreached), the
  gate's collision samples (red dots), unsupported samples (magenta), and
  too-steep waypoints (purple). Failed cases also get a thin red
  start-to-goal intent line and a known/ layer: the planner graph on the
  incremental map at plan time, i.e. what the robot knew when it failed.
"""

from __future__ import annotations

from typing import TYPE_CHECKING

import numpy as np
import rerun as rr
import rerun.blueprint as rrb

from dimos.navigation.nav_3d.evaluator.final_map import load_or_build_final_map
from dimos.navigation.nav_3d.evaluator.recording import load_trajectory

if TYPE_CHECKING:
    from pathlib import Path

    from numpy.typing import NDArray

    from dimos.navigation.nav_3d.evaluator.cases import Suite
    from dimos.navigation.nav_3d.evaluator.config import EvalConfig
    from dimos.navigation.nav_3d.evaluator.runner import PlannerArtifacts, PlanOutcome, Report

WALKED_PATH_COLOR = [255, 255, 255]
START_COLOR = [0, 255, 255]
GOAL_COLOR = [255, 140, 0]
COLLISION_COLOR = [255, 0, 0]
UNSUPPORTED_COLOR = [255, 0, 255]
STEEP_COLOR = [160, 32, 240]
NEGATIVE_INTENT_COLOR = [255, 255, 0]

VALID_PATH_COLOR = [0, 220, 0]
INVALID_PATH_COLOR = [255, 0, 0]
UNREACHED_PATH_COLOR = [255, 200, 0]

CLEARANCE_CLAMP_M = 1.0
# Cells colored red as too close to a wall. Display threshold only.
CLEARANCE_RED_M = 0.1


def turbo_by_height(points: NDArray[np.float32]) -> NDArray[np.uint8]:
    # Lazy: matplotlib is a heavy viz-only dependency.
    import matplotlib.pyplot as plt

    z = points[:, 2].astype(np.float64)
    span = float(z.max() - z.min()) if len(z) else 0.0
    t = (z - z.min()) / max(span, 1e-6)
    return np.asarray(plt.get_cmap("turbo")(t)[:, :3] * 255, dtype=np.uint8)


def _clearance_colors(clearance: NDArray[np.float32], hard_clearance: float) -> NDArray[np.uint8]:
    norm = np.clip(np.nan_to_num(clearance / CLEARANCE_CLAMP_M, nan=1.0, posinf=1.0), 0.0, 1.0)
    blocked = np.array([4.0, 8.0, 48.0])
    clear = np.array([150.0, 200.0, 255.0])
    out = np.asarray(blocked + norm[:, None] * (clear - blocked), dtype=np.uint8)
    out[clearance < hard_clearance] = (255, 0, 0)
    return out


def _edge_cost_colors(costs: NDArray[np.float32]) -> NDArray[np.uint8]:
    t = np.log1p(np.maximum(costs, 0.0))
    t = t / max(float(t.max()), 1e-6)
    low = np.array([220.0, 220.0, 220.0])
    high = np.array([255.0, 40.0, 40.0])
    return np.asarray(low + t[:, None] * (high - low), dtype=np.uint8)


def _log_planner(entity: str, artifacts: PlannerArtifacts | None, cfg: EvalConfig) -> None:
    if artifacts is None:
        return
    surface = artifacts.surface_clearance
    if surface.size:
        rr.log(
            f"{entity}/surface",
            rr.Points3D(
                surface[:, :3],
                colors=_clearance_colors(surface[:, 3], CLEARANCE_RED_M),
                radii=cfg.voxel_size / 4,
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
                radii=0.008,
            ),
            static=True,
        )


def _outcome_color(outcome: PlanOutcome) -> list[int]:
    if outcome.success:
        return VALID_PATH_COLOR
    if outcome.planned and not outcome.valid:
        return INVALID_PATH_COLOR
    return UNREACHED_PATH_COLOR


def _log_path(entity: str, outcome: PlanOutcome, radius: float) -> None:
    if not outcome.waypoints:
        return
    rr.log(
        entity,
        rr.LineStrips3D([outcome.waypoints], colors=[_outcome_color(outcome)], radii=radius),
        static=True,
    )
    if outcome.collisions:
        rr.log(
            f"{entity}/collisions",
            rr.Points3D(outcome.collisions, colors=[COLLISION_COLOR], radii=radius * 3),
            static=True,
        )
    if outcome.unsupported:
        rr.log(
            f"{entity}/unsupported",
            rr.Points3D(outcome.unsupported, colors=[UNSUPPORTED_COLOR], radii=radius * 3),
            static=True,
        )
    if outcome.steep:
        rr.log(
            f"{entity}/steep",
            rr.Points3D(outcome.steep, colors=[STEEP_COLOR], radii=radius * 3),
            static=True,
        )


def _dataset_view(root: str, case_ids: list[str]) -> rrb.Spatial3DView:
    """One view per dataset, planner graph edges hidden until toggled on."""
    hidden = [f"{root}/planner_final/edges"]
    hidden += [f"{root}/cases/{cid}/known/edges" for cid in case_ids]
    return rrb.Spatial3DView(
        origin=f"/{root}",
        name=root,
        overrides={path: rrb.EntityBehavior(visible=False) for path in hidden},
    )


def write_rrd(report: Report, suites: list[Suite], cfg: EvalConfig, out: Path) -> None:
    rr.init("nav3d_eval", recording_id="nav3d_eval")
    rr.save(str(out))

    suites_by_dataset = {suite.dataset: suite for suite in suites}
    for dataset in report.datasets:
        suite = suites_by_dataset[dataset.dataset]
        db_path = suite.db_path()
        final = load_or_build_final_map(db_path, suite, cfg)
        trajectory = load_trajectory(db_path, suite.odom_stream)
        root = dataset.dataset

        rr.log(
            f"{root}/map/obstacles",
            rr.Points3D(
                final.occupied,
                colors=turbo_by_height(final.occupied),
                radii=cfg.voxel_size / 4,
            ),
            static=True,
        )
        foot = trajectory.positions - np.array([0.0, 0.0, cfg.robot_height], dtype=np.float32)
        rr.log(
            f"{root}/walked_path",
            rr.LineStrips3D([foot], colors=[WALKED_PATH_COLOR], radii=0.015),
            static=True,
        )

        _log_planner(f"{root}/planner_final", dataset.final_artifacts, cfg)

        for case in dataset.cases:
            base = f"{root}/cases/{case.id}"
            rr.log(
                f"{base}/start",
                rr.Points3D([case.start], colors=[START_COLOR], radii=0.05),
                static=True,
            )
            rr.log(
                f"{base}/goal",
                rr.Points3D([case.goal], colors=[GOAL_COLOR], radii=0.05),
                static=True,
            )
            if case.expect_fail:
                # Always visible, so a correct refusal is reviewable too.
                rr.log(
                    f"{base}/intent",
                    rr.LineStrips3D(
                        [[case.start, case.goal]], colors=[NEGATIVE_INTENT_COLOR], radii=0.006
                    ),
                    static=True,
                )
            elif not case.online.success:
                rr.log(
                    f"{base}/intent",
                    rr.LineStrips3D(
                        [[case.start, case.goal]], colors=[INVALID_PATH_COLOR], radii=0.003
                    ),
                    static=True,
                )
                _log_planner(f"{base}/known", case.online_artifacts, cfg)
            _log_path(f"{base}/online", case.online, radius=0.04)
            _log_path(f"{base}/final", case.final, radius=0.02)

    views = [_dataset_view(d.dataset, [c.id for c in d.cases]) for d in report.datasets]
    rr.send_blueprint(rrb.Blueprint(rrb.Tabs(*views) if len(views) > 1 else views[0]))

    print(f"wrote {out}")
    print(f"open with: rerun {out}")
