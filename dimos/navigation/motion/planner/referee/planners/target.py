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

"""The autoresearch subject. Seeded as the HONEST gold: the same SE(2)
search, but its world model is built from the cloud — no truth peeking.

Two factories: `make_py` ("target-py") is the python port spec, `make`
("target") adapts the rust candidate (dimos_motion2_target) — same algorithm,
the crate is the surface autoresearch agents rewrite.
"""

from __future__ import annotations

from typing import Any

import numpy as np

from ..geometry import AvoidanceConfig
from ..scenarios import Embodiment, Scenario, se2_search
from ..types import Path, PointCloud2
from .gold import densify_states, pose_stamped

Z_BAND = (0.05, 0.45)  # cloud slice that can touch the body
FINE = 0.05
PAD = 1.5

BUILD_CMD = (
    "uv run maturin develop --uv --release -m dimos/navigation/motion/planner/rust/Cargo.toml"
)


def band_mask(pts: np.ndarray) -> np.ndarray:
    """The slice of an (already anchored) cloud this planner treats as obstacles."""
    return (pts[:, 2] > Z_BAND[0]) & (pts[:, 2] < Z_BAND[1])


class TargetEpisode:
    def __init__(self, emb: Embodiment, resolution: float) -> None:
        self._emb = emb
        self._res = resolution

    def reset(self) -> None:
        pass

    def plan(
        self, cloud: PointCloud2, pose: tuple[float, float, float], goal: tuple[float, float]
    ) -> Path:
        from scipy.spatial import cKDTree

        pts, _ = cloud.as_numpy()
        pts = np.asarray(pts, dtype=float).reshape(-1, 3)
        band = pts[band_mask(pts)][:, :2]

        xs = [pose[0], goal[0]] + ([] if not len(band) else [band[:, 0].min(), band[:, 0].max()])
        ys = [pose[1], goal[1]] + ([] if not len(band) else [band[:, 1].min(), band[:, 1].max()])
        x0, y0 = min(xs) - PAD, min(ys) - PAD
        x1, y1 = max(xs) + PAD, max(ys) + PAD
        fgx = np.arange(x0 - 0.6, x1 + 0.6, FINE)
        fgy = np.arange(y0 - 0.6, y1 + 0.6, FINE)
        if len(band):
            FX, FY = np.meshgrid(fgx, fgy, indexing="ij")
            d, _ = cKDTree(band).query(np.column_stack([FX.ravel(), FY.ravel()]))
            sdf_grid = d.reshape(len(fgx), len(fgy))
        else:
            sdf_grid = np.full((len(fgx), len(fgy)), np.inf)

        states = se2_search(
            fgx, fgy, sdf_grid, (x0, y0, x1, y1), pose, goal, self._emb, self._emb.precision
        )
        if states is None:
            return Path(frame_id="world", poses=[pose_stamped(*pose)])
        dense = densify_states(states, self._res)
        return Path(frame_id="world", poses=[pose_stamped(x, y, yaw) for x, y, yaw in dense])


class RustTargetEpisode:
    """The dimos_motion2_target extension behind the episode protocol."""

    def __init__(self, emb: Embodiment, resolution: float) -> None:
        import dimos_motion2_target

        self._mod = dimos_motion2_target
        self._emb = emb
        self._res = resolution

    def reset(self) -> None:
        pass

    def plan(
        self, cloud: PointCloud2, pose: tuple[float, float, float], goal: tuple[float, float]
    ) -> Path:
        pts, _ = cloud.as_numpy()
        pts = np.ascontiguousarray(np.asarray(pts, dtype=np.float64).reshape(-1, 3))
        e = self._emb
        out = self._mod.plan(
            pts,
            (float(pose[0]), float(pose[1]), float(pose[2])),
            (float(goal[0]), float(goal[1])),
            (e.length, e.width, e.center_off, e.comfort, e.precision, e.strafe, e.reverse, e.yaw_w),
            self._res,
        )
        if out is None or not len(out):
            return Path(frame_id="world", poses=[pose_stamped(*pose)])
        return Path(frame_id="world", poses=[pose_stamped(x, y, yaw) for x, y, yaw in out])


def make_py(sc: Scenario, cfg: AvoidanceConfig | None = None, **_: Any) -> TargetEpisode:
    res = (cfg or AvoidanceConfig()).resolution
    return TargetEpisode(sc.emb, res)


def make(sc: Scenario, cfg: AvoidanceConfig | None = None, **_: Any) -> RustTargetEpisode:
    try:
        import dimos_motion2_target  # noqa: F401
    except ImportError as e:
        raise ImportError(f"dimos_motion2_target is not built; run: {BUILD_CMD}") from e
    res = (cfg or AvoidanceConfig()).resolution
    return RustTargetEpisode(sc.emb, res)
