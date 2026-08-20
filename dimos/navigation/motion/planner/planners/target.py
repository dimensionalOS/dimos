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

from dimos.msgs.nav_msgs.Path import Path
from dimos.navigation.motion.embodiment import Embodiment
from dimos.navigation.motion.geometry import AvoidanceConfig
from dimos.navigation.motion.scenarios import (
    COMMIT_MARGIN,
    FINE,
    PERIOD,
    Scenario,
    anchor,
    se2_search,
)

from .gold import densify_states, pose_stamped, states_of

PAD = 1.5
# Free space around the working area, in whole periods -- se2_path's own.
GRID_PAD = 3 * PERIOD

BUILD_CMD = (
    "uv run maturin develop --uv --release -m dimos/navigation/motion/planner/rust/Cargo.toml"
)


class TargetEpisode:
    def __init__(self, emb: Embodiment, resolution: float) -> None:
        self._emb = emb
        self._res = resolution

    def reset(self) -> None:
        pass

    def plan(
        self,
        obstacles: np.ndarray,
        pose: tuple[float, float, float],
        goal: tuple[float, float],
        incumbent: Path | None = None,
    ) -> Path:
        from scipy.spatial import cKDTree

        band = np.asarray(obstacles, dtype=float).reshape(-1, 2)

        xs = [pose[0], goal[0]] + ([] if not len(band) else [band[:, 0].min(), band[:, 0].max()])
        ys = [pose[1], goal[1]] + ([] if not len(band) else [band[:, 1].min(), band[:, 1].max()])
        # Anchored on the world frame's own lattice, exactly as se2_path is: a
        # return past the cloud's low corner may add rows, never move a sample.
        x0, y0 = anchor(min(xs) - PAD), anchor(min(ys) - PAD)
        x1, y1 = max(xs) + PAD, max(ys) + PAD
        fgx = np.arange(x0 - GRID_PAD, x1 + GRID_PAD, FINE)
        fgy = np.arange(y0 - GRID_PAD, y1 + GRID_PAD, FINE)
        if len(band):
            FX, FY = np.meshgrid(fgx, fgy, indexing="ij")
            d, _ = cKDTree(band).query(np.column_stack([FX.ravel(), FY.ravel()]))
            sdf_grid = d.reshape(len(fgx), len(fgy))
        else:
            sdf_grid = np.full((len(fgx), len(fgy)), np.inf)

        states = se2_search(
            fgx,
            fgy,
            sdf_grid,
            (x0, y0, x1, y1),
            pose,
            goal,
            self._emb,
            self._emb.precision,
            incumbent=states_of(incumbent),
        )
        if states is None:
            return Path(ts=0.0, frame_id="world", poses=[pose_stamped(*pose)])
        dense = densify_states(states, self._res)
        return Path(
            ts=0.0, frame_id="world", poses=[pose_stamped(x, y, yaw) for x, y, yaw in dense]
        )


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
        self,
        obstacles: np.ndarray,
        pose: tuple[float, float, float],
        goal: tuple[float, float],
        incumbent: Path | None = None,
    ) -> Path:
        pts = np.ascontiguousarray(np.asarray(obstacles, dtype=np.float64).reshape(-1, 2))
        inc = states_of(incumbent)
        e = self._emb
        out = self._mod.plan(
            pts,
            (float(pose[0]), float(pose[1]), float(pose[2])),
            (float(goal[0]), float(goal[1])),
            (
                e.length,
                e.width,
                e.center_off,
                e.comfort,
                e.precision,
                e.strafe,
                e.reverse,
                e.yaw_w,
                e.envelope,
                e.arc_inflate,
            ),
            self._res,
            None if inc is None else np.ascontiguousarray(inc, dtype=np.float64),
            # One copy of the constant, crossing the boundary the way the
            # envelope does: python owns it, the crate is handed it.
            COMMIT_MARGIN,
        )
        if out is None or not len(out):
            return Path(ts=0.0, frame_id="world", poses=[pose_stamped(*pose)])
        return Path(ts=0.0, frame_id="world", poses=[pose_stamped(x, y, yaw) for x, y, yaw in out])


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
