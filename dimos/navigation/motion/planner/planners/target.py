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

"""The shipped planner: an SE(2) search whose world model is built from the
cloud rather than from any ground truth.

Two factories: `make_py` ("target-py") is the python port spec, and `make`
("target") adapts the rust crate (dimos_motion2_target) — same algorithm, and
the crate is what runs on a robot.
"""

from __future__ import annotations

from typing import Any

import numpy as np

from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.nav_msgs.Path import Path
from dimos.navigation.motion.embodiment import Embodiment

from .base import RESOLUTION, densify_states, pose_stamped, states_of
from .se2 import COMMIT_MARGIN, PERIOD, SdfGrid, anchor, se2_search

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
        pose: Pose,
        goal: Pose,
        incumbent: Path | None = None,
    ) -> Path:
        band = np.asarray(obstacles, dtype=float).reshape(-1, 2)

        xs = [pose.x, goal.x] + ([] if not len(band) else [band[:, 0].min(), band[:, 0].max()])
        ys = [pose.y, goal.y] + ([] if not len(band) else [band[:, 1].min(), band[:, 1].max()])
        # Anchored on the world frame's own lattice, exactly as se2_path is: a
        # return past the cloud's low corner may add rows, never move a sample.
        x0, y0 = anchor(min(xs) - PAD), anchor(min(ys) - PAD)
        x1, y1 = max(xs) + PAD, max(ys) + PAD
        grid = SdfGrid.from_obstacles(
            (x0 - GRID_PAD, y0 - GRID_PAD, x1 + GRID_PAD, y1 + GRID_PAD), band
        )

        states = se2_search(
            grid,
            (x0, y0, x1, y1),
            (pose.x, pose.y, pose.yaw),
            (goal.x, goal.y),
            self._emb,
            self._emb.precision,
            incumbent=states_of(incumbent),
        )
        if states is None:
            return Path(ts=0.0, frame_id="world", poses=[pose_stamped(pose.x, pose.y, pose.yaw)])
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
        pose: Pose,
        goal: Pose,
        incumbent: Path | None = None,
    ) -> Path:
        pts = np.ascontiguousarray(np.asarray(obstacles, dtype=np.float64).reshape(-1, 2))
        inc = states_of(incumbent)
        e = self._emb
        out = self._mod.plan(
            pts,
            (pose.x, pose.y, pose.yaw),
            (goal.x, goal.y),
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
                (e.max_speed, e.min_speed, e.speed_clearance, e.max_yaw_rate),
            ),
            self._res,
            None if inc is None else np.ascontiguousarray(inc, dtype=np.float64),
            # One copy of the constant, crossing the boundary the way the
            # envelope does: python owns it, the crate is handed it.
            COMMIT_MARGIN,
        )
        if out is None or not len(out):
            return Path(ts=0.0, frame_id="world", poses=[pose_stamped(pose.x, pose.y, pose.yaw)])
        return Path(ts=0.0, frame_id="world", poses=[pose_stamped(x, y, yaw) for x, y, yaw in out])


def make_py(emb: Embodiment, resolution: float = RESOLUTION, **_: Any) -> TargetEpisode:
    return TargetEpisode(emb, resolution)


def make(emb: Embodiment, resolution: float = RESOLUTION, **_: Any) -> RustTargetEpisode:
    try:
        import dimos_motion2_target  # noqa: F401
    except ImportError as e:
        raise ImportError(f"dimos_motion2_target is not built; run: {BUILD_CMD}") from e
    return RustTargetEpisode(emb, resolution)
