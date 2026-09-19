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

"""The shipped planner: SE(2) search over the cloud. `make_py` is the python
spec, `make` the rust crate (dimos_local_planner) that runs on the robot."""

from __future__ import annotations

from typing import Any

import numpy as np
from numpy.typing import NDArray

from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.nav_msgs.Path import Path
from dimos.navigation.embodiment.base import Embodiment

from .base import RESOLUTION, densify_states, pose_stamped, states_of
from .se2 import COMMIT_MARGIN, VOXEL, SdfGrid, anchor, pitches, se2_search

PAD = 1.5

BUILD_CMD = "uv run maturin develop --uv --release --features python -m dimos/navigation/local_planner/rust/Cargo.toml"


class TargetEpisode:
    def __init__(
        self, emb: Embodiment, resolution: float, pointcloud_resolution: float = VOXEL
    ) -> None:
        self._emb = emb
        self._res = resolution
        self._fine, self._cell, self._period = pitches(pointcloud_resolution)

    def reset(self) -> None:
        pass

    def plan(
        self,
        obstacles: NDArray[np.floating[Any]],
        pose: Pose,
        goal: Pose,
        incumbent: Path | None = None,
        ground: NDArray[np.floating[Any]] | None = None,
        unseen_cost: float = 1.0,
    ) -> Path:
        # ponytail: the python port has no explored layer yet; the crate is
        # what ships. Port se2.py when parity on it is wanted.
        band = np.asarray(obstacles, dtype=float).reshape(-1, 2)

        xs = [pose.x, goal.x] + ([] if not len(band) else [band[:, 0].min(), band[:, 0].max()])
        ys = [pose.y, goal.y] + ([] if not len(band) else [band[:, 1].min(), band[:, 1].max()])
        # Anchored on the world lattice: a new return may add rows, never move a sample.
        x0, y0 = anchor(min(xs) - PAD, self._period), anchor(min(ys) - PAD, self._period)
        x1, y1 = max(xs) + PAD, max(ys) + PAD
        pad = 3 * self._period  # free space around the working area
        grid = SdfGrid.from_obstacles(
            (x0 - pad, y0 - pad, x1 + pad, y1 + pad), band, pitch=self._fine
        )

        states = se2_search(
            grid,
            (x0, y0, x1, y1),
            (pose.x, pose.y, pose.yaw),
            (goal.x, goal.y),
            self._emb,
            self._emb.precision,
            cell=self._cell,
            incumbent=states_of(incumbent),
        )
        if states is None:
            return Path(ts=0.0, frame_id="world", poses=[pose_stamped(pose.x, pose.y, pose.yaw)])
        dense = densify_states(states, self._res)
        return Path(
            ts=0.0, frame_id="world", poses=[pose_stamped(x, y, yaw) for x, y, yaw in dense]
        )


class RustTargetEpisode:
    """The dimos_local_planner extension behind the episode protocol."""

    def __init__(
        self, emb: Embodiment, resolution: float, pointcloud_resolution: float = VOXEL
    ) -> None:
        import dimos_local_planner

        self._mod = dimos_local_planner
        # the body crosses as the same dict the native modules are configured with
        self._emb = emb.to_json()
        self._res = resolution
        self._voxel = pointcloud_resolution

    def reset(self) -> None:
        pass

    def plan(
        self,
        obstacles: NDArray[np.floating[Any]],
        pose: Pose,
        goal: Pose,
        incumbent: Path | None = None,
        ground: NDArray[np.floating[Any]] | None = None,
        unseen_cost: float = 1.0,
    ) -> Path:
        pts = np.ascontiguousarray(np.asarray(obstacles, dtype=np.float64).reshape(-1, 2))
        inc = states_of(incumbent)
        out = self._mod.plan(
            pts,
            (pose.x, pose.y, pose.yaw),
            (goal.x, goal.y),
            self._emb,
            self._res,
            None if inc is None else np.ascontiguousarray(inc, dtype=np.float64),
            COMMIT_MARGIN,
            None
            if ground is None
            else np.ascontiguousarray(np.asarray(ground, dtype=np.float64).reshape(-1, 2)),
            unseen_cost,
            self._voxel,
        )
        if out is None or not len(out):
            return Path(ts=0.0, frame_id="world", poses=[pose_stamped(pose.x, pose.y, pose.yaw)])
        return Path(ts=0.0, frame_id="world", poses=[pose_stamped(x, y, yaw) for x, y, yaw in out])


def make_py(
    emb: Embodiment,
    resolution: float = RESOLUTION,
    pointcloud_resolution: float = VOXEL,
    **_: Any,
) -> TargetEpisode:
    return TargetEpisode(emb, resolution, pointcloud_resolution)


def make(
    emb: Embodiment,
    resolution: float = RESOLUTION,
    pointcloud_resolution: float = VOXEL,
    **_: Any,
) -> RustTargetEpisode:
    try:
        import dimos_local_planner  # noqa: F401
    except ImportError as e:
        raise ImportError(f"dimos_local_planner is not built; run: {BUILD_CMD}") from e
    return RustTargetEpisode(emb, resolution, pointcloud_resolution)
