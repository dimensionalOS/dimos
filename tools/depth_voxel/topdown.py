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

"""Side-by-side top-down views of saved voxel-key maps, colored by height."""

from __future__ import annotations

from pathlib import Path
import sys

import matplotlib

matplotlib.use("Agg")
from matplotlib import pyplot
import numpy as np

from dimos.memory.cli.dataset import open_store
from tools.depth_voxel.maps import VOXEL_SIZE_METERS, shared_window
from tools.depth_voxel.pipeline import PoseTrack
from tools.depth_voxel.render import RENDER_CLIP_METERS


def odometry_path(db_path: Path) -> np.ndarray:
    """Point-lio ``odom -> mid360_link`` xy track over the window both maps were built on."""
    store = open_store(str(db_path))
    with store:
        start, end = shared_window(store)
        poses = PoseTrack.from_store(store)
    inside = (poses.timestamps >= start) & (poses.timestamps <= end)
    return poses.positions[inside, :2]


TOPDOWN_MAX_HEIGHT_METERS = 1.5
"""Ceiling returns sit above everything and hide the floor/wall structure from directly
above, so the top-down view drops them and rescales the colormap to what is left."""

TOPDOWN_CLIP_METERS = (
    RENDER_CLIP_METERS[0],
    RENDER_CLIP_METERS[1],
    (RENDER_CLIP_METERS[2][0], TOPDOWN_MAX_HEIGHT_METERS),
)


def column_heights(path: Path) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    centers = (np.load(path).astype(np.float64) + 0.5) * VOXEL_SIZE_METERS
    inside = np.ones(len(centers), dtype=bool)
    for axis, (low, high) in enumerate(TOPDOWN_CLIP_METERS):
        inside &= (centers[:, axis] >= low) & (centers[:, axis] <= high)
    centers = centers[inside]
    order = np.argsort(centers[:, 2])
    centers = centers[order]
    return centers[:, 0], centers[:, 1], centers[:, 2]


def main(out_path: str, db_path: str, *labelled_paths: str) -> None:
    panels = [entry.split("=", 1) for entry in labelled_paths]
    (low_z, high_z) = TOPDOWN_CLIP_METERS[2]
    clouds = [column_heights(Path(key_path)) for _, key_path in panels]
    track = odometry_path(Path(db_path))
    margin_meters = 1.0
    x_limits = (
        min(cloud[0].min() for cloud in clouds) - margin_meters,
        max(cloud[0].max() for cloud in clouds) + margin_meters,
    )
    y_limits = (
        min(cloud[1].min() for cloud in clouds) - margin_meters,
        max(cloud[1].max() for cloud in clouds) + margin_meters,
    )
    figure, axes = pyplot.subplots(1, len(panels), figsize=(8 * len(panels), 8), facecolor="white")
    for axis, (label, _), (x, y, z) in zip(np.atleast_1d(axes), panels, clouds, strict=True):
        scatter = axis.scatter(
            x, y, c=z, s=1.2, cmap="turbo", vmin=low_z, vmax=high_z, linewidths=0
        )
        axis.plot(track[:, 0], track[:, 1], color="black", linewidth=1.6, label="point-lio odom")
        axis.plot(track[0, 0], track[0, 1], "o", color="black", markersize=6)
        axis.legend(loc="upper right")
        axis.set_title(f"{label}  ({len(x)} voxels shown)")
        axis.set_xlabel("x (m)")
        axis.set_ylabel("y (m)")
        axis.set_xlim(*x_limits)
        axis.set_ylim(*y_limits)
        axis.set_aspect("equal")
        axis.grid(alpha=0.2)
        figure.colorbar(scatter, ax=axis, label="height (m)", shrink=0.8)
    figure.tight_layout()
    figure.savefig(out_path, dpi=140)
    print(f"wrote {out_path}")


if __name__ == "__main__":
    main(sys.argv[1], *sys.argv[2:])
