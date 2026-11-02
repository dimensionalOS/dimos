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

"""Score the ray tracer's plane heights against the lidar returns they came from.

Reads the `voxels.f32` a `replay_bench --dump` run writes and the frames it
replayed. Each level voxel with a plane is scored: the truth is the median z of
every registered return in it and the voxel right below it, the pair the
tracer fits, against the plane's height at its column center and against its
top face, the whole-voxel model. The returns are the truth because they are what the map
was built from; the recording's separate ground-truth cloud comes from another
trajectory and sits centimeters off this one.

Usage:
    uv run python -m dimos.navigation.global_planner.mls_planner.utils.surface_error \\
        dump_dir china.frames
"""

from __future__ import annotations

from collections.abc import Iterator
import json
from pathlib import Path

import numpy as np
from numpy.typing import NDArray
from scipy.spatial.transform import Rotation
import typer

from dimos.navigation.global_planner.mls_planner.utils.export_frames import FRAME_HEADER

# Floats per dumped voxel: center xyz, plane normal xyz, plane offset.
SURFACE_STRIDE = 7


def registered_frames(path: Path) -> Iterator[NDArray[np.float32]]:
    """Each exported frame's returns, placed in the world by the frame's pose."""
    with path.open("rb") as f:
        while header := f.read(FRAME_HEADER.size):
            _stamp, px, py, pz, qx, qy, qz, qw, n = FRAME_HEADER.unpack(header)
            points = np.frombuffer(f.read(12 * n), dtype="<f4").reshape(-1, 3)
            origin = np.array([px, py, pz])
            world = Rotation.from_quat([qx, qy, qz, qw]).apply(points) + origin
            yield world.astype(np.float32)


def dump_voxel_size(dump: Path) -> float:
    """The voxel size a `replay_bench --dump` run mapped at."""
    return float(json.loads((dump / "meta.json").read_text())["voxel_size"])


def voxel_keys(xyz: NDArray[np.float32], voxel_size: float) -> NDArray[np.int64]:
    """One int64 per voxel, 21 bits an axis, for sorting and matching."""
    ijk = np.floor(xyz / voxel_size).astype(np.int64) & 0x1FFFFF
    return (ijk[:, 0] << 42) | (ijk[:, 1] << 21) | ijk[:, 2]


def plane_heights(voxels: NDArray[np.float32], voxel_size: float) -> NDArray[np.float32]:
    """Each dumped voxel's plane height at its column center, held within a voxel of its bottom face as the planner holds it."""
    bottom = voxels[:, 2] - 0.5 * voxel_size
    local = np.clip(0.5 * voxel_size + voxels[:, 6] / voxels[:, 5], -voxel_size, voxel_size)
    return (bottom + local).astype(np.float32)


def main(
    dump: Path,
    truth: Path = typer.Argument(..., help="The frames the dump's run replayed"),
    max_tilt_deg: float = typer.Option(10.0, help="Steepest plane scored as level ground"),
    min_truth_points: int = typer.Option(5, help="Fewest truth points a voxel needs"),
) -> None:
    voxel_size = dump_voxel_size(dump)
    voxels = np.fromfile(dump / "voxels.f32", dtype="<f4").reshape(-1, SURFACE_STRIDE)
    normal_z, offset = voxels[:, 5], voxels[:, 6]
    level = np.isfinite(offset) & (normal_z >= np.cos(np.radians(max_tilt_deg)))
    voxels = voxels[level]
    plane_z = plane_heights(voxels, voxel_size)
    top_z = voxels[:, 2] + 0.5 * voxel_size

    # Every return in a scored voxel or the one below it, as (voxel, z).
    keys = voxel_keys(voxels[:, :3], voxel_size)
    order = np.argsort(keys)
    sorted_keys = keys[order]
    hits: list[tuple[NDArray[np.int64], NDArray[np.float32]]] = []
    for world in registered_frames(truth):
        for dz in (-1, 0):
            shifted = world - (0.0, 0.0, dz * voxel_size)
            want = voxel_keys(shifted, voxel_size)
            at = np.minimum(np.searchsorted(sorted_keys, want), len(sorted_keys) - 1)
            found = sorted_keys[at] == want
            hits.append((order[at[found]], world[found, 2]))
    voxel = np.concatenate([h[0] for h in hits])
    z = np.concatenate([h[1] for h in hits])
    by_voxel = np.lexsort((z, voxel))
    voxel, z = voxel[by_voxel], z[by_voxel]
    starts = np.flatnonzero(np.r_[True, voxel[1:] != voxel[:-1]])
    counts = np.diff(np.r_[starts, len(voxel)])
    keep = counts >= min_truth_points
    starts, counts = starts[keep], counts[keep]
    truth_height = z[starts + counts // 2]
    scored = voxel[starts]
    plane = np.abs(plane_z[scored] - truth_height)
    top = np.abs(top_z[scored] - truth_height)
    print(
        json.dumps(
            {
                "voxels_scored": len(plane),
                "plane_median_m": float(np.median(plane)),
                "plane_p90_m": float(np.quantile(plane, 0.9)),
                "whole_voxel_median_m": float(np.median(top)),
                "whole_voxel_p90_m": float(np.quantile(top, 0.9)),
            },
            indent=2,
        )
    )


if __name__ == "__main__":
    typer.run(main)
