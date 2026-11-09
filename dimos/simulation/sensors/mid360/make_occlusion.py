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

"""Measure which directions a robot's own body blocks, as the OcclusionMap a SimLidar reads.

The cast itself cannot answer this: a lidar sits inside its mount's collision hull rather than
proud of its surface, and a hull is a few convex shapes around a shape that is neither -- the
G1's Mid-360 is inside the head's, so a cast against colliders says "blocked" in every
direction including straight up. The G1's head leaves a gap on each side; the hull does not.

So the mount is left out of the cast (``MujocoView.raycaster(mounted_at=...)``) and what it
really blocks is measured here against the robot's *visual* meshes, which are the shape of the
actual shell. The result is a probability per one-degree cell.

    python -m dimos.simulation.sensors.mid360.make_occlusion \\
        --robot g1 --frame mid360_link --out data/g1_sim/g1_mid360_occlusion.npz
"""

from __future__ import annotations

import argparse
from pathlib import Path
from typing import Any

import mujoco
import numpy as np
from numpy.typing import NDArray

# How far a hit has to be to count as the robot's own body rather than the world. The robot is
# a metre tall and a sensor on it is centimetres from its shell, so anything past this is room.
_SELF_RANGE = 1.2
# Samples per cell, jittered inside it: a cell on the edge of the shell is blocked some of the
# time, and that fraction is the number the lidar wants.
_SAMPLES = 16


def occlusion_grid(
    model: Any,
    data: Any,
    site: str,
    *,
    az_edges: NDArray[np.float64],
    el_edges: NDArray[np.float64],
    samples: int = _SAMPLES,
    seed: int = 0,
) -> NDArray[np.float32]:
    """The fraction of each cell's rays the robot's own visual geometry stops."""
    index = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, site)
    if index < 0:
        raise ValueError(f"the model has no site named {site!r}")
    origin = data.site_xpos[index].copy()
    world_from_sensor = data.site_xmat[index].reshape(3, 3)
    # Group 2 is where a robot's visual meshes sit: the shell as it is, not its convex hull.
    mask = np.zeros(mujoco.mjNGROUP, np.uint8)
    mask[2] = 1

    rng = np.random.default_rng(seed)
    rows, cols = len(el_edges) - 1, len(az_edges) - 1
    grid = np.zeros((rows, cols), np.float32)
    for row in range(rows):
        el = np.radians(
            el_edges[row] + rng.random(cols * samples) * (el_edges[row + 1] - el_edges[row])
        )
        az = np.radians(
            np.repeat(az_edges[:-1], samples)
            + rng.random(cols * samples) * (az_edges[1] - az_edges[0])
        )
        # In the sensor's own frame, the way the pattern and the map are both written.
        sensor_dirs = np.column_stack(
            [np.cos(el) * np.cos(az), np.cos(el) * np.sin(az), np.sin(el)]
        )
        world_dirs = sensor_dirs @ world_from_sensor.T
        n = len(world_dirs)
        dist = np.full(n, -1.0)
        geom = np.full(n, -1, np.int32)
        normals = np.zeros(3 * n)
        mujoco.mj_multiRay(
            model,
            data,
            origin,
            np.ascontiguousarray(world_dirs).ravel(),
            mask,
            1,
            -1,
            geom,
            dist,
            normals,
            n,
            _SELF_RANGE,
        )
        blocked = (geom >= 0) & (dist > 0)
        grid[row] = blocked.reshape(cols, samples).mean(axis=1)
    return grid


def write_occlusion(path: Path, grid: NDArray[np.float32], az_edges, el_edges) -> None:
    np.savez_compressed(
        path,
        probability=grid.astype(np.float32),
        az_edges=np.asarray(az_edges, np.float32),
        el_edges=np.asarray(el_edges, np.float32),
    )


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mjcf", required=True, help="the robot's MJCF, alone (no scene)")
    parser.add_argument("--frame", required=True, help="the sensor's site, e.g. mid360_link")
    parser.add_argument("--out", required=True, type=Path)
    parser.add_argument("--el-from", type=float, default=-8.0)
    parser.add_argument("--el-to", type=float, default=56.0)
    args = parser.parse_args()

    model = mujoco.MjModel.from_xml_path(args.mjcf)
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    az_edges = np.arange(-180.0, 181.0)
    el_edges = np.arange(args.el_from, args.el_to + 1.0)
    grid = occlusion_grid(model, data, args.frame, az_edges=az_edges, el_edges=el_edges)
    write_occlusion(args.out, grid, az_edges, el_edges)
    print(
        f"{args.out}: {grid.shape[0]}x{grid.shape[1]} cells, "
        f"{float((grid > 0.5).mean()) * 100:.1f}% blocked, "
        f"{float((grid > 0).mean() - (grid > 0.99).mean()) * 100:.1f}% partial"
    )


if __name__ == "__main__":
    main()
