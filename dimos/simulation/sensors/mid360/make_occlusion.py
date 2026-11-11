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


from __future__ import annotations

import argparse
from pathlib import Path

import mujoco
import numpy as np
from numpy.typing import NDArray

_SELF_RANGE = 1.2
_SAMPLES = 16


def occlusion_grid(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    site: str,
    *,
    az_edges: NDArray[np.float64],
    el_edges: NDArray[np.float64],
    samples: int = _SAMPLES,
    seed: int = 0,
) -> NDArray[np.float32]:
    index = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, site)
    if index < 0:
        raise ValueError(f"the model has no site named {site!r}")
    origin = data.site_xpos[index].copy()
    world_from_sensor = data.site_xmat[index].reshape(3, 3)
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


def write_occlusion(
    path: Path,
    grid: NDArray[np.float32],
    az_edges: NDArray[np.float64],
    el_edges: NDArray[np.float64],
) -> None:
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
    az_edges = np.arange(-180.0, 181.0, dtype=np.float64)
    el_edges = np.arange(args.el_from, args.el_to + 1.0, dtype=np.float64)
    grid = occlusion_grid(model, data, args.frame, az_edges=az_edges, el_edges=el_edges)
    write_occlusion(args.out, grid, az_edges, el_edges)
    print(
        f"{args.out}: {grid.shape[0]}x{grid.shape[1]} cells, "
        f"{float((grid > 0.5).mean()) * 100:.1f}% blocked, "
        f"{float((grid > 0).mean() - (grid > 0.99).mean()) * 100:.1f}% partial"
    )


if __name__ == "__main__":
    main()
