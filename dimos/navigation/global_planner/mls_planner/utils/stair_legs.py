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

"""Plan between two frames of a replay on its final map, once per stairway leg.

Reads the `voxels.f32` a `replay_bench --dump` run writes, the frames it
replayed and its configs. Each leg plans from the ground under one frame's pose
to another's and is scored against the distance the robot walked between them.
A leg that finds no path or runs past the configs' detour ratio fails.
`--planner KEY=VALUE` overrides a planner setting, so a lower wall penalty
can tell a stairway the planner avoids from one it cannot reach.

Usage:
    uv run python -m dimos.navigation.global_planner.mls_planner.utils.stair_legs \\
        dump_dir china.frames configs.json --leg "A up:90:260" --leg "A down:260:90"
"""

from __future__ import annotations

import inspect
import json
from pathlib import Path

import numpy as np
from numpy.typing import NDArray
import typer

from dimos.navigation.global_planner.mls_planner.mls_planner import MLSPlanner
from dimos.navigation.global_planner.mls_planner.utils.export_frames import FRAME_HEADER
from dimos.navigation.global_planner.mls_planner.utils.surface_error import SURFACE_STRIDE


def sensor_positions(path: Path) -> NDArray[np.float64]:
    """Each exported frame's sensor position."""
    positions = []
    with path.open("rb") as f:
        while header := f.read(FRAME_HEADER.size):
            _stamp, px, py, pz, *_rotation, n = FRAME_HEADER.unpack(header)
            f.seek(12 * n, 1)
            positions.append((px, py, pz))
    return np.array(positions)


def main(
    dump: Path = typer.Argument(..., help="A replay_bench --dump directory"),
    frames: Path = typer.Argument(..., help="The frames the dump's run replayed"),
    configs: Path = typer.Argument(..., help="The configs the dump's run used"),
    leg: list[str] = typer.Option(..., help="NAME:FROM:TO, frame indices of the leg's ends"),
    planner_setting: list[str] = typer.Option(
        [], "--planner", help="KEY=VALUE overriding a planner setting, VALUE as JSON"
    ),
) -> None:
    settings = json.loads(configs.read_text())
    accepted = inspect.signature(MLSPlanner).parameters
    kwargs = {k: v for k, v in settings["planner"].items() if k in accepted}
    kwargs |= {k: json.loads(v) for k, v in (s.split("=", 1) for s in planner_setting)}
    planner = MLSPlanner(**kwargs)
    voxels = np.fromfile(dump / "voxels.f32", dtype="<f4").reshape(-1, SURFACE_STRIDE)
    planner.update_global_map(voxels)
    ground = sensor_positions(frames) - [0.0, 0.0, settings["sensor_height_m"]]

    failed = 0
    for spec in leg:
        name, start, end = spec.rsplit(":", 2)
        a, b = int(start), int(end)
        if a == b:
            raise typer.BadParameter(f"{spec}: a leg needs two different frames")
        walked = ground[min(a, b) : max(a, b) + 1]
        trajectory = float(np.linalg.norm(np.diff(walked, axis=0), axis=1).sum())
        path = planner.plan(tuple(map(float, ground[a])), tuple(map(float, ground[b])))
        if path is None:
            failed += 1
            print(f"{name}: FAIL no path, trajectory {trajectory:.1f} m")
            continue
        length = float(np.linalg.norm(np.diff(path, axis=0), axis=1).sum())
        ratio = length / trajectory
        ok = ratio <= settings["detour_ratio"]
        failed += not ok
        print(
            f"{name}: {'ok' if ok else 'FAIL'} path {length:.1f} m vs trajectory {trajectory:.1f} m "
            f"({ratio:.2f}x), z {path[0, 2]:.2f} -> {path[-1, 2]:.2f}, "
            f"middle at {np.round(path[len(path) // 2, :2], 1).tolist()}"
        )
    raise typer.Exit(1 if failed else 0)


if __name__ == "__main__":
    typer.run(main)
