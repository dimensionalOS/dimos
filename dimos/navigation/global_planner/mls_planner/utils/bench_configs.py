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

"""Write the Go2 nav_3d ray tracer and planner configs for the native replay bench.

Usage:
    uv run python -m dimos.navigation.global_planner.mls_planner.utils.bench_configs out.json
"""

from __future__ import annotations

import json
from pathlib import Path

import typer

from dimos.robot.unitree.go2.constants import BASE_LINK_HEIGHT
from dimos.robot.unitree.go2.go2_mid360_static_transforms import FRAMES
from dimos.robot.unitree.go2.nav_3d_config import mls_planner_config, ray_tracing_config


def main(
    out: Path,
    goal_spacing_m: float = typer.Option(2.0, help="Trajectory distance between goals"),
    detour_ratio: float = typer.Option(
        1.5, help="A leg path longer than this times its trajectory counts as a detour"
    ),
    stair_corridor_m: float = typer.Option(
        0.4, help="Half width of the corridor along a climb whose cells are scored"
    ),
) -> None:
    # Walk the lidar's mount chain up to base_link. No link above the lidar is rotated.
    parents = {name: (parent, xyz) for name, parent, xyz, _rpy in FRAMES}
    mount_z, frame = 0.0, "mid360_link"
    while (link := parents[frame])[0] is not None:
        mount_z += link[1][2]
        frame = link[0]
    configs = {
        "tracer": ray_tracing_config.to_config_dict(),
        "planner": mls_planner_config.to_config_dict(),
        "sensor_height_m": BASE_LINK_HEIGHT + mount_z,
        "goal_spacing_m": goal_spacing_m,
        "detour_ratio": detour_ratio,
        "stair_corridor_m": stair_corridor_m,
    }
    out.write_text(json.dumps(configs, indent=2))
    print(f"sensor {configs['sensor_height_m']:.3f} m above ground -> {out}")


if __name__ == "__main__":
    typer.run(main)
