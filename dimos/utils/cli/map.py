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

from collections.abc import Callable
import time
from typing import Any

import rerun as rr
import rerun.blueprint as rrb
import typer

from dimos.mapping.voxels import VoxelMapTransformer
from dimos.memory2.store.sqlite import SqliteStore
from dimos.memory2.type.observation import Observation
from dimos.utils.data import resolve_named_path
from dimos.visualization.rerun.init import rerun_init


def progress(total: int, label: str = "") -> Callable[[Observation[Any]], None]:
    seen = 0
    wall_start: float | None = None
    last_wall: float | None = None
    first_ts: float | None = None

    def _progress(obs: Observation[Any]) -> None:
        nonlocal seen, wall_start, last_wall, first_ts
        now = time.monotonic()
        if wall_start is None:
            wall_start = now
            first_ts = obs.ts
        assert first_ts is not None  # narrowed by the same `if` above
        frame_ms = (now - last_wall) * 1000 if last_wall is not None else 0.0
        last_wall = now
        seen += 1
        pct = 100 * seen // total if total else 100
        wall = now - wall_start
        data = obs.ts - first_ts
        speed = data / wall if wall > 0 else 0.0
        end = "\n" if seen >= total else ""
        prefix = f"{label} " if label else ""
        print(
            f"\r{prefix}{pct:>3}% [{seen}/{total}] {data:.1f}s ({speed:.1f} x rt) {frame_ms:.0f}ms/frame",
            end=end,
            flush=True,
        )

    return _progress


def main(
    dataset: str = typer.Argument(..., help="Dataset .db: bare name (cwd or data/) or path"),
    voxel: float = typer.Option(0.05, "--voxel", help="Voxel size for the rebuild"),
    device: str = typer.Option(
        "CUDA:0", "--device", help="Open3D compute device (e.g. CUDA:0, CPU:0)"
    ),
    pgo: bool = typer.Option(
        False, "--pgo", help="Run pose graph optimization before rebuilding (twopass)"
    ),
    block_count: int = typer.Option(
        2_000_000, "--block-count", help="VoxelBlockGrid capacity (--pgo only)"
    ),
    export: bool = typer.Option(
        False,
        "--export",
        help="Export PGO twopass map to ./<dataset>.pc2.lcm in cwd (implies --pgo)",
    ),
    no_gui: bool = typer.Option(False, "--no-gui", help="Skip rerun visualization"),
) -> None:
    db_path = resolve_named_path(dataset, ".db")
    if export:
        pgo = True

    store = SqliteStore(path=db_path)
    lidar = store.streams.lidar

    print(lidar.summary())

    path: list[tuple[float, float, float]] = []

    def collect_path(obs: Observation[Any]) -> None:
        if obs.pose is None:
            return
        # Reject placeholder poses at the world origin (translation = 0,0,0).
        if obs.pose[0] == 0 and obs.pose[1] == 0 and obs.pose[2] == 0:
            return
        path.append((obs.pose[0], obs.pose[1], obs.pose[2]))

    pgo_map = None
    pgo_path: list[tuple[float, float, float]] = []
    if pgo:
        from dimos.mapping.relocalization.pgo import pgo_then_voxels

        total = lidar.count()
        print("running PGO twopass map...")
        pgo_map = pgo_then_voxels(
            lidar,
            voxel_size=voxel,
            block_count=block_count,
            device=device,
            corrected_path_out=pgo_path,
            pass1_on_frame=progress(total, "pgo pass 1 (optimizing)"),
            pass2_on_frame=progress(total, "pgo pass 2 (rebuilding)"),
        )

    global_map = (
        lidar.tap(collect_path)
        .transform(VoxelMapTransformer(voxel_size=voxel, device=device))
        .tap(progress(lidar.count(), "reconstructing global map"))
        .last()
        .data
    )

    if not no_gui:
        rerun_init("dimos map tool", spawn=True)
        rr.send_blueprint(rrb.Blueprint(rrb.Spatial3DView(origin="world")))
        rr.log("world/raw_map/pointcloud", global_map.to_rerun(size=voxel), static=True)
        if path:
            rr.log(
                "world/raw_map/path",
                rr.LineStrips3D(strips=[path], colors=[[231, 76, 60]], radii=[0.05]),
                static=True,
            )
        if pgo_map is not None:
            rr.log("world/pgo_map/pointcloud", pgo_map.to_rerun(size=voxel), static=True)
        if pgo_path:
            rr.log(
                "world/pgo_map/path",
                rr.LineStrips3D(strips=[pgo_path], colors=[[255, 255, 255]], radii=[0.05]),
                static=True,
            )

    if export and pgo_map is not None:
        from pathlib import Path

        out_path = Path.cwd() / f"{db_path.stem}.pc2.lcm"
        print(f"exporting PGO twopass map to {out_path}...")
        out_path.write_bytes(pgo_map.lcm_encode())
        print(f"wrote {out_path}")
        print()
        print("load back with:")
        print("    from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2")
        print(f'    pcd = PointCloud2.lcm_decode(open("{out_path.name}", "rb").read())')


if __name__ == "__main__":
    typer.run(main)
