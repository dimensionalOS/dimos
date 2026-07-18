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

"""Dump golden data for the Rust mapper ports (issue #2820, task: golden tests).

Feeds the go2_short.db lidar frames through the Python VoxelGrid +
height_cost_occupancy (blueprint configs) and records, at checkpoints:

  - frames.bin      all input frames (u32 n, then n * 3 f32 LE xyz), the exact
                    float32 positions the Python grid consumed (points_f32)
  - voxels_N.bin    sorted occupied voxel keys after frame N (u32 m, m * 3 i32)
  - map_N.bin       emitted global-map centers after frame N (u32 m, m * 3 f32)
  - cost_N.json/bin costmap computed from that emitted map (meta + i8 grid)

The Rust integration test (dimos/mapping/rust/tests/golden.rs) replays
frames.bin and asserts key-set equality / grid tolerance at each checkpoint.

Usage:
    uv run python scripts/benchmarks/dump_golden.py
"""

from dataclasses import asdict
import json
from pathlib import Path
import struct

import numpy as np

from dimos.mapping.pointclouds.occupancy import HeightCostConfig, height_cost_occupancy
from dimos.mapping.voxels import VoxelGrid
from dimos.memory2.store.sqlite import SqliteStore

DB = "data/go2_short.db"
OUT = Path("dimos/mapping/rust/tests/golden_data")
CHECKPOINTS = [5, 50, 150, 300, 460]  # multiples of emit_every=5


def write_array(path: Path, arr: np.ndarray) -> None:
    with open(path, "wb") as f:
        f.write(struct.pack("<I", len(arr)))
        f.write(np.ascontiguousarray(arr).tobytes())


def main() -> None:
    OUT.mkdir(parents=True, exist_ok=True)
    (OUT / ".gitignore").write_text("*\n!.gitignore\n")  # generated data, never committed

    store = SqliteStore(path=DB, must_exist=True)
    store.start()
    lidar = store.replay().stream("lidar")

    grid = VoxelGrid(voxel_size=0.05, carve_columns=True, device="CUDA:0")
    cost_cfg = asdict(HeightCostConfig())

    frames_f = open(OUT / "frames.bin", "wb")
    for i, frame in enumerate(lidar.iterate(), 1):
        pts = frame.points_f32()
        frames_f.write(struct.pack("<I", len(pts)))
        frames_f.write(np.ascontiguousarray(pts, dtype=np.float32).tobytes())

        grid.add_frame(frame)

        if i in CHECKPOINTS:
            active = grid._voxel_hashmap.active_buf_indices()
            keys = grid._voxel_hashmap.key_tensor()[active].numpy().astype(np.int32)
            keys = keys[np.lexsort((keys[:, 2], keys[:, 1], keys[:, 0]))]
            write_array(OUT / f"voxels_{i}.bin", keys)

            gm = grid.get_global_pointcloud2()
            centers = gm.points_f32()
            write_array(OUT / f"map_{i}.bin", centers.astype(np.float32))

            og = height_cost_occupancy(gm, **cost_cfg)
            meta = {
                "width": int(og.width),
                "height": int(og.height),
                "origin_x": float(og.origin.position.x),
                "origin_y": float(og.origin.position.y),
                "resolution": float(og.resolution),
            }
            (OUT / f"cost_{i}.json").write_text(json.dumps(meta))
            (OUT / f"cost_{i}.bin").write_bytes(og.grid.astype(np.int8).tobytes())

            print(f"checkpoint {i}: voxels={len(keys)} grid={og.width}x{og.height}")

    frames_f.close()
    grid.dispose()
    store.stop()
    print(f"golden data written to {OUT}")


if __name__ == "__main__":
    main()
