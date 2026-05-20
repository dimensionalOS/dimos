# Copyright 2026 Dimensional Inc.
# SPDX-License-Identifier: Apache-2.0

"""Render the global OccupancyGrid derived from the recorded lidar stream.

Fuses lidar into a global voxel map, converts to a 2D occupancy grid,
overlays the robot's odom trajectory and the hand-authored map_objects /
map_structural points, and saves a PNG.
"""

from __future__ import annotations

import argparse
from pathlib import Path as FsPath

import cv2

from dimos.mapping.occupancy.visualizations import visualize_occupancy_grid
from dimos.mapping.pointclouds.occupancy import simple_occupancy
from dimos.mapping.voxels import VoxelMapTransformer
from dimos.memory2.store.sqlite import SqliteStore
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.nav_msgs.Path import Path as DimosPath
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--db", type=FsPath, required=True, help="Recording .db path (required).")
    parser.add_argument(
        "--out", type=FsPath, default=None, help="Output PNG path (default: /tmp/<db-stem>_map.png)"
    )
    args = parser.parse_args()

    if not args.db.exists():
        raise SystemExit(f"recording not found: {args.db}")
    out_png = args.out or FsPath("/tmp") / f"{args.db.stem}_map.png"

    store = SqliteStore(path=str(args.db))

    print("[viz] fusing lidar -> global voxel map (voxel_size=0.10)...")
    lidar = store.stream("lidar", PointCloud2)
    global_cloud = lidar.transform(VoxelMapTransformer(voxel_size=0.10)).last().data
    print(f"      {len(global_cloud)} voxels  bbox={global_cloud.bounding_box_dimensions}")

    print("[viz] simple_occupancy(resolution=0.10)...")
    grid = simple_occupancy(global_cloud, resolution=0.10)
    print(
        f"      grid {grid.width}x{grid.height}  origin=({grid.origin.position.x:.2f}, "
        f"{grid.origin.position.y:.2f})  res={grid.resolution}"
    )

    # Trajectory from odom — convert observations into a Path message
    print("[viz] building odom trajectory...")
    odom_obs = store.stream("odom").to_list()
    poses: list[PoseStamped] = [o.data for o in odom_obs]
    traj = DimosPath(poses=poses)
    print(f"      {len(poses)} pose stamps spanning {odom_obs[-1].ts - odom_obs[0].ts:.1f}s")

    img = visualize_occupancy_grid(grid, palette="turbo", path=traj)
    # Flip first so +y is up; overlays go on top of the flipped image.
    bgr = cv2.flip(img.data, 0).copy()

    H = bgr.shape[0]

    def _world_to_px(wx: float, wy: float) -> tuple[int, int]:
        gx, gy, _ = grid.world_to_grid([wx, wy, 0.0])
        return int(gx), (H - 1) - int(gy)

    # Upscale before drawing labels so text is readable.
    SCALE = 5
    up = cv2.resize(
        bgr, (bgr.shape[1] * SCALE, bgr.shape[0] * SCALE), interpolation=cv2.INTER_NEAREST
    )

    def _overlay(stream_name: str, color_bgr: tuple[int, int, int]) -> None:
        for obs in store.stream(stream_name, str).to_list():
            if obs.pose is None:
                continue
            px, py = _world_to_px(obs.pose[0], obs.pose[1])
            if not (0 <= px < grid.width and 0 <= py < grid.height):
                continue
            cx, cy = px * SCALE + SCALE // 2, py * SCALE + SCALE // 2
            cv2.circle(up, (cx, cy), 6, (0, 0, 0), -1)
            cv2.circle(up, (cx, cy), 4, color_bgr, -1)
            # Short label — drop trailing words after the first 2-3
            short = " ".join(obs.data.split()[:3])
            cv2.putText(
                up,
                short,
                (cx + 8, cy + 4),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.45,
                (0, 0, 0),
                3,
                cv2.LINE_AA,
            )
            cv2.putText(
                up,
                short,
                (cx + 8, cy + 4),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.45,
                color_bgr,
                1,
                cv2.LINE_AA,
            )

    _overlay("map_objects", (0, 0, 255))  # red dots — objects
    _overlay("map_structural", (0, 220, 220))  # yellow dots — structural

    # Legend bar on top
    legend = [
        ("red = occupied", (0, 0, 255)),
        ("blue = free", (180, 100, 30)),
        ("grey = unknown", (180, 180, 180)),
        ("black line = trajectory", (0, 0, 0)),
        ("red dot = map_objects", (0, 0, 255)),
        ("yellow dot = map_structural", (0, 220, 220)),
    ]
    y = 18
    for txt, col in legend:
        cv2.putText(up, txt, (8, y), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 0), 3, cv2.LINE_AA)
        cv2.putText(up, txt, (8, y), cv2.FONT_HERSHEY_SIMPLEX, 0.45, col, 1, cv2.LINE_AA)
        y += 14

    cv2.imwrite(str(out_png), up)
    print(f"[viz] wrote {out_png}  ({up.shape[1]}x{up.shape[0]} px)")

    store.stop()


if __name__ == "__main__":
    main()
