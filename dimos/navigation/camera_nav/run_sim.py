# Copyright 2025-2026 Dimensional Inc.
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

"""Visualise a scanned environment in MuJoCo.

Loads the occupancy grid exported by CostMapper (.npy) and opens an
interactive MuJoCo viewer showing the reconstructed room geometry.

Imports ONLY numpy + mujoco — no dimos package, no mujoco_playground.

Usage::

    python -m dimos.navigation.camera_nav.run_sim
    python -m dimos.navigation.camera_nav.run_sim --costmap ~/Downloads/map_costmap.npy
"""

from __future__ import annotations

import argparse
from pathlib import Path

import mujoco
import mujoco.viewer
import numpy as np


# ---------------------------------------------------------------------------
# Occupancy grid → MuJoCo XML (self-contained, no dimos imports)
# ---------------------------------------------------------------------------

def _largest_rect(mask: np.ndarray) -> tuple[int, int, int, int] | None:
    """Find the largest axis-aligned rectangle of 255 in `mask`."""
    h, w = mask.shape
    heights = np.zeros(w, dtype=int)
    best: tuple[int, int, int, int] | None = None
    best_area = 0

    for row in range(h):
        heights = np.where(mask[row] == 255, heights + 1, 0)
        stack: list[int] = []
        for col in range(w + 1):
            h_cur = heights[col] if col < w else 0
            while stack and heights[stack[-1]] > h_cur:
                height = heights[stack.pop()]
                width = col if not stack else col - stack[-1] - 1
                x0 = (col - width) if not stack else stack[-1] + 1
                y0 = row - height + 1
                area = height * width
                if area > best_area:
                    best_area = area
                    best = (x0, y0, x0 + width, y0 + height)
            stack.append(col)

    return best


def _scene_xml_from_grid(
    grid: np.ndarray,
    resolution: float = 0.05,
    wall_height: float = 0.5,
) -> str:
    """Generate MuJoCo XML from a 2D occupancy grid (int8, 100=occupied)."""
    occupied = ((grid == 100) | (grid == -1)).astype(np.uint8) * 255
    remaining = occupied.copy()

    walls: list[tuple[int, int, int, int]] = []
    for _ in range(10_000):
        rect = _largest_rect(remaining)
        if rect is None:
            break
        x0, y0, x1, y1 = rect
        walls.append(rect)
        remaining[y0:y1, x0:x1] = 0

    lines = [
        '<?xml version="1.0" ?>',
        '<mujoco model="scanned_env">',
        '  <compiler angle="radian"/>',
        '  <visual><map znear="0.01" zfar="500"/></visual>',
        '  <asset>',
        '    <material name="wall" rgba="0.55 0.55 0.65 1"/>',
        '    <texture type="2d" name="floor_tex" builtin="checker" '
        'rgb1="0.9 0.9 0.9" rgb2="0.75 0.75 0.75" width="200" height="200"/>',
        '    <material name="floor" texture="floor_tex" texrepeat="4 4"/>',
        '  </asset>',
        '  <worldbody>',
        '    <light directional="true" pos="0 0 4" dir="0 0 -1" diffuse="0.8 0.8 0.8"/>',
        '    <geom name="floor" size="0 0 0.01" type="plane" material="floor"/>',
    ]

    for i, (x0, y0, x1, y1) in enumerate(walls):
        cx = (x0 + x1) / 2 * resolution
        cy = (y0 + y1) / 2 * resolution
        hx = (x1 - x0) / 2 * resolution
        hy = (y1 - y0) / 2 * resolution
        hz = wall_height / 2
        lines.append(
            f'    <geom name="w{i}" type="box" '
            f'size="{hx:.4f} {hy:.4f} {hz:.4f}" '
            f'pos="{cx:.4f} {cy:.4f} {hz:.4f}" '
            f'material="wall"/>'
        )

    lines += ["  </worldbody>", "</mujoco>"]
    return "\n".join(lines)


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="View scanned environment in MuJoCo")
    parser.add_argument(
        "--costmap",
        default="~/Downloads/map_costmap.npy",
        help="Occupancy grid exported by CostMapper (.npy)",
    )
    parser.add_argument(
        "--resolution",
        type=float,
        default=0.05,
        help="Grid cell size in metres (default 0.05)",
    )
    args = parser.parse_args()

    costmap_path = Path(args.costmap).expanduser()
    if not costmap_path.exists():
        raise FileNotFoundError(
            f"Costmap not found: {costmap_path}\n"
            "Run '--zed-only' or '--trial' first to scan and export the map."
        )

    grid = np.load(costmap_path)
    print(f"Loaded costmap {costmap_path}  shape={grid.shape}")

    xml = _scene_xml_from_grid(grid, resolution=args.resolution)
    model = mujoco.MjModel.from_xml_string(xml)
    data = mujoco.MjData(model)

    print("Opening MuJoCo viewer — drag to orbit, scroll to zoom, Esc to quit")
    mujoco.viewer.launch(model, data)
