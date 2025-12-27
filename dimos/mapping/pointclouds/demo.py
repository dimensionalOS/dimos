#!/usr/bin/env python3
# Copyright 2025 Dimensional Inc.
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

import cv2
from open3d.geometry import PointCloud  # type: ignore[import-untyped]
import typer

from dimos.mapping.occupancy.gradient import gradient
from dimos.mapping.occupancy.visualizations import visualize_occupancy_grid
from dimos.mapping.pointclouds.occupancy import simple_occupancy
from dimos.mapping.pointclouds.util import (
    height_colorize,
    read_pointcloud,
    visualize,
)
from dimos.msgs.nav_msgs import OccupancyGrid
from dimos.msgs.sensor_msgs import PointCloud2
from dimos.utils.data import get_data

app = typer.Typer()


def _get_sum_map() -> PointCloud:
    return read_pointcloud(get_data("apartment") / "sum.ply")


def _get_occupancy_grid() -> OccupancyGrid:
    resolution = 0.05
    min_height = 0.15
    max_height = 0.6
    occupancygrid = simple_occupancy(
        PointCloud2(_get_sum_map()),
        resolution=resolution,
        min_height=min_height,
        max_height=max_height,
    )
    return occupancygrid


def _show_occupancy_grid(og: OccupancyGrid) -> None:
    cost_map = visualize_occupancy_grid(og, "turbo").to_opencv()
    cost_map = cv2.flip(cost_map, 0)

    # Resize to make the image larger (scale by 4x)
    height, width = cost_map.shape[:2]
    cost_map = cv2.resize(cost_map, (width * 4, height * 4), interpolation=cv2.INTER_NEAREST)

    cv2.namedWindow("Occupancy Grid", cv2.WINDOW_NORMAL)
    cv2.imshow("Occupancy Grid", cost_map)
    cv2.waitKey(0)
    cv2.destroyAllWindows()


@app.command()
def view_sum() -> None:
    pointcloud = _get_sum_map()
    height_colorize(pointcloud)
    visualize(pointcloud)


@app.command()
def view_map() -> None:
    og = _get_occupancy_grid()
    _show_occupancy_grid(og)


@app.command()
def view_map_inflated() -> None:
    og = gradient(_get_occupancy_grid(), max_distance=1.5)
    _show_occupancy_grid(og)


if __name__ == "__main__":
    app()
