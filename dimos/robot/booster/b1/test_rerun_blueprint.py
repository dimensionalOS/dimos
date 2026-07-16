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

import rerun.blueprint as rrb

from dimos.robot.all_blueprints import all_blueprints
from dimos.robot.booster.b1.blueprints.basic.booster_b1_camera_rerun import (
    booster_b1_camera_rerun,
)
from dimos.robot.booster.b1.rerun_blueprint import (
    booster_camera_rerun_blueprint,
    camera_info_metrics_grid,
    camera_metrics_grid,
)
from dimos.robot.get_all_blueprints import get_blueprint_by_name
from dimos.visualization.rerun.bridge import RerunBridgeModule


def test_camera_metrics_grid_separates_streams_and_metric_categories() -> None:
    grid = camera_metrics_grid()

    assert isinstance(grid, rrb.Grid)
    assert grid.name == "Camera Metrics"
    assert grid.grid_columns == 2
    assert [view.name for view in grid.contents] == [
        "Color FPS",
        "Color bandwidth",
        "Color frame age",
        "Color bridge time",
        "Color timestamp errors",
        "Depth FPS",
        "Depth bandwidth",
        "Depth frame age",
        "Depth bridge time",
        "Depth timestamp errors",
    ]
    assert [view.contents for view in grid.contents] == [
        ["metrics/camera/color/fps"],
        ["metrics/camera/color/payload_mbps"],
        ["metrics/camera/color/frame_age/**"],
        ["metrics/camera/color/bridge/**"],
        [
            "metrics/camera/color/invalid_timestamps",
            "metrics/camera/color/negative_ages",
        ],
        ["metrics/camera/depth/fps"],
        ["metrics/camera/depth/payload_mbps"],
        ["metrics/camera/depth/frame_age/**"],
        ["metrics/camera/depth/bridge/**"],
        [
            "metrics/camera/depth/invalid_timestamps",
            "metrics/camera/depth/negative_ages",
        ],
    ]


def test_camera_info_metrics_grid_separates_streams_and_metric_categories() -> None:
    grid = camera_info_metrics_grid()

    assert isinstance(grid, rrb.Grid)
    assert grid.name == "Camera Info Metrics"
    assert grid.grid_columns == 2
    assert [view.name for view in grid.contents] == [
        "Color Camera Info FPS",
        "Color Camera Info bandwidth",
        "Color Camera Info frame age",
        "Color Camera Info bridge time",
        "Color Camera Info timestamp errors",
        "Depth Camera Info FPS",
        "Depth Camera Info bandwidth",
        "Depth Camera Info frame age",
        "Depth Camera Info bridge time",
        "Depth Camera Info timestamp errors",
    ]
    assert [view.contents for view in grid.contents] == [
        ["metrics/camera/color_camera_info/fps"],
        ["metrics/camera/color_camera_info/payload_mbps"],
        ["metrics/camera/color_camera_info/frame_age/**"],
        ["metrics/camera/color_camera_info/bridge/**"],
        [
            "metrics/camera/color_camera_info/invalid_timestamps",
            "metrics/camera/color_camera_info/negative_ages",
        ],
        ["metrics/camera/depth_camera_info/fps"],
        ["metrics/camera/depth_camera_info/payload_mbps"],
        ["metrics/camera/depth_camera_info/frame_age/**"],
        ["metrics/camera/depth_camera_info/bridge/**"],
        [
            "metrics/camera/depth_camera_info/invalid_timestamps",
            "metrics/camera/depth_camera_info/negative_ages",
        ],
    ]


def test_booster_camera_rerun_blueprint_contains_three_tabs() -> None:
    blueprint = booster_camera_rerun_blueprint()

    assert [part.name for part in blueprint.root_container.contents] == [
        "Images",
        "Camera Metrics",
        "Camera Info Metrics",
    ]


def test_booster_camera_rerun_is_an_opt_in_bridge_blueprint() -> None:
    [atom] = booster_b1_camera_rerun.blueprints

    assert atom.module is RerunBridgeModule
    assert atom.kwargs["blueprint"] is booster_camera_rerun_blueprint
    assert atom.kwargs["max_hz"] == {
        "world/color_image": 8.0,
        "world/depth_image": 3.0,
    }
    assert "visual_override" not in atom.kwargs
    assert all_blueprints["booster-b1-camera-rerun"].endswith(":booster_b1_camera_rerun")
    assert get_blueprint_by_name("booster-b1-camera-rerun") is booster_b1_camera_rerun
