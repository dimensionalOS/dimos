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

"""Booster B1 camera-specific Rerun dashboard layout."""

import rerun.blueprint as rrb


def _camera_metrics_views(streams: tuple[str, ...]) -> list[rrb.TimeSeriesView]:
    views: list[rrb.TimeSeriesView] = []
    for stream in streams:
        root = f"metrics/camera/{stream}"
        stream_name = stream.replace("_", " ").title()
        views.extend(
            [
                rrb.TimeSeriesView(
                    origin=root,
                    contents=[f"{root}/fps"],
                    name=f"{stream_name} FPS",
                ),
                rrb.TimeSeriesView(
                    origin=root,
                    contents=[f"{root}/payload_mbps"],
                    name=f"{stream_name} bandwidth",
                ),
                rrb.TimeSeriesView(
                    origin=root,
                    contents=[f"{root}/frame_age/**"],
                    name=f"{stream_name} frame age",
                ),
                rrb.TimeSeriesView(
                    origin=root,
                    contents=[f"{root}/bridge/**"],
                    name=f"{stream_name} bridge time",
                ),
                rrb.TimeSeriesView(
                    origin=root,
                    contents=[
                        f"{root}/invalid_timestamps",
                        f"{root}/negative_ages",
                    ],
                    name=f"{stream_name} timestamp errors",
                ),
            ]
        )
    return views


def camera_metrics_grid() -> rrb.Grid:
    """Create separate image metric charts grouped by stream and category."""
    views = _camera_metrics_views(("color", "depth"))
    return rrb.Grid(*views, grid_columns=2, name="Camera Metrics")


def camera_info_metrics_grid() -> rrb.Grid:
    """Create metric charts for the color and depth camera-info topics."""
    views = _camera_metrics_views(
        ("color_camera_info", "depth_camera_info"),
    )
    return rrb.Grid(*views, grid_columns=2, name="Camera Info Metrics")


def booster_camera_rerun_blueprint() -> rrb.Blueprint:
    """Create the Booster camera image and metrics dashboard."""
    return rrb.Blueprint(
        rrb.Tabs(
            rrb.Horizontal(
                rrb.Spatial2DView(origin="world/color_image", name="Color"),
                rrb.Spatial2DView(origin="world/depth_image", name="Depth"),
                column_shares=[1, 1],
                name="Images",
            ),
            camera_metrics_grid(),
            camera_info_metrics_grid(),
            name="Booster Camera",
        )
    )
