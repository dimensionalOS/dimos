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

"""Rerun visualization helpers for Spot blueprints."""

from __future__ import annotations

import rerun as rr
import rerun.blueprint as rrb


def _grayscale_origin(suffix: str) -> str:
    return f"world/grayscale_image_{suffix}"


def _depth_origin(suffix: str) -> str:
    return f"world/depth_image_{suffix}"


def _camera_view(origin: str, name: str) -> rrb.Spatial2DView:
    return rrb.Spatial2DView(origin=origin, name=name)


def spot_camera_layout() -> rrb.Blueprint:
    """A 3D view of Spot beside tabbed 2D camera panels.

    Left: the robot's tf tree and every camera frustum in place. Right: tabs of
    grayscale/depth feeds labelled by URDF mount position.
    """
    world_view = rrb.Spatial3DView(
        origin="world",
        name="3D",
        background=rrb.Background(kind="SolidColor", color=[0, 0, 0]),
        line_grid=rrb.LineGrid3D(plane=rr.components.Plane3D.XY.with_distance(0.0)),
    )

    # Each rrb view may live in exactly one place in the blueprint tree, so every
    # container below builds its own fresh view instances rather than sharing one.
    front_tab = rrb.Horizontal(
        _camera_view(_grayscale_origin("front_right"), "front right"),
        _camera_view(_grayscale_origin("front_left"), "front left"),
        _camera_view(_depth_origin("front_right"), "front right (depth)"),
        _camera_view(_depth_origin("front_left"), "front left (depth)"),
        name="front",
    )

    grayscale_tab = rrb.Vertical(
        rrb.Horizontal(
            _camera_view(_grayscale_origin("front_right"), "front right"),
            _camera_view(_grayscale_origin("front_left"), "front left"),
        ),
        rrb.Horizontal(
            _camera_view(_grayscale_origin("left"), "left"),
            _camera_view(_grayscale_origin("right"), "right"),
        ),
        _camera_view(_grayscale_origin("back"), "back"),
        name="grayscale",
    )

    depth_tab = rrb.Vertical(
        rrb.Horizontal(
            _camera_view(_depth_origin("front_right"), "front right (depth)"),
            _camera_view(_depth_origin("front_left"), "front left (depth)"),
        ),
        rrb.Horizontal(
            _camera_view(_depth_origin("left"), "left"),
            _camera_view(_depth_origin("right"), "right"),
        ),
        _camera_view(_depth_origin("back"), "back"),
        name="depth",
    )

    return rrb.Blueprint(
        rrb.Horizontal(
            world_view,
            rrb.Tabs(front_tab, grayscale_tab, depth_tab),
            column_shares=[2, 1],
        ),
        collapse_panels=True,
    )
