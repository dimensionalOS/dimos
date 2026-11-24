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

"""Obstacle rendering dispatch, without a running Viser server."""

from unittest.mock import MagicMock

import numpy as np
import pytest

pytest.importorskip("viser", reason="Viser optional dependency is not installed")

from dimos.manipulation.planning.spec.enums import ObstacleType
from dimos.manipulation.planning.spec.models import WORK_SURFACE_OBSTACLE, Obstacle
from dimos.manipulation.visualization.viser.scene import (
    BELOW_SURFACE_COLOR,
    HEIGHT_RAMP,
    WORK_SURFACE_COLOR,
    ViserManipulationScene,
)
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Vector3 import Vector3


def _scene() -> ViserManipulationScene:
    return ViserManipulationScene(MagicMock(), MagicMock())


def _octree(points: np.ndarray, resolution: float = 0.025) -> Obstacle:
    return Obstacle(
        name="mapping/voxel-map",
        obstacle_type=ObstacleType.OCTREE,
        pose=PoseStamped(frame_id="world"),
        points=points,
        octree_resolution=resolution,
    )


def test_octree_obstacles_draw_their_occupied_cells() -> None:
    """Before this the dispatch raised and drew a grey proxy box over the map."""
    scene = _scene()
    points = np.array([[0.0, 0.0, 0.0], [0.05, 0.0, 0.0]], dtype=np.float32)

    scene.add_vis_obstacle("mapping/voxel-map", _octree(points))

    assert scene._obstacle_render_failures == {}
    call = scene.server.scene.add_point_cloud.call_args
    assert np.allclose(call.kwargs["points"], points)
    assert call.kwargs["point_size"] == pytest.approx(0.025)


def test_octree_cells_are_coloured_by_height() -> None:
    scene = _scene()
    points = np.array([[0.0, 0.0, z] for z in np.linspace(0.0, 0.2, 11)], dtype=np.float32)

    scene.add_vis_obstacle("mapping/voxel-map", _octree(points))

    colors = scene.server.scene.add_point_cloud.call_args.kwargs["colors"]
    assert colors.dtype == np.uint8
    assert tuple(colors[0]) == (40, 70, 220)  # lowest cell blue
    assert tuple(colors[-1]) == (220, 50, 40)  # highest cell red


def test_an_octree_with_its_own_colour_keeps_it() -> None:
    scene = _scene()
    points = np.array([[0.0, 0.0, 0.0], [0.0, 0.0, 0.2]], dtype=np.float32)
    obstacle = _octree(points)
    obstacle.color = (1.0, 0.0, 0.0, 1.0)

    scene.add_vis_obstacle("mapping/voxel-map", obstacle)

    colors = scene.server.scene.add_point_cloud.call_args.kwargs["colors"]
    assert {tuple(c) for c in colors} == {(255, 0, 0)}


def test_an_empty_octree_is_a_render_failure_not_a_blank_cloud() -> None:
    """A map with no occupied cells is a broken mapping chain, not a clear table."""
    scene = _scene()

    scene.add_vis_obstacle("mapping/voxel-map", _octree(np.zeros((0, 3), dtype=np.float32)))

    assert "mapping/voxel-map" in scene._obstacle_render_failures


def _surface_box(name: str, x: float, top: float, side: float = 0.4) -> Obstacle:
    return Obstacle(
        name=name,
        obstacle_type=ObstacleType.BOX,
        pose=PoseStamped(frame_id="world", position=Vector3(x, 0.0, top - 0.015)),
        dimensions=(side, side, 0.03),
    )


def test_with_work_surfaces_only_what_stands_on_them_is_coloured() -> None:
    scene = _scene()
    points = np.array(
        [
            [0.0, 0.0, 0.09],  # on the table, top 0.08
            [0.0, 0.0, 0.3],  # higher on the table
            [1.0, 0.0, 0.05],  # on the low table, top 0.04
            [0.0, 0.0, -0.3],  # under the table
            [0.5, 0.0, 0.2],  # between the tables: a chair, say
        ],
        dtype=np.float32,
    )
    scene.add_vis_obstacle("mapping/voxel-map", _octree(points))

    scene.add_vis_obstacle(f"{WORK_SURFACE_OBSTACLE}/1", _surface_box("t", 0.0, 0.08))
    scene.add_vis_obstacle(f"{WORK_SURFACE_OBSTACLE}/2", _surface_box("t", 1.0, 0.04))

    # The map is drawn again, in focus on the two surfaces.
    colors = scene.server.scene.add_point_cloud.call_args.kwargs["colors"]
    assert tuple(colors[1]) == HEIGHT_RAMP[-1]
    assert tuple(colors[0]) != BELOW_SURFACE_COLOR
    assert tuple(colors[2]) != BELOW_SURFACE_COLOR
    assert tuple(colors[3]) == BELOW_SURFACE_COLOR
    assert tuple(colors[4]) == BELOW_SURFACE_COLOR


def test_work_surfaces_are_drawn_in_their_own_colour() -> None:
    scene = _scene()

    scene.add_vis_obstacle(f"{WORK_SURFACE_OBSTACLE}/1", _surface_box("t", 0.0, 0.08))

    assert scene.server.scene.add_box.call_args.kwargs["color"] == WORK_SURFACE_COLOR
