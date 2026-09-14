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
from dimos.manipulation.planning.spec.models import Obstacle
from dimos.manipulation.visualization.viser.scene import ViserManipulationScene
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped


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


def test_an_empty_octree_is_a_render_failure_not_a_blank_cloud() -> None:
    """A map with no occupied cells is a broken mapping chain, not a clear table."""
    scene = _scene()

    scene.add_vis_obstacle("mapping/voxel-map", _octree(np.zeros((0, 3), dtype=np.float32)))

    assert "mapping/voxel-map" in scene._obstacle_render_failures
