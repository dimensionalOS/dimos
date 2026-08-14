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

"""Display-only Viser adapter for the canonical pick-and-place workflow."""

from __future__ import annotations

import numpy as np

from dimos.core.core import rpc
from dimos.core.module import Module
from dimos.core.stream import In
from dimos.manipulation.pick_and_place import PickAndPlaceState
from dimos.manipulation.visualization.layers import (
    LineSetElement,
    PointCloudElement,
    VisualizationLayer,
)
from dimos.manipulation.visualization_spec import ManipulationVisualizationSpec
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.perception.experimental.object import Object


class PickAndPlaceVisualizationAdapter(Module):
    """Render pick-and-place observations without participating in workflow decisions."""

    _visualization: ManipulationVisualizationSpec
    objects: In[list[Object]]
    pick_and_place_state: In[PickAndPlaceState]

    @rpc
    def start(self) -> None:
        super().start()
        self.objects.subscribe(self._on_objects)
        self.pick_and_place_state.subscribe(self._on_state)

    def _on_objects(self, objects: list[Object]) -> None:
        elements: list[PointCloudElement] = []
        for index, obj in enumerate(objects):
            points = obj.pointcloud.points_f32()
            if len(points):
                color = np.asarray(([80, 180, 255], [130, 230, 130], [230, 150, 230])[index % 3])
                elements.append(
                    PointCloudElement(
                        f"object-{index}", points, np.repeat([color], len(points), axis=0)
                    )
                )
        self._visualization.set_visualization_layer(
            VisualizationLayer("pick-and-place/objects", "world", tuple(elements))
        )

    def _on_state(self, state: PickAndPlaceState) -> None:
        candidate_elements: list[LineSetElement] = []
        for rank, candidate in enumerate(state.candidates.candidates[:10]):
            selected = rank == state.candidates.selected_index
            candidate_elements.append(
                self._axes(
                    f"rank-{rank}",
                    PoseStamped(
                        ts=state.candidates.header.timestamp,
                        frame_id=state.candidates.header.frame_id,
                        position=candidate.pose.position,
                        orientation=candidate.pose.orientation,
                    ),
                    color=np.asarray([255, 220, 70] if selected else [100, 190, 255]),
                )
            )
        self._visualization.set_visualization_layer(
            VisualizationLayer("pick-and-place/candidates", "world", tuple(candidate_elements))
        )
        target_elements = tuple(
            self._axes(name, pose, color=np.asarray(color))
            for name, pose, color in (
                ("grasp", state.selected_grasp, [255, 70, 70]),
                ("pregrasp", state.selected_pregrasp, [70, 255, 120]),
            )
            if pose is not None
        )
        self._visualization.set_visualization_layer(
            VisualizationLayer("pick-and-place/targets", "world", target_elements)
        )

    @staticmethod
    def _axes(name: str, pose: PoseStamped, *, color: np.ndarray) -> LineSetElement:
        origin = np.asarray(pose.position.as_tuple, dtype=np.float32)
        axes = pose.orientation.to_rotation_matrix().astype(np.float32) * 0.06
        vertices = np.vstack((origin, origin + axes.T))
        return LineSetElement(
            name,
            vertices,
            np.asarray([[0, 1], [0, 2], [0, 3]]),
            colors=np.repeat([color], 3, axis=0),
            line_width=2.0,
        )
