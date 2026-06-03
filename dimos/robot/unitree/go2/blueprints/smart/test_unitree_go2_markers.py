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

from dimos.core.coordination.blueprints import Blueprint
from dimos.msgs.vision_msgs.Detection2DArray import Detection2DArray
from dimos.msgs.vision_msgs.Detection3DArray import Detection3DArray
from dimos.perception.fiducial.marker_module import MarkerModule
from dimos.perception.fiducial.marker_tf_module import MarkerTfModule
from dimos.robot.unitree.go2.blueprints.smart.unitree_go2 import unitree_go2_markers


def test_unitree_go2_markers_uses_detector_backed_tf_stack() -> None:
    assert isinstance(unitree_go2_markers, Blueprint)

    modules = [bp.module for bp in unitree_go2_markers.blueprints]
    assert MarkerModule in modules
    assert MarkerTfModule in modules

    detector = next(
        bp for bp in unitree_go2_markers.blueprints if bp.module is MarkerModule
    )
    assert detector.kwargs["marker_length_m"] == 0.1
    assert detector.kwargs["camera_info"].frame_id == "camera_optical"
    assert (
        unitree_go2_markers.transport_map[("detections_3d", Detection3DArray)].topic.topic
        == "/marker_detection/detections_3d"
    )
    assert (
        unitree_go2_markers.transport_map[("detections_2d", Detection2DArray)].topic.topic
        == "/marker_detection/detections_2d"
    )
