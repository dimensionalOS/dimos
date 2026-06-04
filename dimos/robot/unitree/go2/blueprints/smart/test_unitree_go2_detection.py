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
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.vision_msgs.Detection2DArray import Detection2DArray
from dimos.msgs.vision_msgs.Detection3DArray import Detection3DArray
from dimos.perception.detection.module3D import Detection3DModule
from dimos.robot.unitree.go2.blueprints.smart.unitree_go2_detection import unitree_go2_detection


def test_unitree_go2_detection_transports_follow_renamed_fanio_ports() -> None:
    """The migrated module splits the old single 2D out into detections_2d /
    detections_3d; the blueprint must route both (keeping the wire topics LCM
    consumers already listen on) and must not keep a transport for the retired
    "detections" port - that key would silently publish nothing."""
    assert isinstance(unitree_go2_detection, Blueprint)
    transports = unitree_go2_detection.transport_map

    detections_2d = transports[("detections_2d", Detection3DModule)]
    assert detections_2d.topic.topic == "/detector3d/detections"  # topic survives the rename
    assert detections_2d.topic.lcm_type is Detection2DArray

    detections_3d = transports[("detections_3d", Detection3DModule)]
    assert detections_3d.topic.topic == "/detector3d/detections_3d"
    assert detections_3d.topic.lcm_type is Detection3DArray

    for index in range(3):
        pointcloud_out = transports[(f"detected_pointcloud_{index}", Detection3DModule)]
        assert pointcloud_out.topic.topic == f"/detector3d/pointcloud/{index}"
        image_out = transports[(f"detected_image_{index}", Detection3DModule)]
        assert image_out.topic.topic == f"/detector3d/image/{index}"

    assert all(name != "detections" for name, _ in transports)

    # The detector fuses against the accumulated go2 map, not a raw lidar scan.
    assert unitree_go2_detection.remapping_map[(Detection3DModule, "pointcloud")] == "global_map"


def test_unitree_go2_detection_routes_only_declared_ports() -> None:
    """Every transported or remapped port name must exist on the module: a
    stale key left behind by a port rename never errors at deploy, it just
    stops publishing (or stops remapping) silently."""
    module = Detection3DModule(
        camera_info=CameraInfo.from_intrinsics(
            fx=600.0, fy=600.0, cx=320.0, cy=240.0, width=640, height=480, frame_id="camera"
        ),
        detector=lambda: None,
    )
    try:
        transported = {
            name
            for name, owner in unitree_go2_detection.transport_map
            if owner is Detection3DModule
        }
        assert transported
        assert transported <= set(module.outputs)

        remapped = {
            name
            for owner, name in unitree_go2_detection.remapping_map
            if owner is Detection3DModule
        }
        assert remapped <= set(module.inputs)
    finally:
        module.stop()
