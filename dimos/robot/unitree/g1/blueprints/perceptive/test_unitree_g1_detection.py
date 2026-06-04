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
from dimos.perception.detection.moduleDB import ObjectDBModule
from dimos.perception.detection.person_tracker import PersonTracker
from dimos.robot.unitree.g1.blueprints.perceptive.unitree_g1_detection import unitree_g1_detection


def test_unitree_g1_detection_routes_both_fanio_detector_stacks() -> None:
    """Detection3DModule and ObjectDBModule each split the old single 2D out
    into detections_2d / detections_3d. Both stacks must keep their original
    wire topics under their own prefix, and no transport may still target the
    retired "detections" port (it would silently publish nothing)."""
    assert isinstance(unitree_g1_detection, Blueprint)
    transports = unitree_g1_detection.transport_map

    for module_cls, prefix in (
        (Detection3DModule, "/detector3d"),
        (ObjectDBModule, "/detectorDB"),
    ):
        detections_2d = transports[("detections_2d", module_cls)]
        assert detections_2d.topic.topic == f"{prefix}/detections"  # topic survives the rename
        assert detections_2d.topic.lcm_type is Detection2DArray

        detections_3d = transports[("detections_3d", module_cls)]
        assert detections_3d.topic.topic == f"{prefix}/detections_3d"
        assert detections_3d.topic.lcm_type is Detection3DArray

        for index in range(3):
            pointcloud_out = transports[(f"detected_pointcloud_{index}", module_cls)]
            assert pointcloud_out.topic.topic == f"{prefix}/pointcloud/{index}"
            image_out = transports[(f"detected_image_{index}", module_cls)]
            assert image_out.topic.topic == f"{prefix}/image/{index}"

    assert all(name != "detections" for name, _ in transports)

    # The person tracker consumes the renamed 2D output stream.
    assert unitree_g1_detection.remapping_map[(PersonTracker, "detections")] == "detections_2d"


def test_unitree_g1_detection_routes_only_declared_ports() -> None:
    """Every transported or remapped port name must exist on its module: a
    stale key left behind by a port rename never errors at deploy, it just
    stops publishing (or stops remapping) silently."""
    camera_info = CameraInfo.from_intrinsics(
        fx=600.0, fy=600.0, cx=320.0, cy=240.0, width=640, height=480, frame_id="camera"
    )
    modules = {
        Detection3DModule: Detection3DModule(camera_info=camera_info, detector=lambda: None),
        ObjectDBModule: ObjectDBModule(camera_info=camera_info, detector=lambda: None),
        PersonTracker: PersonTracker(cameraInfo=camera_info),
    }
    try:
        for module_cls, module in modules.items():
            transported = {
                name
                for name, owner in unitree_g1_detection.transport_map
                if owner is module_cls
            }
            assert transported <= set(module.outputs), module_cls.__name__

            remapped = {
                name
                for owner, name in unitree_g1_detection.remapping_map
                if owner is module_cls
            }
            assert remapped <= set(module.inputs), module_cls.__name__
    finally:
        for module in modules.values():
            module.stop()


def test_g1_detection_blueprint_has_no_unknown_remap_owners() -> None:
    """Catches the inverse failure: a remapping whose owner module is not part
    of this blueprint at all (e.g. left behind after a module swap)."""
    blueprint_modules = {bp.module for bp in unitree_g1_detection.blueprints}
    for owner, _name in unitree_g1_detection.remapping_map:
        assert owner in blueprint_modules, (
            f"remapping references {owner.__name__}, which the blueprint does not deploy"
        )
