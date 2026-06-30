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

from collections.abc import Generator

import numpy as np
import pytest
from pytest_mock import MockerFixture

from dimos.manipulation.grasping.grasping import GraspingModule
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.grasping_msgs.GraspCandidate import GraspCandidate
from dimos.msgs.grasping_msgs.GraspCandidateArray import GraspCandidateArray
from dimos.msgs.perception_msgs.RegisteredObject import RegisteredObject
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.msgs.std_msgs.Header import Header


class FakeGPDGraspGenModule:
    def __init__(self, result: object) -> None:
        self.result = result
        self.calls: list[tuple[PointCloud2, PointCloud2 | None]] = []

    def generate_grasps(
        self,
        pointcloud: PointCloud2,
        scene_pointcloud: PointCloud2 | None = None,
    ) -> object:
        self.calls.append((pointcloud, scene_pointcloud))
        if isinstance(self.result, Exception):
            raise self.result
        return self.result


@pytest.fixture(autouse=True)
def _stop_created_modules(mocker: MockerFixture) -> Generator[None, None, None]:
    modules: list[GraspingModule] = []
    original_init = GraspingModule.__init__

    def tracked_init(self: GraspingModule, **kwargs: object) -> None:
        original_init(self, **kwargs)
        modules.append(self)

    mocker.patch.object(GraspingModule, "__init__", tracked_init)
    yield
    for module in modules:
        module.stop()


def _candidate_array() -> GraspCandidateArray:
    return GraspCandidateArray(
        header=Header(12.5, "world"),
        candidates=[
            GraspCandidate(
                pose=Pose(Vector3(1.0, 2.0, 3.0), Quaternion(0.0, 0.0, 0.0, 1.0)),
                jaw_width=0.05,
                score=0.9,
                id="vgn-0",
            )
        ],
    )


def test_generate_grasps_for_object_unknown_object_id_returns_clear_message(
    mocker: MockerFixture,
) -> None:
    module = GraspingModule()
    scene_registration = mocker.Mock()
    scene_registration.get_object_by_object_id.return_value = None
    tsdf_grasp_gen = mocker.Mock()
    mocker.patch.object(module, "_scene_registration", scene_registration, create=True)
    mocker.patch.object(module, "_tsdf_grasp_gen", tsdf_grasp_gen, create=True)

    message = module.generate_grasps_for_object("missing-object")

    assert message == "No registered object found with object_id 'missing-object'."
    tsdf_grasp_gen.generate_grasps_for_target_bounds.assert_not_called()


def test_generate_grasps_for_object_forwards_bounds_and_publishes_pose_array(
    mocker: MockerFixture,
) -> None:
    module = GraspingModule()
    target = RegisteredObject(
        object_id="obj-1",
        name="mug",
        center=Vector3(0.1, 0.2, 0.3),
        size=Vector3(0.4, 0.5, 0.6),
        frame_id="camera",
        ts=42.0,
    )
    scene_registration = mocker.Mock()
    scene_registration.get_object_by_object_id.return_value = target
    candidates = _candidate_array()
    tsdf_grasp_gen = mocker.Mock()
    tsdf_grasp_gen.generate_grasps_for_target_bounds.return_value = candidates
    mocker.patch.object(module, "_scene_registration", scene_registration, create=True)
    mocker.patch.object(module, "_tsdf_grasp_gen", tsdf_grasp_gen, create=True)
    published = []
    module.grasps.subscribe(published.append)

    message = module.generate_grasps_for_object("obj-1", cushion_m=0.07)

    tsdf_grasp_gen.generate_grasps_for_target_bounds.assert_called_once_with(
        target_center=target.center,
        target_size=target.size,
        target_frame_id="camera",
        target_ts=42.0,
        cushion_m=0.07,
    )
    assert len(published) == 1
    assert published[0].poses == candidates.to_pose_array().poses
    assert "Generated 1" in message


def test_generate_grasps_pointcloud_flow_unchanged(mocker: MockerFixture) -> None:
    module = GraspingModule()
    scene_registration = mocker.Mock()
    object_pc = mocker.Mock()
    scene_pc = mocker.Mock()
    scene_registration.get_object_pointcloud_by_object_id.return_value = object_pc
    scene_registration.get_full_scene_pointcloud.return_value = scene_pc
    grasp_gen = mocker.Mock()
    grasp_gen.generate_grasps.return_value = _candidate_array().to_pose_array()
    mocker.patch.object(module, "_scene_registration", scene_registration, create=True)
    mocker.patch.object(module, "_grasp_gen", grasp_gen, create=True)
    published = []
    module.grasps.subscribe(published.append)

    message = module.generate_grasps(object_name="mug", object_id="obj-1", filter_collisions=True)

    scene_registration.get_object_pointcloud_by_object_id.assert_called_once_with("obj-1")
    scene_registration.get_full_scene_pointcloud.assert_called_once_with(exclude_object_id="obj-1")
    grasp_gen.generate_grasps.assert_called_once_with(object_pc, scene_pc)
    assert len(published) == 1
    assert "Generated 1" in message


def test_generate_grasps_routes_registered_object_pointcloud_through_gpd_detector(
    mocker: MockerFixture,
) -> None:
    module = GraspingModule()
    scene_registration = mocker.Mock()
    object_pc = PointCloud2.from_numpy(
        np.array([[0.0, 0.0, 0.0], [0.02, 0.0, 0.0]], dtype=np.float32),
        frame_id="object",
    )
    scene_pc = PointCloud2.from_numpy(
        np.array([[1.0, 1.0, 1.0]], dtype=np.float32),
        frame_id="scene",
    )
    scene_registration.get_object_pointcloud_by_object_id.return_value = object_pc
    scene_registration.get_full_scene_pointcloud.return_value = scene_pc
    pose_array = _candidate_array().to_pose_array()
    gpd_detector = FakeGPDGraspGenModule(pose_array)
    mocker.patch.object(module, "_scene_registration", scene_registration, create=True)
    mocker.patch.object(module, "_grasp_gen", gpd_detector, create=True)
    published = []
    module.grasps.subscribe(published.append)

    message = module.generate_grasps(object_name="mug", object_id="obj-1", filter_collisions=True)

    assert gpd_detector.calls == [(object_pc, scene_pc)]
    assert len(published) == 1
    assert published[0] is pose_array
    assert "Generated 1" in message


def test_generate_grasps_empty_gpd_output_returns_clear_message_and_does_not_publish(
    mocker: MockerFixture,
) -> None:
    module = GraspingModule()
    scene_registration = mocker.Mock()
    object_pc = PointCloud2.from_numpy(
        np.array([[0.0, 0.0, 0.0]], dtype=np.float32),
        frame_id="object",
    )
    scene_registration.get_object_pointcloud_by_name.return_value = object_pc
    gpd_detector = FakeGPDGraspGenModule(None)
    mocker.patch.object(module, "_scene_registration", scene_registration, create=True)
    mocker.patch.object(module, "_grasp_gen", gpd_detector, create=True)
    publish = mocker.patch.object(module.grasps, "publish")

    message = module.generate_grasps(object_name="mug", filter_collisions=False)

    assert gpd_detector.calls == [(object_pc, None)]
    publish.assert_not_called()
    assert message == "Pointcloud grasp generator returned no grasps for 'mug'."


def test_generate_grasps_failed_gpd_output_returns_clear_message_and_does_not_publish(
    mocker: MockerFixture,
) -> None:
    module = GraspingModule()
    scene_registration = mocker.Mock()
    object_pc = PointCloud2.from_numpy(
        np.array([[0.0, 0.0, 0.0]], dtype=np.float32),
        frame_id="object",
    )
    scene_registration.get_object_pointcloud_by_name.return_value = object_pc
    gpd_detector = FakeGPDGraspGenModule(RuntimeError("GPD backend unavailable"))
    mocker.patch.object(module, "_scene_registration", scene_registration, create=True)
    mocker.patch.object(module, "_grasp_gen", gpd_detector, create=True)
    publish = mocker.patch.object(module.grasps, "publish")

    message = module.generate_grasps(object_name="mug", filter_collisions=False)

    assert gpd_detector.calls == [(object_pc, None)]
    publish.assert_not_called()
    assert message == "Grasp generation failed: GPD backend unavailable"
