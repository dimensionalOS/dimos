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

from __future__ import annotations

import os
from pathlib import Path

from dimos_gpd_grasp_demo.blueprint import GPD_GRASP_DEMO_ENV_NAME, GPD_GRASP_DEMO_PROJECT
from dimos_gpd_grasp_demo.gpd_grasp_gen_module import GPDGraspGenModule
import pytest
from pytest_mock import MockerFixture

from dimos.core.coordination.module_coordinator import ModuleCoordinator
from dimos.core.runtime_environment import PythonProjectRuntimeEnvironment
from dimos.manipulation.grasping.grasping import GraspingModule
from dimos.manipulation.grasping.pointcloud_grasp_demo_controller import (
    PointcloudGraspDemoController,
)
from dimos.manipulation.grasping.target_grasp_demo_controller import TargetGraspDemoController
from dimos.manipulation.grasping.vgn_grasp_gen_module import VGNGraspGenModule
from dimos.manipulation.pick_and_place_module import PickAndPlaceModule
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.grasping_msgs.GraspCandidateArray import GraspCandidateArray
from dimos.msgs.perception_msgs.RegisteredObject import RegisteredObject
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.perception.object_scene_registration import ObjectSceneRegistrationModule
from dimos.perception.reconstruction import SceneReconstructionModule
from dimos.robot.all_blueprints import all_blueprints
from dimos.robot.manipulators.xarm.blueprints.simulation import (
    gpd_mujoco_grasp_demo,
    vgn_mujoco_grasp_demo,
    xarm_perception_sim,
)
from dimos.simulation.engines.mujoco_sim_module import MujocoSimModule
from dimos.visualization.rerun.bridge import RerunBridgeModule


def test_gpd_mujoco_grasp_demo_blueprint_is_opt_in() -> None:
    demo_modules = {atom.module for atom in gpd_mujoco_grasp_demo.blueprints}
    perception_modules = {atom.module for atom in xarm_perception_sim.blueprints}
    vgn_modules = {atom.module for atom in vgn_mujoco_grasp_demo.blueprints}

    assert demo_modules == {
        MujocoSimModule,
        ObjectSceneRegistrationModule,
        GPDGraspGenModule,
        GraspingModule,
        PointcloudGraspDemoController,
        RerunBridgeModule,
    }
    assert GPDGraspGenModule not in perception_modules
    assert PointcloudGraspDemoController not in perception_modules
    assert PickAndPlaceModule not in demo_modules
    assert TargetGraspDemoController in vgn_modules
    assert VGNGraspGenModule in vgn_modules
    assert PointcloudGraspDemoController not in vgn_modules


def test_gpd_mujoco_grasp_demo_places_only_generator_in_gpd_runtime() -> None:
    placements = dict(gpd_mujoco_grasp_demo.runtime_placement_map)
    environment = gpd_mujoco_grasp_demo.runtime_environment_registry.resolve(
        GPD_GRASP_DEMO_ENV_NAME
    )

    assert placements == {GPDGraspGenModule: GPD_GRASP_DEMO_ENV_NAME}
    assert isinstance(environment, PythonProjectRuntimeEnvironment)
    assert environment.project == GPD_GRASP_DEMO_PROJECT


def test_gpd_mujoco_grasp_demo_uses_stable_rerun_topics() -> None:
    assert (
        gpd_mujoco_grasp_demo.remapping_map[(GPDGraspGenModule, "grasp_candidates")]
        == "gpd_grasp_candidates"
    )
    assert hasattr(GraspCandidateArray, "to_rerun")


def test_gpd_mujoco_grasp_demo_configuration_does_not_change_vgn_or_perception() -> None:
    assert SceneReconstructionModule not in {
        atom.module for atom in gpd_mujoco_grasp_demo.blueprints
    }
    assert GPDGraspGenModule not in {atom.module for atom in vgn_mujoco_grasp_demo.blueprints}

    sim_atom = next(
        atom for atom in gpd_mujoco_grasp_demo.blueprints if atom.module is MujocoSimModule
    )
    controller_atom = next(
        atom
        for atom in gpd_mujoco_grasp_demo.blueprints
        if atom.module is PointcloudGraspDemoController
    )

    assert sim_atom.kwargs["enable_pointcloud"] is True
    assert sim_atom.kwargs["enable_depth"] is True
    assert controller_atom.kwargs["target_name"] == "sphere"
    assert controller_atom.kwargs["filter_collisions"] is False


def test_gpd_mujoco_grasp_demo_registry_name_is_stable() -> None:
    assert (
        all_blueprints["gpd-mujoco-grasp-demo"]
        == "dimos.robot.manipulators.xarm.blueprints.simulation:gpd_mujoco_grasp_demo"
    )


class _SceneRegistrationFake:
    def __init__(self, target: RegisteredObject) -> None:
        self.target = target
        self.prompts: list[list[str]] = []

    def set_prompts(self, text: list[str] | None = None, bboxes: object | None = None) -> None:
        del bboxes
        self.prompts.append(text or [])

    def get_registered_objects(self) -> list[RegisteredObject]:
        return [self.target]

    def get_object_by_object_id(self, object_id: str) -> RegisteredObject | None:
        return self.target if object_id == self.target.object_id else None

    def get_object_pointcloud_by_name(self, name: str) -> PointCloud2 | None:
        del name
        return None

    def get_object_pointcloud_by_object_id(self, object_id: str) -> PointCloud2 | None:
        del object_id
        return None

    def get_full_scene_pointcloud(
        self,
        exclude_object_id: str | None = None,
        depth_trunc: float = 2.0,
        voxel_size: float = 0.01,
    ) -> PointCloud2 | None:
        del exclude_object_id, depth_trunc, voxel_size
        return None


class _PointcloudGraspingFake:
    def __init__(self) -> None:
        self.calls: list[tuple[str, str | None, bool]] = []

    def generate_grasps(
        self,
        object_name: str = "object",
        object_id: str | None = None,
        filter_collisions: bool = True,
    ) -> str:
        self.calls.append((object_name, object_id, filter_collisions))
        return "generated"


def test_pointcloud_demo_controller_calls_pointcloud_generation_only(mocker: MockerFixture) -> None:
    target = RegisteredObject(
        object_id="obj-1",
        name="sphere",
        center=Vector3(0.45, 0.0, 0.18),
        size=Vector3(0.05, 0.05, 0.05),
    )
    scene = _SceneRegistrationFake(target)
    grasping = _PointcloudGraspingFake()
    controller = PointcloudGraspDemoController(
        target_name="sphere",
        detection_timeout_s=0.01,
        pointcloud_settle_s=0.0,
        filter_collisions=False,
    )
    controller._scene_registration = scene
    controller._grasping = grasping
    published = []
    controller.grasp_target_bounds.subscribe(published.append)
    forbidden = mocker.Mock(side_effect=AssertionError("execution API must not be called"))

    try:
        controller._run_demo()

        forbidden.assert_not_called()
        assert scene.prompts == [["sphere"]]
        assert grasping.calls == [("sphere", "obj-1", False)]
        assert published[0].label == "sphere:obj-1"
    finally:
        controller.stop()


@pytest.mark.skipif(
    not Path("packages/dimos-gpd-grasp-demo/.venv/bin/python").exists(),
    reason=(
        "GPD prepared runtime is not available; run `uv run dimos runtime prepare "
        "gpd-mujoco-grasp-demo --runtime dimos-gpd-grasp-demo` first"
    ),
)
@pytest.mark.skipif(
    os.environ.get("DIMOS_RUN_GPD_MUJOCO_SMOKE") != "1",
    reason="Set DIMOS_RUN_GPD_MUJOCO_SMOKE=1 to build the full GPD MuJoCo demo smoke",
)
def test_gpd_mujoco_grasp_demo_prepared_runtime_smoke_gate() -> None:
    coordinator = ModuleCoordinator.build(
        gpd_mujoco_grasp_demo,
        {"g": {"viewer": "none", "n_workers": 1}},
    )
    coordinator.stop()
