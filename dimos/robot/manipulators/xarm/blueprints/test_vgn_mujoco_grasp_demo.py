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

from dimos.manipulation.grasping.grasping import GraspingModule
from dimos.manipulation.grasping.target_grasp_demo_controller import TargetGraspDemoController
from dimos.manipulation.grasping.vgn_grasp_gen_module import VGNGraspGenModule
from dimos.perception.object_scene_registration import ObjectSceneRegistrationModule
from dimos.perception.reconstruction import SceneReconstructionModule
from dimos.robot.manipulators.xarm.blueprints.simulation import (
    vgn_mujoco_grasp_demo,
    xarm_perception_sim,
)
from dimos.simulation.engines.mujoco_sim_module import MujocoSimModule
from dimos.visualization.rerun.bridge import RerunBridgeModule


def test_vgn_mujoco_grasp_demo_blueprint_is_opt_in() -> None:
    demo_modules = {atom.module for atom in vgn_mujoco_grasp_demo.blueprints}
    existing_modules = {atom.module for atom in xarm_perception_sim.blueprints}

    assert demo_modules == {
        MujocoSimModule,
        SceneReconstructionModule,
        ObjectSceneRegistrationModule,
        VGNGraspGenModule,
        GraspingModule,
        TargetGraspDemoController,
        RerunBridgeModule,
    }
    assert VGNGraspGenModule not in existing_modules
    assert SceneReconstructionModule not in existing_modules


def test_vgn_mujoco_grasp_demo_uses_stable_rerun_topics() -> None:
    assert (
        vgn_mujoco_grasp_demo.remapping_map[(SceneReconstructionModule, "tsdf")] == "tsdf_surface"
    )
    assert vgn_mujoco_grasp_demo.remapping_map[(VGNGraspGenModule, "tsdf")] == "tsdf_surface"


def test_vgn_mujoco_grasp_demo_uses_target_controller_not_workspace_auto_generation() -> None:
    grasp_atom = next(
        atom for atom in vgn_mujoco_grasp_demo.blueprints if atom.module is VGNGraspGenModule
    )
    controller_atom = next(
        atom
        for atom in vgn_mujoco_grasp_demo.blueprints
        if atom.module is TargetGraspDemoController
    )
    reconstruction_atom = next(
        atom
        for atom in vgn_mujoco_grasp_demo.blueprints
        if atom.module is SceneReconstructionModule
    )

    assert grasp_atom.kwargs["auto_generate_on_tsdf"] is False
    assert grasp_atom.kwargs["quality_threshold"] == 0.05
    assert grasp_atom.kwargs["width_filter_max_voxels"] == 1000.0
    assert grasp_atom.kwargs["filter_candidates_to_target_bounds"] is True
    assert grasp_atom.kwargs["debug_export_dir"] == "/tmp/opencode/dimos-vgn-tsdf-debug"
    assert controller_atom.kwargs["target_name"] == "sphere"
    assert controller_atom.kwargs["cushion_m"] == 0.2
    assert grasp_atom.kwargs.get("auto_generate_on_tsdf") is False
    assert reconstruction_atom.kwargs["workspace_center"] == (0.45, 0.0, 0.18)
    assert reconstruction_atom.kwargs["resolution"] == 40

    sim_atom = next(
        atom for atom in vgn_mujoco_grasp_demo.blueprints if atom.module is MujocoSimModule
    )
    assert sim_atom.kwargs["headless"] is False
    assert sim_atom.kwargs["initial_joint_positions"] == [
        0.0,
        -0.247,
        0.0,
        0.909,
        0.0,
        1.15644,
        0.0,
    ]
