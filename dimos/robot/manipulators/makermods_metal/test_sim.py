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

"""The Metal MJCF is what the URDF and the sim module expect, and the sim blueprint wires it."""

from pathlib import Path

import mujoco

from dimos.control.coordinator import ControlCoordinator, ControlCoordinatorConfig
from dimos.core.coordination.blueprint_config.parser import BlueprintConfigParser
from dimos.manipulation.manipulation_module import ManipulationModule, ManipulationModuleConfig
from dimos.robot.manipulators.makermods_metal.blueprints.sim import metal_sim
from dimos.robot.manipulators.makermods_metal.config import (
    METAL_BENCH_OBJECTS,
    METAL_BENCH_PATH,
    METAL_GRIPPER_RANGE,
    METAL_JOINT3_LIMITS,
    METAL_MESHDIR,
    METAL_MJCF_PATH,
    metal_sim_joints,
)
from dimos.robot.manipulators.makermods_metal.mjcf import (
    METAL_CAMERA_NAME,
    METAL_SIM_SPEC,
    metal_mjcf_spec,
    write_metal_mjcf,
)
from dimos.simulation.engines.mujoco_engine import MujocoEngine
from dimos.simulation.engines.mujoco_sim_module import MujocoSimModule, MujocoSimModuleConfig
from dimos.simulation.engines.robot_sim_binding import resolve_robot_sim_binding
from dimos.simulation.utils.xml_parser import build_joint_mappings


def _name(model: mujoco.MjModel, kind: mujoco.mjtObj, index: int) -> str:
    return str(mujoco.mj_id2name(model, kind, index))


def test_checked_in_mjcf_is_the_generated_one(tmp_path: Path) -> None:
    write_metal_mjcf(tmp_path / "metal.xml")
    assert (tmp_path / "metal.xml").read_text() == METAL_MJCF_PATH.read_text()


def test_mjcf_drives_every_arm_joint_and_the_jaw() -> None:
    spec = metal_mjcf_spec()
    spec.meshdir = str(METAL_MESHDIR)
    model = spec.compile()
    assert [_name(model, mujoco.mjtObj.mjOBJ_JOINT, i) for i in range(model.njnt)] == [
        f"joint{i}" for i in range(1, 9)
    ]
    assert [model.actuator(i).trnid[0] for i in range(model.nu)] == list(range(7))
    assert tuple(model.actuator("gripper").ctrlrange) == METAL_GRIPPER_RANGE
    assert tuple(model.joint("joint3").range) == METAL_JOINT3_LIMITS
    # joint8 mirrors joint7, as the URDF's mimic says.
    assert model.neq == 1
    assert list(model.eq_data[0][:2]) == [0.0, -1.0]
    camera = model.camera(METAL_CAMERA_NAME)
    assert _name(model, mujoco.mjtObj.mjOBJ_BODY, camera.bodyid[0]) == "gripper_base"


def test_bench_composes_with_the_arm_the_way_the_sim_module_reads_it() -> None:
    scene = mujoco.MjSpec.from_file(str(METAL_BENCH_PATH))
    robot = mujoco.MjSpec.from_file(str(METAL_MJCF_PATH))
    robot.meshdir = str(METAL_MESHDIR)
    scene.attach(robot, prefix=None, frame=scene.worldbody.add_frame())
    model = scene.compile()
    binding = resolve_robot_sim_binding(model, METAL_SIM_SPEC, build_joint_mappings(None, model))
    assert [mapping.name for mapping in binding.joint_mappings] == [
        f"/joint{i}" for i in range(1, 8)
    ]
    assert mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, f"/{METAL_CAMERA_NAME}") >= 0
    for body in METAL_BENCH_OBJECTS:
        assert model.body(body).jntnum[0] == 1
    # The bench's free bodies come first in the model; none of them is the arm's root.
    engine = MujocoEngine(
        config_path=METAL_MJCF_PATH, headless=True, model=model, robot_sim_spec=METAL_SIM_SPEC
    )
    assert engine.root_qpos_adr is None


def test_sim_blueprint_wires_the_arm_the_jaw_and_the_bench() -> None:
    parsed = BlueprintConfigParser(metal_sim).parse([], environ={})
    atoms = metal_sim.blueprints
    [sim] = [a for a in atoms if a.module is MujocoSimModule]
    [coordinator] = [a for a in atoms if issubclass(a.module, ControlCoordinator)]
    [planner] = [a for a in atoms if a.module is ManipulationModule]
    sim_config = MujocoSimModuleConfig.model_validate(parsed.module_kwargs(sim.name))
    assert sim_config.dof == 6
    assert sim_config.tracked_bodies == list(METAL_BENCH_OBJECTS)
    assert sim_config.robot_mjcf == METAL_MJCF_PATH
    assert sim_config.scene_xml == METAL_BENCH_PATH
    assert sim_config.robot_sim_spec == METAL_SIM_SPEC
    assert sim_config.camera_name == METAL_CAMERA_NAME
    control = ControlCoordinatorConfig.model_validate(parsed.module_kwargs(coordinator.name))
    [arm] = control.hardware
    assert (arm.adapter_type, arm.joints) == ("sim_mujoco", metal_sim_joints())
    trajectory, gripper = control.tasks
    assert trajectory.joint_names == metal_sim_joints()[:6]
    assert (gripper.type, gripper.joint_names, gripper.params["open_at"]) == (
        "gripper",
        ["arm/joint7"],
        "lower",
    )
    planner_config = ManipulationModuleConfig.model_validate(parsed.module_kwargs(planner.name))
    assert planner_config.gripper_tasks == {"arm": gripper.name}
    assert planner_config.model.home_joints == sim_config.reset_joint_positions
