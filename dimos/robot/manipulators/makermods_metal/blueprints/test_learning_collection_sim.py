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

"""The Quest collection twin records the real stack's profile from the Metal in MuJoCo."""

from pathlib import Path
from typing import Any

from dimos.control.coordinator import ControlCoordinator, ControlCoordinatorConfig
from dimos.control.teleop_coordinator import TeleopControlCoordinator
from dimos.core.coordination.blueprint_config.parser import BlueprintConfigParser
from dimos.core.coordination.blueprints import BlueprintAtom
from dimos.imitation.collection.recorder import CollectionModule
from dimos.imitation.dataprep.schema import DEFAULT_FPS
from dimos.robot.manipulators.makermods_metal.blueprints.learning_collection import (
    METAL_QUEST_PROFILE,
    metal_quest_collection,
)
from dimos.robot.manipulators.makermods_metal.blueprints.learning_collection_sim import (
    metal_quest_collection_sim,
)
from dimos.robot.manipulators.makermods_metal.config import metal_sim_joints
from dimos.simulation.engines.mujoco_sim_module import MujocoSimModule, MujocoSimModuleConfig


def _coordinator(atoms: tuple[BlueprintAtom, ...]) -> BlueprintAtom:
    [coordinator] = [atom for atom in atoms if issubclass(atom.module, ControlCoordinator)]
    return coordinator


def _task_shapes(atom: BlueprintAtom) -> list[tuple[Any, ...]]:
    return [
        (task.name, task.type, task.priority, task.joint_names, task.stream_bind)
        for task in atom.kwargs["tasks"]
    ]


def test_twin_records_the_quest_profile_from_the_simulated_metal(tmp_path: Path) -> None:
    parsed = BlueprintConfigParser(metal_quest_collection_sim).parse(
        ["--recorder.recording", str(tmp_path / "session"), "--episodes.task", "pick up the cube"],
        environ={},
    )
    atoms = metal_quest_collection_sim.active_blueprints
    real = metal_quest_collection.active_blueprints
    [sim] = [atom for atom in atoms if atom.module is MujocoSimModule]
    coordinator = _coordinator(atoms)

    assert issubclass(atoms[0].module, CollectionModule)
    assert atoms[0].kwargs["recording_schema"] == real[0].kwargs["recording_schema"]
    assert parsed.module_configs["recorder"]["recording"] == tmp_path / "session"
    assert metal_quest_collection_sim.remapping_map[(sim.name, "color_image")] == "wrist_image"
    camera = MujocoSimModuleConfig.model_validate(parsed.module_kwargs(sim.name))
    image = METAL_QUEST_PROFILE.observations["observation.images.wrist"]
    assert (camera.height, camera.width, 3) == image.shape
    assert camera.fps == DEFAULT_FPS
    assert issubclass(coordinator.module, TeleopControlCoordinator)
    control = ControlCoordinatorConfig.model_validate(parsed.module_kwargs(coordinator.name))
    [arm] = control.hardware
    assert (arm.adapter_type, arm.joints) == ("sim_mujoco", metal_sim_joints())
    assert _task_shapes(coordinator) == _task_shapes(_coordinator(real))
