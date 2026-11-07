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

"""The lab blueprint takes its session at launch and hands the actor no movement tool."""

import inspect
from pathlib import Path

from dimos.agents.mcp.mcp_server import McpServer
from dimos.autoresearch.module import AutoresearchModule, AutoresearchModuleConfig
from dimos.autoresearch.truth_scene import TruthSceneModule
from dimos.core.coordination.blueprint_config.parser import BlueprintConfigParser
from dimos.imitation.collection.episode_monitor import EpisodeMonitorModule
from dimos.imitation.collection.recorder import CollectionModule
from dimos.manipulation.manipulation_skills import ManipulationSkills
from dimos.manipulation.pick_and_place_module import PickAndPlaceModule
from dimos.perception.experimental.object_scene_registration import ObjectSceneRegistrationModule
from dimos.robot.manipulators.makermods_metal.blueprints.autoresearch import autoresearch_metal_sim
from dimos.simulation.engines.mujoco_sim_module import MujocoSimModule

LAB_TOOLS = {
    "task",
    "scene",
    "submit",
    "run",
    "stop_run",
    "results",
    "leaderboard",
    "freeze",
    "note",
}


def _skills(module: type) -> set[str]:
    return {
        name
        for name in dir(module)
        if hasattr(inspect.getattr_static(module, name, None), "__skill__")
    }


def test_lab_blueprint_takes_the_session_at_launch_and_offers_only_the_lab_tools(
    tmp_path: Path,
) -> None:
    session = tmp_path / "lift"
    parsed = BlueprintConfigParser(autoresearch_metal_sim).parse(
        [
            "--autoresearchmodule.session-dir",
            str(session),
            "--autoresearchmodule.task",
            "lift the cube",
            "--autoresearchmodule.budget.dev-trials",
            "2",
            "--recorder.recording",
            str(session / "recording"),
            "--episodes.task",
            "lift the cube",
        ],
        environ={},
    )
    atoms = autoresearch_metal_sim.active_blueprints
    modules = {atom.module for atom in atoms}

    assert issubclass(atoms[0].module, CollectionModule)
    assert {AutoresearchModule, TruthSceneModule, McpServer, EpisodeMonitorModule} <= modules
    assert MujocoSimModule in modules
    assert not modules & {ManipulationSkills, PickAndPlaceModule, ObjectSceneRegistrationModule}
    assert {module for module in modules if _skills(module)} == {AutoresearchModule, McpServer}
    assert _skills(AutoresearchModule) == LAB_TOOLS
    assert autoresearch_metal_sim.remapping_map[("recorder", "wrist_image")] == "color_image"

    config = AutoresearchModuleConfig.model_validate(parsed.module_kwargs("autoresearchmodule"))
    assert (config.session_dir, config.task, config.budget.dev_trials) == (
        session,
        "lift the cube",
        2,
    )
    assert parsed.module_configs["recorder"]["recording"] == session / "recording"
    assert parsed.module_kwargs("episodes")["task"] == "lift the cube"
