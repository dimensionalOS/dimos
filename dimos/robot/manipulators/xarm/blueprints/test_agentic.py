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

import platform
import runpy
import sys

import pytest

# The tests extra excludes yourdfpy on Linux ARM because embreex has no wheel.
if sys.platform == "linux" and platform.machine() == "aarch64":
    pytest.importorskip(
        "yourdfpy", reason="yourdfpy is unavailable in the Linux ARM test environment"
    )

from dimos.agents.agent import Agent
from dimos.robot.manipulators.common.agent_prompts import MANIPULATION_AGENT_SYSTEM_PROMPT
from dimos.robot.manipulators.xarm.blueprints import agentic, grasp
from dimos.robot.manipulators.xarm.blueprints.agentic import xarm_perception_sim_agent


def test_perception_sim_scan_has_concrete_scene_queries() -> None:
    prompt = next(
        atom.kwargs["system_prompt"]
        for atom in xarm_perception_sim_agent.solidify().active_blueprints
        if atom.module is Agent
    )
    labels = (
        "red cube, blue cube, green block, yellow bar, orange cylinder, "
        "purple ball, teal ball, basket, red bowl, blue bowl"
    )

    assert f'localize(objects="{labels}", start=-30, duration=30)' in prompt
    assert '"Scan for objects"' in prompt
    assert "Report only objects returned by localize" in prompt
    scan_workflow = prompt.split("# Scan Workflow for the Default Simulation Scene")[1]
    assert "For each scan, including a repeated request:" in scan_workflow
    assert (
        scan_workflow.index("Call **state** until it reads ready before collecting viewpoints")
        < scan_workflow.index("Collect fresh viewpoints")
        < scan_workflow.index("Call **localize**")
    )


@pytest.fixture(params=["xarm_grasp_agent", "xarm_grasp_graspgenx_agent"])
def grasp_agent_name(request):
    return request.param


def _grasp_prompt(monkeypatch, name: str, simulated: bool) -> str:
    # Evaluate composition without changing the imported blueprint module.
    monkeypatch.setattr(grasp, "SIMULATED", simulated)
    blueprints = runpy.run_path(agentic.__file__)
    return next(
        atom.kwargs["system_prompt"]
        for atom in blueprints[name].solidify().active_blueprints
        if atom.module is Agent
    )


def test_simulated_grasp_scan_uses_its_own_scene_labels(monkeypatch, grasp_agent_name):
    prompt = _grasp_prompt(monkeypatch, grasp_agent_name, simulated=True)
    labels = ", ".join(grasp.XARM_GRASP_PROMPTS)

    assert f'localize(objects="{labels}", start=-30, duration=30)' in prompt
    assert "red cube" not in prompt


def test_hardware_grasp_agent_does_not_assume_simulation_objects(monkeypatch, grasp_agent_name):
    prompt = _grasp_prompt(monkeypatch, grasp_agent_name, simulated=False)

    assert prompt == MANIPULATION_AGENT_SYSTEM_PROMPT
