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

"""DimSim-specific Go2 agent composition.

Run it through the normal DimOS CLI without adding a generated core registry
entry:

    uv run python -m dimos.simulation.dimsim.agentic_blueprint \
      --simulation dimsim run unitree-go2-agentic-dimsim
"""

from dimos.agents.mcp.mcp_client import McpClient
from dimos.agents.skills.navigation import NavigationSkillContainer
from dimos.core.coordination.blueprints import autoconnect
from dimos.perception.perceive_loop_skill import PerceiveLoopSkill
from dimos.perception.spatial_perception import SpatialMemory
from dimos.robot.unitree.go2.blueprints.agentic.unitree_go2_agentic import (
    unitree_go2_agentic,
)
from dimos.robot.unitree.go2.connection import GO2Connection
from dimos.robot.unitree.unitree_skill_container import UnitreeSkillContainer
from dimos.simulation.dimsim.go2_connection import DimSimGO2Connection
from dimos.simulation.dimsim.mcp_client import DimSimMcpClient
from dimos.simulation.dimsim.navigation_skill_container import (
    DimSimNavigationSkillContainer,
)
from dimos.simulation.dimsim.perceive_loop_skill import DimSimPerceiveLoopSkill
from dimos.simulation.dimsim.spatial_memory import DimSimSpatialMemory
from dimos.simulation.dimsim.unitree_skill_container import DimSimUnitreeSkillContainer

unitree_go2_agentic_dimsim = autoconnect(
    unitree_go2_agentic.disabled_modules(
        UnitreeSkillContainer,
        McpClient,
        NavigationSkillContainer,
        PerceiveLoopSkill,
        SpatialMemory,
        GO2Connection,
    ),
    DimSimGO2Connection.blueprint(),
    DimSimMcpClient.blueprint(),
    DimSimNavigationSkillContainer.blueprint(),
    DimSimPerceiveLoopSkill.blueprint(),
    DimSimSpatialMemory.blueprint(),
    DimSimUnitreeSkillContainer.blueprint(),
).global_config(simulation="dimsim")

_BLUEPRINT_NAME = "unitree-go2-agentic-dimsim"


def main() -> None:
    from dimos.robot.all_blueprints import all_blueprints

    all_blueprints[_BLUEPRINT_NAME] = (
        "dimos.simulation.dimsim.agentic_blueprint:unitree_go2_agentic_dimsim"
    )

    # Import after registering so get_all_blueprints computes suggestions from
    # the augmented registry and the standard CLI still owns lifecycle/logging.
    from dimos.robot.cli.dimos import cli_main

    cli_main()


if __name__ == "__main__":
    main()
