"""Version-controlled copy of the PawTrack `dimos run` blueprint.

`dimos run <name>` resolves names from a registry that the generator builds by
scanning only the dimos source tree, so the blueprint object has to live inside
that tree. This file is the canonical copy; install it with:

    cp integration/pawtrack_agentic.py \\
        "$DIMOS_HOME/dimos/robot/unitree/go2/blueprints/agentic/"
    pytest "$DIMOS_HOME/dimos/robot/test_all_blueprints_generation.py"  # regen

Then (with the `pawtrack` package importable):

    PYTHONPATH=src dimos run pawtrack-agentic --robot-ip <dog_ip>

It rebuilds the four parts of the stock ``unitree-go2-agentic`` blueprint but
swaps in an ``McpClient`` with the PawTrack prompt and adds the PawTrack
subject-tracking skill container -- reusing the stock stack, no duplicate
agent.
"""

from __future__ import annotations

import os

from dimos.agents.mcp.mcp_client import McpClient
from dimos.agents.mcp.mcp_server import McpServer
from dimos.core.coordination.blueprints import autoconnect
from dimos.robot.unitree.go2.blueprints.agentic._common_agentic import (
    _common_agentic,
)
from dimos.robot.unitree.go2.blueprints.smart.unitree_go2_spatial import (
    unitree_go2_spatial,
)

from pawtrack.skill_container import PawTrackSkillContainer
from pawtrack.system_prompt import PAWTRACK_PROMPT

_MODEL = os.environ.get("PAWTRACK_MODEL", "openai:gpt-5.1")

pawtrack_agentic = autoconnect(
    unitree_go2_spatial,
    McpServer.blueprint(),
    McpClient.blueprint(model=_MODEL, system_prompt=PAWTRACK_PROMPT),
    _common_agentic,
    PawTrackSkillContainer.blueprint(),
)
