"""PawTrack agentic blueprint with optional prebuilt-map relocalization.

Install this next to ``pawtrack_agentic.py`` in the DimOS source tree, then
regenerate ``all_blueprints``:

    cp integration/pawtrack_agentic_relocalization.py \
        "$DIMOS_HOME/dimos/robot/unitree/go2/blueprints/agentic/"
    pytest -o addopts="" \
        "$DIMOS_HOME/dimos/robot/test_all_blueprints_generation.py"

Run with a prebuilt map:

    PYTHONPATH=src dimos run pawtrack-agentic-relocalization \
        --robot-ip <dog_ip> \
        -o relocalizationmodule.map_file=<premap_name>

This intentionally keeps the existing ``pawtrack-agentic`` blueprint
unchanged.
Use this blueprint only when you want ``subject_map_pose`` in addition to the
always-on ``subject_world_pose``.
"""

from __future__ import annotations

import os

from dimos.agents.mcp.mcp_client import McpClient
from dimos.agents.mcp.mcp_server import McpServer
from dimos.core.coordination.blueprints import autoconnect
from dimos.mapping.relocalization.module import RelocalizationModule
from dimos.robot.unitree.go2.blueprints.agentic._common_agentic import (
    _common_agentic,
)
from dimos.robot.unitree.go2.blueprints.smart.unitree_go2_spatial import (
    unitree_go2_spatial,
)

from pawtrack.skill_container import PawTrackSkillContainer
from pawtrack.system_prompt import PAWTRACK_PROMPT

_MODEL = os.environ.get("PAWTRACK_MODEL", "openai:gpt-5.1")

pawtrack_agentic_relocalization = autoconnect(
    unitree_go2_spatial,
    RelocalizationModule.blueprint(),
    McpServer.blueprint(),
    McpClient.blueprint(model=_MODEL, system_prompt=PAWTRACK_PROMPT),
    _common_agentic,
    PawTrackSkillContainer.blueprint(),
)
