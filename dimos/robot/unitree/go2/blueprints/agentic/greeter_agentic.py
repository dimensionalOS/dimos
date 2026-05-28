"""Version-controlled copy of the `greeter-agentic` `dimos run` blueprint.

The greeter dog: wander (DimOS patrol) until a person sitting on a chair is
found, pick a random not-yet-greeted one, walk over, circle to face them, and
wave hello -- then resume the patrol. The autonomous loop lives in
``GreeterSkillContainer``; start/stop it via MCP.

Install it next to the stock agentic blueprints in the DimOS source tree and
regenerate the registry:

    cp integration/greeter_agentic.py \\
        "$DIMOS_HOME/dimos/robot/unitree/go2/blueprints/agentic/"
    pytest -o addopts="" \\
        "$DIMOS_HOME/dimos/robot/test_all_blueprints_generation.py"

Then (with the `pawtrack` package importable):

    PYTHONPATH=src PAWTRACK_MODEL=openai:deepseek-chat \
      dimos run greeter-agentic --robot-ip <dog_ip>
    dimos mcp call start_greeting     # or tell the agent: "go greet people"
    dimos mcp call stop_greeting      # halt

This is the stock ``unitree-go2-agentic`` general agent **plus** the greeter
skill container -- i.e. a normal Go2 agent that can navigate, speak, run sport
commands, etc., and *also* knows how to greet. It uses the agent's default
general system prompt (no narrow greeter prompt); the agent discovers the
greeting from the skill docstrings (``start_greeting`` / ``stop_greeting`` /
``greeter_status`` / ``wave_hello``) and calls it when asked, like any other
skill. The wander/greet loop itself is autonomous once started. Model defaults
to the agent's default; override with ``PAWTRACK_MODEL``.
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

from pawtrack.greeter_container import GreeterSkillContainer

_MODEL = os.environ.get("PAWTRACK_MODEL")

# Stock unitree-go2-agentic (general agent, default prompt) + the greeter skill.
_agent = (
    McpClient.blueprint(model=_MODEL) if _MODEL else McpClient.blueprint()
)

greeter_agentic = autoconnect(
    unitree_go2_spatial,
    McpServer.blueprint(),
    _agent,
    _common_agentic,
    GreeterSkillContainer.blueprint(),
)
