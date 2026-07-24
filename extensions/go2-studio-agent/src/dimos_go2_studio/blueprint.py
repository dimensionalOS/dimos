"""External Blueprint loaded by ``dimos run dimos-go2-studio.go2``."""

from dimos.agents.mcp.mcp_server import McpServer
from dimos.core.coordination.blueprints import autoconnect
from dimos.navigation.movement_manager.movement_manager import MovementManager
from dimos.robot.unitree.go2.blueprints.smart.unitree_go2 import unitree_go2

from .skills import Go2StudioSkills

go2_studio_agentic = (
    autoconnect(
        # Keep the official Go2 camera, lidar map, navigation, obstacle avoidance,
        # and native Rerun visualization.  Deliberately do not include
        # SpatialMemory/PerceiveLoopSkill: this baseline must not continuously
        # capture and classify camera frames.
        unitree_go2,
        MovementManager.blueprint(
            max_nav_linear_speed=0.1,
            allow_nav_reverse=False,
        ),
        # DimOS ships its MCP server as a first-party module.  An external Agent
        # (Codex or another MCP client) can use these tools without an embedded
        # cloud model or API key.
        McpServer.blueprint(),
        Go2StudioSkills.blueprint(),
    )
    .global_config(nerf_speed=0.18)
)
