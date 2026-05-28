"""unitree-go2-hackathon blueprint.

Build chain (avoids SecurityModule which requires CUDA/EdgeTAM):
  unitree_go2_basic → unitree_go2 → +SpatialMemory +PerceiveLoopSkill
                                    (SecurityModule intentionally excluded)
Then adds: McpServer + hackathon skill containers.
"""

from dimos.agents.mcp.mcp_client import McpClient
from dimos.agents.mcp.mcp_server import McpServer
from dimos.agents.skills.navigation import NavigationSkillContainer
from dimos.agents.skills.speak_skill import SpeakSkill
from dimos.agents.web_human_input import WebInput
from dimos.core.coordination.blueprints import autoconnect
from dimos.perception.perceive_loop_skill import PerceiveLoopSkill
from dimos.perception.spatial_perception import SpatialMemory
from dimos.robot.unitree.go2.blueprints.smart.unitree_go2 import unitree_go2
from dimos.robot.unitree.go2.connection import GO2Connection
from dimos.robot.unitree.unitree_skill_container import UnitreeSkillContainer
from dimos.robot.unitree.go2.blueprints.hackathon.dog_mode import DogModeModule
from dimos.robot.unitree.go2.blueprints.hackathon.find import FindSkillContainer
from dimos.robot.unitree.go2.blueprints.hackathon.perception_loop import PerceptionLoopModule
from dimos.robot.unitree.go2.blueprints.hackathon.smart_follow import SmartFollowSkillContainer

# Spatial layer without SecurityModule (no CUDA required)
_unitree_go2_spatial_no_security = autoconnect(
    unitree_go2,
    SpatialMemory.blueprint(),
    PerceiveLoopSkill.blueprint(),
).global_config(n_workers=8)

_hackathon_agentic = autoconnect(
    NavigationSkillContainer.blueprint(),
    PerceptionLoopModule.blueprint(camera_info=GO2Connection.camera_info_static),
    SmartFollowSkillContainer.blueprint(camera_info=GO2Connection.camera_info_static),
    FindSkillContainer.blueprint(),
    DogModeModule.blueprint(camera_info=GO2Connection.camera_info_static),
    UnitreeSkillContainer.blueprint(),
    WebInput.blueprint(),
    SpeakSkill.blueprint(),
)

unitree_go2_hackathon = autoconnect(
    _unitree_go2_spatial_no_security,
    McpServer.blueprint(),
    McpClient.blueprint(),
    _hackathon_agentic,
)

__all__ = ["unitree_go2_hackathon"]
