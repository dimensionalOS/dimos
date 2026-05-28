from __future__ import annotations

from dataclasses import dataclass

from dimos.experimental.dogops.dashboard import DogOpsDashboardModule
from dimos.experimental.dogops.nav_eval import DogOpsNavEvalModule
from dimos.experimental.dogops.observation_module import DogOpsObservationModule
from dimos.experimental.dogops.skills import DogOpsSkillContainer


@dataclass(frozen=True)
class DogOpsBlueprintMetadata:
    name: str
    modules: tuple[str, ...]
    robot_model: str = "unitree_go2"
    requires_mcp_client: bool = False
    fallback: bool = True


def build_unitree_go2_dogops_blueprint() -> object:
    try:  # pragma: no cover - exercised only inside a full DimOS checkout.
        from dimos.agents.mcp.mcp_server import McpServer
        from dimos.core.coordination.blueprints import autoconnect
        from dimos.robot.unitree.go2.blueprints.smart.unitree_go2 import unitree_go2_markers
    except ModuleNotFoundError:
        return DogOpsBlueprintMetadata(
            name="unitree-go2-dogops",
            modules=(
                "unitree_go2_markers",
                "DogOpsObservationModule",
                "DogOpsSkillContainer",
                "McpServer",
                "DogOpsDashboardModule",
                "DogOpsNavEvalModule",
            ),
        )

    return autoconnect(
        unitree_go2_markers,
        DogOpsObservationModule.blueprint(),
        DogOpsSkillContainer.blueprint(),
        DogOpsDashboardModule.blueprint(),
        DogOpsNavEvalModule.blueprint(),
        McpServer.blueprint(),
    ).global_config(n_workers=12, robot_model="unitree_go2")


unitree_go2_dogops = build_unitree_go2_dogops_blueprint()
