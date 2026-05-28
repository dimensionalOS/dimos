from dimos.experimental.dogops.blueprints import (
    DogOpsBlueprintMetadata,
    unitree_go2_dogops,
)
from dimos.robot.unitree.go2.blueprints.agentic.unitree_go2_dogops import (
    unitree_go2_dogops as exported_unitree_go2_dogops,
)


def test_unitree_go2_dogops_blueprint_fallback_is_no_key() -> None:
    blueprint_text = repr(exported_unitree_go2_dogops)
    assert "DogOpsSkillContainer" in blueprint_text
    assert "DogOpsObservationModule" in blueprint_text
    assert "DogOpsDashboardModule" in blueprint_text
    assert "DogOpsNavEvalModule" in blueprint_text
    assert "McpServer" in blueprint_text
    assert "McpClient" not in blueprint_text

    if isinstance(unitree_go2_dogops, DogOpsBlueprintMetadata):
        assert unitree_go2_dogops.name == "unitree-go2-dogops"
        assert unitree_go2_dogops.requires_mcp_client is False
