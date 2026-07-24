"""Smoke tests for the mission-ready external Go2 blueprint."""

from dimos_go2_studio.blueprint import go2_studio_agentic

from dimos.core.coordination.blueprints import Blueprint


def test_go2_studio_blueprint_reuses_dimos_autonomy_and_visualization() -> None:
    assert isinstance(go2_studio_agentic, Blueprint)

    module_names = {item.module.__name__ for item in go2_studio_agentic.blueprints}

    assert "GO2Connection" in module_names
    assert "WavefrontFrontierExplorer" in module_names
    assert "NavigationSkillContainer" in module_names
    assert "SpatialMemory" in module_names
    assert "McpClient" in module_names
    assert "WebsocketVisModule" in module_names
    assert "Go2StudioSkills" in module_names


def test_go2_studio_adds_no_second_direct_velocity_controller() -> None:
    module_names = [item.module.__name__ for item in go2_studio_agentic.blueprints]

    assert module_names.count("GO2Connection") == 1
    assert module_names.count("Go2StudioSkills") == 1
    assert "DirectVelocityMissionController" not in module_names
    assert "UnitreeSkillContainer" not in {
        item.module.__name__ for item in go2_studio_agentic.active_blueprints
    }
    assert "PersonFollowSkillContainer" not in {
        item.module.__name__ for item in go2_studio_agentic.active_blueprints
    }

    movement_manager = next(
        item
        for item in go2_studio_agentic.blueprints
        if item.module.__name__ == "MovementManager"
    )
    assert movement_manager.kwargs["max_nav_linear_speed"] == 0.1
    assert movement_manager.kwargs["allow_nav_reverse"] is False
