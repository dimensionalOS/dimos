"""Smoke tests for the mission-ready external Go2 blueprint."""

from dimos_go2_studio.blueprint import go2_studio_agentic

from dimos.core.coordination.blueprints import Blueprint


def test_go2_studio_blueprint_reuses_dimos_autonomy_and_visualization() -> None:
    assert isinstance(go2_studio_agentic, Blueprint)

    module_names = {item.module.__name__ for item in go2_studio_agentic.blueprints}

    assert "GO2Connection" in module_names
    assert "WavefrontFrontierExplorer" in module_names
    assert "McpServer" in module_names
    assert "UnitreeSkillContainer" in module_names
    assert "SemanticWorld" in module_names
    assert "MissionExecutor" in module_names
    assert "WebsocketVisModule" in module_names
    assert "Go2StudioSkills" in module_names


def test_go2_studio_is_lightweight_and_has_no_continuous_frame_agent() -> None:
    module_names = [item.module.__name__ for item in go2_studio_agentic.blueprints]

    assert module_names.count("GO2Connection") == 1
    assert module_names.count("Go2StudioSkills") == 1
    assert module_names.count("UnitreeSkillContainer") == 1
    assert module_names.count("SemanticWorld") == 1
    assert module_names.count("MissionExecutor") == 1
    assert "DirectVelocityMissionController" not in module_names
    assert "SpatialMemory" not in module_names
    assert "PerceiveLoopSkill" not in module_names
    assert "NavigationSkillContainer" not in module_names
    assert "McpClient" not in module_names
    assert "SpeakSkill" not in module_names

    movement_manager = next(
        item
        for item in go2_studio_agentic.blueprints
        if item.module.__name__ == "MovementManager"
    )
    assert movement_manager.kwargs["max_nav_linear_speed"] == 0.1
    assert movement_manager.kwargs["allow_nav_reverse"] is False

    mission_executor = next(
        item
        for item in go2_studio_agentic.blueprints
        if item.module.__name__ == "MissionExecutor"
    )
    assert [ref.name for ref in mission_executor.module_refs] == [
        "_navigation",
        "_destination_resolver",
    ]
