# Copyright 2026 Dimensional Inc.
# Licensed under the Apache License, Version 2.0.

from pathlib import Path

from dimos.agents.mcp.mcp_client import McpClient
from dimos.agents.mcp.mcp_server import McpServer
from dimos.agents.skills.navigation import NavigationSkillContainer
from dimos.benchmark.vlnce_r2r.blueprint import (
    VlnceObservationRecorder,
    vlnce_r2r_eval_blueprint,
)
from dimos.benchmark.vlnce_r2r.connection import VlnceConnection
from dimos.benchmark.vlnce_r2r.evaluation import SYSTEM_PROMPT
from dimos.benchmark.vlnce_r2r.prompt import vlnce_task_prompt
from dimos.navigation.replanning_a_star.module import ReplanningAStarPlanner
from dimos.perception.experimental.spatial_perception import SpatialMemory


def test_blueprint_is_only_the_public_navigation_stack(tmp_path: Path) -> None:
    blueprint = vlnce_r2r_eval_blueprint(
        socket_path=tmp_path / "gateway.sock", recording_path=tmp_path / "recording.db"
    )
    modules = {atom.module for atom in blueprint.blueprints}
    assert {
        VlnceConnection,
        ReplanningAStarPlanner,
        SpatialMemory,
        NavigationSkillContainer,
        VlnceObservationRecorder,
    } <= modules
    assert McpClient not in modules and McpServer not in modules
    assert blueprint.remapping_map[(ReplanningAStarPlanner.name, "nav_cmd_vel")] == "cmd_vel"
    assert blueprint.global_config_overrides["robot_model"] == "vlnce_habitat_cylinder"
    recorder = next(
        atom for atom in blueprint.blueprints if atom.module is VlnceObservationRecorder
    )
    assert recorder.kwargs["stream_sample_intervals"]["depth_pointcloud"] == 1.0
    assert recorder.kwargs["record_tf"] is False


def test_agent_prompts_require_visual_closed_loop_submission() -> None:
    instruction = "Exit the bedroom, enter the bathroom, wait at the toilet. "
    prompt = vlnce_task_prompt(instruction)
    assert instruction in prompt and "submit_route()" in prompt
    for guidance in (
        "`color_image`",
        "`global_costmap`",
        "Never issue a long or unbounded movement loop",
        "navigate_to_position(x, y)",
        "fresh RGB evidence",
    ):
        assert guidance in SYSTEM_PROMPT
