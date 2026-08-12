# Copyright 2026 Dimensional Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from pathlib import Path

import numpy as np
import pytest

from dimos.agents.mcp.mcp_client import McpClient
from dimos.agents.mcp.mcp_server import McpServer
from dimos.agents.skills.navigation import NavigationSkillContainer
from dimos.benchmark.vlnce_r2r.blueprint import (
    VlnceObservationRecorder,
    vlnce_r2r_eval_blueprint,
)
from dimos.benchmark.vlnce_r2r.connection import VlnceConnection
from dimos.benchmark.vlnce_r2r.evaluation import SYSTEM_PROMPT
from dimos.benchmark.vlnce_r2r.models import VlnceTaskManifest
from dimos.benchmark.vlnce_r2r.prompt import vlnce_task_prompt
from dimos.core.global_config import global_config
from dimos.memory2.store.sqlite import SqliteStore
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.navigation.replanning_a_star.module import ReplanningAStarPlanner
from dimos.perception.experimental.spatial_perception import SpatialMemory
from dimos.visualization.rerun.bridge import RerunBridgeModule
from dimos.visualization.rerun.constants import ViewerBackend


def test_case_bound_blueprint_contains_only_public_navigation_stack(tmp_path) -> None:
    blueprint = vlnce_r2r_eval_blueprint(
        socket_path=tmp_path / "public.sock",
        attempt_id="attempt-1",
        case_id="case-1",
        episode_id="515",
        recording_path=tmp_path / "recording.db",
    )
    modules = [atom.module for atom in blueprint.blueprints]

    assert modules.count(VlnceConnection) == 1
    assert modules.count(ReplanningAStarPlanner) == 1
    assert modules.count(SpatialMemory) == 1
    assert modules.count(NavigationSkillContainer) == 1
    assert modules.count(VlnceObservationRecorder) == 1
    assert McpClient not in modules
    assert McpServer not in modules
    assert all("runtime.vlnce_runtime" not in module.__module__ for module in modules)

    connection = next(atom for atom in blueprint.blueprints if atom.module is VlnceConnection)
    planner = next(atom for atom in blueprint.blueprints if atom.module is ReplanningAStarPlanner)
    spatial_memory = next(atom for atom in blueprint.blueprints if atom.module is SpatialMemory)
    recorder = next(
        atom for atom in blueprint.blueprints if atom.module is VlnceObservationRecorder
    )
    assert connection.kwargs["socket_path"] == str(tmp_path / "public.sock")
    assert connection.kwargs["attempt_id"] == "attempt-1"
    assert connection.kwargs["episode_id"] == "515"
    assert planner.kwargs["robot_width"] == 0.0
    assert planner.kwargs["robot_rotation_diameter"] == 0.0
    assert spatial_memory.kwargs["db_path"] == str(tmp_path / "spatial-memory/chromadb")
    assert spatial_memory.kwargs["visual_memory_path"] == str(
        tmp_path / "spatial-memory/visual-memory.pkl"
    )
    assert spatial_memory.kwargs["output_dir"] == str(tmp_path / "spatial-memory")
    assert recorder.kwargs["stream_codecs"] == {"depth_image": "pickle"}
    assert blueprint.global_config_overrides["robot_model"] == "vlnce_habitat_cylinder"
    assert blueprint.global_config_overrides["transport"] == "zenoh"
    assert blueprint.global_config_overrides["configure_system"] is False


def test_blueprint_wires_static_map_motion_and_public_recording(tmp_path) -> None:
    blueprint = vlnce_r2r_eval_blueprint(
        socket_path=tmp_path / "public.sock",
        attempt_id="attempt-1",
        case_id="case-1",
        episode_id="515",
    )

    assert blueprint.remapping_map[(ReplanningAStarPlanner.name, "nav_cmd_vel")] == "cmd_vel"
    assert (
        blueprint.remapping_map[(ReplanningAStarPlanner.name, "odometry")]
        == "unused_benchmark_odometry"
    )
    assert (
        blueprint.remapping_map[(VlnceObservationRecorder.name, "depth_pointcloud")] == "pointcloud"
    )
    assert (VlnceObservationRecorder.name, "global_map") not in blueprint.remapping_map


def test_task_prompt_preserves_instruction_and_explains_submission() -> None:
    instruction = "Exit the bedroom, enter the bathroom, wait at the toilet. "

    prompt = vlnce_task_prompt(instruction)

    assert instruction in prompt
    assert "submit_route()" in prompt
    assert "VLN-CE STOP" in prompt
    assert "irreversible" in prompt
    assert "does not reveal whether the route succeeded" in prompt


def test_system_prompt_names_observation_streams_and_requires_closed_loop_evidence() -> None:
    assert "`color_image`" in SYSTEM_PROMPT
    assert "`global_costmap`" in SYSTEM_PROMPT
    assert "`depth_pointcloud` is only" in SYSTEM_PROMPT
    assert "Never issue a long or unbounded movement loop" in SYSTEM_PROMPT
    assert "without a duration" in SYSTEM_PROMPT
    assert "navigate_to_position(x, y)" in SYSTEM_PROMPT
    assert "navigation_status()" in SYSTEM_PROMPT
    assert "ordered phase checklist" in SYSTEM_PROMPT
    assert "do not return to the bedroom" in SYSTEM_PROMPT
    assert "fresh RGB evidence" in SYSTEM_PROMPT
    assert "submit immediately" in SYSTEM_PROMPT


def test_depth_image_pickle_codec_preserves_metric_values(tmp_path: Path) -> None:
    depth = Image(
        data=np.array([[0.25, 1.5]], dtype=np.float32),
        format=ImageFormat.DEPTH,
    )

    with SqliteStore(path=str(tmp_path / "recording.db")) as store:
        stream = store.stream("depth_image", Image, codec="pickle")
        stream.append(depth)
        restored = stream.last().data

    assert restored.format is ImageFormat.DEPTH
    np.testing.assert_array_equal(restored.data, depth.data)


@pytest.mark.parametrize(("viewer", "has_rerun"), [("none", False), ("rerun", True)])
def test_viewer_choice_changes_only_public_presentation_modules(
    tmp_path: Path, viewer: ViewerBackend, has_rerun: bool
) -> None:
    case_path = Path(__file__).parent / "cases/mp3d-example-episode-515/task.json"
    fingerprint_before = VlnceTaskManifest.model_validate_json(case_path.read_bytes()).fingerprint
    original = global_config.model_dump()
    try:
        global_config.update(viewer=viewer)
        blueprint = vlnce_r2r_eval_blueprint(
            socket_path=tmp_path / "public.sock",
            attempt_id="attempt-1",
            case_id="case-1",
            episode_id="515",
            recording_path=tmp_path / "recording.db",
        )
    finally:
        global_config.update(**original)

    modules = {atom.module for atom in blueprint.blueprints}
    fingerprint_after = VlnceTaskManifest.model_validate_json(case_path.read_bytes()).fingerprint
    assert (RerunBridgeModule in modules) is has_rerun
    assert fingerprint_after == fingerprint_before
