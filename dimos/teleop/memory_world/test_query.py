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

from collections.abc import Iterator
import json
from pathlib import Path
from types import SimpleNamespace

import numpy as np
from pydantic import ValidationError
import pytest

from dimos.memory.store.sqlite import SqliteStore
from dimos.teleop.memory_world.module import MemoryWorldModule
from dimos.teleop.memory_world.query import MemoryQueryResult


def _empty_store(path: Path) -> None:
    store = SqliteStore(path=str(path))
    store.start()
    store.stop()


@pytest.fixture
def memory_world(tmp_path: Path) -> Iterator[MemoryWorldModule]:
    db_path = tmp_path / "recording.db"
    _empty_store(db_path)
    module = MemoryWorldModule(store_path=str(db_path))
    yield module
    module.stop()


def test_memory_query_result_validates_spatial_geometry() -> None:
    result = MemoryQueryResult.model_validate(
        {
            "answer": "The east room is brighter.",
            "focus_point": [1, 2, 0],
            "regions": [{"points": [[0, 0, 0], [2, 0, 0], [2, 3, 0]]}],
            "evidence_paths": [{"points": [[0, 0, 0.1], [1, 2, 0.1]]}],
        }
    )

    assert result.focus_point == (1.0, 2.0, 0.0)
    assert result.regions[0].opacity == 0.35
    assert result.evidence_paths[0].color == "#ffd166"


@pytest.mark.parametrize(
    "result",
    [
        {"answer": "bad point", "focus_point": [float("nan"), 0, 0]},
        {"answer": "bad color", "points": [{"position": [0, 0, 0], "color": "red"}]},
        {"answer": "too many points", "points": [{"position": [0, 0, 0]}] * 129},
    ],
)
def test_memory_query_result_rejects_unsafe_geometry(result: dict[str, object]) -> None:
    with pytest.raises(ValidationError):
        MemoryQueryResult.model_validate(result)


def test_bare_store_name_resolves_through_data_registry(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    db_path = tmp_path / "recording.db"
    _empty_store(db_path)
    monkeypatch.setattr("dimos.teleop.memory_world.module.get_data", lambda name: db_path)

    module = MemoryWorldModule(store_path="recording.db")
    try:
        assert module.config.store_path == str(db_path.resolve())
    finally:
        module.stop()


def test_lidar_cloud_does_not_fall_back_to_unrelated_pickle(
    memory_world: MemoryWorldModule, monkeypatch: pytest.MonkeyPatch
) -> None:
    memory_world.config.cloud_source = "lidar"
    monkeypatch.setattr(memory_world, "_build_voxel_cloud_from_lidar", lambda: None)

    with pytest.raises(RuntimeError, match="produced no cloud"):
        memory_world._build_cloud()


def test_top_down_map_uses_the_rendered_cloud(
    memory_world: MemoryWorldModule, tmp_path: Path
) -> None:
    positions = np.asarray([[10.0, 20.0, 0.5], [14.0, 22.0, 0.5]], dtype=np.float32)
    colors = np.zeros((2, 3), dtype=np.uint8)
    memory_world._cached_cloud = (
        {"n": 2},
        positions.tobytes() + colors.tobytes(),
    )
    unrelated_map = tmp_path / "unrelated.pickle"
    unrelated_map.write_bytes(b"not a point cloud")
    memory_world.config.global_map_path = str(unrelated_map)

    built = memory_world._build_top_down_map()

    assert built is not None
    header, _payload = built
    assert (header["x_min"] + header["x_max"]) / 2 == pytest.approx(12.0)
    assert (header["y_min"] + header["y_max"]) / 2 == pytest.approx(21.0)


def test_analyze_memory_publishes_and_replaces_result(memory_world: MemoryWorldModule) -> None:
    class Client:
        def __init__(self) -> None:
            self.messages: list[str] = []

        def send_threadsafe(self, message: str) -> None:
            self.messages.append(message)

    client = Client()
    memory_world._world_clients.add(client)  # type: ignore[arg-type]
    code = (
        "result = {"
        "'answer': 'Start highlighted', "
        "'focus_point': np.array([1.0, 2.0, 0.0]), "
        "'points': [{'position': [1.0, 2.0, 0.0]}]"
        "}"
    )

    first = memory_world.analyze_memory(code, timeout=10)
    second = memory_world.analyze_memory("result = {'answer': 'Replacement'}", timeout=10)

    assert first.success and second.success
    assert memory_world._active_query_result is not None
    assert memory_world._active_query_result["answer"] == "Replacement"
    assert memory_world._active_query_result["revision"] == 2
    messages = [json.loads(message) for message in client.messages]
    assert [message["revision"] for message in messages] == [1, 2]
    assert messages[0]["query_id"] != messages[1]["query_id"]


def test_analyze_memory_rejects_missing_result(memory_world: MemoryWorldModule) -> None:
    outcome = memory_world.analyze_memory("print('no structured result')", timeout=10)

    assert not outcome.success
    assert outcome.error_code == "EXECUTION_FAILED"
    assert "must assign a dictionary" in outcome.message


def test_analyze_memory_times_out(memory_world: MemoryWorldModule) -> None:
    outcome = memory_world.analyze_memory("import time; time.sleep(1)", timeout=0.01)

    assert not outcome.success
    assert outcome.error_code == "EXECUTION_TIMEOUT"


def test_analyze_memory_rejects_oversized_result(memory_world: MemoryWorldModule) -> None:
    memory_world.config.memory_analysis_max_output_chars = 100

    outcome = memory_world.analyze_memory("result = {'answer': 'x' * 200}", timeout=10)

    assert not outcome.success
    assert outcome.error_code == "RESULT_TOO_LARGE"


def test_viewer_pose_accepts_only_finite_xyz(memory_world: MemoryWorldModule) -> None:
    memory_world._on_client_message(None, {"type": "viewer_pose", "position": [1, 2.5, 3]})  # type: ignore[arg-type]
    assert memory_world._viewer_position == (1.0, 2.5, 3.0)

    memory_world._on_client_message(
        None,  # type: ignore[arg-type]
        {"type": "viewer_pose", "position": [float("nan"), 5, 6]},
    )
    assert memory_world._viewer_position == (1.0, 2.5, 3.0)


def test_route_is_generated_by_server(
    memory_world: MemoryWorldModule, monkeypatch: pytest.MonkeyPatch
) -> None:
    planned_path = SimpleNamespace(
        poses=[SimpleNamespace(x=1.0, y=2.0, z=0.0), SimpleNamespace(x=3.0, y=4.0, z=0.0)]
    )
    costmap_stream = SimpleNamespace(last=lambda: SimpleNamespace(data="costmap"))
    store = SimpleNamespace(
        list_streams=lambda: ["global_costmap"],
        streams=SimpleNamespace(global_costmap=costmap_stream),
    )
    monkeypatch.setattr(memory_world, "_ensure_store", lambda: store)
    monkeypatch.setattr(
        "dimos.teleop.memory_world.module.min_cost_astar",
        lambda costmap, *, goal, start: planned_path,
    )
    memory_world._viewer_position = (0.0, 0.0, 0.0)
    result = MemoryQueryResult.model_validate(
        {
            "answer": "Route ready",
            "focus_point": [3, 4, 0],
            "route": {"points": [[90, 90, 0], [91, 91, 0]]},
        }
    )

    memory_world._add_route_to_result(result)

    assert result.route is not None
    assert result.route.points == [(1.0, 2.0, 0.08), (3.0, 4.0, 0.08)]
