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
import pytest_mock

from dimos.memory.store.sqlite import SqliteStore
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
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


def test_pack_cloud_clips_to_the_height_slab_and_caps_points(
    memory_world: MemoryWorldModule,
) -> None:
    memory_world.config.map_z_min = 0.0
    memory_world.config.map_z_max = 1.0
    memory_world.config.max_points = 2
    xyz = np.asarray(
        [[0, 0, -1.0], [1, 0, 0.5], [2, 0, 0.6], [3, 0, 0.7], [4, 0, 5.0]], dtype=np.float32
    )

    packed = memory_world._pack_cloud(xyz)

    assert packed is not None
    header, payload = packed
    assert header["n"] == 2
    kept = np.frombuffer(payload, dtype=np.float32, count=6).reshape(2, 3)
    assert kept[:, 0].tolist() == [1.0, 3.0]
    assert memory_world._pack_cloud(np.asarray([[0, 0, 9.0]], dtype=np.float32)) is None


def test_top_down_map_uses_the_rendered_cloud(memory_world: MemoryWorldModule) -> None:
    positions = np.asarray([[10.0, 20.0, 0.5], [14.0, 22.0, 0.5]], dtype=np.float32)
    colors = np.zeros((2, 3), dtype=np.uint8)
    built = memory_world._build_top_down_map(({"n": 2}, positions.tobytes() + colors.tobytes()))

    assert built is not None
    header, _payload = built
    assert (header["x_min"] + header["x_max"]) / 2 == pytest.approx(12.0)
    assert (header["y_min"] + header["y_max"]) / 2 == pytest.approx(21.0)


def test_start_initializes_only_memory_world_server(
    memory_world: MemoryWorldModule, mocker: pytest_mock.MockerFixture
) -> None:
    web_interface = mocker.patch("dimos.teleop.memory_world.module.RobotWebInterface")
    setup_routes = mocker.patch.object(memory_world, "_setup_routes")

    memory_world.start()

    web_interface.assert_called_once_with(host="0.0.0.0", port=8443)
    setup_routes.assert_called_once_with()
    assert not hasattr(memory_world, "_control_loop_thread")


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


def test_sample_pose_path_uses_documented_stream_api(tmp_path: Path) -> None:
    db_path = tmp_path / "recording.db"
    store = SqliteStore(path=str(db_path))
    store.start()
    odom = store.stream("odom", float)
    for index in range(10):
        odom.append(
            float(index),
            ts=float(index),
            pose=(float(index), float(index * 2), 0.0, 0.0, 0.0, 0.0, 1.0),
        )
    store.stop()
    module = MemoryWorldModule(store_path=str(db_path))
    try:
        outcome = module.analyze_memory(
            "path = sample_pose_path('odom', max_points=3)\n"
            "result = {'answer': 'Path sampled', "
            "'evidence_paths': [{'points': path}]}\n",
            timeout=10,
        )

        assert outcome.success
        assert module._active_query_result is not None
        assert module._active_query_result["evidence_paths"][0]["points"] == [
            [0.0, 0.0, 0.0],
            [4.0, 8.0, 0.0],
            [8.0, 16.0, 0.0],
        ]
    finally:
        module.stop()


def test_viewer_pose_accepts_only_finite_xyz(memory_world: MemoryWorldModule) -> None:
    memory_world._on_client_message(None, {"type": "viewer_pose", "position": [1, 2.5, 3]})  # type: ignore[arg-type]
    assert memory_world._viewer_position == (1.0, 2.5, 3.0)

    memory_world._on_client_message(
        None,  # type: ignore[arg-type]
        {"type": "viewer_pose", "position": [float("nan"), 5, 6]},
    )
    assert memory_world._viewer_position == (1.0, 2.5, 3.0)


def test_planner_path_becomes_the_route_of_the_active_answer(
    memory_world: MemoryWorldModule,
) -> None:
    sent: list[str] = []
    memory_world._broadcast = sent.append  # type: ignore[method-assign]
    memory_world._publish_query_result(MemoryQueryResult(answer="Fountain found"))
    path = SimpleNamespace(
        poses=[
            SimpleNamespace(position=SimpleNamespace(x=1.0, y=2.0, z=0.0)),
            SimpleNamespace(position=SimpleNamespace(x=3.0, y=4.0, z=0.0)),
        ]
    )

    memory_world._on_path(path)  # type: ignore[arg-type]

    assert memory_world._active_query_result is not None
    route = memory_world._active_query_result["route"]
    assert route["points"] == [[1.0, 2.0, 0.08], [3.0, 4.0, 0.08]]
    assert memory_world._active_query_result["answer"] == "Fountain found"
    assert len(sent) == 2

    memory_world._on_path(SimpleNamespace(poses=[]))  # type: ignore[arg-type]

    assert memory_world._active_query_result["route"] is None
    assert len(sent) == 3


def test_later_answers_keep_the_planner_route(memory_world: MemoryWorldModule) -> None:
    memory_world._broadcast = lambda message: None  # type: ignore[method-assign]
    path = SimpleNamespace(
        poses=[
            SimpleNamespace(position=SimpleNamespace(x=0.0, y=0.0, z=0.0)),
            SimpleNamespace(position=SimpleNamespace(x=1.0, y=1.0, z=0.0)),
        ]
    )
    memory_world._on_path(path)  # type: ignore[arg-type]

    memory_world._publish_query_result(MemoryQueryResult(answer="Another answer"))

    assert memory_world._active_query_result is not None
    assert memory_world._active_query_result["route"]["points"] == [
        [0.0, 0.0, 0.08],
        [1.0, 1.0, 0.08],
    ]


def test_height_colours_stay_in_the_blue_band(memory_world: MemoryWorldModule) -> None:
    """Warm colours are reserved for highlights, so no height may turn red or yellow."""
    # A frame that is not floor-aligned (floor near -1) with a few stray
    # returns far above the ceiling.
    room = np.linspace(-1.0, 1.4, 24)
    positions = np.array([[0.0, 0.0, z] for z in [*room, 9.0]])
    colours = memory_world._height_colors(positions).astype(int)
    assert colours.shape == (25, 3)
    assert (colours[:, 2] >= colours[:, 0]).all()  # blue never below red: cool half only
    assert (colours[:, 2] >= colours[:, 1]).all()  # blue never below green
    assert (np.diff(colours[:, 1]) >= 0).all()  # cooler and brighter going up
    # The ramp is anchored to the floor, so the stray return does not stretch
    # it: the top of the room already uses the bright end.
    assert colours[0].sum() < 320
    assert colours[23].sum() > 0.8 * colours[24].sum()


def test_concurrent_clients_build_the_world_once(
    memory_world: MemoryWorldModule, monkeypatch: pytest.MonkeyPatch
) -> None:
    """Two headsets connecting together must not each voxelise the recording."""
    import threading
    import time

    builds = []

    def slow_poses() -> tuple[tuple[dict[str, object], bytes], list[bytes]]:
        builds.append(threading.get_ident())
        time.sleep(0.05)
        return ({"n": 0}, b""), []

    monkeypatch.setattr(memory_world, "_build_image_poses", slow_poses)
    monkeypatch.setattr(memory_world, "_build_trail", lambda: ({"n": 0}, b""))

    threads = [threading.Thread(target=memory_world._ensure_world_cache) for _ in range(4)]
    for thread in threads:
        thread.start()
    for thread in threads:
        thread.join()

    assert len(builds) == 1
    assert memory_world._cached_image_poses is not None


def _map(points: list[list[float]], ts: float) -> PointCloud2:
    return PointCloud2.from_numpy(np.asarray(points, np.float32), frame_id="odom", timestamp=ts)


def test_mapper_snapshots_become_the_timeline(memory_world: MemoryWorldModule) -> None:
    memory_world.config.replay_keyframe_interval_s = 1.0

    memory_world._on_global_map(_map([[0, 0, 0], [1, 0, 0]], 10.0))
    memory_world._on_global_map(_map([[0, 0, 0], [2, 0, 0]], 11.5))

    index = memory_world._replay_index_json()
    assert index["scans"] == [10.0, 11.5]
    assert [keyframe["scan"] for keyframe in index["keyframes"]] == [0, 1]
    assert index["complete"] is False
    assert memory_world._replay_progress == "recording"
    assert memory_world._ensure_replay().segment(1)[0]["scans"] == [
        {"index": 1, "ts": 11.5, "n": 2}
    ]


def test_a_timeline_covering_the_recording_is_kept_across_starts(tmp_path: Path) -> None:
    db_path = tmp_path / "recording.db"
    store = SqliteStore(path=str(db_path))
    store.start()
    store.stream("lidar", PointCloud2).append(_map([[0, 0, 0]], 11.0), ts=11.0)
    store.stop()

    first = MemoryWorldModule(store_path=str(db_path))
    try:
        first._on_global_map(_map([[0, 0, 0]], 10.0))
        first._on_global_map(_map([[0, 0, 0], [1, 0, 0]], 11.0))
        assert first._replay_index_json()["complete"] is True
    finally:
        first.stop()

    second = MemoryWorldModule(store_path=str(db_path))
    try:
        second._open_replay()
        assert second._replay_progress == "ready"
        second._on_global_map(_map([[5, 5, 5]], 10.0))
        assert second._recorder is None
        assert second._replay_index_json()["scans"] == [10.0, 11.0]
    finally:
        second.stop()
