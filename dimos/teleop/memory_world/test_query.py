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


def test_lidar_cloud_failure_is_reported(
    memory_world: MemoryWorldModule, monkeypatch: pytest.MonkeyPatch
) -> None:
    monkeypatch.setattr(memory_world, "_build_voxel_cloud_from_lidar", lambda: None)

    with pytest.raises(RuntimeError, match="produced no cloud"):
        memory_world._build_cloud()


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


def test_route_is_generated_by_server(
    memory_world: MemoryWorldModule, monkeypatch: pytest.MonkeyPatch
) -> None:
    planned_path = SimpleNamespace(
        poses=[SimpleNamespace(x=1.0, y=2.0, z=0.0), SimpleNamespace(x=3.0, y=4.0, z=0.0)]
    )
    # A costmap with a real resolution: the waypoints come back as grid_to_world
    # CORNERS and have to be moved to the centres of the cells A* planned through.
    costmap_stream = SimpleNamespace(
        last=lambda: SimpleNamespace(data=SimpleNamespace(resolution=0.5))
    )
    store = SimpleNamespace(
        list_streams=lambda: ["global_costmap"],
        streams=SimpleNamespace(global_costmap=costmap_stream),
    )
    monkeypatch.setattr(memory_world, "_ensure_store", lambda: store)
    monkeypatch.setattr(
        "dimos.teleop.memory_world.hyperspace_answers.min_cost_astar",
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
    # Half a cell on from the corners A* returned, not the corners themselves.
    assert result.route.points == [(1.25, 2.25, 0.08), (3.25, 4.25, 0.08)]


def test_height_colours_run_from_purple_to_light_green(memory_world: MemoryWorldModule) -> None:
    """Floor purple, then blue, cyan, light green at the top; never yellow, orange or red,
    which the heat map and the answer markers keep for themselves."""
    # Not floor-aligned: this recording's ground sits near -1.
    positions = np.array([[0.0, 0.0, z] for z in np.linspace(-1.0, 1.4, 25)])
    colours = memory_world._height_colors(positions).astype(int)
    assert colours.shape == (25, 3)
    r, g, b = colours[:, 0], colours[:, 1], colours[:, 2]
    assert b[0] > r[0] > g[0], "floor is purple"
    assert b[8] > 180 and g[8] < 160, "then blue"
    assert g[16] > 180 and b[16] > 180 and r[16] < 120, "cyan"
    assert g[-1] > 220 and r[-1] < 170 and b[-1] < 170 and g[-1] > r[-1], "light green at the top"
    assert (r[12:] <= g[12:]).all(), "the upper half never turns yellow or orange"


def test_height_colours_separate_the_storeys(memory_world: MemoryWorldModule) -> None:
    """A multi-storey recording must not paint every floor the same shade."""
    ground = np.linspace(0.0, 2.4, 50)
    upstairs = np.linspace(4.0, 6.4, 50)
    positions = np.array([[0.0, 0.0, z] for z in [*ground, *upstairs]])
    colours = memory_world._height_colors(positions).astype(int)
    # The two levels land in clearly different parts of the ramp.
    assert colours[:50].mean(axis=0).sum() + 60 < colours[50:].mean(axis=0).sum()


def test_a_sparse_outlier_does_not_flatten_the_ramp(memory_world: MemoryWorldModule) -> None:
    """One stray return far above the building must not squash everything else."""
    room = np.linspace(0.0, 2.4, 199)
    positions = np.array([[0.0, 0.0, z] for z in [*room, 90.0]])
    colours = memory_world._height_colors(positions).astype(int)
    # The percentile cut ignores the stray, so the room still spans the ramp:
    # purple at the floor, orange at the ceiling, far apart in colour.
    assert colours[0][2] > colours[0][0], "floor is purple"
    assert colours[198][1] > 220 and colours[198][1] > colours[198][0], "ceiling is light green"
    assert np.abs(colours[198] - colours[0]).sum() > 250


def test_concurrent_clients_build_the_world_once(
    memory_world: MemoryWorldModule, monkeypatch: pytest.MonkeyPatch
) -> None:
    """Two headsets connecting together must not each voxelise the recording."""
    import threading
    import time

    builds = []

    def slow_cloud() -> tuple[dict[str, object], bytes]:
        builds.append(threading.get_ident())
        time.sleep(0.05)
        positions = np.asarray([[0.0, 0.0, 0.0]], dtype=np.float32)
        return {"n": 1}, positions.tobytes() + np.zeros((1, 3), dtype=np.uint8).tobytes()

    monkeypatch.setattr(memory_world, "_build_cloud", slow_cloud)
    monkeypatch.setattr(memory_world, "_build_image_poses", lambda: (({"n": 0}, b""), []))
    monkeypatch.setattr(memory_world, "_build_trail", lambda: ({"n": 0}, b""))

    threads = [threading.Thread(target=memory_world._ensure_world_cache) for _ in range(4)]
    for thread in threads:
        thread.start()
    for thread in threads:
        thread.join()

    assert len(builds) == 1
    assert memory_world._cached_top_down is not None


def test_an_answers_ids_reach_the_viewer_as_marker_ids_whatever_the_engine_counted_in() -> None:
    """The viewer holds thumbnails for markers and nothing else, so that is what it is sent.

    Marker ids and the store's observation ids are both small integers and overlap, so
    the engine states which space it means: only the agent skill answers in store ids.
    Deciding by whether the numbers happen to match would highlight the wrong photo.
    """
    from dimos.teleop.memory_world.query import HighlightPoint

    # Marker k was built from observation 100*k, so the two spaces overlap at 0 and
    # nowhere else -- exactly the case a value test gets wrong.
    module = SimpleNamespace(
        _cached_image_poses=({"ids": [0, 1, 2], "source_ids": [0, 100, 200]}, b""),
        _markers_near=lambda positions: [7] * len(positions),
    )
    snap = MemoryWorldModule._marker_ids_for

    for engine in ("hyperspace", "siglip"):
        already = MemoryQueryResult(answer="a", engine=engine, observation_ids=[1, 2])
        assert snap(module, already) == [1, 2]  # already marker ids: untouched

    agent = MemoryQueryResult(answer="a", engine="agent", observation_ids=[100, 200])
    assert snap(module, agent) == [1, 2]  # store ids, translated through the markers

    # An agent answer naming frames no marker was built from falls back to where it points.
    elsewhere = MemoryQueryResult(
        answer="a",
        engine="agent",
        observation_ids=[55, 66],
        points=[HighlightPoint(position=(1.0, 2.0, 3.0), label="x")],
    )
    assert snap(module, elsewhere) == [7]

    # ... and with nowhere to point either, it says so rather than inventing ids.
    nothing = MemoryQueryResult(answer="a", engine="agent", observation_ids=[55])
    assert (
        snap(SimpleNamespace(**{**module.__dict__, "_markers_near": lambda p: []}), nothing) == []
    )

    # An mcap numbers each windowed read from zero, so the server publishes -1 for
    # "no real id"; -1 must never be matched as though it were one.
    mcap = SimpleNamespace(
        _cached_image_poses=({"ids": [0, 1], "source_ids": [-1, -1]}, b""),
        _markers_near=lambda positions: [3],
    )
    from_mcap = MemoryQueryResult(
        answer="a",
        engine="agent",
        observation_ids=[-1],
        points=[HighlightPoint(position=(0.0, 0.0, 0.0), label="x")],
    )
    assert snap(mcap, from_mcap) == [3]

    # An answer can point somewhere and name no frame at all. The markers nearest where
    # it points are still its evidence; returning nothing left it with no photograph.
    by_focus = MemoryQueryResult(
        answer="a", engine="agent", focus_point=(9.0, 9.0, 9.0), observation_ids=[]
    )
    assert snap(module, by_focus) == [7]
    by_points = MemoryQueryResult(
        answer="a",
        engine="agent",
        observation_ids=[],
        points=[
            HighlightPoint(position=(1.0, 0.0, 0.0), label="x"),
            HighlightPoint(position=(2.0, 0.0, 0.0), label="y"),
        ],
    )
    assert snap(module, by_points) == [7, 7]  # one marker per place it points at


def test_stop_does_not_close_the_store_under_a_read_in_flight(tmp_path: Path) -> None:
    """The evidence and adopt threads are not joined by stop(), so it must take the lock.

    Closing the store out from under a read in flight gives a page of sqlite
    ProgrammingError on every `memworld --stop` issued after a question. Read-only, so
    noise rather than corruption, but it is noise that hides a real failure.
    """
    import threading
    import time

    db_path = tmp_path / "recording.db"
    _empty_store(db_path)
    module = MemoryWorldModule(store_path=str(db_path))
    module._ensure_store()
    assert module._store is not None

    # A stand-in for the SigLIP index a query or an adopt build would be reading.
    torn_down = threading.Event()
    module._visual_index = SimpleNamespace(stop=torn_down.set)

    holding = threading.Event()
    may_finish = threading.Event()

    def a_read_in_flight() -> None:
        with module._store_lock:
            holding.set()
            may_finish.wait(10)

    reader = threading.Thread(target=a_read_in_flight, name="reader")
    reader.start()
    try:
        assert holding.wait(5), "the reader never took the lock"
        stopped = threading.Thread(target=module.stop, name="stop")
        stopped.start()
        # While the read holds the lock, NOTHING may be torn down -- not the store, and
        # not the index and model the same reader is using.
        time.sleep(1.0)
        assert module._store is not None, "stop() closed the store under a read in flight"
        assert not torn_down.is_set(), "stop() dismantled the index under a read in flight"
        may_finish.set()
        stopped.join(timeout=20)
        assert not stopped.is_alive()
        # And once the read is done, it does close it.
        assert module._store is None
        assert torn_down.is_set(), "the index was never torn down at all"
    finally:
        may_finish.set()
        reader.join(timeout=5)


def test_the_store_is_closed_while_the_lock_is_still_held(tmp_path: Path) -> None:
    """Not just swapped out under the lock and closed after it.

    Dropping `self._store` under the lock and then calling `store.stop()` outside it leaves
    a window where a thread waking in `_ensure_store()` sees None, reopens the recording,
    and closes the old handle beside it. Nothing observable came of it during shutdown, but
    the comment above the block claims the close is covered, and a comment that is not true
    is how the next reader gets it wrong. The test the round-41 fix came with cannot see
    this: it asserts WHAT was torn down, not WHERE.
    """
    import threading

    db_path = tmp_path / "recording.db"
    _empty_store(db_path)
    module = MemoryWorldModule(store_path=str(db_path))
    real_store = module._ensure_store()
    assert real_store is not None

    held_during_close: dict[str, bool] = {}

    def is_the_lock_free_right_now() -> None:
        # From ANOTHER thread, because an RLock is re-entrant for its owner and would say
        # yes to stop()'s own thread whether or not it holds it.
        if module._store_lock.acquire(blocking=False):
            held_during_close["held"] = False
            module._store_lock.release()  # same thread that took it, as an RLock requires
        else:
            held_during_close["held"] = True

    class StoreThatLooksAtTheLockAsItCloses:
        def __init__(self, inner: object) -> None:
            self._inner = inner

        def stop(self) -> None:
            probe = threading.Thread(target=is_the_lock_free_right_now, name="probe")
            probe.start()
            probe.join(5)
            self._inner.stop()  # type: ignore[attr-defined]

        def __getattr__(self, name: str) -> object:
            return getattr(self._inner, name)

    module._store = StoreThatLooksAtTheLockAsItCloses(real_store)  # type: ignore[assignment]
    module.stop()

    assert held_during_close.get("held") is True, (
        "the store was closed after the lock was released, not while it was held"
    )


def test_the_agent_is_told_how_many_frames_were_actually_lit(
    memory_world: MemoryWorldModule, monkeypatch: pytest.MonkeyPatch
) -> None:
    """`analyze_memory` reported the agent's own input back to it.

    `_publish_query_result` translates whatever the engine counted in into MARKER ids --
    the only frames the viewer holds thumbnails for -- and it did that with `model_copy`,
    a new object and a local rebind. The caller kept the original, so the number the agent
    was told was the number it had sent, not the number that lit. The translation itself
    was already tested; the report about it was not.
    """
    lit: list[int] = [7]
    monkeypatch.setattr(memory_world, "_marker_ids_for", lambda result: list(lit))

    result = MemoryQueryResult.model_validate(
        {"answer": "two frames, says the agent", "observation_ids": [101, 202]}
    )
    memory_world._publish_query_result(result)

    # The object the caller still holds is the one it reports on.
    assert result.observation_ids == [7], "the caller was left with the untranslated ids"

    # And the other direction: an answer naming no frame still lights the nearest marker.
    empty = MemoryQueryResult.model_validate({"answer": "no frames, says the agent"})
    memory_world._publish_query_result(empty)
    assert empty.observation_ids == [7]
