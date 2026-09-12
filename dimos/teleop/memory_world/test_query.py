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
import struct
from types import SimpleNamespace
from unittest import mock

import cv2
import numpy as np
from pydantic import ValidationError
import pytest
import pytest_mock

from dimos.memory.store.sqlite import SqliteStore
from dimos.teleop.memory_world.messages import MSG_HEATMAP
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
            self.messages: list[str | bytes] = []

        def send_threadsafe(self, message: str | bytes) -> None:
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
    texts = [json.loads(m) for m in client.messages if isinstance(m, str)]
    results = [m for m in texts if m.get("type") == "query_result"]
    assert [message["revision"] for message in results] == [1, 2]
    assert results[0]["query_id"] != results[1]["query_id"]

    # Neither answer is a Hyperspace one, so each also takes the previous answer's
    # overlays off the screen -- an empty heat map and an empty frustum list, per answer.
    # Clearing only the server's copy left them drawn in every connected viewer.
    heatmaps = [m for m in client.messages if isinstance(m, bytes) and m[0] == MSG_HEATMAP]
    assert len(heatmaps) == 2
    for frame, result in zip(heatmaps, results, strict=True):
        size = struct.unpack("<I", frame[1:5])[0]
        header = json.loads(frame[5 : 5 + size])
        assert header["n"] == 0 and header["query_id"] == result["query_id"]
        assert header["seconds"] == 0.0  # the tour card reads this without guarding it
    pyramids = [m for m in texts if m.get("type") == "query_pyramids"]
    assert [m["pyramids"] for m in pyramids] == [[], []]


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


def test_two_ids_naming_one_marker_light_and_count_as_one() -> None:
    """The viewer selects through a Set; the direct translation did not.

    `_selectedImageIds = new Set(result.observation_ids)` (scene.js), so an answer naming
    one marker twice lights ONE photograph. The direct branch kept both, so `analyze_memory`
    told the agent two frames were lit when one was -- the same wrong number the previous
    commit fixed, one layer further in. The nearest-marker branch has always deduplicated;
    this one now matches it.
    """
    module = SimpleNamespace(
        _cached_image_poses=({"ids": [0, 1, 2], "source_ids": [0, 100, 200]}, b""),
        _markers_near=lambda positions: [7] * len(positions),
    )
    snap = MemoryWorldModule._marker_ids_for

    # The same observation twice, and two observations that are genuinely different.
    twice = MemoryQueryResult(answer="a", engine="agent", observation_ids=[100, 100])
    assert snap(module, twice) == [1], "one marker, named twice, is one lit photograph"

    both = MemoryQueryResult(answer="a", engine="agent", observation_ids=[100, 200])
    assert snap(module, both) == [1, 2], "two markers stay two"

    # Order is the agent's own, and survives the deduplication.
    reversed_order = MemoryQueryResult(answer="a", engine="agent", observation_ids=[200, 100, 200])
    assert snap(module, reversed_order) == [2, 1]


def test_every_place_the_answer_names_can_still_be_given_a_picture() -> None:
    """The image budget must be at least what the answer can name.

    At 64, with MAX_CLUSTERS=12 places of EVIDENCE_PER_CLUSTER=8 evidence each, the budget
    was spent FIRST-COME and the last places got no photograph at all -- while `n_evidence`
    went on reporting how many they had. Measured on sf_office1_2/main.db: "a monitor"
    reported [8,8,8,6,6,6,6,5,4,4,4,4] and published [8,8,8,6,6,6,6,5,4,4,3,0]. This pins
    the relationship rather than the number, so raising either constant cannot re-open it.
    """
    from dimos.teleop.memory_world.hyperspace_answers import (
        EVIDENCE_CLUSTERS,
        EVIDENCE_IMAGES_MAX,
    )
    from dimos.teleop.memory_world.hyperspace_search import (
        EVIDENCE_PER_CLUSTER,
        MAX_CLUSTERS,
    )

    # Narrow on purpose, and worth saying so: EVIDENCE_IMAGES_MAX is DEFINED as that
    # product, so this is a constant against its own definition and can only fail if
    # someone replaces the expression with a literal -- which is exactly how the 64 got
    # there. The behaviour is guarded by
    # test_hyperspace_search.py::test_the_last_place_an_answer_names_is_still_sent_its_pictures,
    # which runs the publish loop; this one only pins the intent of the constant.
    assert EVIDENCE_IMAGES_MAX >= MAX_CLUSTERS * EVIDENCE_PER_CLUSTER, (
        "the last places an answer names would get no picture"
    )
    # And the outer cap must not bite before the ranking has had its say.
    assert EVIDENCE_CLUSTERS >= MAX_CLUSTERS


def test_a_configured_stream_name_that_is_empty_is_refused_at_startup(tmp_path: Path) -> None:
    """A name the operator gave is still only a name.

    `--...-tf-stream-name tf` against a recording holding the empty `tf` a killed ingest
    leaves behind used to be kept, and `_load_tf_tree` then cached a NON-None, zero-frame
    tree -- which defeats every `tree is None` fallback, so map, trail and index all came
    out empty with nothing reported. Cleared to "" now, which is what those fallbacks
    look for.
    """
    from dimos.msgs.geometry_msgs.Quaternion import Quaternion
    from dimos.msgs.geometry_msgs.Transform import Transform
    from dimos.msgs.geometry_msgs.Vector3 import Vector3
    from dimos.msgs.tf2_msgs.TFMessage import TFMessage

    def moving(ts: float) -> TFMessage:
        return TFMessage(
            Transform(
                translation=Vector3(ts, 0.0, 0.0),
                rotation=Quaternion(0.0, 0.0, 0.0, 1.0),
                frame_id="odom",
                child_frame_id="base_link",
                ts=ts,
            )
        )

    db_path = tmp_path / "recording.db"
    store = SqliteStore(path=str(db_path))
    store.start()
    try:
        # The empty `tf` a killed ingest leaves, and NOTHING for detection to fall back
        # to. With a replacement present the old code already did the right thing -- it is
        # this case, where `detected["tf"] is None` takes the other half of the condition,
        # that kept an unusable name. A first version of this test seeded a populated
        # `robot_tf` and passed against the unfixed code for that reason.
        store.stream("tf", TFMessage)
        assert moving(0.0) is not None  # the helper is real; the recording just has no tf
    finally:
        store.stop()

    module = MemoryWorldModule(store_path=str(db_path), tf_stream_name="tf")
    try:
        opened = module._ensure_store()
        module._name_streams(opened)
        assert module.config.tf_stream_name != "tf", (
            "an empty stream was kept because it was named on the command line"
        )
    finally:
        module.stop()


@pytest.mark.parametrize(
    ("seed_it", "expected"),
    [(True, "empty"), (False, "not in the recording")],
)
def test_the_operator_is_told_which_way_their_named_stream_was_unusable(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
    seed_it: bool,
    expected: str,
) -> None:
    """Absent and empty are different things to be told, and both were told as one.

    Clearing an unusable configured name also clears the local the "using %r (%s)" line
    reads, so its `no %r in the recording` arm became unreachable, and the replacement
    warning said "is empty" for both cases. An operator who mistyped a stream name and one
    whose recording was truncated by a killed ingest need different next steps.

    The warnings are captured by replacing the module's logger, not with `caplog`: this
    package logs through structlog, which does not go through the stdlib handlers pytest
    installs. caplog sees nothing here and the test would pass for the wrong reason.
    """
    from dimos.msgs.tf2_msgs.TFMessage import TFMessage

    db_path = tmp_path / "recording.db"
    store = SqliteStore(path=str(db_path))
    store.start()
    try:
        if seed_it:
            store.stream("front_camera", TFMessage)  # present, and holding nothing
    finally:
        store.stop()

    said: list[str] = []

    class Recorder:
        def warning(self, message: str, *args: object) -> None:
            said.append(message % args if args else message)

        def __getattr__(self, _name: str):  # info, exception, ... are not under test
            return lambda *a, **k: None

    # recording.logger, not module.logger: name_streams lives beside the helpers it uses.
    monkeypatch.setattr("dimos.teleop.memory_world.recording.logger", Recorder())

    module = MemoryWorldModule(store_path=str(db_path), image_stream_name="front_camera")
    try:
        module._name_streams(module._ensure_store())
        about = [line for line in said if "front_camera" in line]
        assert about, f"the operator was not told their flag was ignored; saw {said}"
        assert expected in about[0], about[0]
        # The other wording must not appear: telling them apart is the whole point.
        other = "not in the recording" if expected == "empty" else "empty"
        assert other not in about[0], about[0]
    finally:
        module.stop()


def test_a_second_start_does_not_orphan_the_server_that_is_serving(tmp_path: Path) -> None:
    """`stop()` shut the handle that never bound, and the real listener survived it.

    `start()` had no guard. A second call built a new RobotWebInterface over the first
    handle, logged "memory-world server started", and only then failed to bind inside the
    thread, where the Errno 48 reaches nobody. After that `self._web_server` pointed at a
    server that never bound while the FIRST was still listening -- so `stop()` shut the
    dead one, the port kept serving a module whose store had just been closed, and
    `memworld`'s probe (which takes any answer on the port as success) would print its
    URLs over the PREVIOUS recording's world.

    Seen in the live log: two "server started" lines from one pid, two hours apart, with
    `[Errno 48] address already in use` between them.
    """
    import socket
    import threading

    db_path = tmp_path / "recording.db"
    _empty_store(db_path)

    with socket.socket() as probe:  # a port nothing else is on
        probe.bind(("127.0.0.1", 0))
        port = probe.getsockname()[1]

    started: list[object] = []

    class FakeWeb:
        def __init__(self, **kwargs: object) -> None:
            from fastapi import FastAPI

            self.app = FastAPI()  # _setup_routes registers on it
            self.done = threading.Event()
            started.append(self)

        def run(self, **kwargs: object) -> None:
            self.done.wait(30)  # stays up like a bound server, and stops when asked

        def shutdown(self) -> None:
            self.done.set()

    module = MemoryWorldModule(store_path=str(db_path), server_port=port)
    monkey = mock.patch("dimos.teleop.memory_world.module.RobotWebInterface", FakeWeb)
    monkey.start()
    try:
        module.start()
        first = module._web_server
        module.start()  # the second one, which used to take over
        assert module._web_server is first, "a second start replaced the live server"
        assert len(started) == 1, "a second server was built over the first"
    finally:
        monkey.stop()
        module.stop()


def test_a_module_that_was_stopped_refuses_to_start_again(tmp_path: Path) -> None:
    """start() after stop() must refuse, and must NOT bind a port.

    A stopped module is finished, not idle: our stop() calls super().stop(), which latches
    core's `_module_closed`. Clearing `_web_server` in stop() was right -- "already
    serving" is a lie once the server is down -- but it made a restart reachable for the
    first time, and what came up was a module that bound the port and could do nothing.
    `_stopping` stays set, so `_prepare` returns before loading search, `_build_replay`
    refuses, and every replay and orbit request answers 503 "stopping".

    That is the orphaned-listener hazard start()'s own guard was written against, because
    `memworld`'s probe takes any answer on the port as success and would print its URLs
    over a world that cannot answer. So the second start() builds NO server.
    """
    import socket
    import threading

    db_path = tmp_path / "recording.db"
    _empty_store(db_path)

    with socket.socket() as probe:
        probe.bind(("127.0.0.1", 0))
        port = probe.getsockname()[1]

    built: list[object] = []

    class FakeWeb:
        def __init__(self, **kwargs: object) -> None:
            from fastapi import FastAPI

            self.app = FastAPI()
            self.done = threading.Event()
            built.append(self)

        def run(self, **kwargs: object) -> None:
            self.done.wait(30)

        def shutdown(self) -> None:
            self.done.set()

    module = MemoryWorldModule(store_path=str(db_path), server_port=port)
    monkey = mock.patch("dimos.teleop.memory_world.module.RobotWebInterface", FakeWeb)
    monkey.start()
    try:
        module.start()
        module.stop()
        assert module._stopping.is_set(), "stop() is expected to latch it"
        assert module._module_closed, "our stop() calls super().stop(), which closes it"

        module.start()
        assert len(built) == 1, (
            "a stopped module built a second web server: it binds the port and then"
            " refuses every request, which memworld's probe reads as a healthy launch"
        )
        assert module._web_server is None, "a refused start must leave no server handle"
    finally:
        monkey.stop()
        module.stop()


def test_the_capture_markers_are_built_from_the_poses_on_the_images(tmp_path: Path) -> None:
    """`_build_image_poses` end to end, because nothing else in this suite runs it.

    It was moved to world_cache.py in the extraction that shrank module.py, and the move
    dropped its `body_style_quaternion` import. Every test still passed, ruff still
    passed -- this repo's config lists F821 in its ignore list, so an undefined name is
    never reported -- and the live server logged "failed to build image poses" into a
    swallowed exception. The suite could not see it because it never called this.
    """
    from dimos.msgs.sensor_msgs.CompressedImage import CompressedImage

    pixels = np.zeros((8, 12, 3), dtype=np.uint8)
    pixels[:, :, 1] = 200
    ok, encoded = cv2.imencode(".jpg", pixels)
    assert ok

    db_path = tmp_path / "poses.db"
    store = SqliteStore(path=str(db_path))
    store.start()
    images = store.stream("color_image", CompressedImage)
    for k in range(3):  # no tf stream: the pose on the observation is the one used
        images.append(
            CompressedImage(data=encoded.tobytes(), format="jpeg", frame_id="cam", ts=float(k)),
            ts=float(k),
            pose=(float(k), 0.0, 0.0, 0.0, 0.0, 0.0, 1.0),
        )
    store.stop()

    module = MemoryWorldModule(store_path=str(db_path))
    try:
        (header, payload), thumbnails = module._build_image_poses()
        n = header["n"]
        assert n >= 2, header
        assert len(payload) == n * 12 + n * 16  # xyz float32 then xyzw float32
        assert len(thumbnails) == n
        assert any(thumbnails), "every thumbnail failed to encode"
        xs = np.frombuffer(payload[: n * 12], dtype="<f4").reshape(n, 3)[:, 0]
        assert xs[0] == pytest.approx(0.0) and xs[-1] > xs[0]  # walked along +x
        quats = np.frombuffer(payload[n * 12 :], dtype="<f4").reshape(n, 4)
        assert np.allclose(np.linalg.norm(quats, axis=1), 1.0, atol=1e-5)
    finally:
        module.stop()


def test_the_voxel_cloud_packs_what_survives_the_height_filter(
    memory_world: MemoryWorldModule, monkeypatch: pytest.MonkeyPatch
) -> None:
    """`_build_voxel_cloud_from_lidar` end to end, because nothing else in this suite runs it.

    It is the third of the builders moved into world_cache.py, and the only one still
    uncovered after a dropped import in that same move broke a sibling on the live
    server while every test passed. Stubbing the SOURCE of the scans is the point --
    the function under test is the filtering, striding, colouring and packing below it.
    """
    memory_world.config.build_replay_on_start = False
    memory_world.config.map_z_min = 0.0
    memory_world.config.map_z_max = 2.0
    # Two inside the height band, one under the floor and one through the ceiling.
    cloud = np.array(
        [[0.0, 0.0, 0.5], [1.0, 2.0, 1.5], [3.0, 3.0, -9.0], [4.0, 4.0, 9.0]],
        dtype=np.float64,
    )
    monkeypatch.setattr(memory_world, "_accumulated_cloud", lambda: cloud)

    built = memory_world._build_voxel_cloud_from_lidar()
    assert built is not None, "a cloud with points inside the band must produce one"
    header, payload = built

    assert header["n"] == 2, header  # the two outside the band are gone
    assert header["has_colors"] is True
    assert len(payload) == 2 * 12 + 2 * 3  # xyz float32, then rgb uint8
    xyz = np.frombuffer(payload[: 2 * 12], dtype="<f4").reshape(2, 3)
    assert sorted(float(v) for v in xyz[:, 2]) == [0.5, 1.5]
    assert header["bounds"]["x_min"] == 0.0 and header["bounds"]["x_max"] == 1.0
    # The planner gets the whole filtered map, not the strided copy the viewer gets.
    assert memory_world._map_xyz is not None and memory_world._map_xyz.shape == (2, 3)

    # Nothing inside the band is not a cloud at all, rather than an empty one.
    monkeypatch.setattr(memory_world, "_accumulated_cloud", lambda: cloud[2:])
    assert memory_world._build_voxel_cloud_from_lidar() is None


def test_a_global_map_stream_is_preferred_over_accumulating_the_scans(
    memory_world: MemoryWorldModule, monkeypatch: pytest.MonkeyPatch
) -> None:
    """A recording that carries a ray-traced `global_map` must be shown THAT.

    The walkable world used to come from the replay's final keyframe, falling back to a
    plain accumulation of the raw scans. When the voxel replay streams were deleted from
    grocery.db the map fell to the accumulation and got about three times thinner --
    782,688 voxels to 253,496 -- and started keeping returns 26 m up that the ray tracing
    had cleared. A global map built once, ahead of time, from a deskewed registered cloud
    is better than either, so it wins when it is there.

    The fallbacks are asserted too: an absent stream, and an EMPTY one, must both fall
    through rather than showing the user a world with nothing in it.
    """
    from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2

    memory_world.config.build_replay_on_start = False
    memory_world.config.map_z_min = None
    memory_world.config.map_z_max = None
    accumulated = np.array([[9.0, 9.0, 9.0]], dtype=np.float64)
    monkeypatch.setattr(memory_world, "_accumulated_cloud", lambda: accumulated)

    # No global_map stream: the accumulation is what shows.
    built = memory_world._build_voxel_cloud_from_lidar()
    assert built is not None and built[0]["n"] == 1
    assert memory_world._map_xyz is not None
    assert memory_world._map_xyz.tolist() == [[9.0, 9.0, 9.0]], "fell through wrongly"

    # Declared but empty: still the accumulation, not an empty world.
    store = memory_world._ensure_store()
    stream = store.stream(memory_world.config.global_map_stream_name, PointCloud2)
    built = memory_world._build_voxel_cloud_from_lidar()
    assert built is not None and built[0]["n"] == 1, "an empty global_map emptied the world"

    # Written: the global map wins.
    world = np.array([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [2.0, 0.0, 0.0]], dtype=np.float32)
    stream.append(PointCloud2.from_numpy(world, frame_id="world", timestamp=1.0), ts=1.0)
    built = memory_world._build_voxel_cloud_from_lidar()
    assert built is not None, "a global map with points must produce a cloud"
    assert built[0]["n"] == 3, "the accumulation was shown over the global map"
    assert memory_world._map_xyz is not None
    assert sorted(float(v) for v in memory_world._map_xyz[:, 0]) == [0.0, 1.0, 2.0]

    # Turned off by config: back to the accumulation even though the stream is there.
    memory_world.config.global_map_stream_name = ""
    built = memory_world._build_voxel_cloud_from_lidar()
    assert built is not None and built[0]["n"] == 1, "the off switch did not turn it off"


def test_a_global_map_that_cannot_be_read_falls_through_to_the_next_source(
    memory_world: MemoryWorldModule, monkeypatch: pytest.MonkeyPatch
) -> None:
    """The global map is the one source in the chain matched by NAME alone.

    It never goes through `detect_streams`, so nothing checks its message type. A
    recording whose `global_map` is a `nav_msgs/OccupancyGrid` -- an ordinary ROS name
    for a 2-D map -- reached `.points_f32()` and raised. `from_replay` contains its own
    failures and this source did not, so the error left the whole loop: the replay and
    the accumulated scans were never tried and the viewer got "world load failed" with a
    usable map sitting in the recording. Falling through is what the comment above the
    loop already promised.
    """
    memory_world.config.build_replay_on_start = False
    memory_world.config.map_z_min = None
    memory_world.config.map_z_max = None
    accumulated = np.array([[9.0, 9.0, 9.0]], dtype=np.float64)
    monkeypatch.setattr(memory_world, "_accumulated_cloud", lambda: accumulated)

    def not_a_cloud() -> np.ndarray:
        raise AttributeError("'OccupancyGrid' object has no attribute 'points_f32'")

    monkeypatch.setattr(memory_world, "_global_map_cloud", not_a_cloud)

    built = memory_world._build_voxel_cloud_from_lidar()
    assert built is not None, "an unreadable global_map took the whole world down"
    assert built[0]["n"] == 1
    assert memory_world._map_xyz is not None
    assert memory_world._map_xyz.tolist() == [[9.0, 9.0, 9.0]]


def test_a_global_map_in_another_frame_is_moved_into_the_world_frame(
    memory_world: MemoryWorldModule, monkeypatch: pytest.MonkeyPatch
) -> None:
    """A cloud says which frame it is in, and it is not safe to assume that is ours.

    Everything else in the world -- the capture poses, the trajectory, the planner's map
    -- is in `world_frame`. Reading the global map's coordinates raw put a map written in
    any other frame somewhere else entirely while looking perfectly reasonable: with a
    10 m offset between the frames the voxels landed 10 m from the photographs of the same
    place, and `_map_xyz` (which the planner routes on) went with them.

    When tf cannot place the cloud's frame at all, it is REFUSED rather than placed
    wrongly, and the next source is used.
    """
    from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2

    memory_world.config.build_replay_on_start = False
    memory_world.config.map_z_min = None
    memory_world.config.map_z_max = None
    memory_world.config.world_frame = "odom"
    monkeypatch.setattr(
        memory_world, "_accumulated_cloud", lambda: np.array([[9.0, 9.0, 9.0]], dtype=np.float64)
    )

    store = memory_world._ensure_store()
    stream = store.stream(memory_world.config.global_map_stream_name, PointCloud2)
    world = np.array([[0.0, 0.0, 1.0]], dtype=np.float32)
    stream.append(PointCloud2.from_numpy(world, frame_id="map", timestamp=1.0), ts=1.0)

    # tf knows where "map" is: the cloud is moved, not taken literally.
    shifted = np.eye(4)
    shifted[0, 3] = 10.0
    monkeypatch.setattr(memory_world, "_frame_pose_at", lambda frame, ts: shifted)
    built = memory_world._build_voxel_cloud_from_lidar()
    assert built is not None and memory_world._map_xyz is not None
    assert memory_world._map_xyz.tolist() == [[10.0, 0.0, 1.0]], (
        "the global map was read in its own frame and placed 10 m from everything else"
    )

    # tf cannot place it: refuse and fall through rather than misplace it.
    monkeypatch.setattr(memory_world, "_frame_pose_at", lambda frame, ts: None)
    built = memory_world._build_voxel_cloud_from_lidar()
    assert built is not None and memory_world._map_xyz is not None
    assert memory_world._map_xyz.tolist() == [[9.0, 9.0, 9.0]], (
        "an unplaceable global map was used anyway instead of falling back"
    )


def test_a_global_map_entirely_outside_the_height_band_falls_back(
    memory_world: MemoryWorldModule, monkeypatch: pytest.MonkeyPatch
) -> None:
    """A global map whose voxels are all filtered out is as useless as an absent one.

    The source was chosen BEFORE the height filter ran, so a global map sitting entirely
    outside map_z_min/max returned no cloud at all -- and `_build_cloud` turns that into
    "world load failed" for the viewer -- while the scans it could have accumulated were
    sitting in the same recording.
    """
    from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2

    memory_world.config.build_replay_on_start = False
    memory_world.config.map_z_min = 0.0
    memory_world.config.map_z_max = 2.0
    memory_world.config.world_frame = "odom"
    monkeypatch.setattr(
        memory_world, "_accumulated_cloud", lambda: np.array([[3.0, 3.0, 1.0]], dtype=np.float64)
    )

    store = memory_world._ensure_store()
    stream = store.stream(memory_world.config.global_map_stream_name, PointCloud2)
    # Every voxel through the ceiling.
    high = np.array([[0.0, 0.0, 10.0], [1.0, 0.0, 11.0]], dtype=np.float32)
    stream.append(PointCloud2.from_numpy(high, frame_id="odom", timestamp=1.0), ts=1.0)

    built = memory_world._build_voxel_cloud_from_lidar()
    assert built is not None, "a filtered-out global map cost the viewer its whole world"
    assert memory_world._map_xyz is not None
    assert memory_world._map_xyz.tolist() == [[3.0, 3.0, 1.0]], "the fallback was not used"


def test_the_camera_frame_comes_from_the_images_and_the_config_overrides_it(
    memory_world: MemoryWorldModule, tmp_path: Path
) -> None:
    """`_camera_frame` decides which tf frame every camera pose is looked up in.

    A reviewer asked which load-bearing code the suite would not notice the loss of, and
    this was on the list: returning a constant left all 213 tests green. It is the same
    shape of blind spot that let a dropped import break the live server.
    """
    from dimos.msgs.sensor_msgs.CompressedImage import CompressedImage

    pixels = np.zeros((4, 4, 3), dtype=np.uint8)
    ok, encoded = cv2.imencode(".jpg", pixels)
    assert ok
    db_path = tmp_path / "frames.db"
    store = SqliteStore(path=str(db_path))
    store.start()
    store.stream("color_image", CompressedImage).append(
        CompressedImage(data=encoded.tobytes(), format="jpeg", frame_id="d455_optical", ts=1.0),
        ts=1.0,
    )
    store.stop()

    module = MemoryWorldModule(store_path=str(db_path))
    try:
        assert module._camera_frame() == "d455_optical"  # read off the images themselves
        module._camera_frame_cache = None
        module.config.camera_optical_frame = "told_so"
        assert module._camera_frame() == "told_so"  # the operator outranks the recording
    finally:
        module.stop()


def test_the_field_of_view_falls_back_to_seventy_degrees_without_intrinsics(
    memory_world: MemoryWorldModule,
) -> None:
    """`_camera_hfov` sets how wide every camera frustum is drawn.

    Also on the reviewer's list of code no test exercised: returning 0.0 left the suite
    green, and a zero field of view draws every frustum as a line.
    """
    memory_world.config.camera_info_stream_name = None
    memory_world._camera_hfov_deg = None
    assert memory_world._camera_hfov() == pytest.approx(70.0)


def test_the_operator_can_say_whether_the_lidar_is_already_in_the_world_frame(
    memory_world: MemoryWorldModule,
) -> None:
    """`_lidar_world_aligned` decides whether scans get transformed by tf at all.

    Third of the four functions a reviewer found the suite would not miss: returning a
    constant left every test green, and getting this wrong puts the whole map in the
    wrong frame. The configured answer is the one an operator reaches for when the
    detection is wrong, so it is the one that must not silently stop working.
    """
    memory_world.config.lidar_world_frame = True
    memory_world._lidar_world_aligned_cache = None
    assert memory_world._lidar_world_aligned() is True

    memory_world.config.lidar_world_frame = False
    memory_world._lidar_world_aligned_cache = None
    assert memory_world._lidar_world_aligned() is False


@pytest.mark.parametrize(
    ("value", "acceptable"),
    [
        (0, True),
        (-3, True),
        (1.5, True),
        (10**30, True),  # wide, but a real place
        (10**400, False),  # np.isfinite raised TypeError here; math.isfinite OverflowError
        (float("inf"), False),
        (float("nan"), False),
        (True, False),  # a bool is not a coordinate
        ("2", False),
        (None, False),
    ],
)
def test_the_viewer_pose_guard_never_raises_on_what_it_rejects(
    value: object, acceptable: bool
) -> None:
    """This guard has now been broken twice, the same way both times.

    It exists to reject a bad `viewer_pose`, and twice it has died on one instead and taken
    the websocket loop with it: `np.isfinite` raises TypeError on a python int wider than
    int64, and `math.isfinite` raises OverflowError converting one to a float. The second
    was introduced fixing the first, which is what a test here would have caught.
    """
    from dimos.teleop.memory_world.module import _is_finite_number

    assert _is_finite_number(value) is acceptable


@pytest.mark.parametrize(
    ("configured", "expected"),
    [
        ("/memory_world", "/memory_world"),
        (
            "/custom/",
            "/custom",
        ),  # registered "/custom//replay/index"; the viewer asks "/custom/..."
        ("/", ""),  # registered "//ws"; the viewer asks "/ws"
        ("walk", "/walk"),
    ],
)
def test_the_client_route_is_normalised_the_way_the_viewer_normalises_it(
    tmp_path: Path, configured: str, expected: str
) -> None:
    """The viewer computes its base as `pathname.replace(/\\/$/, "")`, and every API path
    and the websocket hang off it on both sides. When the two rules disagree the page
    loads and everything under it 404s, which looks like a working server."""
    from fastapi.testclient import TestClient

    from dimos.web.robot_web_interface import RobotWebInterface

    db_path = tmp_path / "route.db"
    _empty_store(db_path)
    module = MemoryWorldModule(store_path=str(db_path), client_route=configured)
    try:
        assert module.config.client_route == expected

        # ...and the page is actually SERVED there. Asserting the normalised string alone
        # passed while the root case did not work at all: RobotWebInterface registers
        # dimos's generic index at "/" first, Starlette answers with the first full match,
        # and the viewer's own page was registered and reachable by nothing.
        module._web_server = RobotWebInterface(host="127.0.0.1", port=0)
        module._setup_routes()
        response = TestClient(module._web_server.app).get(expected or "/")
        assert response.status_code == 200
        assert "DimOS Memory World" in response.text, (
            f"{expected or '/'} served something else: {response.text[:120]}"
        )
    finally:
        module.stop()
