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

"""`analyze_memory` and the question that reaches it.

Split out of test_query.py, which had reached the repo's 75KB per-file limit: the hook
rejects the whole commit at that point and reports only a size, so the failure reads as
something unrelated to whatever you were editing. These tests are the most self-contained
group in that file -- they drive the analysis subprocess and the routing into it. The one
fixture they share, `memory_world`, moved to conftest.py rather than being imported across
test modules.
"""

from __future__ import annotations

import json
import struct
from types import SimpleNamespace

import pytest

from dimos.teleop.memory_world.messages import MSG_HEATMAP
from dimos.teleop.memory_world.module import MemoryWorldModule


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


def test_a_question_is_answered_by_clip_even_when_hyperspace_is_loaded(
    memory_world: MemoryWorldModule, monkeypatch: pytest.MonkeyPatch
) -> None:
    """One engine answers, always.

    `find_in_memory` used to prefer Hyperspace whenever its index happened to be ready and
    fall back to the embeddings otherwise, so the same recording could answer two different
    ways depending on whether a sidecar had finished loading. Nothing tested the choice --
    deleting the branch outright left all 267 tests green -- which is exactly why it gets a
    test now rather than a comment.
    """
    called: list[str] = []
    monkeypatch.setattr(
        memory_world, "_find_with_siglip", lambda phrase, started: called.append("clip") or "ok"
    )
    if hasattr(memory_world, "_find_with_hyperspace"):
        monkeypatch.setattr(
            memory_world,
            "_find_with_hyperspace",
            lambda phrase, started: called.append("hyperspace") or "ok",
        )
    # Hyperspace present and ready, which is what used to win the routing. It needs a
    # close(), because stop() closes whatever is there and a bare object() fails teardown.
    memory_world._hyperspace = SimpleNamespace(close=lambda: None)
    assert memory_world._hyperspace_ready() is True

    memory_world.find_in_memory("where did I see a whiteboard")

    assert called == ["clip"], f"answered with {called}, not the embedding index"


def test_an_empty_question_is_refused_before_any_engine_runs(
    memory_world: MemoryWorldModule, monkeypatch: pytest.MonkeyPatch
) -> None:
    """`search_phrase` can reduce a question to nothing; that is not a search."""
    monkeypatch.setattr(
        memory_world, "_find_with_siglip", lambda *a: pytest.fail("searched for nothing")
    )

    outcome = memory_world.find_in_memory("   ?  ")

    assert not outcome.success and outcome.error_code == "INVALID_QUERY"


def test_analyze_memory_rejects_missing_result(memory_world: MemoryWorldModule) -> None:
    outcome = memory_world.analyze_memory("print('no structured result')", timeout=10)

    assert not outcome.success
    assert outcome.error_code == "EXECUTION_FAILED"
    assert "must assign a dictionary" in outcome.message


def test_analyze_memory_reports_a_child_that_died_after_printing_its_answer(
    memory_world: MemoryWorldModule,
) -> None:
    """A result on stdout is not the same as a run that worked.

    The result is read off the child's stdout and its EXIT STATUS was never looked at, so
    a child that printed the sentinel and then died -- a teardown that raises, a native
    library faulting as it closes its handles -- came back from this method as
    `success=True` with the failure discarded. The `atexit` below is the cheap way to
    stage exactly that ordering: the answer is printed first, the process dies after.
    """
    code = (
        "import atexit, os, sys\n"
        # Flush FIRST: the answer has to reach the parent's stdout, otherwise this stages
        # the already-covered "no result at all" case instead of the one under test.
        "atexit.register(lambda: (sys.stdout.flush(), os._exit(3)))\n"
        "result = {'answer': 'ok'}"
    )

    outcome = memory_world.analyze_memory(code, timeout=10)

    assert not outcome.success, "a child that died after printing was reported as success"
    assert outcome.error_code == "EXECUTION_FAILED"
    assert "3" in outcome.message


def test_two_frames_a_fraction_of_a_millisecond_apart_are_not_one_cached_frame(
    memory_world: MemoryWorldModule, monkeypatch: pytest.MonkeyPatch
) -> None:
    """The replay frame cache is keyed by the stamp, so the stamp has to be the stamp.

    It was rounded to four decimals first. Two observations inside the same tenth of a
    millisecond then shared a bucket, and the second one was served the FIRST one's JPEG
    and camera pose -- a picture of somewhere else, with a pose to match.
    """
    first = SimpleNamespace(ts=1.00001, data="FIRST")
    second = SimpleNamespace(ts=1.000049, data="SECOND")
    stream = SimpleNamespace(at=lambda ts, tolerance: [first, second])
    monkeypatch.setattr(
        memory_world, "_ensure_store", lambda: SimpleNamespace(streams={"": stream})
    )
    monkeypatch.setattr(memory_world.config, "image_stream_name", "")
    encodes: list[str] = []

    def encode(data, *a):  # type: ignore[no-untyped-def]
        encodes.append(str(data))
        return str(data).encode()

    monkeypatch.setattr(memory_world, "_encode_jpeg", encode)
    monkeypatch.setattr(memory_world, "_camera_hfov", lambda: 60.0)
    monkeypatch.setattr(memory_world, "_camera_pose_of", lambda obs: None)

    got_first, _ = memory_world._replay_frame(1.00001)
    got_second, _ = memory_world._replay_frame(1.000049)

    assert got_first == b"FIRST"
    assert got_second == b"SECOND", "served the neighbouring frame's picture from the cache"
    # ...and the cache is still a cache. Counting ENCODES is the only thing that shows
    # that: comparing the returned bytes and the dict length passes just as well with the
    # cache lookup deleted outright, because a re-encode returns equal bytes.
    assert encodes == ["FIRST", "SECOND"]
    assert memory_world._replay_frame(1.00001)[0] == b"FIRST"
    assert encodes == ["FIRST", "SECOND"], "re-encoded a frame it had already cached"
    assert len(memory_world._replay_frames) == 2


def test_analyze_memory_survives_analysis_that_printed_without_a_newline(
    memory_world: MemoryWorldModule,
) -> None:
    """Analysis code may leave stdout mid-line, and often does.

    The marker is found by looking for a line that STARTS with it. The bootstrap printed
    it with no leading newline, so a `print(..., end="")` anywhere in the analysis put
    that output and the marker on one line and the answer vanished -- EXECUTION_FAILED on
    a run that worked. Found by a reviewer immediately after the line-based parse landed:
    fixing the substring search had opened this next to it.
    """
    outcome = memory_world.analyze_memory(
        "print('progress', end=''); result = {'answer': 'ok'}", timeout=10
    )

    assert outcome.success, f"unterminated stdout swallowed the answer: {outcome.message}"
    assert outcome.message == "ok"


def test_analyze_memory_reads_the_sentinel_as_a_line_not_a_substring(
    memory_world: MemoryWorldModule,
) -> None:
    """The answer's own text must not be mistaken for the marker announcing it.

    The bootstrap prints the sentinel at the start of a line and the JSON after it on the
    SAME line, so searching the whole of stdout for the last occurrence found the copy
    sitting INSIDE the answer -- later in the string than the real one -- and sliced from
    there. A perfectly good result came back as EXECUTION_FAILED.
    """
    outcome = memory_world.analyze_memory(
        "result = {'answer': 'the marker __DIMOS_MEMORY_RESULT__= appears in my text'}",
        timeout=10,
    )

    assert outcome.success, f"a valid answer quoting the marker was rejected: {outcome.message}"
    assert "appears in my text" in outcome.message


def test_analyze_memory_times_out(memory_world: MemoryWorldModule) -> None:
    outcome = memory_world.analyze_memory("import time; time.sleep(1)", timeout=0.01)

    assert not outcome.success
    assert outcome.error_code == "EXECUTION_TIMEOUT"


def test_analyze_memory_rejects_oversized_result(memory_world: MemoryWorldModule) -> None:
    memory_world.config.memory_analysis_max_output_chars = 100

    outcome = memory_world.analyze_memory("result = {'answer': 'x' * 200}", timeout=10)

    assert not outcome.success
    assert outcome.error_code == "RESULT_TOO_LARGE"


def test_navigate_works_against_an_embedding_answer(
    memory_world: MemoryWorldModule, monkeypatch: pytest.MonkeyPatch
) -> None:
    """Navigate has to route to a place the embeddings found.

    `_navigate_to` reads the answer's places off `_last_answer`, and only the Hyperspace
    path ever set it -- so with Hyperspace gone every Navigate press answered 409, "the
    last answer is not a Hyperspace one". Nothing caught that: no test drove /navigate
    after a SigLIP answer, because until now there was always a Hyperspace answer to use.
    """
    from dimos.teleop.memory_world.hyperspace_answers import NavigateRequest

    places = [SimpleNamespace(position=(3.0, 0.0, 0.5)), SimpleNamespace(position=(9.0, 0.0, 0.5))]
    query_id = "q-clip"
    memory_world._active_query_result = {"query_id": query_id}
    memory_world._active_query_images = []
    with memory_world._clients_lock:
        memory_world._last_answer = (
            SimpleNamespace(
                clusters=[
                    SimpleNamespace(index=i, centre=tuple(p.position), radius=None)
                    for i, p in enumerate(places)
                ]
            ),
            query_id,
        )
    monkeypatch.setattr(memory_world, "_ground_under_viewer", lambda: (0.0, 0.0, 0.0))
    monkeypatch.setattr(memory_world, "_broadcast", lambda *a, **k: None)
    monkeypatch.setattr(
        memory_world,
        "_planner",
        lambda: SimpleNamespace(
            plan=lambda start, goal: SimpleNamespace(
                points=[(0.0, 0.0, 0.0), (float(goal[0]), 0.0, 0.0)],
                length_m=float(goal[0]),
                cells=4,
                planner="mls",
            )
        ),
    )

    payload = memory_world._navigate_to(NavigateRequest(cluster=1, query_id=query_id))

    assert payload["length_m"] == 9.0, "did not route to the second place the embeddings found"
    assert payload["goal"] == [9.0, 0.0, 0.5]


def test_an_analysis_answer_does_not_draw_a_route_to_where_you_are_standing(
    memory_world, monkeypatch
) -> None:
    """`_navigate_to` refuses a route whose two ends are the same point, with a comment
    recording it measured live -- three copies of one pose returned as a successful 200.

    `_add_route_to_result` calls the same planner and only counted the points. It runs
    automatically on every analysis answer carrying a focus point, so a zero-length line
    was drawn on the map saying "here is the way there" about somewhere the viewer was
    already standing.
    """
    from types import SimpleNamespace

    from dimos.teleop.memory_world.query import MemoryQueryResult

    here = (1.25, 2.25, 0.0)
    memory_world._viewer_position = here

    costmap = SimpleNamespace(resolution=0.1)
    streams = SimpleNamespace(
        global_costmap=SimpleNamespace(last=lambda: SimpleNamespace(data=costmap))
    )
    monkeypatch.setattr(
        memory_world,
        "_ensure_store",
        lambda: SimpleNamespace(list_streams=lambda: ["global_costmap"], streams=streams),
    )

    def one_place(costmap, goal, start):  # the shape `_navigate_to`'s comment describes
        pose = SimpleNamespace(x=here[0], y=here[1], z=0.0)
        return SimpleNamespace(poses=[pose, pose, pose])

    monkeypatch.setattr("dimos.teleop.memory_world.hyperspace_answers.min_cost_astar", one_place)

    result = MemoryQueryResult(answer="x", focus_point=here)
    memory_world._add_route_to_result(result)
    assert result.route is None, f"drew a route that goes nowhere: {result.route}"

    # A real route still arrives.
    def a_real_route(costmap, goal, start):
        return SimpleNamespace(
            poses=[SimpleNamespace(x=1.25, y=2.25, z=0.0), SimpleNamespace(x=4.25, y=2.25, z=0.0)]
        )

    monkeypatch.setattr("dimos.teleop.memory_world.hyperspace_answers.min_cost_astar", a_real_route)
    memory_world._add_route_to_result(result)
    assert result.route is not None and len(result.route.points) == 2


def test_analysis_cannot_write_to_the_recording(memory_world, tmp_path) -> None:  # type: ignore[no-untyped-def]
    """Analysis READS the recording. `open_recording` hands back a read-write store --
    there is no read-only mode -- so a snippet that appended a stream left it in the
    operator's recording for good, and `analyze_memory` reported success.

    Measured before the guard: an injected stream survived and the `.db` grew by 24 KB.
    """
    import os

    before = os.path.getsize(memory_world.config.store_path)
    outcome = memory_world.analyze_memory(
        code=(
            "store.stream('injected_by_analysis', dict).append({'x': 1}, ts=1.0)\n"
            "result = {'answer': 'wrote'}\n"
        ),
        timeout=30.0,
    )
    after = os.path.getsize(memory_world.config.store_path)

    assert not outcome.success, "analysis wrote to the recording and was told it worked"
    assert "cannot write" in (outcome.message or ""), outcome.message
    assert after == before, f"the recording grew by {after - before} bytes"
    assert "injected_by_analysis" not in memory_world._ensure_store().list_streams()

    # Reading still works, which is the whole point of the skill.
    read = memory_world.analyze_memory(
        code="result = {'answer': f'{len(store.list_streams())} streams'}\n", timeout=30.0
    )
    assert read.success, read.message


def test_a_filtered_view_of_a_stream_cannot_write_either(memory_world, tmp_path) -> None:  # type: ignore[no-untyped-def]
    """A stream hands back MORE STREAMS.

    `limit`, `after`, `near`, `order_by` and a dozen others each return another view of
    the same table, with the same `append` on it -- so forwarding them handed the analysis
    a writable object through the read-only wrapper, and
    `store.streams[name].limit(1).append(...)` put a row in the operator's recording.
    """
    import os

    # The recording needs something in it to filter. The module's own store writes it:
    # the rule under test is about what ANALYSIS may do, not the module.
    memory_world._ensure_store().stream("measurements", dict).append({"x": 1}, ts=1.0)

    before = os.path.getsize(memory_world.config.store_path)
    outcome = memory_world.analyze_memory(
        code=(
            "store.streams['measurements'].limit(1).append({'x': 99}, ts=2.0)\n"
            "result = {'answer': 'wrote'}\n"
        ),
        timeout=30.0,
    )
    after = os.path.getsize(memory_world.config.store_path)

    assert not outcome.success, "analysis wrote through a filtered view and was told it worked"
    assert "cannot write" in (outcome.message or ""), outcome.message
    assert after == before, f"the recording grew by {after - before} bytes"

    # `save(target)` is the same hole by another name: it appends every observation into
    # the TARGET's backend, and the target is reached through the wrapper too.
    memory_world._ensure_store().stream("copy_here", dict).append({"x": 0}, ts=0.0)
    copied = memory_world.analyze_memory(
        code=(
            "n = store.streams['measurements'].save(store.streams['copy_here']).drain()\n"
            "result = {'answer': 'copied %d' % n}\n"
        ),
        timeout=30.0,
    )
    assert not copied.success, "analysis copied rows into the recording through save()"
    assert "cannot write" in (copied.message or ""), copied.message

    # Filtering still READS, which is what those methods are for.
    read = memory_world.analyze_memory(
        code="result = {'answer': f\"{len(list(store.streams['measurements'].limit(1)))} row\"}\n",
        timeout=30.0,
    )
    assert read.success, read.message
    assert "1 row" in (memory_world._active_query_result or {}).get("answer", "")


def test_a_grandchild_that_ignores_sigterm_is_killed_anyway(memory_world, tmp_path) -> None:  # type: ignore[no-untyped-def]
    """The second signal is not a question about the direct child.

    `_end_group` returned as soon as the child it launched was gone, so the SIGKILL pass
    never ran -- and the group is exactly where the things that ignore SIGTERM are.
    Measured: the child died on the TERM, its grandchild went on appending to a file every
    0.2 s for as long as the machine was up, and `analyze_memory` reported
    EXECUTION_TIMEOUT.
    """
    import sys
    import time

    beat = tmp_path / "stubborn.txt"
    stubborn = tmp_path / "stubborn.py"
    stubborn.write_text(
        "import signal, time\n"
        "signal.signal(signal.SIGTERM, signal.SIG_IGN)\n"
        f"handle = open({str(beat)!r}, 'a')\n"
        "while True:\n"
        "    handle.write('x')\n"
        "    handle.flush()\n"
        "    time.sleep(0.2)\n"
    )

    outcome = memory_world.analyze_memory(
        code=(
            "import subprocess, sys, time\n"
            f"subprocess.Popen([sys.executable, {str(stubborn)!r}])\n"
            "time.sleep(30)\n"
            "result = {'answer': 'never'}\n"
        ),
        timeout=2.0,
    )
    assert not outcome.success and outcome.error_code == "EXECUTION_TIMEOUT", outcome.message

    time.sleep(0.5)
    grew = beat.stat().st_size if beat.exists() else 0
    assert grew > 0, "the fixture's grandchild never ran, so it proves nothing"
    time.sleep(1.0)
    still = beat.stat().st_size if beat.exists() else 0
    assert still == grew, (
        f"a grandchild that ignores SIGTERM outlived the timeout ({grew} -> {still} bytes)"
    )
    assert sys.executable  # the interpreter the grandchild ran under, for the record


def test_a_timed_out_analysis_takes_what_it_started_with_it(memory_world, tmp_path) -> None:  # type: ignore[no-untyped-def]
    """`subprocess.run(timeout=...)` signals only the child it launched, and analysis
    code is free to spawn.

    Measured before: a snippet that started a background loop and then slept past the
    timeout was reported as EXECUTION_TIMEOUT while that loop went on writing, unmanaged,
    with nothing left holding a handle to it.
    """
    import time

    beat = tmp_path / "beat.txt"
    outcome = memory_world.analyze_memory(
        code=(
            "import subprocess, time\n"
            f"subprocess.Popen(['bash','-c','for i in $(seq 1 200); do echo x >> {beat};"
            " sleep 0.05; done'])\n"
            "time.sleep(30)\n"
            "result = {'answer': 'never'}\n"
        ),
        timeout=1.5,
    )
    assert not outcome.success and outcome.error_code == "EXECUTION_TIMEOUT"

    time.sleep(0.4)
    grew = beat.stat().st_size if beat.exists() else 0
    time.sleep(1.0)
    still = beat.stat().st_size if beat.exists() else 0
    assert still == grew, (
        f"the analysis was reported as stopped but what it started kept running"
        f" ({grew} -> {still} bytes)"
    )
