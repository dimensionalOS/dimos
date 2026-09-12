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


"""Loading, adopting and reloading the Hyperspace index.

Split out of `test_hyperspace_search.py`, which had grown past the repo's 75 KB
file hook. These are the tests that drive `_load_hyperspace` and
`_adopt_an_index_that_appeared` through `_loader_module`: what happens when a
load fails, when it fails repeatedly, when it recovers, and when an index
changes underneath a module that is already serving one.
"""

from __future__ import annotations

import sqlite3
import threading
import time
from types import SimpleNamespace
from unittest import mock

from dimos.teleop.memory_world.hyperspace_search import (
    KEYFRAME_STREAM,
    PATCH_STREAM,
    memory_db_index_stamp,
    memory_db_ready,
)


def _loader_module(tmp_path, make_search):  # type: ignore[no-untyped-def]
    """A HyperspaceAnswers with only what `_load_hyperspace` touches."""
    from dimos.teleop.memory_world.hyperspace_answers import HyperspaceAnswers

    class Module(HyperspaceAnswers):
        def __init__(self) -> None:
            self._hyperspace = None
            self._hyperspace_error = None
            self._hyperspace_lock = threading.Lock()
            self._hyperspace_build_lock = threading.Lock()
            self._adopting = threading.Lock()
            self._adopted_stamp = (0, 0.0)
            self._failed_stamp = None
            self._failed_count = 0
            self._failed_at = 0.0
            self._stopping = threading.Event()
            self._prepare_job = SimpleNamespace(
                status=lambda: {"embedding": "idle", "progress": 0.0}
            )
            self.config = SimpleNamespace(
                store_path=str(tmp_path / "walk.db"),
                hyperspace_model_name="m",
                world_frame="odom",
                hyperspace_voxel_size=0.1,
                hyperspace_device="cpu",
                hyperspace_segments=False,
                hyperspace_refine=False,
            )

        def _map_points(self):  # type: ignore[no-untyped-def]
            return None

        def _broadcast(self, message: bytes | str) -> None:
            pass

    return Module()


def test_a_reload_keeps_answering_until_the_replacement_is_warm(tmp_path) -> None:  # type: ignore[no-untyped-def]
    """The search must stay live while its replacement warms, and survive a failed reload.

    The reload used to close the loaded search and set `_hyperspace = None` BEFORE
    building the new one. Warming takes seconds, and a running ingest re-triggers the
    reload as soon as each one lands, so the reloads chain: with a 12 s warm, 9 of 11
    status polls answered `ready: false, keyframes: 0` and every question got
    "Hyperspace is reloading". The search was down for essentially the whole re-ingest --
    the exact workflow the reload exists to serve.

    Worse, a FAILED reload was unrecoverable: the close had already happened, so the
    error was latched with `_hyperspace` already None, and `_adopt_an_index_that_appeared`
    returns for ever once `_hyperspace_error` is set.
    """
    closed: list[int] = []

    class Search:
        def __init__(self, tag: int) -> None:
            self.tag = tag
            self.keyframe_count = tag
            self.segment_count = 0

        def warm(self) -> None:
            pass

        def close(self) -> None:
            closed.append(self.tag)

    module = _loader_module(tmp_path, None)
    seen_during_warm: list[object] = []

    def build_first(*a, **k):  # type: ignore[no-untyped-def]
        return Search(1)

    with (
        mock.patch(
            "dimos.teleop.memory_world.hyperspace_answers.memory_db_index_stamp",
            lambda _p: (1, 1.0),
        ),
        mock.patch("dimos.teleop.memory_world.hyperspace_answers.HyperspaceSearch", build_first),
    ):
        assert module._load_hyperspace() is True
    first = module._hyperspace
    assert first is not None and first.tag == 1

    # A reload whose warm observes what a query would see at that moment.
    def build_second(*a, **k):  # type: ignore[no-untyped-def]
        seen_during_warm.append(module._hyperspace)
        return Search(2)

    with (
        mock.patch(
            "dimos.teleop.memory_world.hyperspace_answers.memory_db_index_stamp",
            lambda _p: (2, 2.0),
        ),
        mock.patch("dimos.teleop.memory_world.hyperspace_answers.HyperspaceSearch", build_second),
    ):
        assert module._load_hyperspace(reload=True) is True
    assert seen_during_warm == [first], (
        "the old search was dropped before the new one was warm: every question in that"
        " window answers 'Hyperspace is reloading' and the status poll reports 0 keyframes"
    )
    assert module._hyperspace is not None and module._hyperspace.tag == 2
    assert closed == [1], "the replaced search was not closed after the swap"

    # A reload that FAILS keeps the working search and does not latch an error.
    def build_broken(*a, **k):  # type: ignore[no-untyped-def]
        raise SystemExit("the memory db is from another model")

    with (
        mock.patch(
            "dimos.teleop.memory_world.hyperspace_answers.memory_db_index_stamp",
            lambda _p: (3, 3.0),
        ),
        mock.patch("dimos.teleop.memory_world.hyperspace_answers.HyperspaceSearch", build_broken),
    ):
        assert module._load_hyperspace(reload=True) is False
    assert module._hyperspace is not None and module._hyperspace.tag == 2, (
        "a failed reload cost the module its working search"
    )
    assert module._hyperspace_error is None, (
        "a failed reload latched an error, and the adopt returns for ever once it is set"
    )


def test_a_question_can_be_answered_while_a_replacement_warms(tmp_path) -> None:  # type: ignore[no-untyped-def]
    """Warming must not hold the lock questions are answered under.

    The first fix stopped the reload from NULLING the search, but kept the warm inside
    `_hyperspace_lock` -- and `_find_with_hyperspace` holds that same lock for the whole
    of an answer. So every question blocked for the length of the warm instead of failing
    fast. A hung request is not an improvement on a failed one, and the status poll
    reporting `ready: true` throughout made it worse: the viewer had no reason to wait.
    """

    free_during_warm: list[bool] = []

    class Search:
        def __init__(self, tag: int) -> None:
            self.tag = tag
            self.keyframe_count = tag
            self.segment_count = 0

        def warm(self) -> None:
            pass

        def close(self) -> None:
            pass

    module = _loader_module(tmp_path, None)
    with (
        mock.patch(
            "dimos.teleop.memory_world.hyperspace_answers.memory_db_index_stamp",
            lambda _p: (1, 1.0),
        ),
        mock.patch(
            "dimos.teleop.memory_world.hyperspace_answers.HyperspaceSearch",
            lambda *a, **k: Search(1),
        ),
    ):
        assert module._load_hyperspace() is True

    def build_second(*a, **k):  # type: ignore[no-untyped-def]
        # What a question would find at the moment the replacement is being warmed.
        got = module._hyperspace_lock.acquire(blocking=False)
        free_during_warm.append(got)
        if got:
            module._hyperspace_lock.release()
        return Search(2)

    with (
        mock.patch(
            "dimos.teleop.memory_world.hyperspace_answers.memory_db_index_stamp",
            lambda _p: (2, 2.0),
        ),
        mock.patch("dimos.teleop.memory_world.hyperspace_answers.HyperspaceSearch", build_second),
    ):
        assert module._load_hyperspace(reload=True) is True

    assert free_during_warm == [True], (
        "the query lock was held while the replacement warmed: every question blocks for"
        " the whole warm, and the status poll says ready throughout"
    )
    assert module._hyperspace is not None and module._hyperspace.tag == 2


def test_a_reload_that_keeps_failing_does_not_retry_for_ever(tmp_path) -> None:  # type: ignore[no-untyped-def]
    """A failed reload must be recorded, or every status poll starts another warm.

    Two bugs in sequence here. The reload used to close the search first, so a failure
    latched `_hyperspace_error` with nothing behind it and the adopt gave up for ever.
    Fixing that by returning early traded it for the opposite: the adopt compares the
    db's stamp against `_adopted_stamp`, so an unchanged stamp with no error meant every
    poll spawned a fresh build and warm -- 7 attempts over 10 polls and climbing --
    each one loading the text tower and every keyframe, while the status kept reporting
    `ready: true` and the OLD keyframe count.

    Recording the stamp says "this index has been tried". An index that changes again
    still gets a fresh attempt, which is the one case worth retrying.
    """

    from dimos.teleop.memory_world.hyperspace_answers import RELOAD_ATTEMPTS

    class Search:
        keyframe_count = 1
        segment_count = 0

        def warm(self) -> None:
            pass

        def close(self) -> None:
            pass

    module = _loader_module(tmp_path, None)
    with (
        mock.patch(
            "dimos.teleop.memory_world.hyperspace_answers.memory_db_index_stamp",
            lambda _p: (1, 1.0),
        ),
        mock.patch(
            "dimos.teleop.memory_world.hyperspace_answers.HyperspaceSearch",
            lambda *a, **k: Search(),
        ),
    ):
        assert module._load_hyperspace() is True
    working = module._hyperspace
    assert working is not None

    attempts: list[int] = []

    def build_broken(*a, **k):  # type: ignore[no-untyped-def]
        attempts.append(1)
        raise SystemExit("the memory db is from another model")

    def settle() -> None:
        for _ in range(200):
            if module._adopting.acquire(blocking=False):
                module._adopting.release()
                return
            time.sleep(0.01)

    adopt = type(module)._adopt_an_index_that_appeared
    with (
        mock.patch(
            "dimos.teleop.memory_world.hyperspace_answers.memory_db_index_stamp",
            lambda _p: (2, 2.0),  # the db changed once and then stopped changing
        ),
        mock.patch("dimos.teleop.memory_world.hyperspace_answers.HyperspaceSearch", build_broken),
        # Zero the spacing so this asserts the COUNT bound; the spacing is a separate
        # property and asserting both here would only make the test slow.
        mock.patch("dimos.teleop.memory_world.hyperspace_answers.RELOAD_BACKOFF_S", 0.0),
    ):
        for _ in range(10):
            adopt(module)
            settle()

    assert len(attempts) == RELOAD_ATTEMPTS, (
        f"a failing reload was attempted {len(attempts)} times over 10 polls; it should"
        f" get {RELOAD_ATTEMPTS} spaced tries, not one and not one per poll"
    )
    assert module._hyperspace is working, "the working search was lost"
    assert module._hyperspace_error is None, "the module is not broken; nothing should latch"

    # An index that changes AGAIN is still worth one more attempt.
    with (
        mock.patch(
            "dimos.teleop.memory_world.hyperspace_answers.memory_db_index_stamp",
            lambda _p: (3, 3.0),
        ),
        mock.patch("dimos.teleop.memory_world.hyperspace_answers.HyperspaceSearch", build_broken),
    ):
        adopt(module)
        settle()
    assert len(attempts) == RELOAD_ATTEMPTS + 1, (
        "a genuinely NEW index was not retried: the attempt budget is per-index, not a"
        " global give-up, so a db that changes again always gets a fresh look"
    )


def test_a_failed_reload_waits_longer_before_each_retry(tmp_path) -> None:  # type: ignore[no-untyped-def]
    """The retries are SPACED, which is half of what the backoff is for.

    The two reload tests either side of this one patch RELOAD_BACKOFF_S to 0.0 --
    deliberately, to isolate the COUNT bound -- so between them nothing exercised the
    spacing their docstrings promise, and deleting the gap left the whole suite green.

    Two things this needs that are easy to get wrong. The backoff only applies while
    REPLACING a working search: a first load that fails latches `_hyperspace_error`
    instead, so a working one has to be in place first. And time is moved by winding
    `_failed_at` backwards rather than by patching the clock, because `time.monotonic` is
    what threading itself waits on -- replacing it globally makes the adopt thread's own
    lock behave strangely and the test measures the mock.
    """

    class Search:
        keyframe_count = 1
        segment_count = 0

        def warm(self) -> None:
            pass

        def close(self) -> None:
            pass

    module = _loader_module(tmp_path, None)
    with (
        mock.patch(
            "dimos.teleop.memory_world.hyperspace_answers.memory_db_index_stamp",
            lambda _p: (1, 1.0),
        ),
        mock.patch(
            "dimos.teleop.memory_world.hyperspace_answers.HyperspaceSearch",
            lambda *a, **k: Search(),
        ),
    ):
        assert module._load_hyperspace() is True

    attempts: list[int] = []

    def build_broken(*a, **k):  # type: ignore[no-untyped-def]
        attempts.append(1)
        raise SystemExit("the memory db is from another model")

    def settle() -> None:
        for _ in range(200):
            if module._adopting.acquire(blocking=False):
                module._adopting.release()
                return
            time.sleep(0.01)

    adopt = type(module)._adopt_an_index_that_appeared
    with (
        mock.patch(
            "dimos.teleop.memory_world.hyperspace_answers.memory_db_index_stamp",
            lambda _p: (2, 2.0),  # a DIFFERENT index, so a reload is due
        ),
        mock.patch("dimos.teleop.memory_world.hyperspace_answers.HyperspaceSearch", build_broken),
        mock.patch("dimos.teleop.memory_world.hyperspace_answers.RELOAD_BACKOFF_S", 10.0),
    ):
        adopt(module)
        settle()
        assert len(attempts) == 1, attempts
        assert module._failed_count == 1

        # Inside the first gap (10 s x 1 failure): refused without building anything.
        module._failed_at = time.monotonic() - 9.0
        adopt(module)
        settle()
        assert len(attempts) == 1, "retried before the backoff had elapsed"

        # Past it.
        module._failed_at = time.monotonic() - 11.0
        adopt(module)
        settle()
        assert len(attempts) == 2, "never retried after the backoff elapsed"
        assert module._failed_count == 2

        # The second gap is 10 s x 2 failures, so 11 s is now still INSIDE it. That is
        # what "spaced further apart each time" means; a flat gap fails this line.
        module._failed_at = time.monotonic() - 11.0
        adopt(module)
        settle()
        assert len(attempts) == 2, "the second gap was no longer than the first"

        module._failed_at = time.monotonic() - 21.0
        adopt(module)
        settle()
        assert len(attempts) == 3, attempts


def test_a_reload_that_fails_once_recovers_when_it_stops_failing(tmp_path) -> None:  # type: ignore[no-untyped-def]
    """A TRANSIENT failure must not be permanent.

    The first fix for the infinite-retry bug recorded a failed stamp as simply "tried",
    which made one unlucky warm -- an allocation that lost a race with another heavy
    process -- permanent: the module went on answering from the older index for the life
    of the process, reporting `ready: true` with no error, while a perfectly good index
    sat on disk. Verified against that version with a single injected MemoryError: it
    made one attempt and stayed on the stale index.

    So the record is "attempted N times", not "tried", and the attempts are spaced.
    """
    from dimos.teleop.memory_world.hyperspace_answers import HyperspaceAnswers

    class Search:
        def __init__(self, n: int) -> None:
            self.keyframe_count = n
            self.segment_count = 0

        def warm(self) -> None:
            pass

        def close(self) -> None:
            pass

    module = _loader_module(tmp_path, None)
    with (
        mock.patch(
            "dimos.teleop.memory_world.hyperspace_answers.memory_db_index_stamp",
            lambda _p: (10, 10.0),
        ),
        mock.patch(
            "dimos.teleop.memory_world.hyperspace_answers.HyperspaceSearch",
            lambda *a, **k: Search(10),
        ),
    ):
        assert module._load_hyperspace() is True
    assert module._hyperspace.keyframe_count == 10

    calls: list[int] = []

    def build_flaky(*a, **k):  # type: ignore[no-untyped-def]
        calls.append(1)
        if len(calls) == 1:
            raise MemoryError("lost a race with another heavy process")
        return Search(20)

    def settle() -> None:
        for _ in range(200):
            if module._adopting.acquire(blocking=False):
                module._adopting.release()
                return
            time.sleep(0.01)

    adopt = HyperspaceAnswers._adopt_an_index_that_appeared
    with (
        mock.patch(
            "dimos.teleop.memory_world.hyperspace_answers.memory_db_index_stamp",
            lambda _p: (20, 20.0),
        ),
        mock.patch("dimos.teleop.memory_world.hyperspace_answers.HyperspaceSearch", build_flaky),
        mock.patch("dimos.teleop.memory_world.hyperspace_answers.RELOAD_BACKOFF_S", 0.0),
    ):
        adopt(module)  # fails
        settle()
        assert module._hyperspace.keyframe_count == 10, "the working index was lost"
        adopt(module)  # and now it works
        settle()

    assert len(calls) == 2, f"the newer index was attempted {len(calls)} times, not 2"
    assert module._hyperspace.keyframe_count == 20, (
        "a single transient failure permanently suppressed the newer index: the module"
        " keeps answering from the stale one, reporting ready with no error"
    )
    assert module._failed_stamp is None, "the failure record survived a success"


def test_a_transient_failure_on_the_very_first_load_is_not_permanent(tmp_path) -> None:  # type: ignore[no-untyped-def]
    """The sibling above proves a RELOAD recovers. The first load did not.

    With nothing loaded yet, `_load_hyperspace` latched `_hyperspace_error` on the first
    failure, and `_adopt_an_index_that_appeared` returns at its very first line whenever
    that is set -- so no later poll could ever try again. One MemoryError, or one of the
    MPS hiccups this package sees on macOS, cost the index for the life of the process,
    and the only recovery the UI offers is re-running the whole GPU ingest.

    Same injected failure, on the first load instead of a reload.
    """
    from dimos.teleop.memory_world.hyperspace_answers import HyperspaceAnswers

    class Search:
        def __init__(self, n: int) -> None:
            self.keyframe_count = n
            self.segment_count = 0

        def warm(self) -> None:
            pass

        def close(self) -> None:
            pass

    calls: list[int] = []

    def build_flaky(*a, **k):  # type: ignore[no-untyped-def]
        calls.append(1)
        if len(calls) == 1:
            raise MemoryError("lost a race with another heavy process")
        return Search(20)

    def settle() -> None:
        for _ in range(200):
            if module._adopting.acquire(blocking=False):
                module._adopting.release()
                return
            time.sleep(0.01)

    module = _loader_module(tmp_path, None)
    assert module._hyperspace is None  # nothing loaded: this is the first-load path

    adopt = HyperspaceAnswers._adopt_an_index_that_appeared
    with (
        mock.patch(
            "dimos.teleop.memory_world.hyperspace_answers.memory_db_index_stamp",
            lambda _p: (20, 20.0),
        ),
        mock.patch("dimos.teleop.memory_world.hyperspace_answers.HyperspaceSearch", build_flaky),
        mock.patch("dimos.teleop.memory_world.hyperspace_answers.RELOAD_BACKOFF_S", 0.0),
    ):
        adopt(module)  # fails
        settle()
        assert module._hyperspace_error is None, (
            "one transient failure latched an error no retry can get past"
        )
        adopt(module)  # and now it works
        settle()

    assert len(calls) == 2, f"the index was attempted {len(calls)} times, not 2"
    assert module._hyperspace is not None and module._hyperspace.keyframe_count == 20


def test_keyframes_without_patches_are_not_an_index(tmp_path) -> None:  # type: ignore[no-untyped-def]
    """An index whose patch stream is empty must not read as ready.

    It takes two streams to answer a question: the patches are the searchable content and
    the keyframes only say where each was seen from. Counting keyframes alone made
    "the keyframe stream is not empty" mean "ready".

    Seen for real on grocery.db, which is why this exists: an ingest reported a clean
    summary ending `kept: 3462`, wrote 3,462 keyframes and ZERO patches, and left a
    stream whose name, presence and entirely plausible count all said "index". Loading it
    would have reported `ready: true` with 3,462 keyframes and answered nothing.
    """
    recording = tmp_path / "half.db"
    db = sqlite3.connect(recording)
    db.execute("CREATE TABLE _streams (name TEXT)")
    db.execute(f"INSERT INTO _streams VALUES ('{KEYFRAME_STREAM}'), ('{PATCH_STREAM}')")
    db.execute(f'CREATE TABLE "{KEYFRAME_STREAM}" (id INTEGER, ts REAL)')
    db.execute(f'CREATE TABLE "{PATCH_STREAM}" (id INTEGER)')
    db.executemany(
        f'INSERT INTO "{KEYFRAME_STREAM}" VALUES (?, ?)', [(i, 100.0 + i) for i in range(3462)]
    )
    db.commit()

    assert not memory_db_ready(recording), "keyframes with no patches read as a whole index"
    assert memory_db_index_stamp(recording) == (0, 0.0), "a half index got an identity"

    db.executemany(f'INSERT INTO "{PATCH_STREAM}" VALUES (?)', [(i,) for i in range(10)])
    db.commit()
    db.close()
    assert memory_db_ready(recording), "a complete index was refused"
    assert memory_db_index_stamp(recording)[0] == 3462
