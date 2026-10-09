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


import asyncio
from collections.abc import Callable
from pathlib import Path
import sqlite3
from typing import Any

import pytest

from experimental.gateway.utils.events import Bus
from experimental.gateway.utils.models import DimosEvent, UploadedByPath, UploadList
from experimental.gateway.utils.uploads import Queue, Rate, Uploads, check_path


def queue_with(n: int) -> tuple[Queue, list[str]]:
    queue = Queue()
    ids = [queue.enqueue(Path(f"/r/{i}.mcap"), 1000, None, None)[0]["id"] for i in range(n)]
    return queue, ids


def test_fifo_one_at_a_time() -> None:
    queue, ids = queue_with(3)
    assert queue.next_queued() == ids[0]
    queue.start(ids[0])
    assert queue.next_queued() is None
    queue.succeed(ids[0], {"uploadId": "abc", "skipped": False, "quota": {"state": "ok"}})
    assert queue.get(ids[0])["state"] == "done"  # type: ignore[index]
    assert queue.next_queued() == ids[1]


def test_same_path_is_not_queued_twice() -> None:
    queue = Queue()
    first, added = queue.enqueue(Path("/r/a.mcap"), 5, "", None)
    assert added and first["robotId"] is None
    again, added = queue.enqueue(Path("/r/a.mcap"), 5, None, None)
    assert not added and again["id"] == first["id"]
    queue.start(first["id"])
    queue.fail(first["id"], "network", "offline", None)
    assert queue.enqueue(Path("/r/a.mcap"), 5, None, None)[1]


def test_cancel_retry_clear() -> None:
    queue, ids = queue_with(3)
    queue.start(ids[0])
    assert queue.cancel(ids[0]) == "running"
    queue.cancelled(ids[0])
    assert queue.cancel(ids[1]) == "cancelled"
    assert queue.next_queued() == ids[2]
    assert queue.retry(ids[0])["state"] == "queued"
    assert queue.items[-1]["id"] == ids[0]
    with pytest.raises(ValueError):
        queue.retry(ids[2])
    assert queue.cancel(ids[1]) == "removed" and queue.get(ids[1]) is None
    queue.start(ids[2])
    queue.fail(ids[2], "quota", "full", None)
    queue.clear_finished()
    assert [u["id"] for u in queue.items] == [ids[0]]
    assert queue.cancel("nope") is None


def test_not_logged_in_waits_then_resumes() -> None:
    queue, ids = queue_with(2)
    queue.start(ids[0])
    upload = queue.needs_login(ids[0], "Not logged in")
    assert upload is not None and (upload["state"], upload["errorCode"]) == (
        "queued",
        "not_logged_in",
    )
    assert queue.next_queued() is None
    assert queue.logged_in() and not queue.logged_in()
    assert queue.next_queued() == ids[0]


def test_restore_requeues_the_running_one() -> None:
    queue, ids = queue_with(2)
    queue.start(ids[0])
    queue.progress(ids[0], "upload", 10, 100, 1.0)
    restored = Queue([dict(u) for u in queue.items])
    assert restored.items[0]["state"] == "queued" and restored.next_queued() == ids[0]
    assert restored.enqueue(Path("/r/new.mcap"), 1, None, None)[0]["id"] == "u3"


def test_progress_rate_and_eta() -> None:
    queue, (id,) = queue_with(1)
    queue.start(id)
    u = queue.progress(id, "compress", 0, 0, 0.0)
    assert u is not None and (u["phase"], u["rateBps"]) == ("compress", None)
    u = queue.progress(id, "upload", 200, 1200, 1.0)
    assert u is not None and (u["bytesDone"], u["rateBps"]) == (200, None)
    u = queue.progress(id, "upload", 300, 1200, 2.0)
    assert u is not None and (u["rateBps"], u["etaSeconds"]) == (100.0, 9.0)
    u = queue.progress(id, "upload", 1200, 1200, 4.0)
    assert u is not None and (u["phase"], u["etaSeconds"]) == ("finishing", None)
    u = queue.succeed(
        id, {"uploadId": "x", "skipped": True, "quota": {"state": "warn", "message": "90% used"}}
    )
    assert u is not None and u["skipped"] and u["notice"] == "90% used"


def test_eta_needs_a_second_and_counts_time_since_the_last_tick() -> None:
    rate = Rate()
    rate.tick(0.0, 0)
    rate.tick(0.3, 30_000)
    assert rate.bps() is not None and rate.eta(1_000_000, 0.3) is None
    rate = Rate()
    rate.tick(0.0, 0)
    rate.tick(1.0, 1000)
    assert rate.eta(10_000, 4.0) == 7.0
    assert rate.eta(10_000, 100.0) == 1.0
    rate.tick(2.0, 10)
    assert rate.bps() is None


def test_checks_paths(tmp_path: Path) -> None:
    (tmp_path / "a.mcap").write_bytes(b"1234")
    (tmp_path / "b.txt").write_bytes(b"x")
    (tmp_path / "c.db-wal").write_bytes(b"x")
    with sqlite3.connect(tmp_path / "memory.db") as db:
        db.execute("CREATE TABLE _streams (name TEXT, config TEXT)")
    with sqlite3.connect(tmp_path / "other.db") as db:
        db.execute("CREATE TABLE t (x)")
    assert check_path(str(tmp_path / "a.mcap"))[1] == 4
    assert check_path(str(tmp_path / "memory.db"))[0].name == "memory.db"
    for bad, why in [
        ("relative.mcap", "absolute"),
        (str(tmp_path / "gone.mcap"), "no such file"),
        (str(tmp_path / "b.txt"), "not a dimos recording"),
        (str(tmp_path / "c.db-wal"), "not a dimos recording"),
        (str(tmp_path / "other.db"), "not a dimos recording"),
        (str(tmp_path), "not a file"),
    ]:
        with pytest.raises(ValueError, match=why):
            check_path(bad)


async def until(condition: Callable[[], Any], timeout: float = 20) -> None:
    for _ in range(int(timeout / 0.05)):
        if condition():
            return
        await asyncio.sleep(0.05)
    raise AssertionError("timed out")


async def test_uploads_run_in_a_worker_and_are_remembered(
    tmp_path: Path, fake_worker: list[str], check_model: Any
) -> None:
    events: list[dict[str, Any]] = []
    bus = Bus()
    bus.send = events.append  # type: ignore[method-assign, assignment]
    file = tmp_path / "state" / "uploads.json"
    uploads = Uploads(tmp_path, bus, file, tmp_path / "uploads.log", worker=fake_worker)
    worker = asyncio.create_task(uploads.work())
    try:
        mcap = tmp_path / "a.mcap"
        mcap.write_bytes(b"1234")
        upload = uploads.enqueue(str(mcap), None, None)
        await until(lambda: uploads.queue.get(upload["id"])["state"] == "done")  # type: ignore[index]
        done = uploads.queue.get(upload["id"])
        assert done is not None and (done["uploadId"], done["link"]) == (
            "cloud-1",
            "https://console.x/console/data",
        )
        assert {e["type"] for e in events} >= {"upload"}
        one = uploads.uploaded_one(str(mcap))
        assert one is not None and (one["uploadId"], one["size"], one["changed"]) == (
            "cloud-1",
            4,
            False,
        )

        # a running one is cancelled by killing its worker
        slow = tmp_path / "slow.mcap"
        slow.write_bytes(b"x")
        running = uploads.enqueue(str(slow), None, None)
        await until(lambda: uploads.running is not None)
        uploads.cancel(running["id"])
        await until(lambda: uploads.queue.get(running["id"])["state"] == "cancelled")  # type: ignore[index]

        # not logged in: back in the queue, which waits
        nologin = tmp_path / "nologin.mcap"
        nologin.write_bytes(b"x")
        waiting = uploads.enqueue(str(nologin), None, None)
        await until(lambda: uploads.queue.waiting_for_login)
        assert uploads.queue.get(waiting["id"])["errorCode"] == "not_logged_in"  # type: ignore[index]
        assert (await uploads.account(False))["loggedIn"] is True
        assert not uploads.queue.waiting_for_login
        for event in events:
            check_model(DimosEvent, event, f"the {event['type']} event")
        check_model(UploadList, uploads.listing(), "the upload list")
    finally:
        uploads.shutdown()
        worker.cancel()
    # a restart keeps the queue and what is in the cloud; a changed file is marked
    again = Uploads(tmp_path, Bus(), file, tmp_path / "uploads.log", worker=fake_worker)
    assert again.uploaded()["byPath"][str(mcap)]["uploadId"] == "cloud-1"
    check_model(UploadedByPath, again.uploaded(), "the uploaded list")
    mcap.write_bytes(b"123456")
    assert again.uploaded_one(str(mcap))["changed"] is True  # type: ignore[index]


async def test_device_login(tmp_path: Path, fake_worker: list[str]) -> None:
    uploads = Uploads(tmp_path, Bus(), None, tmp_path / "uploads.log", worker=fake_worker)
    login = await uploads.start_login()
    assert (login["state"], login["code"], login["url"]) == (
        "pending",
        "ABCD",
        "https://console/device",
    )
    assert (await uploads.start_login())["code"] == "ABCD", (
        "a pending login is returned, not restarted"
    )
    (tmp_path / "approve").touch()
    await until(lambda: uploads.login_state()["state"] == "approved")
    assert uploads.login_state()["email"] == "a@b.c"
    (tmp_path / "approve").unlink()
    await uploads.start_login()
    assert uploads.cancel_login()["state"] == "idle"
