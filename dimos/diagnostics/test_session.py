# Copyright 2025-2026 Dimensional Inc.
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

"""A session is ready only after every required participant writes fresh evidence."""

import gzip
import json
from threading import Event
import time

import pytest

from dimos.diagnostics.ingest import decode_batch, ingest
from dimos.diagnostics.session import NetworkSettings, Requirement, SessionSettings, SessionStore
from dimos.diagnostics.session_network import SessionNetwork


@pytest.fixture
def session(tmp_path):
    value = SessionStore(tmp_path)
    value.start(SessionSettings(run_id="lap"))
    return value


def browser_batch(batch_id="first"):
    return {
        "batchId": batch_id,
        "manifest": {
            "schema": "dimos.incident/v1",
            "run_id": "lap",
            "process": "browser-test",
            "clock_domain": "browser/performance",
            "health": {"observer_drops": 0},
        },
        "events": [
            {
                "schema": "dimos.trace/v1",
                "process": "browser-test",
                "clock_domain": "browser/performance",
                "mono_ns": 10,
                "event": "video.presented",
                "context": None,
            }
        ],
    }


def test_start_does_not_imply_ready_and_upload_is_durable(session):
    required = [Requirement(name="headset", kind="browser", event="video.presented")]
    assert session.readiness("lap", required)["ready"] is False
    batch = decode_batch(gzip.compress(json.dumps(browser_batch()).encode()), "gzip")
    ingest(session.root, batch)
    assert session.readiness("lap", required)["ready"] is True
    assert len(session.store.incidents("lap")) == 1
    session.control("lap", "stop")
    assert session.readiness("lap", required)["ready"] is False
    with pytest.raises(ValueError, match="active trace"):
        ingest(session.root, batch)


def test_retry_is_idempotent_and_different_payload_same_id_is_rejected(session):
    batch = browser_batch()
    ingest(session.root, batch)
    ingest(session.root, batch)
    assert len(session.store.incidents("lap")) == 1
    batch["events"][0]["mono_ns"] = 11
    with pytest.raises(ValueError, match="identity reused"):
        ingest(session.root, batch)


def test_session_cannot_take_over_another_run_or_reuse_a_stopped_id(session):
    assert session.start(SessionSettings(run_id="lap"))["run_id"] == "lap"
    with pytest.raises(ValueError, match="another diagnostics session"):
        session.start(SessionSettings(run_id="different"))
    session.control("lap", "stop")
    with pytest.raises(ValueError, match="already used"):
        session.start(SessionSettings(run_id="lap"))
    assert session.start(SessionSettings(run_id="different"))["run_id"] == "different"


@pytest.mark.parametrize("change", ["stale", "no_artifact", "io_error", "stopped", "missing_event"])
def test_readiness_rejects_missing_or_unusable_evidence(session, change):
    ingest(session.root, browser_batch())
    path = session.root / "lap/browser-test/status.json"
    status = json.loads(path.read_text())
    if change == "stale":
        status["at_ns"] = time.time_ns() - 20_000_000_000
    elif change == "io_error":
        status["io_errors"] = 1
    elif change == "stopped":
        status["state"] = "stopped"
    elif change == "missing_event":
        status["last_events"] = {}
    else:
        (path.parent / "incident-first.jsonl").unlink()
    path.write_text(json.dumps(status))
    result = session.readiness(
        "lap", [Requirement(name="video", kind="browser", event="video.presented")]
    )
    assert result["ready"] is False
    assert result["checks"][0]["observed"] == 0


def test_oversized_gzip_and_foreign_context_are_rejected():
    with pytest.raises(ValueError, match="oversized"):
        decode_batch(gzip.compress(b"x" * 2_000_000), "gzip")
    batch = browser_batch()
    batch["events"][0]["context"] = {"run_id": "different"}
    with pytest.raises(ValueError, match="invalid event"):
        decode_batch(json.dumps(batch).encode(), None)


def test_symlink_browser_directory_is_rejected(session, tmp_path):
    outside = tmp_path / "outside"
    outside.mkdir()
    (session.root / "lap/browser-test").symlink_to(outside)
    with pytest.raises(ValueError, match="symlink"):
        ingest(session.root, browser_batch())
    assert list(outside.iterdir()) == []


def test_browser_file_retention_is_bounded_independently_of_byte_budget(session):
    for index in range(4):
        ingest(session.root, browser_batch(str(index)), max_files=2)
    assert [item["id"] for item in session.store.incidents("lap")] == [
        "browser-test/incident-2.jsonl",
        "browser-test/incident-3.jsonl",
    ]
    assert session.store.statuses("lap")[0]["storage_evictions_this_batch"] == 1


def test_session_network_runs_clock_probe_until_stopped(tmp_path, mocker):
    entered = Event()
    stopped = Event()

    def monitor(*args, stop, **kwargs):
        entered.set()
        stop.wait(timeout=3)
        stopped.set()

    probe = mocker.patch("dimos.diagnostics.session_network.monitor", side_effect=monitor)
    observer = SessionNetwork()
    try:
        observer.start(
            tmp_path,
            SessionSettings(
                run_id="clock-lap",
                network=NetworkSettings(
                    interface="en0", probe_url="http://robot/diagnostics/clock"
                ),
            ),
        )
        assert entered.wait(timeout=3)
        assert probe.call_args.kwargs["probe_url"] == "http://robot/diagnostics/clock"
    finally:
        observer.stop()
    assert stopped.is_set()
    assert not observer.thread.is_alive()
