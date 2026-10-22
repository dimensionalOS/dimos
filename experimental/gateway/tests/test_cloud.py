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
import threading
import time
from typing import Any

from fastapi.testclient import TestClient
import pytest

from dimos.cli import cloud
from experimental.gateway import uploads
from experimental.gateway.state import State


def until(check: Callable[[], bool], seconds: float = 10.0) -> None:
    deadline = time.monotonic() + seconds
    while not check():
        assert time.monotonic() < deadline, "timed out"
        time.sleep(0.02)


@pytest.fixture
def recording(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> Path:
    path = tmp_path / "run.mcap"
    path.write_bytes(b"x" * 100)
    monkeypatch.setattr("dimos.cloud.data.kind_of", lambda path: "recording")
    return path


@pytest.fixture
def transfer(state: State, monkeypatch: pytest.MonkeyPatch) -> threading.Event:
    release = threading.Event()

    def fake(path: str, robot_id: Any, kind: Any, tick: Callable[..., None]) -> dict[str, Any]:
        tick("upload", 0, 100)
        while not release.wait(0.02):
            tick("upload", 50, 100)
        if "nologin" in path:
            raise PermissionError("not logged in")
        tick("upload", 100, 100)
        return {"upload_id": "cloud-1", "skipped": False, "quota": {}}

    monkeypatch.setattr(state.uploads, "_transfer", fake)
    return release


def work(client: TestClient) -> None:
    state: State = client.app.state.gateway  # type: ignore[attr-defined]
    assert client.portal is not None
    client.portal.call(lambda: state.background.append(asyncio.create_task(state.uploads.work())))


def state_of(client: TestClient, id: str) -> str:
    return next(
        upload["state"]
        for upload in client.get("/cloud/uploads").json()["uploads"]
        if upload["id"] == id
    )


def test_an_upload_goes_through_the_queue(
    client: TestClient, recording: Path, transfer: threading.Event, sent: list[Any]
) -> None:
    work(client)
    upload = client.post("/cloud/uploads", json={"path": str(recording)}).json()
    assert client.post("/cloud/uploads", json={"path": str(recording)}).json()["id"] == upload["id"]
    until(lambda: client.get("/cloud/uploads").json()["uploads"][0]["bytesDone"] == 50)
    transfer.set()
    until(lambda: state_of(client, upload["id"]) == "done")
    done = client.get("/cloud/uploads").json()["uploads"][0]
    assert done["uploadId"] == "cloud-1" and done["bytesDone"] == 100
    assert client.get("/cloud/uploads").json()["byPath"][str(recording)]["uploadId"] == "cloud-1"
    assert any(key == "ns/dimos/events/upload" for key, _ in sent)
    assert client.app.state.gateway.uploads.rates == {}  # type: ignore[attr-defined]
    assert client.delete("/cloud/uploads").json()["uploads"] == []


def test_a_running_upload_can_be_cancelled_and_retried(
    client: TestClient, recording: Path, transfer: threading.Event
) -> None:
    work(client)
    upload = client.post("/cloud/uploads", json={"path": str(recording)}).json()
    until(lambda: state_of(client, upload["id"]) == "uploading")
    assert client.post("/cloud/uploads", json={"retry": upload["id"]}).status_code == 409
    assert client.delete(f"/cloud/uploads?id={upload['id']}").status_code == 200
    until(lambda: state_of(client, upload["id"]) == "cancelled")
    transfer.set()
    assert client.post("/cloud/uploads", json={"retry": upload["id"]}).json()["state"] == "queued"
    until(lambda: state_of(client, upload["id"]) == "done")
    assert client.delete("/cloud/uploads?id=u99").status_code == 404
    assert client.post("/cloud/uploads", json={}).status_code == 400


def test_a_recording_that_grows_during_its_upload_reads_as_changed(
    client: TestClient, recording: Path, transfer: threading.Event
) -> None:
    work(client)
    upload = client.post("/cloud/uploads", json={"path": str(recording)}).json()
    until(lambda: client.get("/cloud/uploads").json()["uploads"][0]["bytesDone"] == 50)
    with recording.open("ab") as file:
        file.write(b"y" * 10)
    transfer.set()
    until(lambda: state_of(client, upload["id"]) == "done")
    entry = client.get("/cloud/uploads").json()["byPath"][str(recording)]
    assert entry["size"] == 100 and entry["changed"] is True


def test_an_upload_cancelled_after_its_last_part_ends_cancelled(
    client: TestClient, recording: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    release_completion = threading.Event()

    def completes_without_progress(
        path: str, robot_id: Any, kind: Any, tick: Callable[..., None]
    ) -> dict[str, Any]:
        tick("upload", 100, 100)
        release_completion.wait(10)
        return {"upload_id": "cloud-1", "skipped": False, "quota": {}}

    state: State = client.app.state.gateway  # type: ignore[attr-defined]
    monkeypatch.setattr(state.uploads, "_transfer", completes_without_progress)
    work(client)
    upload = client.post("/cloud/uploads", json={"path": str(recording)}).json()
    until(lambda: client.get("/cloud/uploads").json()["uploads"][0]["phase"] == "finishing")
    assert client.delete(f"/cloud/uploads?id={upload['id']}").status_code == 200
    release_completion.set()
    until(lambda: state_of(client, upload["id"]) == "cancelled")
    listing = client.get("/cloud/uploads").json()
    assert str(recording) not in listing["byPath"]
    assert listing["uploads"][0]["uploadId"] == "cloud-1" and listing["uploads"][0]["notice"]


def test_only_recordings_are_uploaded(client: TestClient, tmp_path: Path) -> None:
    assert client.post("/cloud/uploads", json={"path": "relative.mcap"}).status_code == 400
    assert (
        client.post("/cloud/uploads", json={"path": str(tmp_path / "gone.mcap")}).status_code == 400
    )


def test_without_a_login_the_queue_waits(
    client: TestClient, recording: Path, transfer: threading.Event
) -> None:
    path = recording.rename(recording.with_name("nologin.mcap"))
    work(client)
    transfer.set()
    upload = client.post("/cloud/uploads", json={"path": str(path)}).json()
    until(lambda: client.get("/cloud/uploads").json()["waitingForLogin"])
    assert state_of(client, upload["id"]) == "queued"


def logged_in_account(logged_in: bool) -> dict[str, Any]:
    return {
        "loggedIn": logged_in,
        "email": "a@b.c" if logged_in else None,
        "scopes": [],
        "source": "stored" if logged_in else None,
        "cloudUrl": "x",
        "error": None,
    }


def test_device_login_then_logout(
    client: TestClient, monkeypatch: pytest.MonkeyPatch, sent: list[Any]
) -> None:
    approved = threading.Event()
    stored: list[str] = []

    def post(path: str, **params: Any) -> dict[str, Any]:
        if path == "/auth/device":
            return {
                "verification_uri": "https://console/device",
                "user_code": "ABCD",
                "expires_in": 600,
                "interval": 0.05,
                "device_code": "d",
            }
        return (
            {"status": "ok", "api_key": "key", "email": "a@b.c"}
            if approved.is_set()
            else {"status": "pending"}
        )

    # TODO(core): dimos.cli.cloud should expose device login / key storage publicly
    monkeypatch.setattr(cloud, "_post", post)
    monkeypatch.setattr(cloud, "_store", stored.append)
    monkeypatch.setattr(uploads, "account", lambda: logged_in_account(bool(stored)))
    pending = client.post("/cloud/login").json()
    assert pending["state"] == "pending" and pending["code"] == "ABCD"
    approved.set()
    until(lambda: client.get("/cloud/account").json()["login"]["state"] == "approved")
    assert stored == ["key"]
    forgot: list[int] = []

    def forget() -> None:
        forgot.append(1)
        stored.clear()

    monkeypatch.setattr(cloud, "_forget", forget)
    logged_out = client.delete("/cloud/login").json()
    assert logged_out["loggedIn"] is False and logged_out["login"]["state"] == "idle"
    assert forgot == [1]
    account_events = [
        payload["account"]["loggedIn"]
        for key, payload in sent
        if key == "ns/dimos/events/cloud-account"
    ]
    assert account_events == [True, False]


def test_deleting_a_login_cancels_it_when_pending_and_logs_out_otherwise(
    client: TestClient, monkeypatch: pytest.MonkeyPatch
) -> None:
    def post(path: str, **params: Any) -> dict[str, Any]:
        if path == "/auth/device":
            return {
                "verification_uri": "https://console/device",
                "user_code": "ABCD",
                "expires_in": 600,
                "interval": 0.05,
                "device_code": "d",
            }
        return {"status": "pending"}

    forgot: list[int] = []
    # TODO(core): dimos.cli.cloud should expose device login / key storage publicly
    monkeypatch.setattr(cloud, "_post", post)
    monkeypatch.setattr(cloud, "_forget", lambda: forgot.append(1))
    monkeypatch.setattr(uploads, "account", lambda: logged_in_account(False))
    assert client.post("/cloud/login").json()["state"] == "pending"
    assert client.delete("/cloud/login").json()["login"]["state"] == "idle"
    assert forgot == []
    assert client.delete("/cloud/login").json()["login"]["state"] == "idle"
    assert forgot == [1]


def test_a_login_cancelled_while_its_token_request_is_out_stores_nothing(
    client: TestClient, monkeypatch: pytest.MonkeyPatch
) -> None:
    token_requested, release_token = threading.Event(), threading.Event()
    stored: list[str] = []
    woken: list[int] = []

    def post(path: str, **params: Any) -> dict[str, Any]:
        if path == "/auth/device":
            return {
                "verification_uri": "https://console/device",
                "user_code": "ABCD",
                "expires_in": 600,
                "interval": 0.05,
                "device_code": "d",
            }
        token_requested.set()
        release_token.wait(10)
        return {"status": "ok", "api_key": "key", "email": "a@b.c"}

    # TODO(core): dimos.cli.cloud should expose device login / key storage publicly
    monkeypatch.setattr(cloud, "_post", post)
    monkeypatch.setattr(cloud, "_store", stored.append)
    monkeypatch.setattr(uploads, "account", lambda: logged_in_account(bool(stored)))
    state: State = client.app.state.gateway  # type: ignore[attr-defined]
    monkeypatch.setattr(state.uploads, "logged_in", lambda: woken.append(1))
    login_thread_done = threading.Event()
    device_login = state.uploads._device_login

    def device_login_then_signal(cancel: threading.Event) -> None:
        device_login(cancel)
        login_thread_done.set()

    monkeypatch.setattr(state.uploads, "_device_login", device_login_then_signal)
    assert client.post("/cloud/login").json()["state"] == "pending"
    assert token_requested.wait(10)
    assert client.delete("/cloud/login").json()["login"]["state"] == "idle"
    release_token.set()
    assert login_thread_done.wait(10)
    assert stored == []
    assert woken == []
    assert client.get("/cloud/account").json()["login"]["state"] == "idle"


def test_account_is_cached(client: TestClient, monkeypatch: pytest.MonkeyPatch) -> None:
    calls: list[int] = []

    def account() -> dict[str, Any]:
        calls.append(1)
        return logged_in_account(True)

    monkeypatch.setattr(uploads, "account", account)
    assert client.get("/cloud/account").json()["email"] == "a@b.c"
    client.get("/cloud/account")
    client.get("/cloud/account?fresh")
    assert len(calls) == 2


def test_rate_gives_a_time_left_after_warming_up() -> None:
    rate = uploads.Rate(since=0.0, at=0.0, done=0)
    rate.tick(0.5, 50)
    assert rate.eta(50) is None
    rate.tick(2.0, 200)
    assert rate.bps == pytest.approx(100, rel=0.2)
    assert rate.eta(800) == pytest.approx(8, rel=0.3)
