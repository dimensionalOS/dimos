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
    monkeypatch.setattr("dimos.cloud.data.kind_of", lambda p: "recording")
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
    return next(u["state"] for u in client.get("/dimos/uploads").json()["uploads"] if u["id"] == id)


def test_an_upload_goes_through_the_queue(
    client: TestClient, recording: Path, transfer: threading.Event, sent: list[Any]
) -> None:
    work(client)
    upload = client.post("/dimos/uploads", json={"path": str(recording)}).json()
    assert client.post("/dimos/uploads", json={"path": str(recording)}).json()["id"] == upload["id"]
    until(lambda: state_of(client, upload["id"]) == "uploading")
    transfer.set()
    until(lambda: state_of(client, upload["id"]) == "done")
    done = client.get("/dimos/uploads").json()["uploads"][0]
    assert done["uploadId"] == "cloud-1" and done["bytesDone"] == 100
    assert client.get(f"/dimos/uploads/uploaded?path={recording}").json()["uploadId"] == "cloud-1"
    assert any(key == "ns/dimos/events/upload" for key, _ in sent)
    assert client.delete("/dimos/uploads").json()["uploads"] == []


def test_a_running_upload_can_be_cancelled_and_retried(
    client: TestClient, recording: Path, transfer: threading.Event
) -> None:
    work(client)
    upload = client.post("/dimos/uploads", json={"path": str(recording)}).json()
    until(lambda: state_of(client, upload["id"]) == "uploading")
    assert client.post(f"/dimos/uploads/{upload['id']}/retry").status_code == 409
    assert client.delete(f"/dimos/uploads/{upload['id']}").json() == {"ok": True}
    until(lambda: state_of(client, upload["id"]) == "cancelled")
    transfer.set()
    assert client.post(f"/dimos/uploads/{upload['id']}/retry").json()["state"] == "queued"
    until(lambda: state_of(client, upload["id"]) == "done")
    assert client.delete("/dimos/uploads/u99").status_code == 404


def test_only_recordings_are_uploaded(client: TestClient, tmp_path: Path) -> None:
    assert client.post("/dimos/uploads", json={"path": "relative.mcap"}).status_code == 400
    assert (
        client.post("/dimos/uploads", json={"path": str(tmp_path / "gone.mcap")}).status_code == 400
    )


def test_without_a_login_the_queue_waits(
    client: TestClient, tmp_path: Path, monkeypatch: pytest.MonkeyPatch, transfer: threading.Event
) -> None:
    monkeypatch.setattr("dimos.cloud.data.kind_of", lambda p: "recording")
    path = tmp_path / "nologin.mcap"
    path.write_bytes(b"x")
    work(client)
    transfer.set()
    upload = client.post("/dimos/uploads", json={"path": str(path)}).json()
    until(lambda: client.get("/dimos/uploads").json()["waitingForLogin"])
    assert state_of(client, upload["id"]) == "queued"


def test_device_login(client: TestClient, monkeypatch: pytest.MonkeyPatch) -> None:
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

    monkeypatch.setattr(cloud, "_post", post)
    monkeypatch.setattr(cloud, "_store", stored.append)
    pending = client.post("/dimos/cloud/login").json()
    assert pending["state"] == "pending" and pending["code"] == "ABCD"
    approved.set()
    until(lambda: client.get("/dimos/cloud/login").json()["state"] == "approved")
    assert stored == ["key"]
    assert client.delete("/dimos/cloud/login").json()["state"] == "approved"


def test_account_is_cached(client: TestClient, monkeypatch: pytest.MonkeyPatch) -> None:
    calls: list[int] = []

    def account() -> dict[str, Any]:
        calls.append(1)
        return {
            "loggedIn": True,
            "email": "a@b.c",
            "scopes": [],
            "source": "stored",
            "cloudUrl": "x",
            "error": None,
        }

    monkeypatch.setattr(uploads, "account", account)
    assert client.get("/dimos/cloud/account").json()["email"] == "a@b.c"
    client.get("/dimos/cloud/account")
    client.get("/dimos/cloud/account?fresh")
    assert len(calls) == 2


def test_rate_gives_a_time_left_after_warming_up() -> None:
    rate = uploads.Rate()
    rate.tick(0.0, 0)
    rate.tick(0.5, 50)
    assert rate.eta(50, 0.5) is None
    rate.tick(2.0, 200)
    assert rate.bps() == pytest.approx(100, rel=0.2)
    assert rate.eta(800, 2.0) == pytest.approx(8, rel=0.3)
