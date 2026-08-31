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

from collections.abc import Callable
import hashlib
import json
import os
from pathlib import Path
import sqlite3
import time
from typing import Any

import pytest

from dimos.cloud import data as cd
from dimos.cloud.data import CloudData, DataApi, MultipartBackend
from dimos.core.global_config import global_config


class FakeTransport:
    """The /v1/data contract in memory: create/status/complete/download + part PUTs."""

    def __init__(self) -> None:
        self.uploads: dict[str, dict[str, Any]] = {}
        self.parts: dict[str, dict[int, bytes]] = {}
        self.part_size = 128 * 1024
        self.fail_at = 0  # fail the Nth put, once

    def request(self, method: str, path: str, body: dict[str, Any] | None = None) -> dict[str, Any]:
        if path.endswith("/uploads") and method == "POST":
            assert body
            for uid, u in self.uploads.items():
                if u["sha256"] == body["sha256"]:
                    return (
                        {"state": "complete", "upload_id": uid, "quota": {}}
                        if u["state"] == "complete"
                        else self._pending(uid)
                    )
            uid = f"u{len(self.uploads)}"
            self.uploads[uid], self.parts[uid] = dict(body, state="pending"), {}
            return self._pending(uid)
        if path.endswith("/quota"):
            return {"state": "ok"}
        if path.endswith("/uploads"):
            return {"uploads": [dict(id=k, **v) for k, v in self.uploads.items()]}
        uid = path.split("/")[4]
        u = self.uploads[uid]
        if path.endswith("/complete"):
            blob = b"".join(v for _, v in sorted(self.parts[uid].items()))
            assert len(blob) == u["size"]
            u.update(state="complete", blob=blob)
            return {"state": "complete", "quota": {"state": "ok"}}
        if path.endswith("/download"):
            return {
                "url": uid,
                "filename": u["filename"],
                "sha256": u["sha256"],
                "content_encoding": u["content_encoding"],
            }
        return {
            "state": u["state"],
            "parts": [{"part_number": n, "etag": "e"} for n in self.parts[uid]][::-1],
        }

    def _pending(self, uid: str) -> dict[str, Any]:
        n = -(-self.uploads[uid]["size"] // self.part_size)
        return {
            "state": "pending",
            "upload_id": uid,
            "part_size": self.part_size,
            "part_urls": [{"part_number": i, "url": f"{uid}/{i}"} for i in range(1, n + 1)],
            "quota": {"state": "ok"},
        }

    def put(self, url: str, body: bytes) -> None:
        uid, n = url.split("/")
        if self.fail_at and len(self.parts[uid]) + 1 == self.fail_at:
            self.fail_at = 0
            raise OSError("link dropped")
        self.parts[uid][int(n)] = body

    def download(self, url: str, dst: Path) -> None:
        dst.write_bytes(self.uploads[url]["blob"])


def recording(dir_: Path, age_s: float = 3600) -> Path:
    db = dir_ / "session_go2_1.db"
    with sqlite3.connect(db) as c:
        c.execute("CREATE TABLE _streams (name TEXT PRIMARY KEY, config TEXT NOT NULL)")
        c.execute(
            "INSERT INTO _streams VALUES ('/lidar', ?)", (json.dumps({"transport": "zenoh"}),)
        )
        c.execute("CREATE TABLE f (x BLOB)")
        c.execute("INSERT INTO f VALUES (?)", (os.urandom(400_000),))  # incompressible -> 3+ parts
    os.utime(db, (time.time() - age_s,) * 2)
    return db


@pytest.fixture
def env(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> tuple[CloudData, FakeTransport, Path]:
    t = FakeTransport()
    monkeypatch.setattr(cd, "RECORDINGS_DIR", tmp_path)
    monkeypatch.setattr(global_config, "dimos_upload_retries", 1)
    cloud = CloudData(MultipartBackend(DataApi(t), "lz4", None, retries=1))
    return cloud, t, recording(tmp_path)


def test_roundtrip_with_resume_and_manifest(env: tuple[CloudData, FakeTransport, Path]) -> None:
    cloud, t, db = env
    t.fail_at = 2  # one part fails, retry covers it
    r = cloud.upload(db, robot_id="go2")
    u = t.uploads[r["upload_id"]]
    assert r["state"] == "complete" and u["content_encoding"] == "lz4"
    assert u["manifest"]["streams"][0]["name"] == "/lidar"
    assert cloud.upload(db)["skipped"]  # idempotent by sha

    out = cloud.pull(r["upload_id"], dest=db.parent / "back.db")
    assert (
        hashlib.sha256(out.read_bytes()).hexdigest() == hashlib.sha256(db.read_bytes()).hexdigest()
    )


def test_resume_sends_only_missing_parts(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    t = FakeTransport()
    monkeypatch.setattr(cd, "RECORDINGS_DIR", tmp_path)
    cloud = CloudData(MultipartBackend(DataApi(t), "lz4", None, retries=0))
    db = recording(tmp_path)
    t.fail_at = 2
    with pytest.raises(RuntimeError, match="failed after"):
        cloud.upload(db)
    uid = next(iter(t.uploads))
    before = set(t.parts[uid])
    sent: list[int] = []
    real_put = t.put

    def spying_put(url: str, body: bytes) -> None:
        sent.append(int(url.split("/")[1]))
        real_put(url, body)

    monkeypatch.setattr(t, "put", spying_put)
    assert cloud.upload(db)["state"] == "complete"
    assert before and set(sent).isdisjoint(before)


@pytest.mark.parametrize(
    "bad,why",
    [("x.db-shm", "sidecar"), ("live.bin", "still being written"), ("missing.db", "missing.db")],
)
def test_upload_guards(env: tuple[CloudData, FakeTransport, Path], bad: str, why: str) -> None:
    cloud, _, db = env
    p = db.parent / bad
    if "missing" not in bad:
        p.write_bytes(b"x")  # fresh mtime == still being written
    with pytest.raises((RuntimeError, OSError), match=why):
        cloud.upload(p)


def test_kind_inferred_from_file(env: tuple[CloudData, FakeTransport, Path]) -> None:
    cloud, t, db = env
    blob = db.parent / "flight.bin"
    blob.write_bytes(os.urandom(1024))
    os.utime(blob, (time.time() - 3600,) * 2)
    cloud.upload(db)
    cloud.upload(blob)
    kinds = {u["filename"].split(".")[0]: u["kind"] for u in t.uploads.values()}
    assert kinds == {"session_go2_1": "recording", "flight": "blob"}
    assert set(cd.recordings()) == {db, blob}  # discovery is format-agnostic


@pytest.mark.parametrize("evil", ["../escaped.db.lz4", "/tmp/evil.db.lz4", "...lz4"])
def test_pull_confines_server_filename(
    env: tuple[CloudData, FakeTransport, Path], evil: str
) -> None:
    cloud, t, db = env
    r = cloud.upload(db)
    t.uploads[r["upload_id"]]["filename"] = evil
    if evil == "...lz4":
        with pytest.raises(RuntimeError, match="invalid filename"):
            cloud.pull(r["upload_id"])
    else:
        assert cloud.pull(r["upload_id"]).parent == db.parent


def test_failed_decode_keeps_existing_dest(env: tuple[CloudData, FakeTransport, Path]) -> None:
    cloud, t, db = env
    r = cloud.upload(db)
    dest = db.parent / "keep.db"
    dest.write_bytes(b"precious")
    t.uploads[r["upload_id"]].update(blob=b"garbage", sha256=hashlib.sha256(b"garbage").hexdigest())
    with pytest.raises(RuntimeError):
        cloud.pull(r["upload_id"], dest=dest)
    assert dest.read_bytes() == b"precious"


def test_recordings_discovery(
    env: tuple[CloudData, FakeTransport, Path], monkeypatch: pytest.MonkeyPatch
) -> None:
    _, _, db = env
    ghost = db.parent / "ghost.db"
    ghost.write_bytes(b"x")
    real = Path.stat
    monkeypatch.setattr(
        Path,
        "stat",
        lambda self, *a, **k: (_ for _ in ()).throw(FileNotFoundError(self))
        if self.name == "ghost.db"
        else real(self, *a, **k),
    )
    assert cd.recordings() == [db] and cd.recordings(10**9) == [db] and cd.recordings(1) == []


def test_not_logged_in(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setattr(cd, "api_key", lambda: None)
    monkeypatch.setattr(global_config, "dimos_api_key", None)
    with pytest.raises(RuntimeError, match="dimos login"):
        CloudData()


def test_pull_with_cross_filesystem_staging(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    t = FakeTransport()
    monkeypatch.setattr(cd, "RECORDINGS_DIR", tmp_path)
    staging = tmp_path / "otherfs"
    staging.mkdir()
    cloud = CloudData(MultipartBackend(DataApi(t), "lz4", staging, retries=0))
    db = recording(tmp_path)
    r = cloud.upload(db)

    real = Path.replace

    def exdev_from_staging(self: Path, target: Any) -> Any:
        if staging in self.parents:
            raise OSError(18, "Invalid cross-device link")
        return real(self, target)

    monkeypatch.setattr(Path, "replace", exdev_from_staging)
    dest = tmp_path / "back.db"
    dest.write_bytes(b"old")
    out = cloud.pull(r["upload_id"], dest=dest)
    assert (
        hashlib.sha256(out.read_bytes()).hexdigest() == hashlib.sha256(db.read_bytes()).hexdigest()
    )


@pytest.mark.parametrize("algo", ["lz4", "gzip", "xz"])
def test_codecs_interchangeable(tmp_path: Path, monkeypatch: pytest.MonkeyPatch, algo: str) -> None:
    t = FakeTransport()
    monkeypatch.setattr(cd, "RECORDINGS_DIR", tmp_path)
    cloud = CloudData(MultipartBackend(DataApi(t), algo, None, retries=0))
    db = recording(tmp_path)
    r = cloud.upload(db)
    assert t.uploads[r["upload_id"]]["content_encoding"] == algo
    out = cloud.pull(r["upload_id"], dest=tmp_path / f"back-{algo}.db")
    assert out.read_bytes() == db.read_bytes()


def test_progress_prefix_and_default_pull(env: tuple[CloudData, FakeTransport, Path]) -> None:
    cloud, t, db = env
    ticks: list[tuple[str, int, int]] = []
    r = cloud.upload(db, progress=lambda ph, d, tot: ticks.append((ph, d, tot)))
    assert (
        ticks[0][0] == "compress"
        and ticks[-1][:1] == ("upload",)
        and ticks[-1][1] == ticks[-1][2] > 0
    )
    uid = r["upload_id"]
    assert cloud.resolve(uid[:6]) == uid and cloud.resolve(None) == uid
    with pytest.raises(RuntimeError, match="no upload matching"):
        cloud.resolve("zz")
    assert cloud.pull(uid[:6], dest=db.parent / "p.db").read_bytes() == db.read_bytes()


def test_upload_refuses_when_staging_partition_is_full(
    env: tuple[CloudData, FakeTransport, Path], monkeypatch: pytest.MonkeyPatch
) -> None:
    cloud, _, db = env
    import shutil

    monkeypatch.setattr(shutil, "disk_usage", lambda p: shutil._ntuple_diskusage(10, 9, 1))  # type: ignore[attr-defined]
    with pytest.raises(RuntimeError, match="free, need"):
        cloud.upload(db)


def test_missing_staging_dir_is_created(tmp_path: Path) -> None:
    where = tmp_path / "scratch" / "deeper"
    b = MultipartBackend(DataApi(FakeTransport()), "lz4", where, retries=0)
    with b._staging(tmp_path):
        assert where.is_dir()


def test_recordings_discovery_recurses_run_dirs(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    monkeypatch.setattr(cd, "RECORDINGS_DIR", tmp_path)
    run = tmp_path / "20260828-181256-unitree-go2"
    run.mkdir()
    db = recording(run)  # recordings/<run>/session_go2_1.db
    (run / "memory.db-wal").write_bytes(b"x")  # sidecar, skipped
    stale = tmp_path / ".dimos-staging-abc"
    stale.mkdir()
    (stale / "memory.db.lz4").write_bytes(b"x")  # staging leftovers, skipped
    assert cd.recordings() == [db]


def test_transport_maps_http_errors(monkeypatch: pytest.MonkeyPatch) -> None:
    """The Aug 2026 outage surfaced as `POST /v1/data/uploads: 500` — pin that a 5xx
    names the endpoint and status, and a 401 says how to fix it."""
    from email.message import Message
    import io
    import urllib.error
    import urllib.request

    from dimos.cloud.cloud_transport import HttpCloudTransport

    t = HttpCloudTransport("https://api.test", "dimos_sk_x", timeout=1)

    def raise_http(code: int) -> Callable[..., Any]:
        def opener(req: Any, timeout: float) -> Any:
            raise urllib.error.HTTPError(req.full_url, code, "err", Message(), io.BytesIO(b"boom"))

        return opener

    monkeypatch.setattr(urllib.request, "urlopen", raise_http(500))
    with pytest.raises(RuntimeError, match=r"POST /v1/data/uploads: 500 boom"):
        t.request("POST", "/v1/data/uploads", {})
    monkeypatch.setattr(urllib.request, "urlopen", raise_http(401))
    with pytest.raises(RuntimeError, match="dimos login"):
        t.request("POST", "/v1/data/uploads", {})


def test_upload_cli_exits_nonzero_on_server_error(
    env: tuple[CloudData, FakeTransport, Path], monkeypatch: pytest.MonkeyPatch
) -> None:
    """`dimos data upload` must fail loudly (exit 1), not swallow a server fault."""
    import typer

    from dimos.cloud import cli

    cloud, t, db = env

    def failing_request(
        method: str, path: str, body: dict[str, Any] | None = None
    ) -> dict[str, Any]:
        raise RuntimeError("POST /v1/data/uploads: 503 storage unavailable")

    monkeypatch.setattr(t, "request", failing_request)
    monkeypatch.setattr(cli, "CloudData", lambda: cloud)
    with pytest.raises(typer.Exit) as e:
        cli.upload(db, None, None, None, None)
    assert e.value.exit_code == 1
