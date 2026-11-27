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

from collections.abc import Callable, Iterator
import contextlib
import errno
import hashlib
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
from multiprocessing.pool import ThreadPool
import os
from pathlib import Path
import shutil
import socket
import sqlite3
import tarfile
import tempfile
import threading
import time
from typing import Any, Literal

import pytest
import requests
import typer

from dimos.cloud import cli, data as cd
from dimos.cloud.cloud_request import PUT_BLOCK, HttpCloudRequest
from dimos.cloud.config import CloudConfig
from dimos.cloud.data import CloudData, DataApi, MultipartBackend
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.sensor_msgs.Joy import Joy


class FakeTransport:
    """The /v1/data contract in memory: create/status/complete/download + part PUTs."""

    def __init__(self) -> None:
        self.uploads: dict[str, dict[str, Any]] = {}
        self.parts: dict[str, dict[int, bytes]] = {}
        self.part_size = 128 * 1024
        self.fail_at = 0  # fail the Nth put, once
        self.epoch = 0  # presign generation; URLs from older epochs are expired
        self.expire_at = 0  # bump epoch before the Nth put, once
        self.previews: dict[str, dict[str, Any]] = {}
        self.put_lock = threading.Lock()

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
            self.uploads[uid], self.parts[uid] = (
                dict(body, state="pending", created_at="2026-08-30T12:00:00+00:00"),
                {},
            )
            return self._pending(uid)
        if path.endswith("/quota"):
            # The real /v1/data/quota always returns the full shape (quota_status
            # in dimos-cloud); a minimal stub here would hide contract breaks.
            return {
                "state": "ok",
                "pct": 0.0,
                "used_total": 0,
                "used_today": 0,
                "limits": {"total_gb": 250, "daily_gb": 25, "max_file_gb": 50, "trust": "new"},
                "message": "storage at 0% of quota",
            }
        if path.endswith("/preview") and method == "PUT":
            assert body
            self.previews[path.split("/")[4]] = body
            return {}
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
            "part_urls": [
                {"part_number": i, "url": f"{uid}/{i}?e={self.epoch}"} for i in range(1, n + 1)
            ],
            "quota": {"state": "ok"},
        }

    def put(self, url: str, body: bytes | memoryview) -> None:
        path, _, epoch = url.partition("?e=")
        uid, n = path.split("/")
        with self.put_lock:
            if self.expire_at and len(self.parts[uid]) + 1 == self.expire_at:
                self.expire_at = 0
                self.epoch += 1
            if int(epoch) != self.epoch:
                raise OSError("403 Forbidden: Request has expired")
            if self.fail_at and len(self.parts[uid]) + 1 == self.fail_at:
                self.fail_at = 0
                raise OSError("link dropped")
            self.parts[uid][int(n)] = bytes(body)

    def download(
        self, url: str, dst: Path, progress: Callable[[int, int], None] | None = None
    ) -> None:
        blob = self.uploads[url]["blob"]
        dst.write_bytes(blob)
        if progress:
            progress(len(blob) // 2, len(blob))
            progress(len(blob), len(blob))


def one_part_per_worker(cloud: CloudData, t: FakeTransport, dir_: Path) -> Path:
    raw = dir_ / "parts.lz4"  # carries the codec suffix, so it uploads as-is
    raw.write_bytes(os.urandom(cloud.backend.parallel_parts * t.part_size))
    return raw


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
    monkeypatch.setattr(cd, "DOWNLOADS_DIR", tmp_path)
    monkeypatch.setattr(cd.time, "sleep", lambda s: None)
    monkeypatch.setattr(cd, "BYTES_PER_CONNECTION", t.part_size)  # a connection per part
    cloud = CloudData(MultipartBackend(DataApi(t), "lz4", None, retries=1, parallel_parts=4))
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


@pytest.mark.parametrize("mode", ["default", "configured", "in_tree", "symlink_root"])
def test_directory_upload(
    env: tuple[CloudData, FakeTransport, Path], monkeypatch: pytest.MonkeyPatch, mode: str
) -> None:
    cloud, transport, db = env
    tree = db.parent / "dataset"
    scratch = tree / "tmp"
    scratch.mkdir(parents=True)
    (tree / ".hidden").write_bytes(b"data")
    (tree / "link").symlink_to("tmp", target_is_directory=True)
    source = tree
    if mode == "symlink_root":
        source = db.parent / "alias"
        source.symlink_to(tree, target_is_directory=True)
    staging = db.parent / "staging" if mode == "configured" else db.parent
    if mode in ("configured", "in_tree"):
        cloud.backend.staging_dir = staging if mode == "configured" else scratch
    if mode == "in_tree":
        monkeypatch.setattr(tempfile, "tempdir", str(scratch))
    mtimes = [p.stat().st_mtime_ns for p in (tree, scratch)]
    tar_paths: list[Path] = []
    real_open = tarfile.open

    def track_tar(name: Path, mode: Literal["w", "r:"]) -> tarfile.TarFile:
        if mode == "w":
            tar_paths.append(name)
            return real_open(name, "w")
        return real_open(name, "r:")

    monkeypatch.setattr(tarfile, "open", track_tar)
    result = cloud.upload(source, robot_id="go2", kind="dataset", chunk_mb=1)
    uid = result["upload_id"]
    uploaded = transport.uploads[uid]
    assert uploaded["robot_id"] == "go2"
    assert uploaded["kind"] == "dataset"
    assert uploaded["part_size"] == 2**20
    name = source.name
    assert uploaded["filename"] == f"{name}.tar.lz4"
    assert uploaded["content_encoding"] == "lz4"
    assert uploaded["blob"].startswith(b"\x04\x22\x4d\x18")  # LZ4 frame magic
    if mode != "in_tree":  # Pull still uses the configured staging directory.
        pulled = cloud.pull(uid)
        assert pulled.name.endswith(f"{name}.tar")
        with tarfile.open(pulled, "r:") as tar:
            assert tar.getnames() == [name, f"{name}/.hidden", f"{name}/link", f"{name}/tmp"]
            link = tar.getmember(f"{name}/link")
            assert link.issym() and link.linkname == "tmp"
            hidden = tar.extractfile(f"{name}/.hidden")
            assert hidden is not None and hidden.read() == b"data"
    retry = cloud.upload(source)
    assert retry["skipped"] and retry["upload_id"] == uid  # FakeTransport dedupes by SHA-256.
    assert [p.stat().st_mtime_ns for p in (tree, scratch)] == mtimes
    assert all(p.parent.parent == staging and not p.parent.exists() for p in tar_paths)
    assert len(tar_paths) == 2 and not list(scratch.iterdir())


def test_directory_upload_cleans_up_after_tar_write_failure(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    tree = tmp_path / "tree"
    tree.mkdir()
    (tree / "file").write_bytes(b"data")
    staging = tmp_path / "staging"
    transport = FakeTransport()
    cloud = CloudData(
        MultipartBackend(DataApi(transport), "", staging, retries=0, parallel_parts=4)
    )

    def disk_usage(path: Path) -> Any:
        assert path.is_relative_to(staging)
        return shutil._ntuple_diskusage(10, 6, 4)  # type: ignore[attr-defined]

    def fail_add(tar: tarfile.TarFile, *args: Any, **kwargs: Any) -> None:
        assert tar.fileobj is not None
        tar.fileobj.write(b"partial tar")
        raise OSError(errno.ENOSPC, "No space left on device")

    monkeypatch.setattr(shutil, "disk_usage", disk_usage)
    monkeypatch.setattr(tarfile.TarFile, "add", fail_add)
    with pytest.raises(OSError, match="No space left on device"):
        cloud.upload(tree)
    assert not transport.uploads and not list(staging.iterdir())


def test_resume_sends_only_missing_parts(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    t = FakeTransport()
    monkeypatch.setattr(cd, "RECORDINGS_DIR", tmp_path)
    cloud = CloudData(MultipartBackend(DataApi(t), "lz4", None, retries=0, parallel_parts=1))
    db = recording(tmp_path)
    t.fail_at = 2
    with pytest.raises(RuntimeError, match="failed after"):
        cloud.upload(db)
    uid = next(iter(t.uploads))
    before = set(t.parts[uid])
    sent: list[int] = []
    real_put = t.put

    def spying_put(url: str, body: bytes | memoryview) -> None:
        sent.append(int(url.split("/")[1].partition("?")[0]))
        real_put(url, body)

    monkeypatch.setattr(t, "put", spying_put)
    assert cloud.upload(db)["state"] == "complete"
    assert before and set(sent).isdisjoint(before)


def test_parallel_upload_sends_each_part_once(
    env: tuple[CloudData, FakeTransport, Path], monkeypatch: pytest.MonkeyPatch
) -> None:
    cloud, t, db = env
    sent: list[int] = []
    completed: list[int] = []
    real_put, real_request = t.put, t.request

    def spying_put(url: str, body: bytes | memoryview) -> None:
        sent.append(int(url.split("/")[1].partition("?")[0]))
        real_put(url, body)

    def spying_request(
        method: str, path: str, body: dict[str, Any] | None = None
    ) -> dict[str, Any]:
        if path.endswith("/complete"):
            assert body
            completed.extend(p["part_number"] for p in body["parts"])
        return real_request(method, path, body)

    monkeypatch.setattr(t, "put", spying_put)
    monkeypatch.setattr(t, "request", spying_request)
    r = cloud.upload(db)
    assert len(sent) > 1 and sorted(sent) == completed == list(range(1, len(sent) + 1))
    out = cloud.pull(r["upload_id"], dest=db.parent / "back.db")
    assert out.read_bytes() == db.read_bytes()


def test_upload_refreshes_expired_part_urls(
    env: tuple[CloudData, FakeTransport, Path], monkeypatch: pytest.MonkeyPatch
) -> None:
    """An upload slower than the presign TTL: URLs from create die mid-run. The
    client must re-create (same sha256 -> fresh URLs) and finish, not abort.
    The parts in flight hit the expiry together and share one re-create."""
    cloud, t, db = env
    cloud.backend.retries = 0
    raw = one_part_per_worker(cloud, t, db.parent)
    t.expire_at = 1  # URLs signed at create expire before any part lands
    in_flight = threading.Barrier(cloud.backend.parallel_parts)
    creates: list[str] = []
    real_put, real_request = t.put, t.request

    def put_together(url: str, body: bytes | memoryview) -> None:
        if url.endswith("?e=0"):
            in_flight.wait(timeout=5)
        real_put(url, body)

    def counting_request(
        method: str, path: str, body: dict[str, Any] | None = None
    ) -> dict[str, Any]:
        if method == "POST" and path.endswith("/uploads"):
            creates.append(path)
        return real_request(method, path, body)

    monkeypatch.setattr(t, "put", put_together)
    monkeypatch.setattr(t, "request", counting_request)
    r = cloud.upload(raw)
    assert r["state"] == "complete" and not r["skipped"]
    assert t.epoch == 1  # completed on re-signed URLs, not the originals
    assert len(creates) == 2  # the create, then one re-sign shared by every part


def test_resign_finding_the_upload_complete_reports_it_skipped(
    env: tuple[CloudData, FakeTransport, Path], monkeypatch: pytest.MonkeyPatch
) -> None:
    """The URLs expire and the re-create answers that the upload is already complete,
    as when another run finished it."""
    cloud, t, db = env
    cloud.backend.retries = 0
    t.expire_at = 1
    real_request = t.request

    def completing_request(
        method: str, path: str, body: dict[str, Any] | None = None
    ) -> dict[str, Any]:
        if method == "POST" and path.endswith("/uploads"):
            for u in t.uploads.values():
                u["state"] = "complete"
        return real_request(method, path, body)

    monkeypatch.setattr(t, "request", completing_request)
    r = cloud.upload(db)
    assert r["skipped"] and r["state"] == "complete"


def test_failed_part_abandons_the_parts_in_flight(
    env: tuple[CloudData, FakeTransport, Path], monkeypatch: pytest.MonkeyPatch
) -> None:
    """Part 1 fails for good while the others are mid-send. The upload raises without
    waiting for them, and when their link drops too they do not try again."""
    cloud, t, db = env
    raw = one_part_per_worker(cloud, t, db.parent)
    in_flight = threading.Barrier(cloud.backend.parallel_parts)
    release = threading.Event()
    attempts: list[str] = []
    waited_out: list[str] = []

    def put(url: str, body: bytes | memoryview) -> None:
        attempts.append(url)
        if attempts.count(url) == 1:
            in_flight.wait(timeout=5)
        if "/1?" not in url and not release.wait(timeout=5):
            waited_out.append(url)
        raise OSError("link dropped")

    monkeypatch.setattr(t, "put", put)
    before = set(threading.enumerate())
    with pytest.raises(RuntimeError, match="part 1 failed after 2 attempts: link dropped"):
        cloud.upload(raw)
    release.set()
    for worker in set(threading.enumerate()) - before:
        worker.join(timeout=5)
    assert not waited_out
    in_flight_urls = {url for url in attempts if "/1?" not in url}
    assert len(in_flight_urls) == cloud.backend.parallel_parts - 1
    assert all(attempts.count(url) == 1 for url in in_flight_urls)


def test_empty_file_uploads_with_no_parts(env: tuple[CloudData, FakeTransport, Path]) -> None:
    cloud, t, db = env
    empty = db.parent / "empty.lz4"  # carries the codec suffix, so it uploads as-is
    empty.write_bytes(b"")
    r = cloud.upload(empty)
    assert r["state"] == "complete" and t.uploads[r["upload_id"]]["blob"] == b""


def test_upload_opens_a_connection_per_share_of_the_file(
    env: tuple[CloudData, FakeTransport, Path], monkeypatch: pytest.MonkeyPatch
) -> None:
    """Four parts that fill two connections between them go out over two."""
    cloud, t, db = env
    raw = one_part_per_worker(cloud, t, db.parent)
    pools: list[int] = []
    monkeypatch.setattr(cd, "BYTES_PER_CONNECTION", 2 * t.part_size)
    monkeypatch.setattr(cd, "ThreadPool", lambda n: pools.append(n) or ThreadPool(n))
    assert cloud.upload(raw)["state"] == "complete"
    assert pools == [2]


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
        cloud.upload(p, skip_recent=True)


def test_explicit_path_uploads_fresh_file(env: tuple[CloudData, FakeTransport, Path]) -> None:
    cloud, _, db = env
    assert cloud.upload(db)["state"] == "complete"


def test_blueprint_recorded_from_run_dir(env: tuple[CloudData, FakeTransport, Path]) -> None:
    cloud, t, db = env
    run = db.parent / "20260830-155733-unitree-go2"
    run.mkdir()
    r = cloud.upload(recording(run))
    assert t.uploads[r["upload_id"]]["manifest"]["blueprint"] == "unitree-go2"
    plain = cloud.upload(db)
    assert "blueprint" not in t.uploads[plain["upload_id"]]["manifest"]


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
    monkeypatch.setattr(cd, "api_key", lambda config: None)
    with pytest.raises(RuntimeError, match="dimos login"):
        CloudData(config=CloudConfig(api_key=None))


def test_pull_with_cross_filesystem_staging(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    t = FakeTransport()
    monkeypatch.setattr(cd, "RECORDINGS_DIR", tmp_path)
    staging = tmp_path / "otherfs"
    staging.mkdir()
    cloud = CloudData(MultipartBackend(DataApi(t), "lz4", staging, retries=0, parallel_parts=4))
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


@pytest.mark.parametrize("algo", ["lz4", "gzip", "bz2", "xz"])
def test_codecs_interchangeable(tmp_path: Path, monkeypatch: pytest.MonkeyPatch, algo: str) -> None:
    t = FakeTransport()
    monkeypatch.setattr(cd, "RECORDINGS_DIR", tmp_path)
    cloud = CloudData(MultipartBackend(DataApi(t), algo, None, retries=0, parallel_parts=4))
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
    assert cloud.resolve(uid[:6])["id"] == uid == cloud.resolve(None)["id"]
    with pytest.raises(RuntimeError, match="no upload matching"):
        cloud.resolve("zz")
    pulls: list[tuple[str, int, int]] = []
    got = cloud.pull(uid[:6], dest=db.parent / "p.db", progress=lambda *a: pulls.append(a))
    assert got.read_bytes() == db.read_bytes()
    phases = [p[0] for p in pulls]
    assert phases[0] == "download" and "verify" in phases
    dl = [p for p in pulls if p[0] == "download" and p[2]]
    assert dl[-1][1] == dl[-1][2] > 0
    out = cloud.pull(None)
    assert out.name == f"20260830-120000-{uid}-{db.name}"
    assert out.read_bytes() == db.read_bytes()


def test_upload_refuses_when_staging_partition_is_full(
    env: tuple[CloudData, FakeTransport, Path], monkeypatch: pytest.MonkeyPatch
) -> None:
    cloud, _, db = env

    monkeypatch.setattr(shutil, "disk_usage", lambda p: shutil._ntuple_diskusage(10, 9, 1))  # type: ignore[attr-defined]
    with pytest.raises(RuntimeError, match="free, need"):
        cloud.upload(db)


def test_pull_refuses_when_disk_is_full(
    env: tuple[CloudData, FakeTransport, Path], monkeypatch: pytest.MonkeyPatch
) -> None:
    cloud, _, db = env
    uid = cloud.upload(db)["upload_id"]

    monkeypatch.setattr(shutil, "disk_usage", lambda p: shutil._ntuple_diskusage(10, 9, 1))  # type: ignore[attr-defined]
    with pytest.raises(RuntimeError, match="free, need"):
        cloud.pull(uid)


def test_missing_staging_dir_is_created(tmp_path: Path) -> None:
    where = tmp_path / "scratch" / "deeper"
    b = MultipartBackend(DataApi(FakeTransport()), "lz4", where, retries=0, parallel_parts=4)
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

    t = HttpCloudRequest("https://api.test", "dimos_sk_x", timeout=1)

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


class PartStore(BaseHTTPRequestHandler):
    """Takes part PUTs over HTTP/1.1, which keeps the connection open between them."""

    protocol_version = "HTTP/1.1"

    def do_PUT(self) -> None:
        body = self.rfile.read(int(self.headers["Content-Length"]))
        self.server.puts.append(  # type: ignore[attr-defined]
            (self.client_address, self.headers["Transfer-Encoding"], body)
        )
        reply = b""
        if "stalled" in self.path:
            reply = b"<Error><Code>RequestTimeout</Code><Message>Idle connections will be closed.</Message></Error>"
        self.send_response(403 if "expired" in self.path else 400 if reply else 200)
        self.send_header("Content-Length", str(len(reply)))
        self.end_headers()
        self.wfile.write(reply)

    def log_message(self, format: str, *args: Any) -> None:
        pass


class PartStoreServer(ThreadingHTTPServer):
    def __init__(self) -> None:
        super().__init__(("127.0.0.1", 0), PartStore)
        self.puts: list[tuple[Any, str | None, bytes]] = []
        self.connections: list[socket.socket] = []

    def get_request(self) -> tuple[socket.socket, Any]:
        connection, address = super().get_request()
        self.connections.append(connection)
        return connection, address


@pytest.fixture
def part_store() -> Iterator[PartStoreServer]:
    server = PartStoreServer()
    threading.Thread(target=server.serve_forever, daemon=True).start()
    yield server
    server.shutdown()
    server.server_close()
    for connection in server.connections:  # ends the threads serving open connections
        with contextlib.suppress(OSError):
            connection.shutdown(socket.SHUT_RDWR)


def test_put_sends_blocks_over_one_connection(
    part_store: PartStoreServer, monkeypatch: pytest.MonkeyPatch
) -> None:
    """The timeout bounds each socket send, so a part sent whole is cut off after that
    long even while data is moving. A thread's parts all go over one connection."""
    sent: list[list[int]] = []
    real_send = requests.Session.send

    def spying_send(
        session: requests.Session, request: requests.PreparedRequest, **kwargs: Any
    ) -> requests.Response:
        sent.append([len(block) for block in request.body])  # type: ignore[union-attr]
        return real_send(session, request, **kwargs)

    monkeypatch.setattr(requests.Session, "send", spying_send)
    t = HttpCloudRequest("https://api.test", "dimos_sk_x", timeout=5)
    url = f"http://127.0.0.1:{part_store.server_address[1]}/part"
    body = os.urandom(2 * PUT_BLOCK + 123)
    t.put(url, memoryview(body))
    t.put(url, memoryview(body))
    assert sent == [[PUT_BLOCK, PUT_BLOCK, 123]] * 2
    (first, encoding, received), (second, _, _) = part_store.puts
    assert first == second and encoding is None and received == body


def test_put_errors_leave_out_the_signed_url(part_store: PartStoreServer) -> None:
    """A 403 in the message triggers a re-sign, and a signature can hold those digits."""
    t = HttpCloudRequest("https://api.test", "dimos_sk_x", timeout=5)
    signed = "X-Amz-Signature=a403b"
    with pytest.raises(OSError, match="HTTP Error 403: Forbidden") as expired:
        t.put(f"http://127.0.0.1:{part_store.server_address[1]}/expired?{signed}", b"x")
    assert signed not in str(expired.value)
    with socket.socket() as unused:
        unused.bind(("127.0.0.1", 0))
        closed_port = unused.getsockname()[1]
    with pytest.raises(OSError, match="Connection refused") as refused:
        t.put(f"http://127.0.0.1:{closed_port}/part?{signed}", b"x")
    assert "403" not in str(refused.value)


def test_put_errors_keep_the_s3_error_code(part_store: PartStoreServer) -> None:
    """S3 gives the reason a part failed only in its XML error body."""
    t = HttpCloudRequest("https://api.test", "dimos_sk_x", timeout=5)
    with pytest.raises(
        OSError,
        match=r"^HTTP Error 400: Bad Request \(RequestTimeout: Idle connections will be closed\.\)$",
    ):
        t.put(f"http://127.0.0.1:{part_store.server_address[1]}/stalled", b"x")


@pytest.mark.parametrize(
    "api_url,console_url,skipped",
    [
        ("https://api.dimensional.org", "https://console.dimensional.org", False),
        ("https://api.dimensional.org/", "https://console.dimensional.org", True),
        ("https://api.staging.dimensional.org", "https://console.staging.dimensional.org", False),
        ("https://api.staging.dimensional.org/", "https://console.staging.dimensional.org", True),
    ],
)
def test_upload_cli_prints_console_link(
    env: tuple[CloudData, FakeTransport, Path],
    monkeypatch: pytest.MonkeyPatch,
    capsys: pytest.CaptureFixture[str],
    skipped: bool,
    api_url: str,
    console_url: str,
) -> None:
    cloud, _, db = env
    monkeypatch.setattr(cli, "CloudData", lambda: cloud)
    monkeypatch.setattr(cloud.config, "url", api_url)
    if skipped:
        cloud.upload(db)

    cli.upload(db, None, None, None, None)

    captured = capsys.readouterr()
    note = "already uploaded" if skipped else "complete"
    assert f"{db.name}: {note} (u0)\n" in captured.out
    assert "  console preview: failed:" in captured.out  # fixture's invalid stream name
    assert f"  console: {console_url}/console/data?open=u0\n" in captured.out
    assert captured.err == ""


@pytest.mark.parametrize(
    "api_url",
    [
        "http://localhost:8000",
        "https://cloud.example/",
        "https://api.dimensional.org.evil.example",
        "http://api.dimensional.org",
        "https://api.dimensional.org/v1",
        "https://api.dimensional.org//",
    ],
)
def test_upload_cli_omits_console_link_for_unknown_api(
    env: tuple[CloudData, FakeTransport, Path],
    monkeypatch: pytest.MonkeyPatch,
    capsys: pytest.CaptureFixture[str],
    api_url: str,
) -> None:
    cloud, _, db = env
    monkeypatch.setattr(cli, "CloudData", lambda: cloud)
    monkeypatch.setattr(cloud.config, "url", api_url)

    cli.upload(db, None, None, None, None)

    captured = capsys.readouterr()
    assert f"{db.name}: complete (u0)\n" in captured.out
    assert "  console:" not in captured.out
    assert captured.err == ""


def test_upload_cli_keeps_quota_warning_without_preview(
    env: tuple[CloudData, FakeTransport, Path],
    monkeypatch: pytest.MonkeyPatch,
    capsys: pytest.CaptureFixture[str],
) -> None:
    cloud, t, db = env
    blob = db.with_suffix(".bin")
    blob.write_bytes(b"raw artifact")
    request = t.request

    def warning_request(
        method: str, path: str, body: dict[str, Any] | None = None
    ) -> dict[str, Any]:
        result = request(method, path, body)
        if path.endswith("/complete"):
            result["quota"] = {"state": "warn", "message": "storage at 90% of quota"}
        return result

    monkeypatch.setattr(t, "request", warning_request)
    monkeypatch.setattr(cli, "CloudData", lambda: cloud)
    monkeypatch.setattr(cloud.config, "url", "https://api.dimensional.org")

    cli.upload(blob, None, None, None, None)

    captured = capsys.readouterr()
    assert captured.out.strip().splitlines() == [
        f"{blob.name}: complete (u0)",
        "  console: https://console.dimensional.org/console/data?open=u0",
    ]
    assert captured.err == "storage at 90% of quota\n"


@pytest.mark.parametrize("endpoint", ["/uploads", "/uploads/u0/complete"])
def test_upload_cli_exits_nonzero_on_server_error(
    env: tuple[CloudData, FakeTransport, Path],
    monkeypatch: pytest.MonkeyPatch,
    capsys: pytest.CaptureFixture[str],
    endpoint: str,
) -> None:
    """`dimos data upload` must fail loudly (exit 1), not swallow a server fault."""
    cloud, t, db = env
    request = t.request

    def failing_request(
        method: str, path: str, body: dict[str, Any] | None = None
    ) -> dict[str, Any]:
        if path == f"/v1/data{endpoint}":
            raise RuntimeError(f"POST {path}: 503 storage unavailable")
        return request(method, path, body)

    monkeypatch.setattr(t, "request", failing_request)
    monkeypatch.setattr(cli, "CloudData", lambda: cloud)
    monkeypatch.setattr(cloud.config, "url", "https://api.dimensional.org")
    with pytest.raises(typer.Exit) as e:
        cli.upload(db, None, None, None, None)
    assert e.value.exit_code == 1
    captured = capsys.readouterr()
    assert captured.out.strip() == ""
    assert f"{db.name}: POST /v1/data{endpoint}: 503 storage unavailable\n" in captured.err


def test_matching_suffix_uploads_raw_and_unstamped(
    env: tuple[CloudData, FakeTransport, Path],
) -> None:
    """A file already named *.lz4 is sent as-is: no encoding stamp, byte-identical pull."""
    cloud, t, db = env
    raw = db.parent / "artifact.lz4"  # arbitrary bytes, NOT lz4-compressed
    raw.write_bytes(b"not actually lz4" * 1024)
    r = cloud.upload(raw)
    assert t.uploads[r["upload_id"]]["content_encoding"] is None
    out = cloud.pull(r["upload_id"], dest=db.parent / "artifact.back.lz4")
    assert out.read_bytes() == raw.read_bytes()


def test_mcap_recording_sends_preview_with_joystick(
    env: tuple[CloudData, FakeTransport, Path],
) -> None:
    writer_mod = pytest.importorskip("mcap.writer")
    cloud, t, db = env
    path = db.parent / "teleop.mcap"
    with path.open("wb") as f:
        w = writer_mod.Writer(f)
        w.start(profile="dimos", library="test")
        chans = {
            name: w.register_channel(
                topic=name,
                message_encoding="lcm",
                schema_id=0,
                metadata={
                    "dimos.payload_type": payload,
                    "dimos.observation_time": "publish_time",
                },
            )
            for name, payload in (
                ("odom", "dimos.msgs.geometry_msgs.PoseStamped.PoseStamped"),
                ("joy", "dimos.msgs.sensor_msgs.Joy.Joy"),
            )
        }
        for i in range(3):
            ns = (100 + i) * 10**9
            pose = PoseStamped(ts=100.0 + i, position=[i, 0, 0.3], frame_id="world")
            joy = Joy(ts=100.0 + i, axes=[0.0, 0.5, -1.0, 0.0], buttons=[1, 0, 0])
            for name, msg in (("odom", pose), ("joy", joy)):
                w.add_message(
                    channel_id=chans[name], log_time=ns, publish_time=ns, data=msg.lcm_encode()
                )
        w.finish()

    r = cloud.upload(path, robot_id="go2")
    assert r["preview"].startswith("sent")
    doc = t.previews[r["upload_id"]]
    assert doc["streams"]["joystick"]["count"] == 3
    assert [s[1:] for s in doc["joy"]] == [[[0.0, 0.5, -1.0, 0.0], [1, 0, 0]]] * 3


def test_mcap_upload_sends_its_preview(env: tuple[CloudData, FakeTransport, Path]) -> None:
    from dimos.memory.store.test_mcap import ros2_odom, write_ros2_mcap

    cloud, _t, db = env
    mcap = db.parent / "run.mcap"
    write_ros2_mcap(mcap, [("/odometry", "nav_msgs/Odometry", 1.0, ros2_odom(1.0, 0, 0))])
    before = mcap.read_bytes()
    assert cloud.upload(mcap)["preview"] == "sent"  # opened read-only in place, not as SQLite
    assert mcap.read_bytes() == before


def test_an_mcap_recording_gets_a_manifest_so_it_can_be_previewed(tmp_path: Path) -> None:
    """No manifest means no console preview, and the manifest used to be read by opening the
    recording as sqlite — which every .mcap fails, whatever the preview could have made of it."""
    writer = pytest.importorskip("mcap.writer")
    path = tmp_path / "run.mcap"
    with path.open("wb") as out:
        w = writer.Writer(out)
        w.start(profile="dimos", library="test")
        w.register_channel(
            topic="dimos/lidar/sensor_msgs.PointCloud2", message_encoding="lcm", schema_id=0
        )
        w.finish()

    manifest = cd._manifest(path)
    assert manifest is not None
    assert [s["name"] for s in manifest["streams"]] == ["lidar"]
    assert manifest["streams"][0]["payload_module"].endswith("PointCloud2")


def test_a_file_that_is_no_recording_has_no_manifest(tmp_path: Path) -> None:
    junk = tmp_path / "notes.txt"
    junk.write_text("not a recording")
    assert cd._manifest(junk) is None
