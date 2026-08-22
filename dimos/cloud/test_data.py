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

import hashlib
import json
from pathlib import Path
import sqlite3
import tempfile
import time
from typing import Any
import urllib.error
import urllib.request

import lz4.frame
import pytest

from dimos.cloud import data as cd
from dimos.cloud.data import CloudData, CloudDataConfig


def _recording(tmp_path: Path, age_s: float = 3600) -> Path:
    db = tmp_path / "session_go2_20260819_120000.db"
    with sqlite3.connect(db) as c:
        c.execute("CREATE TABLE _streams (name TEXT PRIMARY KEY, config TEXT NOT NULL)")
        c.execute(
            "INSERT INTO _streams VALUES (?, ?)",
            ("/lidar", json.dumps({"class": "dimos.memory.Stream"})),
        )
        c.execute("CREATE TABLE payload (x BLOB)")
        import random

        rng = random.Random(7)
        c.execute("INSERT INTO payload VALUES (?)", (rng.randbytes(400_000),))
    old = time.time() - age_s
    import os

    os.utime(db, (old, old))
    return db


class FakeServer:
    """The /v1/data contract, in-memory; presigned PUTs land in self.parts."""

    def __init__(self) -> None:
        self.uploads: dict[str, dict[str, Any]] = {}
        self.parts: dict[str, dict[int, bytes]] = {}
        self.part_size = 128 * 1024

    def req(self, method: str, path: str, body: dict[str, Any] | None) -> dict[str, Any]:
        if path == "/v1/data/uploads" and method == "POST":
            assert body is not None
            for uid, u in self.uploads.items():
                if u["sha256"] == body["sha256"]:
                    if u["state"] == "complete":
                        return {"state": "complete", "upload_id": uid, "quota": self._q()}
                    return self._pending(uid)
            uid = f"u{len(self.uploads)}"
            self.uploads[uid] = dict(body, state="pending")
            self.parts[uid] = {}
            return self._pending(uid)
        uid = path.split("/")[4]
        u = self.uploads[uid]
        if path.endswith("/complete"):
            got = b"".join(v for _, v in sorted(self.parts[uid].items()))
            assert len(got) == u["size"], "size mismatch"
            u.update(state="complete", blob=got)
            return {"state": "complete", "quota": self._q()}
        if path.endswith("/download"):
            return {
                "url": f"fake://{uid}",
                "filename": u["filename"],
                "sha256": u["sha256"],
                "content_encoding": u["content_encoding"],
            }
        return {
            "state": u["state"],
            "parts": [{"part_number": n, "etag": "e"} for n in self.parts[uid]],
        }

    def _pending(self, uid: str) -> dict[str, Any]:
        u = self.uploads[uid]
        n = max(1, -(-u["size"] // self.part_size))
        return {
            "state": "pending",
            "upload_id": uid,
            "part_size": self.part_size,
            "part_urls": [{"part_number": i, "url": f"fake://{uid}/{i}"} for i in range(1, n + 1)],
            "quota": self._q(),
        }

    def _q(self) -> dict[str, Any]:
        return {
            "state": "ok",
            "pct": 1.0,
            "message": "",
            "used_total": 0,
            "limits": {"total_gb": 250, "daily_gb": 25},
        }


@pytest.fixture
def cloud(monkeypatch: pytest.MonkeyPatch, tmp_path: Path) -> tuple[CloudData, FakeServer]:
    srv = FakeServer()
    c = CloudData(CloudDataConfig(api_key="dimos_sk_test", base_url="https://fake"))
    monkeypatch.setattr(c, "_req", lambda m, p, b=None: srv.req(m, p, b))
    monkeypatch.setattr(
        cd,
        "_put",
        lambda url, body: srv.parts[url.split("/")[2]].__setitem__(int(url.split("/")[3]), body),
    )
    monkeypatch.setattr(
        cd, "_download", lambda url, dst: dst.write_bytes(srv.uploads[url.split("/")[2]]["blob"])
    )
    monkeypatch.setattr(cd, "RECORDINGS_DIR", tmp_path / "recordings")
    return c, srv


def test_upload_compress_manifest_roundtrip(
    cloud: tuple[CloudData, FakeServer], tmp_path: Path
) -> None:
    c, srv = cloud
    db = _recording(tmp_path)
    r = c.upload(db, robot_id="go2")
    assert r["state"] == "complete" and not r["skipped"]
    u = next(iter(srv.uploads.values()))
    assert u["content_encoding"] == "lz4" and u["robot_id"] == "go2"
    assert u["manifest"]["streams"][0]["name"] == "/lidar"
    raw = lz4.frame.decompress(u["blob"])
    assert hashlib.sha256(raw).hexdigest() == hashlib.sha256(db.read_bytes()).hexdigest()

    again = c.upload(db)
    assert again["skipped"]  # idempotent by sha


def test_resume_only_missing_parts(cloud: tuple[CloudData, FakeServer], tmp_path: Path) -> None:
    c, srv = cloud
    db = _recording(tmp_path)

    calls = {"n": 0}

    def dying_put(url: str, body: bytes) -> None:
        calls["n"] += 1
        if calls["n"] == 2:
            raise OSError("link dropped")
        srv.parts[url.split("/")[2]].__setitem__(int(url.split("/")[3]), body)

    import unittest.mock

    from dimos.core.global_config import global_config

    with (
        unittest.mock.patch.object(cd, "_put", dying_put),
        unittest.mock.patch.object(global_config, "dimos_upload_retries", 0),
    ):
        with pytest.raises(RuntimeError, match="failed after"):
            c.upload(db)

    uploaded_before = {k: len(v) for k, v in srv.parts.items()}
    r = c.upload(db)  # the re-run resumes
    assert r["state"] == "complete"
    assert sum(uploaded_before.values()) >= 1  # earlier parts were kept, not re-sent


def test_live_session_skipped(cloud: tuple[CloudData, FakeServer], tmp_path: Path) -> None:
    c, _ = cloud
    db = _recording(tmp_path, age_s=1)
    with pytest.raises(RuntimeError, match="still recording"):
        c.upload(db)


def test_pull_verifies_and_decompresses(
    cloud: tuple[CloudData, FakeServer], tmp_path: Path
) -> None:
    c, srv = cloud
    db = _recording(tmp_path)
    r = c.upload(db)
    out = c.pull(r["upload_id"], dest=tmp_path / "back.db")
    assert out.read_bytes() == db.read_bytes()

    srv.uploads[r["upload_id"]]["sha256"] = "0" * 64
    with pytest.raises(RuntimeError, match="sha256 mismatch"):
        c.pull(r["upload_id"], dest=tmp_path / "bad.db")


def test_not_logged_in(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setattr(cd, "api_key", lambda: None)
    from dimos.core.global_config import global_config

    monkeypatch.setattr(global_config, "dimos_api_key", None)
    with pytest.raises(RuntimeError, match="dimos login"):
        assert CloudDataConfig().key


def test_staging_beside_source_and_dest(
    cloud: tuple[CloudData, FakeServer], tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    """tmpfs safety + EXDEV: temp dirs must be created next to the file, never in /tmp."""
    c, srv = cloud
    seen: list[Path] = []
    real = tempfile.TemporaryDirectory

    def spying(*a: Any, **kw: Any) -> Any:
        seen.append(Path(kw["dir"]))
        return real(*a, **kw)

    monkeypatch.setattr(tempfile, "TemporaryDirectory", spying)
    db = _recording(tmp_path)
    r = c.upload(db)
    out = tmp_path / "sub" / "back.db"
    c.pull(r["upload_id"], dest=out)
    assert seen[0] == db.parent and seen[1] == out.parent


def test_requests_carry_timeouts(monkeypatch: pytest.MonkeyPatch) -> None:
    captured: dict[str, Any] = {}

    def fake_urlopen(req: Any, timeout: float | None = None) -> Any:
        captured["timeout"] = timeout
        raise urllib.error.URLError("stop here")

    monkeypatch.setattr(urllib.request, "urlopen", fake_urlopen)
    c = CloudData(CloudDataConfig(api_key="dimos_sk_t", base_url="https://fake"))
    with pytest.raises(RuntimeError):
        c._req("GET", "/v1/data/quota")
    assert captured["timeout"] == 60.0


def test_sqlite_sidecars_rejected(cloud: tuple[CloudData, FakeServer], tmp_path: Path) -> None:
    c, _ = cloud
    shm = tmp_path / "go2_short.db-shm"
    shm.write_bytes(b"x")
    with pytest.raises(RuntimeError, match="sidecar"):
        c.upload(shm)


def test_pull_confines_server_filename(
    cloud: tuple[CloudData, FakeServer], tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    """A malicious endpoint must not write outside the destination directory."""
    c, srv = cloud
    db = _recording(tmp_path)
    r = c.upload(db)
    rec_dir = tmp_path / "recordings"
    monkeypatch.setattr(cd, "RECORDINGS_DIR", rec_dir)
    for evil in ("../escaped.db.lz4", "/tmp/evil.db.lz4"):
        srv.uploads[r["upload_id"]]["filename"] = evil
        out = c.pull(r["upload_id"])
        assert out.parent == rec_dir  # confined, traversal stripped
        out.unlink()
    srv.uploads[r["upload_id"]]["filename"] = "../.."
    with pytest.raises(RuntimeError, match="invalid filename"):
        c.pull(r["upload_id"])


def test_failed_lz4_extraction_preserves_existing_dest(
    cloud: tuple[CloudData, FakeServer], tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    c, srv = cloud
    db = _recording(tmp_path)
    r = c.upload(db)
    dest = tmp_path / "keep.db"
    dest.write_bytes(b"precious existing recording")
    srv.uploads[r["upload_id"]]["blob"] = b"not lz4 at all"
    srv.uploads[r["upload_id"]]["sha256"] = hashlib.sha256(b"not lz4 at all").hexdigest()
    with pytest.raises(RuntimeError):
        c.pull(r["upload_id"], dest=dest)
    assert dest.read_bytes() == b"precious existing recording"


def test_resume_completion_sorts_parts(
    cloud: tuple[CloudData, FakeServer], tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    c, srv = cloud
    real_req = c._req

    def shuffling(m: str, p: str, b: dict[str, Any] | None = None) -> dict[str, Any]:
        r = real_req(m, p, b)
        if m == "GET" and "/uploads/" in p and r.get("parts"):
            r["parts"] = list(reversed(r["parts"]))
        if p.endswith("/complete") and b is not None:
            nums = [x["part_number"] for x in b["parts"]]
            assert nums == sorted(nums), "parts must reach /complete ascending"
        return r

    import unittest.mock

    with unittest.mock.patch.object(c, "_req", shuffling):
        r = c.upload(_recording(tmp_path))
    assert r["state"] == "complete"


def test_suffix_strip_cannot_escape(
    cloud: tuple[CloudData, FakeServer], tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    c, srv = cloud
    r = c.upload(_recording(tmp_path))
    monkeypatch.setattr(cd, "RECORDINGS_DIR", tmp_path / "rec")
    srv.uploads[r["upload_id"]]["filename"] = "...lz4"  # basename ok, stripped -> ".."
    with pytest.raises(RuntimeError, match="invalid filename"):
        c.pull(r["upload_id"])


def test_discovery_tolerates_vanished_files(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    rec = tmp_path / "rec"
    rec.mkdir()
    keep = _recording(rec)
    ghost = rec / "session_ghost_1.db"
    ghost.write_bytes(b"x")
    monkeypatch.setattr(cd, "RECORDINGS_DIR", rec)
    real_stat = Path.stat

    def flaky_stat(self: Path, *a: Any, **k: Any) -> Any:
        if self.name == ghost.name:
            raise FileNotFoundError(self)
        return real_stat(self, *a, **k)

    monkeypatch.setattr(Path, "stat", flaky_stat)
    assert cd.latest_recording() == keep
    assert cd.recordings_since(10**9) == [keep]
