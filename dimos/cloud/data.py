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

"""Dimensional cloud data: upload / pull / ls / status / quota over /v1/data."""

from collections.abc import Callable
import functools
import hashlib
import json
from pathlib import Path
import shutil
import sqlite3
import tempfile
import time
from typing import Any, cast
import urllib.error
import urllib.request

import lz4.frame
from pydantic import BaseModel

from dimos.cli.cloud import api_key
from dimos.constants import RECORDINGS_DIR
from dimos.core.global_config import global_config


class CloudDataConfig(BaseModel):
    base_url: str | None = None
    api_key: str | None = None

    @property
    def url(self) -> str:
        return (self.base_url or global_config.dimos_cloud_url).rstrip("/")

    @property
    def key(self) -> str:
        if k := self.api_key or api_key():
            return k
        raise RuntimeError("not logged in — run `dimos login`")


class CloudData:
    def __init__(self, config: CloudDataConfig | None = None) -> None:
        self.config = config or CloudDataConfig()

    def _req(self, method: str, path: str, body: dict[str, Any] | None = None) -> dict[str, Any]:
        req = urllib.request.Request(
            f"{self.config.url}{path}",
            data=json.dumps(body).encode() if body is not None else None,
            method=method,
            headers={
                "Authorization": f"Bearer {self.config.key}",
                "Content-Type": "application/json",
            },
        )
        try:
            with urllib.request.urlopen(req, timeout=global_config.dimos_http_timeout) as r:
                return cast("dict[str, Any]", json.load(r))
        except urllib.error.HTTPError as e:
            if e.code == 401:
                raise RuntimeError("API key invalid or revoked — run `dimos login`") from e
            raise RuntimeError(f"{method} {path}: {e.code} {e.read().decode()[:300]}") from e
        except (urllib.error.URLError, TimeoutError) as e:
            raise RuntimeError(f"{method} {path}: {e}") from e

    def _retry(self, fn: Callable[[], Any], what: str) -> Any:
        retries = global_config.dimos_upload_retries
        last: Exception | None = None
        for attempt in range(1, retries + 2):
            try:
                return fn()
            except (urllib.error.URLError, TimeoutError, OSError) as e:
                last = e
                if attempt <= retries:
                    time.sleep(attempt)
        raise RuntimeError(f"{what} failed after {retries + 1} attempts: {last}")

    def upload(
        self,
        path: Path,
        robot_id: str | None = None,
        kind: str = "recording",
        compress: bool | None = None,
        chunk_mb: int | None = None,
    ) -> dict[str, Any]:
        path = path.expanduser()
        if not path.is_file():
            raise FileNotFoundError(path)
        if path.suffix in (".db-shm", ".db-wal"):
            raise RuntimeError(
                f"{path.suffix} is a SQLite sidecar, not the recording — upload the .db "
                "(and its presence means the session may still be open)"
            )
        if path.suffix == ".db" and time.time() - path.stat().st_mtime < 30:
            raise RuntimeError("session still recording — skipped")
        if compress is None:
            compress = global_config.dimos_upload_compress
        if chunk_mb is None:
            chunk_mb = global_config.dimos_upload_chunk_mb

        manifest = _manifest(path) if kind == "recording" else None
        # Staged BESIDE the source, never the default temp dir: /tmp is RAM-backed
        # tmpfs on robots, and a multi-GB artifact there is an OOM.
        with tempfile.TemporaryDirectory(prefix=".dimos-upload-", dir=path.parent) as tmp:
            if compress and path.suffix != ".lz4":
                artifact = Path(tmp) / (path.name + ".lz4")
                with path.open("rb") as i, lz4.frame.open(artifact, "wb") as o:
                    shutil.copyfileobj(i, o)
                encoding = "lz4"
            else:
                artifact, encoding = path, None
            sha = _sha256(artifact)
            size = artifact.stat().st_size

            create = self._req(
                "POST",
                "/v1/data/uploads",
                {
                    "filename": artifact.name,
                    "size": size,
                    "sha256": sha,
                    "kind": kind,
                    "content_encoding": encoding,
                    "robot_id": robot_id,
                    "manifest": manifest,
                    "part_size": chunk_mb * 2**20 if chunk_mb else None,
                },
            )
            if create["state"] == "complete":
                return {
                    "upload_id": create["upload_id"],
                    "state": "complete",
                    "skipped": True,
                    "quota": create["quota"],
                }

            uid = create["upload_id"]
            have = {p["part_number"] for p in self._req("GET", f"/v1/data/uploads/{uid}")["parts"]}
            part_size = create["part_size"]
            with artifact.open("rb") as f:
                for part in create["part_urls"]:
                    n = part["part_number"]
                    if n in have:
                        continue
                    f.seek((n - 1) * part_size)
                    self._retry(
                        functools.partial(_put, part["url"], f.read(part_size)), f"part {n}"
                    )

            parts = sorted(
                self._req("GET", f"/v1/data/uploads/{uid}")["parts"], key=lambda p: p["part_number"]
            )
            done = self._req("POST", f"/v1/data/uploads/{uid}/complete", {"parts": parts})
            return {
                "upload_id": uid,
                "state": done["state"],
                "skipped": False,
                "quota": done["quota"],
            }

    def ls(self) -> list[dict[str, Any]]:
        return cast("list[dict[str, Any]]", self._req("GET", "/v1/data/uploads")["uploads"])

    def status(self, upload_id: str) -> dict[str, Any]:
        return self._req("GET", f"/v1/data/uploads/{upload_id}")

    def quota(self) -> dict[str, Any]:
        return self._req("GET", "/v1/data/quota")

    def pull(self, upload_id: str, dest: Path | None = None) -> Path:
        d = self._req("GET", f"/v1/data/uploads/{upload_id}/download")
        # The filename is server-supplied: confine it to a bare basename so a
        # compromised endpoint cannot traverse out of the destination directory.
        name = Path(d["filename"]).name
        plain_name = name.removesuffix(".lz4")
        if not plain_name or plain_name in (".", ".."):
            raise RuntimeError(f"server returned an invalid filename: {d['filename']!r}")
        out = dest or RECORDINGS_DIR / plain_name
        out.parent.mkdir(parents=True, exist_ok=True)
        # Staged beside the destination: same filesystem, so the final rename is
        # atomic and can't hit EXDEV; also keeps multi-GB downloads off tmpfs.
        with tempfile.TemporaryDirectory(prefix=".dimos-pull-", dir=out.parent) as tmp:
            raw = Path(tmp) / name
            self._retry(functools.partial(_download, d["url"], raw), "download")
            if _sha256(raw) != d["sha256"]:
                raise RuntimeError("sha256 mismatch — refusing to keep the file")
            if d.get("content_encoding") == "lz4":
                plain = Path(tmp) / "extracted"
                with lz4.frame.open(raw, "rb") as i, plain.open("wb") as o:
                    shutil.copyfileobj(i, o)
                plain.replace(out)  # atomic: tmp is on out's filesystem
            else:
                raw.replace(out)
        return out


def _recordings() -> list[tuple[float, Path]]:
    found = []
    for p in RECORDINGS_DIR.glob("*.db"):
        try:
            found.append((p.stat().st_mtime, p))
        except FileNotFoundError:
            continue
    return sorted(found)


def latest_recording() -> Path | None:
    found = _recordings()
    return found[-1][1] if found else None


def recordings_since(seconds: float) -> list[Path]:
    cutoff = time.time() - seconds
    return [p for mtime, p in _recordings() if mtime >= cutoff]


def _manifest(db: Path) -> dict[str, Any] | None:
    try:
        with sqlite3.connect(f"file:{db}?mode=ro", uri=True) as conn:
            rows = conn.execute("SELECT name, config FROM _streams").fetchall()
        return {"streams": [{"name": n, **json.loads(c)} for n, c in rows]}
    except sqlite3.Error:
        return None


def _sha256(path: Path) -> str:
    with path.open("rb") as f:
        return hashlib.file_digest(f, "sha256").hexdigest()


def _put(url: str, body: bytes) -> None:
    req = urllib.request.Request(url, data=body, method="PUT")
    with urllib.request.urlopen(req, timeout=global_config.dimos_http_timeout):
        pass


def _download(url: str, dst: Path) -> None:
    with (
        urllib.request.urlopen(url, timeout=global_config.dimos_http_timeout) as r,
        dst.open("wb") as f,
    ):
        shutil.copyfileobj(r, f)
