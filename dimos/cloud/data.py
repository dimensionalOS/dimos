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

"""Dimensional cloud datasets: upload / pull / ls / status / quota.

`CloudData` is a thin facade over a `DatasetBackend`. `MultipartBackend` (today)
speaks the /v1/data blob API; a sync backend plugs in beside it without touching
callers. Every knob lives on GlobalConfig; nothing here is hardcoded."""

from __future__ import annotations

import functools
import hashlib
import json
from pathlib import Path
import shutil
import sqlite3
import tempfile
import time
from typing import Any, Protocol

from dimos.cli.cloud import api_key
from dimos.cloud import codecs
from dimos.cloud.http_transport import HttpTransport, Transport
from dimos.constants import RECORDINGS_DIR
from dimos.core.global_config import global_config


class DatasetBackend(Protocol):
    def upload(
        self, path: Path, *, robot_id: str | None, kind: str, part_size: int | None
    ) -> dict[str, Any]: ...
    def pull(self, upload_id: str, dest: Path | None) -> Path: ...
    def ls(self) -> list[dict[str, Any]]: ...
    def status(self, upload_id: str) -> dict[str, Any]: ...
    def quota(self) -> dict[str, Any]: ...


class MultipartBackend:
    """Blob artifacts via /v1/data: presigned multipart to S3, server-side part
    listing as the only resume state, sha256-verified pulls."""

    API = "/v1/data"

    def __init__(
        self, transport: Transport, codec_id: str, staging_dir: Path | None, retries: int
    ) -> None:
        self.t, self.codec_id, self.staging_dir, self.retries = (
            transport,
            codec_id,
            staging_dir,
            retries,
        )

    def _retry(self, fn: Any, what: str) -> Any:
        for attempt in range(1, self.retries + 2):
            try:
                return fn()
            except (OSError, RuntimeError) as e:
                if attempt > self.retries:
                    raise RuntimeError(f"{what} failed after {attempt} attempts: {e}") from e
                time.sleep(attempt)

    def _staging(self, beside: Path) -> tempfile.TemporaryDirectory[str]:
        return tempfile.TemporaryDirectory(dir=self.staging_dir or beside.parent)

    def upload(
        self, path: Path, *, robot_id: str | None, kind: str, part_size: int | None
    ) -> dict[str, Any]:
        manifest = _manifest(path)
        with self._staging(path) as tmp:
            if self.codec_id and path.suffix != codecs.suffix(self.codec_id):
                artifact = Path(tmp) / (path.name + codecs.suffix(self.codec_id))
                codecs.compress(self.codec_id, path, artifact)
            else:
                artifact = path
            size = artifact.stat().st_size
            create = self.t.request(
                "POST",
                f"{self.API}/uploads",
                {
                    "filename": artifact.name,
                    "size": size,
                    "sha256": _sha256(artifact),
                    "kind": kind,
                    "content_encoding": self.codec_id or None,
                    "robot_id": robot_id,
                    "manifest": manifest,
                    "part_size": part_size,
                },
            )
            if create["state"] == "complete":
                return {**create, "skipped": True}
            uid = create["upload_id"]
            have = {p["part_number"] for p in self.status(uid)["parts"]}
            with artifact.open("rb") as f:
                for part in create["part_urls"]:
                    if part["part_number"] in have:
                        continue
                    f.seek((part["part_number"] - 1) * create["part_size"])
                    self._retry(
                        functools.partial(self.t.put, part["url"], f.read(create["part_size"])),
                        f"part {part['part_number']}",
                    )
            parts = sorted(self.status(uid)["parts"], key=lambda p: p["part_number"])
            done = self.t.request("POST", f"{self.API}/uploads/{uid}/complete", {"parts": parts})
            return {**done, "upload_id": uid, "skipped": False}

    def pull(self, upload_id: str, dest: Path | None) -> Path:
        d = self.t.request("GET", f"{self.API}/uploads/{upload_id}/download")
        wire = d.get("content_encoding") or ""
        name = Path(d["filename"]).name
        plain = name.removesuffix(codecs.suffix(wire)) if wire else name
        if not plain or plain in (".", ".."):
            raise RuntimeError(f"server returned an invalid filename: {d['filename']!r}")
        out = dest or RECORDINGS_DIR / plain
        out.parent.mkdir(parents=True, exist_ok=True)
        with self._staging(out) as tmp:
            raw = Path(tmp) / name
            self._retry(functools.partial(self.t.download, d["url"], raw), "download")
            if _sha256(raw) != d["sha256"]:
                raise RuntimeError("sha256 mismatch — refusing to keep the file")
            if wire:
                decoded = Path(tmp) / plain
                codecs.decompress(wire, raw, decoded)
                _move_into_place(decoded, out)
            else:
                _move_into_place(raw, out)
        return out

    def ls(self) -> list[dict[str, Any]]:
        return list(self.t.request("GET", f"{self.API}/uploads")["uploads"])

    def status(self, upload_id: str) -> dict[str, Any]:
        return self.t.request("GET", f"{self.API}/uploads/{upload_id}")

    def quota(self) -> dict[str, Any]:
        return self.t.request("GET", f"{self.API}/quota")


BACKENDS: dict[str, Any] = {"multipart": MultipartBackend}


class CloudData:
    """Facade: resolves auth + config once, delegates to the configured backend."""

    def __init__(self, backend: DatasetBackend | None = None) -> None:
        if backend is None:
            key = global_config.dimos_api_key or api_key()
            if not key:
                raise RuntimeError("not logged in — run `dimos login`")
            transport = HttpTransport(
                global_config.dimos_cloud_url, key, global_config.dimos_http_timeout
            )
            backend = BACKENDS[global_config.dimos_cloud_backend](
                transport,
                global_config.dimos_upload_codec,
                global_config.dimos_staging_dir,
                global_config.dimos_upload_retries,
            )
        self.backend = backend

    def upload(
        self,
        path: Path,
        robot_id: str | None = None,
        kind: str | None = None,
        chunk_mb: int | None = None,
    ) -> dict[str, Any]:
        path = path.expanduser()
        if not path.is_file():
            raise FileNotFoundError(path)
        if _is_sidecar(path):
            raise RuntimeError(f"{path.name} is a SQLite sidecar, not the data file")
        if time.time() - path.stat().st_mtime < global_config.dimos_upload_quiet_s:
            raise RuntimeError("file still being written — skipped")
        chunk_mb = chunk_mb if chunk_mb is not None else global_config.dimos_upload_chunk_mb
        return self.backend.upload(
            path,
            robot_id=robot_id,
            kind=kind or kind_of(path),
            part_size=chunk_mb * 2**20 if chunk_mb else None,
        )

    def pull(self, upload_id: str, dest: Path | None = None) -> Path:
        return self.backend.pull(upload_id, dest)

    def ls(self) -> list[dict[str, Any]]:
        return self.backend.ls()

    def status(self, upload_id: str) -> dict[str, Any]:
        return self.backend.status(upload_id)

    def quota(self) -> dict[str, Any]:
        return self.backend.quota()


def recordings(newer_than_s: float | None = None) -> list[Path]:
    """Files in RECORDINGS_DIR by mtime, oldest first, any format; sidecars excluded;
    tolerant of files vanishing mid-scan."""
    cutoff = time.time() - newer_than_s if newer_than_s else None
    found = []
    for p in RECORDINGS_DIR.iterdir():
        try:
            if not p.is_file() or _is_sidecar(p):
                continue
            mtime = p.stat().st_mtime
        except FileNotFoundError:
            continue
        if cutoff is None or mtime >= cutoff:
            found.append((mtime, p))
    return [p for _mtime, p in sorted(found)]


def kind_of(path: Path) -> str:
    """Server category inferred from the file: a dimos recording (mem2 sqlite with a
    `_streams` table, or mcap) is `recording`; everything else is a generic `blob`.
    `--kind` overrides for video/pointcloud/log."""
    if path.suffix == ".mcap" or _manifest(path) is not None:
        return "recording"
    return "blob"


def _move_into_place(src: Path, out: Path) -> None:
    """Atomic replace; a cross-filesystem staging dir falls back to copy-then-rename
    so the destination is still never left truncated."""
    try:
        src.replace(out)
    except OSError:
        part = out.with_name(out.name + ".part")
        shutil.move(str(src), part)
        part.replace(out)


def _is_sidecar(path: Path) -> bool:
    return path.name.endswith(("-shm", "-wal", "-journal"))  # SQLite's own suffixes


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
