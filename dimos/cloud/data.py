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

`CloudData` is a thin facade over `MultipartBackend` (today)
speaks the /v1/data blob API; a sync backend plugs in beside it without touching
callers. Transfer settings live on CloudConfig."""

from __future__ import annotations

from collections.abc import Callable
import contextlib
import ctypes
from datetime import datetime
import functools
import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import sqlite3
import stat
import sys
import tarfile
import tempfile
import time
from typing import Any

from dimos.cli.cloud import api_key
from dimos.cloud import codecs, preview
from dimos.cloud.cloud_request import CloudRequest, HttpCloudRequest
from dimos.cloud.config import CloudConfig
from dimos.constants import DOWNLOADS_DIR, RECORDINGS_DIR
from dimos.core.run_registry import blueprint_from_run_id
from dimos.memory.cli.dataset import open_store

Progress = Callable[[str, int, int], None]  # (phase, done_bytes, total_bytes)
DIRECTORY_MANIFEST = {"format": "dimos.directory.tar", "version": 1}
MIN_TAR = 2**20  # Cloud upload minimum (zero padding is valid tar).
MAX_TAR = 100 * 2**30
MAX_TAR_MEMBERS = 20_000
MAX_TAR_DEPTH = 128  # Bound the open directory descriptors and scratch-tree cleanup.


class DataApi:
    """The /v1/data operations, by name. The only place routes and payloads live."""

    PREFIX = "/v1/data"

    def __init__(self, request: CloudRequest) -> None:
        self.t = request

    def create(self, **spec: Any) -> dict[str, Any]:
        return self.t.request("POST", f"{self.PREFIX}/uploads", spec)

    def status(self, upload_id: str) -> dict[str, Any]:
        return self.t.request("GET", f"{self.PREFIX}/uploads/{upload_id}")

    def complete(self, upload_id: str, parts: list[dict[str, Any]]) -> dict[str, Any]:
        return self.t.request(
            "POST", f"{self.PREFIX}/uploads/{upload_id}/complete", {"parts": parts}
        )

    def download(self, upload_id: str) -> dict[str, Any]:
        return self.t.request("GET", f"{self.PREFIX}/uploads/{upload_id}/download")

    def list(self) -> list[dict[str, Any]]:
        return list(self.t.request("GET", f"{self.PREFIX}/uploads")["uploads"])

    def quota(self) -> dict[str, Any]:
        return self.t.request("GET", f"{self.PREFIX}/quota")

    def put_preview(self, upload_id: str, doc: dict[str, Any]) -> dict[str, Any]:
        return self.t.request("PUT", f"{self.PREFIX}/uploads/{upload_id}/preview", doc)

    def confirm_video(self, upload_id: str) -> dict[str, Any]:
        return self.t.request("POST", f"{self.PREFIX}/uploads/{upload_id}/preview/video")

    def put_part(self, url: str, chunk: bytes) -> None:
        self.t.put(url, chunk)

    def fetch(
        self, url: str, dst: Path, progress: Callable[[int, int], None] | None = None
    ) -> None:
        self.t.download(url, dst, progress)


class MultipartBackend:
    """Blob artifacts via /v1/data: presigned multipart to S3, server-side part
    listing as the only resume state, sha256-verified pulls."""

    def __init__(self, api: DataApi, codec_id: str, staging_dir: Path | None, retries: int) -> None:
        self.api, self.codec_id, self.staging_dir, self.retries = (
            api,
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

    def _staging(self, beside: Path, *, reap: bool = True) -> tempfile.TemporaryDirectory[str]:
        parent = self.staging_dir or beside.parent
        parent.mkdir(parents=True, exist_ok=True)
        if reap:
            _reap_stale(parent)
        return tempfile.TemporaryDirectory(prefix=".dimos-staging-", dir=parent)

    def upload(
        self,
        path: Path,
        *,
        robot_id: str | None,
        kind: str,
        part_size: int | None,
        progress: Progress | None,
    ) -> dict[str, Any]:
        tick = progress or (lambda phase, done, total: None)
        directory = path.is_dir()
        recording = not directory and kind_of(path) == "recording"
        manifest = DIRECTORY_MANIFEST if directory else _manifest(path)
        if not directory and (bp := _blueprint(path)):
            manifest = dict(manifest or {}, blueprint=bp)
        with self._staging(path.absolute() if directory else path, reap=not directory) as tmp:
            # A file already carrying the codec's suffix uploads as-is; it must not be
            # stamped, or pull would decompress bytes we never compressed.
            compress = (
                not directory
                and bool(self.codec_id)
                and path.suffix != codecs.suffix(self.codec_id)
            )
            if directory:
                tick("pack", 0, 0)
                artifact = Path(tmp) / "directory.tar"
                _pack_directory(path, artifact)
            elif compress:
                _require_space(Path(tmp), path.stat().st_size)
                artifact = Path(tmp) / (path.name + codecs.suffix(self.codec_id))
                tick("compress", 0, 0)
                codecs.compress(self.codec_id, path, artifact)
            else:
                artifact = path
            size = artifact.stat().st_size
            spec = dict(
                filename=f"{Path(os.path.abspath(path)).name or 'root'}.tar"
                if directory
                else artifact.name,
                size=size,
                sha256=_sha256(artifact),
                kind=kind,
                content_encoding=self.codec_id if compress else None,
                robot_id=robot_id,
                manifest=manifest,
                part_size=part_size,
            )

            create = self.api.create(**spec)
            uid = create["upload_id"]
            # Dedup is by wire digest. Only a plain tar can alias a directory upload.
            if directory or (not compress and _starts_like_tar(artifact)):
                prior = next((row for row in self.api.list() if row["id"] == uid), None)
                if prior is None or _is_directory(prior.get("manifest")) != directory:
                    raise RuntimeError("identical wire bytes already have a different upload type")
            if create["state"] == "complete":
                # already uploaded: (re)send the preview, so a failed send is repaired
                # by simply running the upload again
                return {
                    **create,
                    "skipped": True,
                    "preview": self._preview(uid, path, Path(tmp)) if recording else None,
                }
            have = {p["part_number"] for p in self.status(uid)["parts"]}
            ps = create["part_size"]
            done = min(len(have) * ps, size)
            tick("upload", done, size)
            urls = {p["part_number"]: p["url"] for p in create["part_urls"]}
            with artifact.open("rb") as f:
                for n in sorted(urls):
                    if n in have:
                        continue
                    f.seek((n - 1) * ps)
                    chunk = f.read(ps)
                    try:
                        self._retry(
                            functools.partial(self.api.put_part, urls[n], chunk), f"part {n}"
                        )
                    except RuntimeError as e:
                        if "403" not in str(e):
                            raise
                        # Part URLs are presigned at create and share one expiry
                        # clock; an upload slower than the TTL outlives them and
                        # S3 answers 403. create is idempotent by sha256 and
                        # re-signs every URL.
                        fresh = self.api.create(**spec)
                        if fresh["upload_id"] != uid:
                            raise RuntimeError("upload ID changed while refreshing part URLs")
                        if fresh["state"] == "complete":
                            return {**fresh, "skipped": True}
                        urls = {p["part_number"]: p["url"] for p in fresh["part_urls"]}
                        self._retry(
                            functools.partial(self.api.put_part, urls[n], chunk), f"part {n}"
                        )
                    done += len(chunk)
                    tick("upload", done, size)
            parts = sorted(self.status(uid)["parts"], key=lambda p: p["part_number"])
            done = self.api.complete(uid, parts)
            return {
                **done,
                "upload_id": uid,
                "skipped": False,
                "preview": self._preview(uid, path, Path(tmp)) if recording else None,
            }

    def _preview(self, upload_id: str, path: Path, tmp: Path) -> str:
        """The console's spatial preview, built here while the recording is at hand.
        A .db is built from a copy: the store opens read-write (WAL), and the user's
        recording must stay byte-identical to what was uploaded; an .mcap opens
        read-only. Best effort: the upload stands without it."""
        try:
            source, clip = path, tmp / "timelapse.mp4"
            if path.suffix != ".mcap":
                source = tmp / "preview.db"
                shutil.copyfile(path, source)
            note = ""
            with open_store(source) as store:
                doc = preview.build(store)
                try:  # the preview stands without its clip
                    video = preview.timelapse(store, clip) if doc is not None else None
                except Exception as e:
                    video, note = None, f" (no timelapse: {e})"
            if doc is None:
                return "none (no lidar, camera or odometry)"
            if video:
                doc["video"] = video
            sent = self.api.put_preview(upload_id, doc)
            if not (video and sent.get("video_url")):
                return "sent" + note
            self.api.put_part(sent["video_url"], clip.read_bytes())
            self.api.confirm_video(upload_id)  # the console links the clip only after this
            return f"sent (+ {video['duration_s']:.0f} s timelapse, {video['bytes'] / 1e6:.1f} MB)"
        except Exception as e:
            return f"failed: {e}"

    def pull(
        self,
        upload_id: str,
        dest: Path | None,
        tag: str = "",
        progress: Progress | None = None,
        size: int = 0,
        directory: bool = False,
    ) -> Path:
        tick = progress or (lambda phase, done, total: None)
        if directory and not re.fullmatch(r"[A-Za-z0-9_-]+", upload_id):
            raise RuntimeError("invalid directory upload id")
        d = self.api.download(upload_id)
        wire = d.get("content_encoding") or ""
        name = _plain_name(d["filename"], wire) if directory or dest is None else ""
        if directory:
            if wire or not name.endswith(".tar") or not 0 < size <= MAX_TAR:
                raise RuntimeError("invalid directory download descriptor")
            name = name.removesuffix(".tar")
        out = dest or DOWNLOADS_DIR / f"{tag}{name}"

        def downloaded(done: int, total: int) -> None:
            if directory and (done > size or (total and total != size)):
                raise RuntimeError("directory wire size mismatch")
            tick("download", done, total)

        out.parent.mkdir(parents=True, exist_ok=True)
        with self._staging(out, reap=not directory) as tmp:
            if size:
                # wire and decompressed copies coexist in staging before the move
                _require_space(Path(tmp), size * 2 if wire else size)
            raw = Path(tmp) / "wire"
            tick("download", 0, 0)
            self._retry(
                functools.partial(self.api.fetch, d["url"], raw, downloaded),
                "download",
            )
            tick("verify", 0, 0)
            if _sha256(raw) != d["sha256"]:
                raise RuntimeError("sha256 mismatch — refusing to keep the file")
            if directory:
                if raw.stat().st_size != size:
                    raise RuntimeError("directory wire size mismatch")
                # Stage the directory on the destination filesystem, even if wire
                # staging is configured on another mount.
                with tempfile.TemporaryDirectory(prefix=".dimos-restore-", dir=out.parent) as stage:
                    tree = Path(stage) / "tree"
                    tree.mkdir()
                    tick("extract", 0, 0)
                    _extract_directory(raw, tree, size)
                    _publish_directory(tree, out)
                return out
            if wire:
                tick("decompress", 0, 0)
                codecs.decompress(wire, raw, plain := Path(tmp) / "plain")
            else:
                raw.rename(plain := Path(tmp) / "plain")
            _move_into_place(plain, out)
        return out

    def ls(self) -> list[dict[str, Any]]:
        return self.api.list()

    def status(self, upload_id: str) -> dict[str, Any]:
        return self.api.status(upload_id)

    def quota(self) -> dict[str, Any]:
        return self.api.quota()


class CloudData:
    """Facade: resolves auth + config once, delegates to the configured backend."""

    def __init__(
        self, backend: MultipartBackend | None = None, *, config: CloudConfig | None = None
    ) -> None:
        self.config = config if config is not None else CloudConfig()
        if backend is None:
            key = api_key(self.config)
            if not key:
                raise RuntimeError("not logged in — run `dimos login`")
            request = HttpCloudRequest(self.config.url, key, self.config.http_timeout)
            backend = MultipartBackend(
                DataApi(request),
                self.config.upload_codec,
                self.config.staging_dir,
                self.config.upload_retries,
            )
        self.backend = backend

    def upload(
        self,
        path: Path,
        robot_id: str | None = None,
        kind: str | None = None,
        chunk_mb: int | None = None,
        progress: Progress | None = None,
        skip_recent: bool = False,
    ) -> dict[str, Any]:
        path = path.expanduser()
        directory = path.is_dir()
        if directory:
            if kind not in (None, "blob"):
                raise RuntimeError("directories can only be uploaded as blobs")
        elif not path.is_file():
            raise FileNotFoundError(path)
        if not directory and _is_sidecar(path):
            raise RuntimeError(f"{path.name} is a SQLite sidecar, not the data file")
        if skip_recent and time.time() - path.stat().st_mtime < self.config.upload_quiet_s:
            raise RuntimeError("file still being written — skipped")
        chunk_mb = chunk_mb if chunk_mb is not None else self.config.upload_chunk_mb
        return self.backend.upload(
            path,
            robot_id=robot_id,
            kind=kind or ("blob" if directory else kind_of(path)),
            part_size=chunk_mb * 2**20 if chunk_mb else None,
            progress=progress,
        )

    def resolve(self, prefix: str | None) -> dict[str, Any]:
        """Upload row from an id prefix (as printed by `ls`); None -> newest complete."""
        rows = [u for u in self.ls() if u["state"] == "complete"]
        hits = [u for u in rows if not prefix or u["id"].startswith(prefix)]
        if not hits:
            raise RuntimeError(f"no upload matching {prefix!r}" if prefix else "no uploads")
        if prefix and len(hits) > 1 and hits[0]["id"] != prefix:
            raise RuntimeError(f"ambiguous id prefix {prefix!r}")
        return hits[0]

    def pull(
        self, upload_id: str | None, dest: Path | None = None, progress: Progress | None = None
    ) -> Path:
        row = self.resolve(upload_id)
        directory = _is_directory(row.get("manifest"))
        if directory and row.get("kind") != "blob":
            raise RuntimeError("directory archive must be a blob")
        return self.backend.pull(
            str(row["id"]),
            dest,
            tag=_tag(row),
            progress=progress,
            size=int(row.get("size") or 0),
            directory=directory,
        )

    def ls(self) -> list[dict[str, Any]]:
        return self.backend.ls()

    def status(self, upload_id: str) -> dict[str, Any]:
        return self.backend.status(upload_id)

    def quota(self) -> dict[str, Any]:
        return self.backend.quota()


def recordings(newer_than_s: float | None = None) -> list[Path]:
    """Files under RECORDINGS_DIR (recursively: runs live in per-run subdirectories) by
    mtime, oldest first, any format; sidecars and staging dirs excluded."""
    cutoff = time.time() - newer_than_s if newer_than_s else None
    found = []
    for p in RECORDINGS_DIR.rglob("*"):
        try:
            if (
                not p.is_file()
                or _is_sidecar(p)
                or any(
                    part.startswith(".dimos-staging-")
                    for part in p.relative_to(RECORDINGS_DIR).parts
                )
            ):
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


def _require_space(where: Path, need: int) -> None:
    """Uploads and pulls stage copies beside the file (or in DIMOS_CLOUD_STAGING_DIR); fail
    before starting rather than at 99% of a 100 GB transfer."""
    free = shutil.disk_usage(where).free
    if free < need:
        raise RuntimeError(
            f"{where} has {free / 2**30:.1f} GB free, need ~{need / 2**30:.1f} GB to stage — "
            "set DIMOS_CLOUD_STAGING_DIR to a larger partition"
        )


def _reap_stale(parent: Path, max_age_s: float = 86400) -> None:
    """A SIGKILL mid-transfer orphans the staging dir beside the file; nothing else
    would ever remove it."""
    for d in parent.glob(".dimos-staging-*"):
        try:
            if d.is_dir() and time.time() - d.stat().st_mtime > max_age_s:
                shutil.rmtree(d, ignore_errors=True)
        except FileNotFoundError:
            continue


def _move_into_place(src: Path, out: Path) -> None:
    """Atomic replace; a cross-filesystem staging dir falls back to copy-then-rename
    so the destination is still never left truncated."""
    try:
        src.replace(out)
    except OSError:
        fd, tmp = tempfile.mkstemp(prefix=out.name + ".", dir=out.parent)
        os.close(fd)
        shutil.move(str(src), tmp)
        Path(tmp).replace(out)


def _is_directory(manifest: Any) -> bool:
    if not isinstance(manifest, dict) or manifest.get("format") != DIRECTORY_MANIFEST["format"]:
        return False
    if type(manifest.get("version")) is not int or manifest["version"] != 1:
        raise RuntimeError("unsupported directory archive marker")
    return True


def _starts_like_tar(path: Path) -> bool:
    with path.open("rb") as stream:
        header = stream.read(tarfile.BLOCKSIZE)
    try:
        tarfile.TarInfo.frombuf(header, sys.getfilesystemencoding(), "surrogateescape")
    except tarfile.HeaderError:
        return header == bytes(tarfile.BLOCKSIZE)  # Empty archive.
    return True


def _pack_directory(root: Path, target: Path) -> None:
    scratch = target.parent.stat()
    count = 0

    def add_tree(archive: tarfile.TarFile, fd: int, prefix: str, depth: int) -> None:
        nonlocal count
        # Bound the sorted list too, not just the number of written members.
        entries_by_name = {}
        with os.scandir(fd) as entries:
            for entry in entries:
                info = entry.stat(follow_symlinks=False)
                if os.path.samestat(info, scratch):
                    continue
                count += 1
                if count > MAX_TAR_MEMBERS or depth > MAX_TAR_DEPTH:
                    raise RuntimeError("directory archive member or depth limit exceeded")
                entries_by_name[entry.name] = info
        for leaf, info in sorted(entries_by_name.items()):
            name = prefix + leaf
            mode = info.st_mode
            directory = stat.S_ISDIR(mode)
            if not (directory or stat.S_ISREG(mode)) or len(os.fsencode(name)) > 4095:
                raise RuntimeError(f"unsupported directory member: {name!r}")
            flags = os.O_DIRECTORY if directory else os.O_NONBLOCK
            child = os.open(leaf, os.O_RDONLY | os.O_NOFOLLOW | flags, dir_fd=fd)
            try:
                st = os.fstat(child)
                if not os.path.samestat(st, info):
                    raise RuntimeError(f"directory member changed while packing: {name!r}")
                member = tarfile.TarInfo(name)
                member.type = tarfile.DIRTYPE if directory else tarfile.REGTYPE
                member.size = 0 if directory else st.st_size
                # Leave room for the GNU name header and tar end padding.
                if archive.offset + member.size + 2 * tarfile.RECORDSIZE > MAX_TAR:
                    raise RuntimeError("directory archive exceeds size limit")
                if directory:
                    archive.addfile(member)
                    add_tree(archive, child, name + "/", depth + 1)
                else:
                    with open(child, "rb", closefd=False) as stream:
                        archive.addfile(member, stream)
            finally:
                os.close(child)

    # Open each component without following links, then walk by directory fd.
    anchor, *parts = root.absolute().parts
    flags = os.O_RDONLY | os.O_DIRECTORY | os.O_NOFOLLOW
    fd = os.open(anchor, flags)
    try:
        for part in parts:
            child = os.open(part, flags, dir_fd=fd)
            os.close(fd)
            fd = child
        with target.open("xb") as stream:
            with tarfile.open(fileobj=stream, mode="w", format=tarfile.GNU_FORMAT) as archive:
                add_tree(archive, fd, "", 1)
            if stream.tell() < MIN_TAR:
                stream.truncate(MIN_TAR)
    finally:
        os.close(fd)


class _DirectoryMember(tarfile.TarInfo):
    @classmethod
    def frombuf(cls, buf: bytes | bytearray, encoding: str, errors: str) -> _DirectoryMember:
        member = super().frombuf(buf, encoding, errors)
        # Reject extensions before tarfile reads their payload. Only GNU long names
        # are needed; PAX/sparse entries could allocate or expand beyond wire size.
        if (
            not (
                (member.type in (tarfile.REGTYPE, tarfile.AREGTYPE) and member.size >= 0)
                or (member.isdir() and member.size == 0)
                or (member.type == tarfile.GNUTYPE_LONGNAME and 0 < member.size <= 4097)
            )
            or member.linkname
        ):
            raise tarfile.ReadError(f"unsupported directory tar member: {member.name!r}")
        return member


def _extract_directory(wire: Path, tree: Path, size: int) -> None:
    try:
        with (
            wire.open("rb") as stream,
            tarfile.open(fileobj=stream, mode="r:", tarinfo=_DirectoryMember) as archive,
        ):
            for i, member in enumerate(archive):
                parts = member.name.split("/")
                if (
                    i >= MAX_TAR_MEMBERS
                    or len(parts) > MAX_TAR_DEPTH
                    or any(p in ("", ".", "..") for p in parts)
                    or member.offset_data + member.size > size
                ):
                    raise RuntimeError(f"unsafe or oversized directory member: {member.name!r}")
                target = tree.joinpath(*parts)
                if member.isdir():
                    target.mkdir()
                else:
                    src = archive.extractfile(member)
                    assert src is not None  # Only regular files reach this branch.
                    with src, target.open("xb") as dst:
                        shutil.copyfileobj(src, dst)
            stream.seek(archive.offset)
            trailer = stream.read(MIN_TAR + 1)
            if (
                not 2 * tarfile.BLOCKSIZE <= len(trailer) <= MIN_TAR
                or len(trailer) % tarfile.BLOCKSIZE
                or any(trailer)
            ):
                raise RuntimeError("invalid directory tar trailer")
    except (tarfile.TarError, RecursionError) as exc:
        raise RuntimeError(f"invalid directory tar: {exc}") from exc


def _publish_directory(source: Path, target: Path) -> None:
    # Path.rename() can replace an empty directory even after an existence check.
    libc = ctypes.CDLL(None, use_errno=True)
    src, dst = os.fsencode(source), os.fsencode(target)
    if sys.platform == "linux" and hasattr(libc, "renameat2"):
        result = libc.renameat2(-100, src, -100, dst, 1)  # AT_FDCWD, RENAME_NOREPLACE
    elif sys.platform == "darwin":
        result = libc.renamex_np(src, dst, 4)  # RENAME_EXCL
    else:
        raise RuntimeError("atomic directory restore requires Linux or macOS")
    if result:
        err = ctypes.get_errno()
        raise OSError(err, os.strerror(err), target)


def _is_sidecar(path: Path) -> bool:
    return path.name.endswith(("-shm", "-wal", "-journal"))  # SQLite's own suffixes


def _sqlite_streams(db: Path) -> list[dict[str, Any]] | None:
    try:
        with contextlib.closing(
            sqlite3.connect(f"{db.absolute().as_uri()}?mode=ro", uri=True)
        ) as conn:
            rows = conn.execute("SELECT name, config FROM _streams").fetchall()
    except sqlite3.Error:
        return None
    return [{"name": n, **json.loads(c)} for n, c in rows]


def _store_streams(recording: Path) -> list[dict[str, Any]] | None:
    """What a recording that isn't a dimos sqlite database holds, asked of the store itself."""
    try:
        with open_store(recording) as store:
            streams = []
            for name in store.list_streams():
                try:
                    payload = store.stream(name).data_type
                except Exception:  # a channel whose type won't import still leaves the rest
                    continue
                streams.append(
                    {
                        "name": name,
                        "payload_module": f"{payload.__module__}.{payload.__qualname__}"
                        if payload
                        else None,
                    }
                )
            return streams
    except Exception:
        return None


def _manifest(recording: Path) -> dict[str, Any] | None:
    """What the recording holds: the server keeps it, and it is what says a preview can be built.

    A dimos sqlite database answers from its own `_streams` table, cheaply and without importing
    any dimos type. Anything else — an .mcap — is opened as a store and asked instead. Without
    that an .mcap had no manifest, and no manifest meant no console preview at all, whatever the
    preview itself could make of it.
    """
    streams = _sqlite_streams(recording)
    if streams is None:
        streams = _store_streams(recording)
    return {"streams": streams} if streams else None


def _plain_name(filename: str, wire: str) -> str:
    name = Path(filename).name.removesuffix(codecs.suffix(wire))
    if not name or name in (".", ".."):
        raise RuntimeError(f"server returned an invalid filename: {filename!r}")
    return name


def _tag(row: dict[str, Any]) -> str:
    """Filename prefix that keeps pulls distinguishable: upload time + id."""
    uid = str(row["id"])[:8]
    try:
        stamp = datetime.fromisoformat(str(row["created_at"]))
    except (KeyError, ValueError):
        return f"{uid}-"
    return f"{stamp:%Y%m%d-%H%M%S}-{uid}-"


def _blueprint(path: Path) -> str | None:
    """Run dirs are named after their run ID (generate_run_id)."""
    return blueprint_from_run_id(path.parent.name)


def _sha256(path: Path) -> str:
    with path.open("rb") as f:
        if sys.version_info >= (3, 11):
            return hashlib.file_digest(f, "sha256").hexdigest()
        # TODO(PY311): drop this fallback for hashlib.file_digest.
        digest = hashlib.sha256()
        while chunk := f.read(2**20):
            digest.update(chunk)
        return digest.hexdigest()
