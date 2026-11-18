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

"""Plain tar transport for directory trees of ordinary files."""

from __future__ import annotations

from collections.abc import Callable, Iterator
from contextlib import contextmanager
import ctypes
import os
from pathlib import Path
import stat
import sys
import tarfile
from typing import Any, BinaryIO

MANIFEST = {"format": "dimos.directory.tar", "version": 1}
MIN_WIRE = 2**20  # Cloud upload minimum; use only valid tar zero padding.
MAX_WIRE = 100 * 2**30
MAX_MEMBERS = 20_000
MAX_NAME = 4095  # Bound GNU long-name records before reading their payload.
BLOCK = tarfile.BLOCKSIZE
ZERO = bytes(BLOCK)


def marked(manifest: Any) -> bool:
    if not isinstance(manifest, dict) or manifest.get("format") != MANIFEST["format"]:
        return False
    if type(manifest.get("version")) is not int or manifest["version"] != MANIFEST["version"]:
        raise RuntimeError("unsupported directory archive version")
    return True


@contextmanager
def _open_at(name: str, parent: int | None = None, *, directory: bool) -> Iterator[int]:
    flags = os.O_DIRECTORY if directory else os.O_NONBLOCK
    fd = os.open(name, os.O_RDONLY | os.O_NOFOLLOW | flags, dir_fd=parent)
    try:
        yield fd
    finally:
        os.close(fd)


@contextmanager
def _open_root(path: Path) -> Iterator[int]:
    """Never follow a source symlink, including intermediate directories."""
    parts = path.absolute().parts
    fd = os.open(parts[0], os.O_RDONLY | os.O_DIRECTORY)
    try:
        for part in parts[1:]:
            child = os.open(part, os.O_RDONLY | os.O_NOFOLLOW | os.O_DIRECTORY, dir_fd=fd)
            os.close(fd)
            fd = child
        yield fd
    finally:
        os.close(fd)


def pack(root: Path, target: Path, require_space: Callable[[Path, int], None]) -> None:
    # Staging can be configured inside the source. Omit only this invocation's
    # private scratch directory, never filter user files by name.
    scratch = target.parent.stat()
    count = 0

    def add_tree(fd: int, prefix: str, archive: tarfile.TarFile) -> None:
        nonlocal count
        for leaf in sorted(os.listdir(fd)):
            name = prefix + leaf
            entry = os.stat(leaf, dir_fd=fd, follow_symlinks=False)
            if os.path.samestat(entry, scratch):
                continue
            directory = stat.S_ISDIR(entry.st_mode)
            if not (directory or stat.S_ISREG(entry.st_mode)):
                raise RuntimeError(f"unsupported directory member: {name!r}")
            count += 1
            if count > MAX_MEMBERS or len(os.fsencode(name)) > MAX_NAME:
                raise RuntimeError(f"directory archive member limit exceeded: {name!r}")
            with _open_at(leaf, fd, directory=directory) as child:
                st = os.fstat(child)
                if not os.path.samestat(entry, st) or (
                    not directory and not stat.S_ISREG(st.st_mode)
                ):
                    raise RuntimeError(f"directory member changed while packing: {name!r}")
                member = tarfile.TarInfo(name)
                member.type = tarfile.DIRTYPE if directory else tarfile.REGTYPE
                member.size = 0 if directory else st.st_size
                header = member.tobuf(
                    tarfile.GNU_FORMAT, sys.getfilesystemencoding(), "surrogateescape"
                )
                blocks = len(header) + member.size + (-member.size % BLOCK)
                end = archive.offset + blocks + 2 * BLOCK
                size = max(MIN_WIRE, end + (-end % tarfile.RECORDSIZE))
                if size > MAX_WIRE:
                    raise RuntimeError("directory archive exceeds size limit")
                require_space(target.parent, size - archive.offset)
                if directory:
                    archive.addfile(member)
                    add_tree(child, name + "/", archive)
                else:
                    with os.fdopen(os.dup(child), "rb") as stream:
                        archive.addfile(member, stream)

    try:
        with _open_root(root) as fd, target.open("xb") as stream:
            require_space(target.parent, MIN_WIRE)
            with tarfile.open(fileobj=stream, mode="w", format=tarfile.GNU_FORMAT) as archive:
                add_tree(fd, "", archive)
            if stream.tell() < MIN_WIRE:
                stream.truncate(MIN_WIRE)
    except (tarfile.TarError, ValueError, RecursionError) as exc:
        raise RuntimeError("cannot pack directory as plain tar") from exc


def _padding(stream: BinaryIO, size: int) -> None:
    padding = -size % BLOCK
    if stream.read(padding) != ZERO[:padding]:
        raise RuntimeError("invalid directory tar padding")


def extract(wire: Path, destination: Path) -> None:
    """Read bounded plain headers into a fresh private staging directory."""
    seen: set[str] = set()
    dirs = {""}
    long_name: str | None = None
    try:
        with wire.open("rb") as stream:
            while (header := stream.read(BLOCK)) != ZERO:
                if len(header) != BLOCK or header[257:265] not in (
                    tarfile.GNU_MAGIC,
                    b"ustar\x0000",
                ):
                    raise RuntimeError("expected a plain directory tar")
                member = tarfile.TarInfo.frombuf(
                    header, sys.getfilesystemencoding(), "surrogateescape"
                )
                if member.type == tarfile.GNUTYPE_LONGNAME:
                    if long_name is not None or not 1 < member.size <= MAX_NAME + 2:
                        raise RuntimeError("invalid directory tar long name")
                    name_bytes = stream.read(member.size)
                    if len(name_bytes) != member.size or not name_bytes.endswith(b"\0"):
                        raise RuntimeError("invalid directory tar long name")
                    long_name = os.fsdecode(name_bytes[:-1])
                    _padding(stream, member.size)
                    continue
                name = long_name if long_name is not None else member.name
                if member.isdir():
                    name = name.removesuffix("/")
                long_name = None
                if (
                    any(part in ("", ".", "..") for part in name.split("/"))
                    or "\0" in name
                    or name.rpartition("/")[0] not in dirs
                    or name in seen
                    or len(seen) >= MAX_MEMBERS
                    or member.linkname
                    or member.type not in (tarfile.REGTYPE, tarfile.AREGTYPE, tarfile.DIRTYPE)
                    or not 0 <= member.size <= MAX_WIRE
                    or (member.isdir() and member.size != 0)
                ):
                    raise RuntimeError(f"unsafe or unsupported directory tar member: {name!r}")
                seen.add(name)
                target = destination / name
                if member.isdir():
                    target.mkdir()
                    dirs.add(name)
                    continue
                with target.open("xb") as dst:
                    remaining = member.size
                    while remaining:
                        chunk = stream.read(min(remaining, 2**20))
                        if not chunk:
                            raise RuntimeError("truncated directory tar member")
                        dst.write(chunk)
                        remaining -= len(chunk)
                _padding(stream, member.size)
            trailer = stream.read(MIN_WIRE + 1)
            if (
                long_name is not None
                or not BLOCK <= len(trailer) <= MIN_WIRE
                or len(trailer) % BLOCK
                or any(trailer)
                or stream.read(1)
            ):
                raise RuntimeError("invalid directory tar trailer")
    except tarfile.TarError as exc:
        raise RuntimeError("invalid directory tar header") from exc


def publish_directory(source: Path, target: Path) -> None:
    """Atomic same-filesystem rename that never replaces an empty directory."""
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
