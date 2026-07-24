"""Descriptor-pinned filesystem topology for one Pi runtime."""

from __future__ import annotations

from collections.abc import Iterator
from contextlib import contextmanager
from dataclasses import dataclass
import os
from pathlib import Path
import stat


class TopologyError(ValueError):
    """Raised when a runtime root is unsafe or overlaps another root."""


@contextmanager
def fresh_directory_cursor(fd: int) -> Iterator[int]:
    """Yield an independent, descriptor-anchored directory enumeration FD."""
    cursor_fd = os.open(
        ".",
        os.O_RDONLY | os.O_DIRECTORY | os.O_NOFOLLOW | os.O_CLOEXEC,
        dir_fd=fd,
    )
    try:
        yield cursor_fd
    finally:
        os.close(cursor_fd)


@dataclass
class PinnedDirectory:
    """A directory pinned by an O_NOFOLLOW descriptor and inode identity."""

    path: Path
    fd: int
    device: int
    inode: int
    ancestors: frozenset[tuple[int, int]] = frozenset()

    @classmethod
    def open(cls, path: Path, *, create: bool = False) -> PinnedDirectory:
        path = Path(path)
        fd, ancestors = _walk_directory(path, create=create)
        info = os.fstat(fd)
        if not stat.S_ISDIR(info.st_mode):
            os.close(fd)
            raise TopologyError(f"runtime root is not a real directory: {path}")
        current = os.fstat(fd)
        return cls(path, fd, current.st_dev, current.st_ino, frozenset(ancestors))

    def mkdir(self, name: str) -> None:
        """Create one child without following a replaced parent or child."""
        if "/" in name or name in ("", ".", ".."):
            raise TopologyError("invalid descriptor-relative child name")
        try:
            os.mkdir(name, mode=0o700, dir_fd=self.fd)
        except FileExistsError:
            child = os.open(name, os.O_RDONLY | os.O_DIRECTORY | os.O_NOFOLLOW, dir_fd=self.fd)
            try:
                info = os.fstat(child)
                if (
                    not stat.S_ISDIR(info.st_mode)
                    or info.st_uid != os.geteuid()
                    or stat.S_IMODE(info.st_mode) != 0o700
                ):
                    raise TopologyError(f"owned child is not a private directory: {name}")
            finally:
                os.close(child)

    def write_bytes(self, name: str, data: bytes) -> None:
        if "/" in name or name in ("", ".", ".."):
            raise TopologyError("invalid descriptor-relative file name")
        fd = os.open(
            name, os.O_WRONLY | os.O_CREAT | os.O_TRUNC | os.O_NOFOLLOW, 0o600, dir_fd=self.fd
        )
        try:
            view = memoryview(data)
            while view:
                view = view[os.write(fd, view) :]
        finally:
            os.close(fd)

    def verify(self) -> None:
        current = os.fstat(self.fd)
        if (current.st_dev, current.st_ino) != (self.device, self.inode):
            raise TopologyError(f"pinned runtime root was replaced: {self.path}")

    def read_bytes(self, name: str) -> bytes:
        _validate_child_name(name)
        fd = os.open(name, os.O_RDONLY | os.O_NOFOLLOW, dir_fd=self.fd)
        try:
            info = os.fstat(fd)
            if not stat.S_ISREG(info.st_mode):
                raise TopologyError(f"owned child is not a regular file: {name}")
            return os.read(fd, info.st_size)
        finally:
            os.close(fd)

    def open_relative(self, relative: str) -> PinnedDirectory:
        """Pin a directory below this descriptor without resolving a pathname."""
        parts = _validate_relative_path(relative)
        current = self
        opened: list[PinnedDirectory] = []
        try:
            for part in parts:
                current = PinnedDirectory.open_at(current, part)
                opened.append(current)
            if not opened:
                return self
            result = opened[-1]
            for item in opened[:-1]:
                item.close()
            return result
        except Exception:
            for item in opened:
                item.close()
            raise

    def read_relative(self, relative: str) -> bytes:
        parts = _validate_relative_path(relative)
        if len(parts) == 1:
            return self.read_bytes(parts[0])
        parent = self.open_relative("/".join(parts[:-1]))
        try:
            return parent.read_bytes(parts[-1])
        finally:
            parent.close()

    def write_relative(self, relative: str, data: bytes) -> None:
        parts = _validate_relative_path(relative)
        if len(parts) == 1:
            self.write_bytes(parts[0], data)
            return
        parent = self.open_relative("/".join(parts[:-1]))
        try:
            parent.write_bytes(parts[-1], data)
        finally:
            parent.close()

    def close(self) -> None:
        if self.fd >= 0:
            os.close(self.fd)
            self.fd = -1

    def __truediv__(self, name: str) -> Path:
        """Expose the descriptive path for legacy adapters, never for access."""
        return self.path / name

    @classmethod
    def open_at(cls, parent: PinnedDirectory, name: str) -> PinnedDirectory:
        if not name or "/" in name or name in (".", ".."):
            raise TopologyError("invalid descriptor-relative directory name")
        try:
            fd = os.open(name, os.O_RDONLY | os.O_DIRECTORY | os.O_NOFOLLOW, dir_fd=parent.fd)
        except OSError as error:
            raise TopologyError(f"could not pin runtime root: {name}") from error
        info = os.fstat(fd)
        return cls(
            parent.path / name,
            fd,
            info.st_dev,
            info.st_ino,
            parent.ancestors | {(info.st_dev, info.st_ino)},
        )

    def __enter__(self) -> PinnedDirectory:
        return self

    def __exit__(self, *_: object) -> None:
        self.close()


@dataclass
class PinnedRuntimeTopology:
    """Trusted staged input plus owned writable workspace/output/private leaves."""

    input: PinnedDirectory
    workspace: PinnedDirectory
    output: PinnedDirectory
    private: PinnedDirectory

    def verify(self) -> None:
        for directory in (self.input, self.workspace, self.output, self.private):
            directory.verify()

    def close(self) -> None:
        for directory in (self.input, self.workspace, self.output, self.private):
            directory.close()


def pin_runtime_topology(
    *,
    input_dir: Path | PinnedDirectory,
    workspace_dir: Path | PinnedDirectory,
    output_dir: Path | PinnedDirectory,
    private_dir: Path | PinnedDirectory,
) -> PinnedRuntimeTopology:
    values = (input_dir, workspace_dir, output_dir, private_dir)
    pinned: list[PinnedDirectory] = []
    try:
        # The walk, rather than Path.resolve()/lexical comparison, is the trust
        # boundary.  It rejects symlinked ancestors and records every component.
        for value in values:
            pinned.append(
                value
                if isinstance(value, PinnedDirectory)
                else PinnedDirectory.open(value, create=True)
            )
        identities = {(item.device, item.inode) for item in pinned}
        if len(identities) != len(pinned):
            raise TopologyError("runtime roots must have distinct directory identities")
        for index in range(len(pinned)):
            for other in range(index + 1, len(pinned)):
                if (pinned[index].device, pinned[index].inode) in pinned[other].ancestors or (
                    pinned[other].device,
                    pinned[other].inode,
                ) in pinned[index].ancestors:
                    raise TopologyError("runtime roots must be distinct and non-overlapping")
        return PinnedRuntimeTopology(*pinned)
    except Exception:
        for item in pinned:
            item.close()
        raise


def _walk_directory(path: Path, *, create: bool) -> tuple[int, set[tuple[int, int]]]:
    raw = str(path)
    absolute = path.absolute()
    current = os.open(
        "/" if absolute.is_absolute() else ".", os.O_RDONLY | os.O_DIRECTORY | os.O_NOFOLLOW
    )
    identities: set[tuple[int, int]] = {(os.fstat(current).st_dev, os.fstat(current).st_ino)}
    try:
        components = absolute.parts[1:] if absolute.is_absolute() else absolute.parts
        for component in components:
            try:
                child = os.open(
                    component,
                    os.O_RDONLY | os.O_DIRECTORY | os.O_NOFOLLOW,
                    dir_fd=current,
                )
            except FileNotFoundError:
                if not create:
                    raise TopologyError(f"runtime root does not exist: {raw}")
                os.mkdir(component, dir_fd=current)
                child = os.open(
                    component,
                    os.O_RDONLY | os.O_DIRECTORY | os.O_NOFOLLOW,
                    dir_fd=current,
                )
            os.close(current)
            current = child
            info = os.fstat(current)
            identities.add((info.st_dev, info.st_ino))
        return current, identities
    except Exception as error:
        os.close(current)
        if isinstance(error, TopologyError):
            raise
        raise TopologyError(f"could not pin runtime root: {raw}") from error


def _validate_child_name(name: str) -> None:
    if "/" in name or name in ("", ".", ".."):
        raise TopologyError("invalid descriptor-relative child name")


def _validate_relative_path(relative: str) -> tuple[str, ...]:
    parts = tuple(relative.split("/"))
    if not parts or any(part in ("", ".", "..") for part in parts):
        raise TopologyError("invalid descriptor-relative path")
    return parts
