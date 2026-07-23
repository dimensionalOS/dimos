# Copyright 2025-2026 Dimensional Inc.
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

from collections.abc import Iterator
from contextlib import contextmanager
from datetime import datetime
from functools import cache
import os
from pathlib import Path, PurePosixPath
import platform
import re
import shutil
import sys
import tarfile
import tempfile

from filelock import FileLock, Timeout as FileLockTimeout
from huggingface_hub import hf_hub_download
from huggingface_hub.errors import EntryNotFoundError, LocalEntryNotFoundError

from dimos.constants import DIMOS_PROJECT_ROOT
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

HF_DATASET_REPO_ID = "playercc7/dimensional"
HF_DATASET_DIR = "data"
HF_DATASET_REVISION = "f262d7f8775c2d507b1bfde62a5aa21cffabb3a1"
# Native locks are released on process death, so healthy materializations may
# hold the lock for as long as their download and extraction require.
ASSET_LOCK_TIMEOUT_SECONDS = -1


def _get_user_data_dir() -> Path:
    """Get platform-specific user data directory."""
    system = platform.system()
    if os.environ.get("VIRTUAL_ENV"):
        return Path(
            f"{os.environ['VIRTUAL_ENV']}/lib/python{sys.version_info.major}.{sys.version_info.minor}/site-packages/dimos/data"
        )

    if system == "Linux":
        xdg_data_home = os.environ.get("XDG_DATA_HOME")
        if xdg_data_home:
            return Path(xdg_data_home) / "dimos"
        return Path.home() / ".local" / "share" / "dimos"
    if system == "Darwin":
        return Path.home() / "Library" / "Application Support" / "dimos"
    return Path.home() / ".dimos"


@cache
def get_project_root() -> Path:
    """Return the checkout root or the synthetic installed-package root."""
    if (DIMOS_PROJECT_ROOT / ".git").exists():
        return DIMOS_PROJECT_ROOT

    # Keep the established installed-package path contract without cloning a
    # source repository. The Hub materializer owns only <root>/data.
    try:
        root = _get_user_data_dir() / "repo"
        root.mkdir(parents=True, exist_ok=True)
        test_file = root / ".write_test"
        test_file.touch()
        test_file.unlink()
    except (OSError, PermissionError):
        root = Path(tempfile.gettempdir()) / "dimos" / "repo"
        root.mkdir(parents=True, exist_ok=True)
    logger.info("Using local user data directory at '%s'", root)
    return root


@cache
def get_data_dir(extra_path: str | None = None) -> Path:
    if extra_path:
        return get_project_root() / "data" / extra_path
    return get_project_root() / "data"


def resolve_named_path(name: str | Path, suffix: str = "") -> Path:
    s = str(name)
    p = Path(s)
    if p.is_absolute() or p.exists():
        return p
    if (DIMOS_PROJECT_ROOT / p).exists():
        return DIMOS_PROJECT_ROOT / p
    if suffix and not s.endswith(suffix):
        p = Path(s + suffix)
        if p.is_absolute() or p.exists():
            return p
        if (DIMOS_PROJECT_ROOT / p).exists():
            return DIMOS_PROJECT_ROOT / p
    return get_data(p.name)


def backup_file(path: str | Path, keep_last: int = 3) -> Path | None:
    path = Path(path)
    if not path.exists():
        return None
    ts = datetime.now().strftime("%Y%m%d%H%M%S")
    backup = path.with_name(f"{path.stem}.{ts}{path.suffix}")
    path.rename(backup)
    pattern = re.compile(rf"^{re.escape(path.stem)}\.\d{{14}}{re.escape(path.suffix)}$")
    backups = sorted(
        p for p in path.parent.glob(f"{path.stem}.*{path.suffix}") if pattern.match(p.name)
    )
    for old in backups[:-keep_last] if keep_last > 0 else backups:
        old.unlink()
    return backup if backup.exists() else None


def _validate_data_reference(name: str | Path) -> Path:
    reference = Path(name)
    if (
        "\\" in str(name)
        or reference.is_absolute()
        or not reference.parts
        or any(part in ("", ".", "..") for part in reference.parts)
    ):
        raise ValueError(f"Unsafe data reference {name!r}; expected a relative path under data/")
    candidate = (get_data_dir() / reference).resolve(strict=False)
    try:
        candidate.relative_to(get_data_dir().resolve())
    except ValueError as exc:
        raise ValueError(f"Unsafe data reference {name!r}; expected a path under data/") from exc
    top_level = reference.parts[0]
    if not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9._-]*", top_level):
        raise ValueError(f"Unsafe top-level data asset name {top_level!r}")
    return reference


def _hub_failure(operation: str, name: str | Path, exc: Exception) -> RuntimeError:
    return RuntimeError(
        f"Hugging Face data {operation} failed for {name!r} from "
        f"{HF_DATASET_REPO_ID}@{HF_DATASET_REVISION}: {exc}"
    )


def _archive_member_path(member: tarfile.TarInfo, top_level: str) -> PurePosixPath:
    name = member.name
    if not name or name.startswith("/") or "\\" in name:
        raise RuntimeError(f"Unsafe archive member path {name!r}")
    parts = name.split("/")
    if parts[-1] == "":
        parts.pop()
    if not parts or any(part in ("", ".", "..") for part in parts):
        raise RuntimeError(f"Unsafe archive member path {name!r}")
    path = PurePosixPath(*parts)
    if path.parts[0] != top_level:
        raise RuntimeError(f"Archive member {name!r} has the wrong root")
    return path


def _path_exists_including_broken_symlink(path: Path) -> bool:
    try:
        os.lstat(path)
    except FileNotFoundError:
        return False
    except OSError as exc:
        raise RuntimeError(f"Could not inspect data asset path {path}: {exc}") from exc
    return True


@contextmanager
def _asset_lock(data_dir: Path, top_level: str) -> Iterator[FileLock]:
    """Serialize one immutable asset with a process-safe, persistent lock file."""
    lock = data_dir / f".{top_level}.lock"
    try:
        data_dir.mkdir(parents=True, exist_ok=True)
        lock_handle = FileLock(str(lock))
    except OSError as exc:
        raise RuntimeError(f"Could not prepare data asset lock {lock}: {exc}") from exc
    try:
        lock_handle.acquire(timeout=ASSET_LOCK_TIMEOUT_SECONDS)
    except FileLockTimeout as exc:
        raise RuntimeError(f"Timed out acquiring data asset lock {lock}") from exc
    except OSError as exc:
        raise RuntimeError(f"Could not acquire data asset lock {lock}: {exc}") from exc
    try:
        yield lock_handle
    finally:
        try:
            lock_handle.release()
        except OSError as exc:
            raise RuntimeError(f"Could not release data asset lock {lock}: {exc}") from exc


class _MissingArchiveMemberError(FileNotFoundError):
    """A requested nested member is absent from an otherwise valid archive."""


def _extract_archive(
    archive: Path,
    staging: Path,
    top_level: str,
    requested: tuple[str, ...],
    name: str | Path,
) -> Path:
    staged_top = staging / top_level
    try:
        with tarfile.open(archive, mode="r:gz") as tar:
            members = tar.getmembers()
            validated: list[tuple[tarfile.TarInfo, PurePosixPath]] = []
            seen: set[PurePosixPath] = set()
            root_kind: str | None = None
            for member in members:
                path = _archive_member_path(member, top_level)
                if path in seen:
                    raise RuntimeError(f"Archive contains duplicate member {member.name!r}")
                seen.add(path)
                if member.issym() or member.islnk() or not (member.isdir() or member.isreg()):
                    raise RuntimeError(f"Archive contains unsafe member {member.name!r}")
                if len(path.parts) == 1:
                    kind = "directory" if member.isdir() else "file"
                    if root_kind is not None:
                        raise RuntimeError(
                            f"Archive contains multiple root entries for {top_level!r}"
                        )
                    root_kind = kind
                elif root_kind == "file":
                    raise RuntimeError(f"File root {top_level!r} cannot contain children")
                validated.append((member, path))

            if root_kind is None:
                raise RuntimeError(f"Archive does not contain expected root {top_level!r}")
            if root_kind == "file":
                if any(len(path.parts) > 1 for _, path in validated):
                    raise RuntimeError(f"File root {top_level!r} cannot contain children")
                if requested:
                    raise _MissingArchiveMemberError(f"Nested data member {name!r} was not found")

            directories: list[tuple[tarfile.TarInfo, Path]] = []
            for member, path in sorted(validated, key=lambda item: len(item[1].parts)):
                target = staging / Path(*path.parts)
                if member.isdir():
                    target.mkdir(parents=True, exist_ok=True)
                    directories.append((member, target))
                    continue
                target.parent.mkdir(parents=True, exist_ok=True)
                source = tar.extractfile(member)
                if source is None:
                    raise RuntimeError(f"Could not read archive member {member.name!r}")
                with source, target.open("xb") as output:
                    shutil.copyfileobj(source, output)
                target.chmod(member.mode & 0o777)

            for member, target in reversed(directories):
                target.chmod(member.mode & 0o777)

            requested_path = staged_top.joinpath(*requested)
            if not requested_path.exists():
                raise _MissingArchiveMemberError(f"Nested data member {name!r} was not found")
            return staged_top
    except _MissingArchiveMemberError:
        raise
    except (OSError, tarfile.TarError) as exc:
        raise RuntimeError(f"Could not safely extract archive for {name!r}: {exc}") from exc


def _cleanup_abandoned_staging(data_dir: Path, top_level: str) -> None:
    prefix = f".{top_level}.tmp-"
    try:
        paths = list(data_dir.iterdir())
    except OSError as exc:
        raise RuntimeError(f"Could not enumerate staging under {data_dir}: {exc}") from exc
    for path in paths:
        try:
            if path.name.startswith(prefix) and path.is_dir() and not path.is_symlink():
                shutil.rmtree(path)
        except OSError as exc:
            raise RuntimeError(f"Could not remove abandoned staging {path}: {exc}") from exc


def _create_data_dir_for_write(name: str | Path) -> Path:
    """Create a relative data directory while cooperating with materialization."""
    reference = _validate_data_reference(name)
    data_dir = get_data_dir()
    top_level = reference.parts[0]
    with _asset_lock(data_dir, top_level):
        _cleanup_abandoned_staging(data_dir, top_level)
        target = data_dir / reference
        try:
            target.mkdir(parents=True, exist_ok=True)
        except OSError as exc:
            raise RuntimeError(f"Could not create writable data directory {target}: {exc}") from exc
        return target


def _materialize_archive(
    archive: Path,
    top_level: str,
    requested: tuple[str, ...],
    data_dir: Path,
    name: str | Path,
) -> None:
    staging: Path | None = None
    try:
        try:
            data_dir.mkdir(parents=True, exist_ok=True)
            staging = Path(tempfile.mkdtemp(prefix=f".{top_level}.tmp-", dir=data_dir))
        except OSError as exc:
            raise _hub_failure("materialization", name, exc) from exc
        assert staging is not None
        destination = data_dir / top_level
        _extract_archive(archive, staging, top_level, requested, name)
        if _path_exists_including_broken_symlink(destination):
            requested_path = destination.joinpath(*requested)
            if requested_path.exists():
                return
            raise FileNotFoundError(f"Data asset {name!r} is absent beneath existing {destination}")
        try:
            os.rename(staging / top_level, destination)
        except FileExistsError:
            requested_path = destination.joinpath(*requested)
            if requested_path.exists():
                return
            raise FileNotFoundError(f"Data asset {name!r} is absent beneath existing {destination}")
    except FileNotFoundError:
        raise
    except OSError as exc:
        raise _hub_failure("materialization", name, exc) from exc
    finally:
        if staging is not None:
            shutil.rmtree(staging, ignore_errors=True)


def get_data(name: str | Path) -> Path:
    """Return a local data path, lazily materializing its Hub asset.

    Absolute existing paths are returned unchanged for compatibility with
    callers that pass local recordings. Missing absolute paths raise
    ``FileNotFoundError``. Relative traversal or unsafe references raise
    ``ValueError`` before any network access.
    """
    requested = Path(name)
    if requested.is_absolute():
        if requested.exists():
            return requested
        raise FileNotFoundError(f"Local data path does not exist: {requested}")
    reference = _validate_data_reference(name)
    data_dir = get_data_dir()
    file_path = data_dir / reference
    if file_path.exists():
        return file_path

    top_level = reference.parts[0]
    remote_archive = f"{HF_DATASET_DIR}/{top_level}.tar.gz"
    destination = data_dir / top_level
    with _asset_lock(data_dir, top_level):
        _cleanup_abandoned_staging(data_dir, top_level)
        if file_path.exists():
            return file_path
        if _path_exists_including_broken_symlink(destination):
            raise FileNotFoundError(f"Data asset {name!r} is absent beneath existing {destination}")
        try:
            archive = Path(
                hf_hub_download(
                    repo_id=HF_DATASET_REPO_ID,
                    filename=remote_archive,
                    repo_type="dataset",
                    revision=HF_DATASET_REVISION,
                    token=False,
                )
            )
        except LocalEntryNotFoundError as exc:
            raise _hub_failure("cache download", name, exc) from exc
        except EntryNotFoundError as exc:
            raise FileNotFoundError(
                f"Data asset {name!r} was not found in {HF_DATASET_REPO_ID} at revision "
                f"{HF_DATASET_REVISION}"
            ) from exc
        except Exception as exc:
            raise _hub_failure("download", name, exc) from exc

        _materialize_archive(archive, top_level, reference.parts[1:], data_dir, name)
        if not file_path.exists():
            raise FileNotFoundError(f"Data asset {name!r} could not be materialized under {data_dir}")
        return file_path


class LfsPath(type(Path())):  # type: ignore[misc]
    """A Path subclass that lazily resolves a data reference."""

    def __new__(cls, filename: str | Path) -> "LfsPath":
        instance: LfsPath = super().__new__(cls, ".")
        object.__setattr__(instance, "_lfs_filename", filename)
        object.__setattr__(instance, "_lfs_resolved_cache", None)
        return instance

    def _ensure_downloaded(self) -> Path:
        resolved: Path | None = object.__getattribute__(self, "_lfs_resolved_cache")
        if resolved is None:
            resolved = get_data(object.__getattribute__(self, "_lfs_filename"))
            object.__setattr__(self, "_lfs_resolved_cache", resolved)
        return resolved

    def __getattribute__(self, name: str) -> object:
        try:
            object.__getattribute__(self, "_lfs_filename")
        except AttributeError:
            return object.__getattribute__(self, name)
        if name in ("_lfs_filename", "_lfs_resolved_cache", "_ensure_downloaded"):
            return object.__getattribute__(self, name)
        return getattr(object.__getattribute__(self, "_ensure_downloaded")(), name)

    def __str__(self) -> str:
        return str(self._ensure_downloaded())

    def __fspath__(self) -> str:
        return str(self._ensure_downloaded())

    def __truediv__(self, other: object) -> "LfsPath":
        filename = object.__getattribute__(self, "_lfs_filename")
        return LfsPath(f"{filename}/{other}")

    def __rtruediv__(self, other: object) -> Path:
        return other / self._ensure_downloaded()  # type: ignore[operator]
