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

from datetime import datetime
from functools import cache
import hashlib
import json
import os
from pathlib import Path
import platform
import re
import shutil
import subprocess
import sys
import tarfile
import tempfile
import time
from uuid import uuid4

from dimos.constants import DIMOS_PROJECT_ROOT
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


def _get_user_data_dir() -> Path:
    """Get platform-specific user data directory."""
    system = platform.system()
    # if virtual env is available, use it to keep venv's from fighting over data
    # a better fix for large files will be added later to minimize storage duplication
    if os.environ.get("VIRTUAL_ENV"):
        venv_data_dir = Path(
            f"{os.environ.get('VIRTUAL_ENV')}/lib/python{sys.version_info.major}.{sys.version_info.minor}/site-packages/dimos/data"
        )
        return venv_data_dir

    if system == "Linux":
        # Use XDG_DATA_HOME if set, otherwise default to ~/.local/share
        xdg_data_home = os.environ.get("XDG_DATA_HOME")
        if xdg_data_home:
            return Path(xdg_data_home) / "dimos"
        return Path.home() / ".local" / "share" / "dimos"
    elif system == "Darwin":  # macOS
        return Path.home() / "Library" / "Application Support" / "dimos"
    else:
        # Fallback for other systems
        return Path.home() / ".dimos"


@cache
def get_project_root() -> Path:
    # Check if running from git repo
    if (DIMOS_PROJECT_ROOT / ".git").exists():
        return DIMOS_PROJECT_ROOT

    # Running as installed package - clone repo to data dir
    try:
        data_dir = _get_user_data_dir()
        data_dir.mkdir(parents=True, exist_ok=True)
        # Test if writable
        test_file = data_dir / ".write_test"
        test_file.touch()
        test_file.unlink()
        logger.info(f"Using local user data directory at '{data_dir}'")
    except (OSError, PermissionError):
        # Fall back to temp dir if data dir not writable
        data_dir = Path(tempfile.gettempdir()) / "dimos"
        data_dir.mkdir(parents=True, exist_ok=True)
        logger.info(f"Using tmp data directory at '{data_dir}'")

    repo_dir = data_dir / "repo"

    # Clone if not already cloned
    if not (repo_dir / ".git").exists():
        try:
            env = os.environ.copy()
            env["GIT_LFS_SKIP_SMUDGE"] = "1"
            subprocess.run(
                [
                    "git",
                    "clone",
                    "--depth",
                    "1",
                    "--branch",
                    "main",
                    "https://github.com/dimensionalOS/dimos.git",
                    str(repo_dir),
                ],
                check=True,
                capture_output=True,
                text=True,
                env=env,
            )
        except subprocess.CalledProcessError as e:
            raise RuntimeError(
                f"Failed to clone dimos repository: {e.stderr}\n"
                f"Make sure you can access https://github.com/dimensionalOS/dimos.git"
            )

    return repo_dir


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


@cache
def _get_lfs_dir() -> Path:
    return get_data_dir() / ".lfs"


def _check_git_lfs_available() -> bool:
    missing = []

    # Check if git is available
    try:
        subprocess.run(["git", "--version"], capture_output=True, check=True, text=True)
    except (subprocess.CalledProcessError, FileNotFoundError):
        missing.append("git")

    # Check if git-lfs is available
    try:
        subprocess.run(["git-lfs", "version"], capture_output=True, check=True, text=True)
    except (subprocess.CalledProcessError, FileNotFoundError):
        missing.append("git-lfs")

    if missing:
        raise RuntimeError(
            f"Missing required tools: {', '.join(missing)}.\n\n"
            "Git LFS installation instructions: https://git-lfs.github.io/"
        )

    return True


def _is_lfs_pointer_file(file_path: Path) -> bool:
    try:
        # LFS pointer files are small (typically < 200 bytes) and start with specific text
        if file_path.stat().st_size > 1024:  # LFS pointers are much smaller
            return False

        with open(file_path, encoding="utf-8") as f:
            first_line = f.readline().strip()
            return first_line.startswith("version https://git-lfs.github.com/spec/")

    except (UnicodeDecodeError, OSError):
        return False


def _lfs_pull(file_path: Path, repo_root: Path, *, retries: int = 2) -> None:
    relative_path = file_path.relative_to(repo_root)

    env = os.environ.copy()
    env["GIT_LFS_FORCE_PROGRESS"] = "1"

    last_err: subprocess.CalledProcessError | None = None
    for attempt in range(1, retries + 2):  # retries + 1 total attempts
        try:
            subprocess.run(
                ["git", "lfs", "pull", "--include", str(relative_path)],
                cwd=repo_root,
                check=True,
                env=env,
            )
            return
        except subprocess.CalledProcessError as e:
            last_err = e
            if attempt <= retries:
                time.sleep(attempt)  # 1s, 2s backoff

    raise RuntimeError(
        f"Failed to pull LFS file {file_path} after {retries + 1} attempts: {last_err}"
    )


def _decompress_archive(filename: str | Path) -> Path:
    target_dir = get_data_dir()
    filename_path = Path(filename)
    with tarfile.open(filename_path, "r:gz") as tar:
        tar.extractall(target_dir)
    return target_dir / filename_path.name.replace(".tar.gz", "")


def _pull_lfs_archive(filename: str | Path) -> Path:
    # Check Git LFS availability first
    _check_git_lfs_available()

    # Find repository root
    repo_root = get_project_root()

    # Construct path to test data file
    file_path = _get_lfs_dir() / (str(filename) + ".tar.gz")

    # Check if file exists
    if not file_path.exists():
        raise FileNotFoundError(
            f"Test file '{filename}' not found at {file_path}. "
            f"Make sure the file is committed to Git LFS in the tests/data directory."
        )

    # If it's an LFS pointer file, ensure LFS is set up and pull the file
    if _is_lfs_pointer_file(file_path):
        _lfs_pull(file_path, repo_root)

        # Verify the file was actually downloaded
        if _is_lfs_pointer_file(file_path):
            raise RuntimeError(
                f"Failed to download LFS file '{filename}'. The file is still a pointer after attempting to pull."
            )

    return file_path


def _get_archive_metadata_path(extracted_path: Path) -> Path:
    """Return the metadata path for an extracted file or directory."""
    METADATA_FILENAME = ".archive_metadata.json"
    if extracted_path.is_dir():
        return extracted_path / METADATA_FILENAME

    return extracted_path.parent / (f".{extracted_path.name}.archive_metadata.json")


def _calculate_md5(file_path: Path, chunk_size: int = 1024 * 1024) -> str:
    digest = hashlib.md5()

    with file_path.open("rb") as file:
        for chunk in iter(lambda: file.read(chunk_size), b""):
            digest.update(chunk)

    return digest.hexdigest()


def _write_archive_md5(extracted_path: Path, md5_str: str) -> None:
    """Write the archive MD5 checksum to a metadata JSON file in the extracted directory."""

    metadata_path = _get_archive_metadata_path(extracted_path)

    metadata = {"archive_md5": md5_str}

    # Write the metadata as UTF-8 encoded JSON.
    metadata_path.write_text(
        json.dumps(metadata, indent=2),
        encoding="utf-8",
    )


def _read_archive_checksum(extracted_path: Path) -> str | None:
    """Read the archive MD5 checksum from the metadata JSON file in the extracted directory."""

    metadata_path = _get_archive_metadata_path(extracted_path)

    if not metadata_path.exists():
        return None

    try:
        metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
        return metadata.get("archive_md5")
    except (
        FileNotFoundError,
        NotADirectoryError,
        json.JSONDecodeError,
        OSError,
    ):
        return None


def get_data(name: str | Path) -> Path:
    """
    Get the path to a test data, downloading from LFS if needed.

    This function will:
    1. Check that Git LFS is available
    2. Locate the file in the tests/data directory
    3. Initialize Git LFS if needed
    4. Download the file from LFS if it's a pointer file
    5. Return the Path object to the actual file or dir

    Supports nested paths like "dataset/subdir/file.jpg" - will download and
    decompress "dataset" archive but return the full nested path.

    Args:
        name: Name of the test file or dir, optionally with nested path
              (e.g., "lidar_sample.bin" or "dataset/frames/001.png")

    Returns:
        Path: Path object to the test file or dir

    Raises:
        RuntimeError: If Git LFS is not available or LFS operations fail
        FileNotFoundError: If the test file doesn't exist

    Usage:
        # Simple file/dir
        file_path = get_data("sample.bin")

        # Nested path - downloads "dataset" archive, returns path to nested file
        frame = get_data("dataset/frames/001.png")
    """

    data_dir = get_data_dir()
    file_path = data_dir / name

    # Extract the archive root from the first path component and preserve any remaining components as the nested path.
    path_parts = Path(name).parts
    archive_name = path_parts[0]
    nested_path = Path(*path_parts[1:]) if len(path_parts) > 1 else None

    # Path to the first-level extracted file or directory.
    extracted_path = data_dir / archive_name

    # Path to the corresponding Git LFS archive.
    archive_file = _get_lfs_dir() / f"{archive_name}.tar.gz"

    # If the requested path already exists, compare the archive and extracted
    # root modification times to determine whether the extraction is stale.
    if file_path.exists():
        archive_is_newer = False
        archive_checksum = None

        # If the archive is missing, preserve and return the existing usable
        # extracted data instead of raising FileNotFoundError from stat().
        if archive_file.exists():
            if extracted_path.exists():
                # calculate archive_file md5
                archive_checksum = _calculate_md5(archive_file)

                # read md5 from extracted_path if reading fail return None and decompress again.
                extracted_checksum = _read_archive_checksum(extracted_path)

                # compare md5
                archive_is_newer = archive_checksum != extracted_checksum

            else:
                archive_is_newer = True

        if archive_is_newer:
            logger.warning(
                "Replacing stale extracted data at %s. A backup will be created first.",
                extracted_path,
            )

            pull_path = _pull_lfs_archive(archive_name)

            # Extract the updated archive into a temporary staging directory so
            # the existing extraction remains usable if extraction fails.
            staging_dir = Path(
                tempfile.mkdtemp(
                    dir=data_dir,
                    prefix=f"{archive_name}._staging_",
                )
            )

            # Use a unique backup path to avoid conflicts with stale backup
            # directories from previous interrupted refresh operations.
            backup_path = data_dir / f"{archive_name}._backup_{uuid4().hex}"

            try:
                with tarfile.open(pull_path, "r:gz") as tar:
                    tar.extractall(staging_dir)

                # Archives are expected to contain a top-level directory whose
                # name matches archive_name.
                new_extracted = staging_dir / archive_name

                if not new_extracted.exists():
                    raise FileNotFoundError(
                        f"Archive does not contain expected root: {archive_name}"
                    )

                # Preserve the current extraction as a backup before replacing
                # it with the newly extracted data.
                extracted_path.rename(backup_path)

                try:
                    # Move the new extraction into the final location.
                    new_extracted.rename(extracted_path)
                except Exception:
                    # Restore the previous extraction if replacement fails.
                    backup_path.rename(extracted_path)
                    raise

                # The replacement succeeded, so the old extraction is no
                # longer needed.
                if backup_path.is_dir() and not backup_path.is_symlink():
                    shutil.rmtree(backup_path)
                elif backup_path.exists():
                    backup_path.unlink()

                # write new md5 to extracted_path if succeed.
                _write_archive_md5(extracted_path, archive_checksum)

                # Return the requested nested path or the extracted root.
                result_path = extracted_path / nested_path if nested_path else extracted_path

                return result_path

            finally:
                # Remove any remaining staging data after success or failure.
                shutil.rmtree(staging_dir, ignore_errors=True)

        # The archive is unchanged or unavailable, so return the existing extracted data.
        return file_path

    # pull archive file first.
    pull_path = _pull_lfs_archive(archive_name)
    # calculate archive_file md5
    archive_checksum = _calculate_md5(archive_file)

    # decompress archive files to produce extract directory
    archive_path = _decompress_archive(pull_path)

    # write current archive_file md5 to extract directory
    _write_archive_md5(extracted_path, archive_checksum)

    # Return the requested nested path or the extracted archive root.
    if nested_path:
        return archive_path / nested_path

    return archive_path


class LfsPath(type(Path())):  # type: ignore[misc]
    """
    A Path subclass that lazily downloads LFS data when accessed.

    This is useful for both lazy loading and differentiating between LFS paths and regular paths.

    This class wraps pathlib.Path and ensures that get_data() is called
    before any meaningful filesystem operation, making LFS data lazy-loaded.

    Usage:
        path = LfsPath("sample_data")
        # No download yet

        with path.open('rb') as f:  # Downloads now if needed
            data = f.read()

        # Or use any Path operation:
        if path.exists():  # Downloads now if needed
            files = list(path.iterdir())
    """

    def __new__(cls, filename: str | Path) -> "LfsPath":
        # Create instance with a placeholder path to satisfy Path.__new__
        # We use "." as a dummy path that always exists
        instance: LfsPath = super().__new__(cls, ".")
        # Store the actual filename as an instance attribute
        object.__setattr__(instance, "_lfs_filename", filename)
        object.__setattr__(instance, "_lfs_resolved_cache", None)
        return instance

    def _ensure_downloaded(self) -> Path:
        """Ensure the LFS data is downloaded and return the resolved path."""
        cache: Path | None = object.__getattribute__(self, "_lfs_resolved_cache")
        if cache is None:
            filename = object.__getattribute__(self, "_lfs_filename")
            cache = get_data(filename)
            object.__setattr__(self, "_lfs_resolved_cache", cache)
        return cache

    def __getattribute__(self, name: str) -> object:
        # During Path.__new__(), _lfs_filename hasn't been set yet.
        # Fall through to normal Path behavior until construction is complete.
        try:
            object.__getattribute__(self, "_lfs_filename")
        except AttributeError:
            return object.__getattribute__(self, name)

        # After construction, allow access to our internal attributes directly
        if name in ("_lfs_filename", "_lfs_resolved_cache", "_ensure_downloaded"):
            return object.__getattribute__(self, name)

        # For all other attributes, ensure download first then delegate to resolved path
        resolved = object.__getattribute__(self, "_ensure_downloaded")()
        return getattr(resolved, name)

    def __str__(self) -> str:
        """String representation returns resolved path."""
        return str(self._ensure_downloaded())

    def __fspath__(self) -> str:
        """Return filesystem path, downloading from LFS if needed."""
        return str(self._ensure_downloaded())

    def __truediv__(self, other: object) -> "LfsPath":
        """Path division operator - returns a new lazy LfsPath (no download)."""
        filename = object.__getattribute__(self, "_lfs_filename")
        return LfsPath(f"{filename}/{other}")

    def __rtruediv__(self, other: object) -> Path:
        """Reverse path division operator."""
        return other / self._ensure_downloaded()  # type: ignore[operator]
