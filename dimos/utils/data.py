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

from functools import cache
import os
from pathlib import Path
import platform
import sys
import tarfile
import tempfile

import boto3
from botocore import UNSIGNED
from botocore.config import Config as BotoConfig
from botocore.exceptions import ClientError

from dimos.constants import DIMOS_PROJECT_ROOT
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

S3_BUCKET = os.environ.get("DIMOS_DATA_S3_BUCKET", "dimos-github-lfs")
S3_PREFIX = os.environ.get("DIMOS_DATA_S3_PREFIX", ".lfs/")
S3_REGION = os.environ.get("DIMOS_DATA_S3_REGION", "us-east-2")


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

    # Running as installed package - use a local data directory
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

    return data_dir


@cache
def get_data_dir(extra_path: str | None = None) -> Path:
    if extra_path:
        return get_project_root() / "data" / extra_path
    return get_project_root() / "data"


@cache
def _get_lfs_dir() -> Path:
    return get_data_dir() / ".lfs"


@cache
def _get_s3_client():  # type: ignore[no-untyped-def]
    """Get a boto3 S3 client, using instance profile / env credentials."""
    try:
        session = boto3.Session(region_name=S3_REGION)
        client = session.client("s3")
        # Quick check that credentials work
        client.head_bucket(Bucket=S3_BUCKET)
        return client
    except ClientError:
        # Fall back to unsigned access (public bucket)
        return boto3.client(
            "s3",
            region_name=S3_REGION,
            config=BotoConfig(signature_version=UNSIGNED),
        )


def _decompress_archive(filename: str | Path) -> Path:
    target_dir = get_data_dir()
    filename_path = Path(filename)
    with tarfile.open(filename_path, "r:gz") as tar:
        tar.extractall(target_dir)
    return target_dir / filename_path.name.replace(".tar.gz", "")


def _pull_s3_archive(filename: str | Path) -> Path:
    """Download an archive from S3 into the local .lfs cache dir."""
    lfs_dir = _get_lfs_dir()
    lfs_dir.mkdir(parents=True, exist_ok=True)

    archive_name = str(filename) + ".tar.gz"
    local_path = lfs_dir / archive_name
    s3_key = S3_PREFIX + archive_name

    if local_path.exists() and local_path.stat().st_size > 1024:
        # Already downloaded (and not an LFS pointer stub)
        return local_path

    logger.info(f"Downloading s3://{S3_BUCKET}/{s3_key} -> {local_path}")
    client = _get_s3_client()
    try:
        client.download_file(S3_BUCKET, s3_key, str(local_path))
    except ClientError as e:
        raise FileNotFoundError(
            f"Data file '{archive_name}' not found in s3://{S3_BUCKET}/{S3_PREFIX}. "
            f"Make sure it has been uploaded with bin/s3_push. ({e})"
        )

    return local_path


def get_data(name: str | Path) -> Path:
    """
    Get the path to test data, downloading from S3 if needed.

    This function will:
    1. Check if the data is already available locally
    2. If not, download the compressed archive from S3
    3. Decompress and return the path

    Supports nested paths like "dataset/subdir/file.jpg" - will download and
    decompress "dataset" archive but return the full nested path.

    Args:
        name: Name of the test file or dir, optionally with nested path
              (e.g., "lidar_sample.bin" or "dataset/frames/001.png")

    Returns:
        Path: Path object to the test file or dir

    Raises:
        FileNotFoundError: If the data file doesn't exist in S3

    Usage:
        # Simple file/dir
        file_path = get_data("sample.bin")

        # Nested path - downloads "dataset" archive, returns path to nested file
        frame = get_data("dataset/frames/001.png")
    """
    data_dir = get_data_dir()
    file_path = data_dir / name

    # already pulled and decompressed, return it directly
    if file_path.exists():
        return file_path

    # extract archive root (first path component) and nested path
    path_parts = Path(name).parts
    archive_name = path_parts[0]
    nested_path = Path(*path_parts[1:]) if len(path_parts) > 1 else None

    # download and decompress the archive root
    archive_path = _decompress_archive(_pull_s3_archive(archive_name))

    # return full path including nested components
    if nested_path:
        return archive_path / nested_path
    return archive_path


class LfsPath(type(Path())):  # type: ignore[misc]
    """
    A Path subclass that lazily downloads data from S3 when accessed.

    This class wraps pathlib.Path and ensures that get_data() is called
    before any meaningful filesystem operation, making data lazy-loaded.

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
        """Ensure the data is downloaded and return the resolved path."""
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
        """Return filesystem path, downloading from S3 if needed."""
        return str(self._ensure_downloaded())

    def __truediv__(self, other: object) -> "LfsPath":
        """Path division operator - returns a new lazy LfsPath (no download)."""
        filename = object.__getattribute__(self, "_lfs_filename")
        return LfsPath(f"{filename}/{other}")

    def __rtruediv__(self, other: object) -> Path:
        """Reverse path division operator."""
        return other / self._ensure_downloaded()  # type: ignore[operator]
