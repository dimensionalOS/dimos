# Copyright 2025 Dimensional Inc.
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
import subprocess
import tarfile
import tempfile

from dimos.constants import DIMOS_PROJECT_ROOT
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


def _get_user_data_dir() -> Path:
    """Get platform-specific user data directory."""
    system = platform.system()

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
def _get_repo_root() -> Path:
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
                    # TODO: Use "main",
                    "dev",
                    "git@github.com:dimensionalOS/dimos.git",
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
                f"Make sure you have access to git@github.com:dimensionalOS/dimos.git"
            )

    return repo_dir


@cache
def _get_data_dir(extra_path: str | None = None) -> Path:
    if extra_path:
        return _get_repo_root() / "data" / extra_path
    return _get_repo_root() / "data"


@cache
def _get_lfs_dir() -> Path:
    return _get_data_dir() / ".lfs"


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


def _lfs_pull(file_path: Path, repo_root: Path) -> None:
    try:
        relative_path = file_path.relative_to(repo_root)

        env = os.environ.copy()
        env["GIT_LFS_FORCE_PROGRESS"] = "1"

        subprocess.run(
            ["git", "lfs", "pull", "--include", str(relative_path)],
            cwd=repo_root,
            check=True,
            env=env,
        )
    except subprocess.CalledProcessError as e:
        raise RuntimeError(f"Failed to pull LFS file {file_path}: {e}")

    return None


def _decompress_archive(filename: str | Path) -> Path:
    target_dir = _get_data_dir()
    filename_path = Path(filename)
    with tarfile.open(filename_path, "r:gz") as tar:
        tar.extractall(target_dir)
    return target_dir / filename_path.name.replace(".tar.gz", "")


def _pull_lfs_archive(filename: str | Path) -> Path:
    # Check Git LFS availability first
    _check_git_lfs_available()

    # Find repository root
    repo_root = _get_repo_root()

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


def get_data(filename: str | Path) -> Path:
    """
    Get the path to a test data, downloading from LFS if needed.

    This function will:
    1. Check that Git LFS is available
    2. Locate the file in the tests/data directory
    3. Initialize Git LFS if needed
    4. Download the file from LFS if it's a pointer file
    5. Return the Path object to the actual file or dir

    Args:
        filename: Name of the test file (e.g., "lidar_sample.bin")

    Returns:
        Path: Path object to the test file

    Raises:
        RuntimeError: If Git LFS is not available or LFS operations fail
        FileNotFoundError: If the test file doesn't exist

    Usage:
        # As string path
        file_path = str(testFile("sample.bin"))

        # As context manager for file operations
        with testFile("sample.bin").open('rb') as f:
            data = f.read()
    """
    data_dir = _get_data_dir()
    file_path = data_dir / filename

    # already pulled and decompressed, return it directly
    if file_path.exists():
        return file_path

    return _decompress_archive(_pull_lfs_archive(filename))
