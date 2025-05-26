"""
Test utilities for handling large data files via Git LFS.

This module provides utilities for downloading and managing large test data files
stored in Git LFS, with automatic initialization and targeted file pulling.
"""

# trigger change 2

import subprocess
from functools import cache
from pathlib import Path


def _check_git_lfs_available() -> None:
    try:
        subprocess.run(["git", "lfs", "version"], capture_output=True, check=True, text=True)
    except (subprocess.CalledProcessError, FileNotFoundError):
        raise RuntimeError(
            "Git LFS is not installed. Please install git-lfs to use test data utilities.\n"
            "Installation instructions: https://git-lfs.github.io/"
        )
    return True


@cache
def _get_repo_root() -> Path:
    try:
        result = subprocess.run(["git", "rev-parse", "--show-toplevel"], capture_output=True, check=True, text=True)
        return Path(result.stdout.strip())
    except subprocess.CalledProcessError:
        raise RuntimeError("Not in a Git repository")


def _is_lfs_pointer_file(file_path: Path) -> bool:
    if not file_path.exists():
        return False

    try:
        # LFS pointer files are small (typically < 200 bytes) and start with specific text
        if file_path.stat().st_size > 1024:  # LFS pointers are much smaller
            return False

        with open(file_path, "r", encoding="utf-8") as f:
            first_line = f.readline().strip()
            return first_line.startswith("version https://git-lfs.github.com/spec/")
    except (UnicodeDecodeError, OSError):
        return False


def _ensure_lfs_initialized(repo_root: Path) -> None:
    try:
        # Check if LFS is already initialized by looking for .git/hooks/pre-push
        lfs_hook = repo_root / ".git" / "hooks" / "pre-push"
        if lfs_hook.exists():
            return

        # Initialize LFS with --skip-smudge to avoid downloading all files
        subprocess.run(["git", "lfs", "install", "--skip-smudge"], cwd=repo_root, check=True, capture_output=True)
    except subprocess.CalledProcessError as e:
        raise RuntimeError(f"Failed to initialize Git LFS: {e}")


def _pull_lfs_file(file_path: Path, repo_root: Path) -> None:
    try:
        # Convert absolute path to relative path from repo root
        relative_path = file_path.relative_to(repo_root)

        subprocess.run(
            ["git", "lfs", "pull", "--include", str(relative_path)], cwd=repo_root, check=True, capture_output=True
        )
    except subprocess.CalledProcessError as e:
        raise RuntimeError(f"Failed to pull LFS file {file_path}: {e}")


def testFile(filename: str) -> Path:
    """
    Get the path to a test data file, downloading from LFS if needed.

    This function will:
    1. Check that Git LFS is available
    2. Locate the file in the tests/data directory
    3. Initialize Git LFS if needed
    4. Download the file from LFS if it's a pointer file
    5. Return the Path object to the actual file

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
    # Check Git LFS availability first
    _check_git_lfs_available()

    # Find repository root
    repo_root = _get_repo_root()

    # Construct path to test data file
    test_data_dir = repo_root / "tests" / "data"
    file_path = test_data_dir / filename

    # Check if file exists
    if not file_path.exists():
        raise FileNotFoundError(
            f"Test file '{filename}' not found at {file_path}. "
            f"Make sure the file is committed to Git LFS in the tests/data directory."
        )

    # If it's an LFS pointer file, ensure LFS is set up and pull the file
    if _is_lfs_pointer_file(file_path):
        _ensure_lfs_initialized(repo_root)
        _pull_lfs_file(file_path, repo_root)

        # Verify the file was actually downloaded
        if _is_lfs_pointer_file(file_path):
            raise RuntimeError(
                f"Failed to download LFS file '{filename}'. The file is still a pointer after attempting to pull."
            )

    return file_path
