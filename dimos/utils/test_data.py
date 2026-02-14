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

import hashlib
import os
from pathlib import Path
import subprocess

import pytest

from dimos.utils import data
from dimos.utils.data import LfsPath


@pytest.mark.heavy
def test_pull_file() -> None:
    repo_root = data._get_repo_root()
    test_file_name = "cafe.jpg"
    test_file_compressed = data._get_lfs_dir() / (test_file_name + ".tar.gz")
    test_file_decompressed = data.get_data_dir() / test_file_name

    # delete decompressed test file if it exists
    if test_file_decompressed.exists():
        test_file_decompressed.unlink()

    # delete lfs archive file if it exists
    if test_file_compressed.exists():
        test_file_compressed.unlink()

    assert not test_file_compressed.exists()
    assert not test_file_decompressed.exists()

    # pull the lfs file reference from git
    env = os.environ.copy()
    env["GIT_LFS_SKIP_SMUDGE"] = "1"
    subprocess.run(
        ["git", "checkout", "HEAD", "--", test_file_compressed],
        cwd=repo_root,
        env=env,
        check=True,
        capture_output=True,
    )

    # ensure we have a pointer file from git (small ASCII text file)
    assert test_file_compressed.exists()
    assert test_file_compressed.stat().st_size < 200

    # trigger a data file pull
    assert data.get_data(test_file_name) == test_file_decompressed

    # validate data is received
    assert test_file_compressed.exists()
    assert test_file_decompressed.exists()

    # validate hashes
    with test_file_compressed.open("rb") as f:
        assert test_file_compressed.stat().st_size > 200
        compressed_sha256 = hashlib.sha256(f.read()).hexdigest()
        assert (
            compressed_sha256 == "b8cf30439b41033ccb04b09b9fc8388d18fb544d55b85c155dbf85700b9e7603"
        )

    with test_file_decompressed.open("rb") as f:
        decompressed_sha256 = hashlib.sha256(f.read()).hexdigest()
        assert (
            decompressed_sha256
            == "55d451dde49b05e3ad386fdd4ae9e9378884b8905bff1ca8aaea7d039ff42ddd"
        )


@pytest.mark.heavy
def test_pull_dir() -> None:
    repo_root = data._get_repo_root()
    test_dir_name = "ab_lidar_frames"
    test_dir_compressed = data._get_lfs_dir() / (test_dir_name + ".tar.gz")
    test_dir_decompressed = data.get_data_dir() / test_dir_name

    # delete decompressed test directory if it exists
    if test_dir_decompressed.exists():
        for item in test_dir_decompressed.iterdir():
            item.unlink()
        test_dir_decompressed.rmdir()

    # delete lfs archive file if it exists
    if test_dir_compressed.exists():
        test_dir_compressed.unlink()

    # pull the lfs file reference from git
    env = os.environ.copy()
    env["GIT_LFS_SKIP_SMUDGE"] = "1"
    subprocess.run(
        ["git", "checkout", "HEAD", "--", test_dir_compressed],
        cwd=repo_root,
        env=env,
        check=True,
        capture_output=True,
    )

    # ensure we have a pointer file from git (small ASCII text file)
    assert test_dir_compressed.exists()
    assert test_dir_compressed.stat().st_size < 200

    # trigger a data file pull
    assert data.get_data(test_dir_name) == test_dir_decompressed
    assert test_dir_compressed.stat().st_size > 200

    # validate data is received
    assert test_dir_compressed.exists()
    assert test_dir_decompressed.exists()

    for [file, expected_hash] in zip(
        sorted(test_dir_decompressed.iterdir()),
        [
            "6c3aaa9a79853ea4a7453c7db22820980ceb55035777f7460d05a0fa77b3b1b3",
            "456cc2c23f4ffa713b4e0c0d97143c27e48bbe6ef44341197b31ce84b3650e74",
        ],
        strict=False,
    ):
        with file.open("rb") as f:
            sha256 = hashlib.sha256(f.read()).hexdigest()
            assert sha256 == expected_hash


# ============================================================================
# LfsPath Tests
# ============================================================================


def test_lfs_path_lazy_creation() -> None:
    """Test that creating LfsPath doesn't trigger download."""
    lfs_path = LfsPath("test_data_file")

    # Check that the object is created
    assert isinstance(lfs_path, LfsPath)

    # Check that cache is None (not downloaded yet)
    cache = object.__getattribute__(lfs_path, "_lfs_resolved_cache")
    assert cache is None

    # Check that filename is stored
    filename = object.__getattribute__(lfs_path, "_lfs_filename")
    assert filename == "test_data_file"


def test_lfs_path_safe_attributes() -> None:
    """Test that safe attributes don't trigger download."""
    lfs_path = LfsPath("test_data_file")

    # Access safe attributes directly
    filename = object.__getattribute__(lfs_path, "_lfs_filename")
    cache = object.__getattribute__(lfs_path, "_lfs_resolved_cache")
    ensure_fn = object.__getattribute__(lfs_path, "_ensure_downloaded")

    # Verify they exist and cache is still None
    assert filename == "test_data_file"
    assert cache is None
    assert callable(ensure_fn)


def test_lfs_path_no_download_on_creation() -> None:
    """Test that LfsPath construction doesn't trigger download.

    Path(lfs_path) extracts internal _raw_paths (\".\") and does NOT
    call __fspath__, so it won't trigger download. The correct way to
    convert is Path(str(lfs_path)), which triggers __str__ -> download.
    """
    lfs_path = LfsPath("nonexistent_file")

    # Construction should not trigger download
    cache = object.__getattribute__(lfs_path, "_lfs_resolved_cache")
    assert cache is None

    # Accessing internal LfsPath attributes should not trigger download
    filename = object.__getattribute__(lfs_path, "_lfs_filename")
    assert filename == "nonexistent_file"
    assert cache is None


@pytest.mark.heavy
def test_lfs_path_with_real_file() -> None:
    """Test LfsPath with a real small LFS file."""
    # Use a small existing LFS file
    filename = "three_paths.png"
    lfs_path = LfsPath(filename)

    # Initially, cache should be None
    cache = object.__getattribute__(lfs_path, "_lfs_resolved_cache")
    assert cache is None

    # Access a Path method - this should trigger download
    exists = lfs_path.exists()

    # Now cache should be populated
    cache = object.__getattribute__(lfs_path, "_lfs_resolved_cache")
    assert cache is not None
    assert isinstance(cache, Path)

    # File should exist after download
    assert exists is True

    # Should be able to get file stats
    stat_result = lfs_path.stat()
    assert stat_result.st_size > 0

    # Should be able to read the file
    content = lfs_path.read_bytes()
    assert len(content) > 0

    # Verify it's a PNG file
    assert content.startswith(b"\x89PNG")


@pytest.mark.heavy
def test_lfs_path_unload_and_reload() -> None:
    """Test unloading and reloading an LFS file."""
    filename = "three_paths.png"
    data_dir = data.get_data_dir()
    file_path = data_dir / filename

    # Clean up if file already exists
    if file_path.exists():
        file_path.unlink()

    # Create LfsPath
    lfs_path = LfsPath(filename)

    # Verify file doesn't exist yet
    assert not file_path.exists()

    # Access the file - this triggers download
    content_first = lfs_path.read_bytes()
    assert file_path.exists()

    # Get hash of first download
    hash_first = hashlib.sha256(content_first).hexdigest()

    # Now unload (delete the file)
    file_path.unlink()
    assert not file_path.exists()

    # Create a new LfsPath instance for the same file
    lfs_path_2 = LfsPath(filename)

    # Access the file again - should re-download
    content_second = lfs_path_2.read_bytes()
    assert file_path.exists()

    # Get hash of second download
    hash_second = hashlib.sha256(content_second).hexdigest()

    # Hashes should match (same file downloaded)
    assert hash_first == hash_second

    # Content should be identical
    assert content_first == content_second


@pytest.mark.heavy
def test_lfs_path_operations() -> None:
    """Test various Path operations with LfsPath."""
    filename = "three_paths.png"
    lfs_path = LfsPath(filename)

    # Test is_file
    assert lfs_path.is_file() is True
    assert lfs_path.is_dir() is False

    # Test absolute path
    abs_path = lfs_path.absolute()
    assert abs_path.is_absolute()

    # Test resolve
    resolved = lfs_path.resolve()
    assert resolved.is_absolute()

    # Test string conversion
    path_str = str(lfs_path)
    assert isinstance(path_str, str)
    assert filename in path_str

    # Test __fspath__
    fspath_result = os.fspath(lfs_path)
    assert isinstance(fspath_result, str)
    assert filename in fspath_result


@pytest.mark.heavy
def test_lfs_path_division_operator() -> None:
    """Test path division operator with LfsPath."""
    # Use a directory for testing
    lfs_path = LfsPath("three_paths.png")

    # Test truediv - this should trigger download and return resolved path
    result = lfs_path / "subpath"
    assert isinstance(result, Path)

    # The result should be the resolved path with subpath appended
    assert "three_paths.png" in str(result)


@pytest.mark.heavy
def test_lfs_path_multiple_instances() -> None:
    """Test that multiple LfsPath instances for same file work correctly."""
    filename = "three_paths.png"

    # Create two separate instances
    lfs_path_1 = LfsPath(filename)
    lfs_path_2 = LfsPath(filename)

    # Both should start with None cache
    cache_1 = object.__getattribute__(lfs_path_1, "_lfs_resolved_cache")
    cache_2 = object.__getattribute__(lfs_path_2, "_lfs_resolved_cache")
    assert cache_1 is None
    assert cache_2 is None

    # Access file through first instance
    content_1 = lfs_path_1.read_bytes()

    # First instance should have cache
    cache_1 = object.__getattribute__(lfs_path_1, "_lfs_resolved_cache")
    assert cache_1 is not None

    # Second instance cache should still be None (separate instance)
    cache_2 = object.__getattribute__(lfs_path_2, "_lfs_resolved_cache")
    assert cache_2 is None

    # Access through second instance
    content_2 = lfs_path_2.read_bytes()

    # Now second instance should also have cache
    cache_2 = object.__getattribute__(lfs_path_2, "_lfs_resolved_cache")
    assert cache_2 is not None

    # Content should be the same
    assert content_1 == content_2

    # Both caches should point to the same file
    assert cache_1 == cache_2
