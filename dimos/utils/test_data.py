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

from collections.abc import Callable
from concurrent.futures import ThreadPoolExecutor
from dataclasses import dataclass
import hashlib
import io
import json
import os
from pathlib import Path
import subprocess
import tarfile

import pytest

from dimos.utils import data
from dimos.utils.data import LfsPath, backup_file


def _make_backups(dir_path: Path, stem: str, suffix: str, timestamps: list[str]) -> None:
    for ts in timestamps:
        (dir_path / f"{stem}.{ts}{suffix}").write_text(ts)


def test_backup_file_missing_is_noop(tmp_path: Path) -> None:
    assert backup_file(tmp_path / "nope.db") is None
    assert list(tmp_path.iterdir()) == []


def test_backup_file_renames_with_timestamp(tmp_path: Path) -> None:
    db = tmp_path / "recording_go2.db"
    db.write_text("live")

    backup = backup_file(db)

    assert backup is not None
    assert not db.exists()
    assert backup.exists()
    assert backup.read_text() == "live"
    # name is "<stem>.<14-digit timestamp><suffix>"
    assert backup.parent == tmp_path
    assert backup.suffix == ".db"
    middle = backup.name[len("recording_go2.") : -len(".db")]
    assert len(middle) == 14 and middle.isdigit()


def test_backup_file_prunes_to_keep_last(tmp_path: Path) -> None:
    db = tmp_path / "recording_go2.db"
    # four pre-existing backups, oldest first
    _make_backups(
        tmp_path,
        "recording_go2",
        ".db",
        ["20260101010101", "20260101010102", "20260101010103", "20260101010104"],
    )
    db.write_text("live")

    backup_file(db, keep_last=3)

    remaining = sorted(p.name for p in tmp_path.glob("recording_go2.*.db"))
    # two oldest pruned; two newest pre-existing + the just-created one == 3
    assert len(remaining) == 3
    assert "recording_go2.20260101010101.db" not in remaining
    assert "recording_go2.20260101010102.db" not in remaining
    assert "recording_go2.20260101010103.db" in remaining
    assert "recording_go2.20260101010104.db" in remaining


def test_backup_file_ignores_non_timestamp_siblings(tmp_path: Path) -> None:
    db = tmp_path / "recording_go2.db"
    decoy = tmp_path / "recording_go2.notes.db"  # not a 14-digit timestamp
    other = tmp_path / "other.db"
    decoy.write_text("keep me")
    other.write_text("unrelated")
    _make_backups(tmp_path, "recording_go2", ".db", ["20260101010101", "20260101010102"])
    db.write_text("live")

    backup_file(db, keep_last=1)

    # only real backups are pruned; decoy and unrelated files survive
    assert decoy.exists()
    assert other.exists()
    ts_backups = sorted(p.name for p in tmp_path.glob("recording_go2.*.db") if p.name != decoy.name)
    assert len(ts_backups) == 1


def test_backup_file_keep_last_zero_removes_all(tmp_path: Path) -> None:
    db = tmp_path / "recording_go2.db"
    _make_backups(tmp_path, "recording_go2", ".db", ["20260101010101"])
    db.write_text("live")

    assert backup_file(db, keep_last=0) is None

    assert list(tmp_path.glob("recording_go2.*.db")) == []


@dataclass
class DataLayout:
    data_dir: Path
    lfs_dir: Path
    state_dir: Path
    staging_root: Path


@pytest.fixture
def data_layout(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> DataLayout:
    layout = DataLayout(
        data_dir=tmp_path / "data",
        lfs_dir=tmp_path / "data" / ".lfs",
        state_dir=tmp_path / "state",
        staging_root=tmp_path / "staging",
    )
    layout.data_dir.mkdir()
    layout.lfs_dir.mkdir()
    layout.staging_root.mkdir()

    monkeypatch.setattr(
        data,
        "get_data_dir",
        lambda extra_path=None: layout.data_dir / extra_path if extra_path else layout.data_dir,
    )
    monkeypatch.setattr(data, "_get_lfs_dir", lambda: layout.lfs_dir)
    monkeypatch.setattr(data, "_lfs_extraction_state_dir", lambda: layout.state_dir)
    monkeypatch.setattr(data, "_check_git_lfs_available", lambda: True)
    return layout


def _write_tar_gz(tar_path: Path, archive_name: str, content: bytes, staging_root: Path) -> None:
    staging = staging_root / archive_name
    staging.mkdir(exist_ok=True)
    (staging / "payload.bin").write_bytes(content)
    with tarfile.open(tar_path, "w:gz") as tar:
        tar.add(staging, arcname=archive_name)


def _write_single_file_tar_gz(tar_path: Path, member_name: str, content: bytes) -> None:
    with tarfile.open(tar_path, "w:gz") as tar:
        info = tarfile.TarInfo(member_name)
        info.size = len(content)
        tar.addfile(info, io.BytesIO(content))


def _counting_decompress(
    monkeypatch: pytest.MonkeyPatch,
) -> Callable[[], int]:
    """Wrap _decompress_archive and return its running call count."""
    call_count = 0
    original_decompress = data._decompress_archive

    def counting_decompress(filename: str | Path) -> Path:
        nonlocal call_count
        call_count += 1
        return original_decompress(filename)

    monkeypatch.setattr(data, "_decompress_archive", counting_decompress)
    return lambda: call_count


def test_get_data_self_heals_legacy_extraction_without_stamp(data_layout: DataLayout) -> None:
    """No stamp file means not trusted: re-extracts once, then self-heals."""
    tar_path = data._lfs_archive_path("dataset")
    _write_tar_gz(tar_path, "dataset", b"current", data_layout.staging_root)

    # archive and extraction both present, but no stamp
    (data_layout.data_dir / "dataset").mkdir()
    (data_layout.data_dir / "dataset" / "payload.bin").write_bytes(b"stale")

    healed = data.get_data("dataset/payload.bin")
    assert healed.read_bytes() == b"current"
    assert data._extraction_stamp_path("dataset").exists()


def test_get_data_returns_local_recording_without_backing_archive(
    data_layout: DataLayout, monkeypatch: pytest.MonkeyPatch
) -> None:
    local_db = data_layout.data_dir / "recording.db"
    local_db.write_bytes(b"local only, no archive behind it")

    def fail_pull(name: str) -> Path:
        raise AssertionError("must not attempt an LFS pull for a local-only recording")

    monkeypatch.setattr(data, "_pull_lfs_archive", fail_pull)

    assert data.get_data("recording.db") == local_db


def test_get_data_reextracts_when_archive_changes_then_stabilizes(
    data_layout: DataLayout, monkeypatch: pytest.MonkeyPatch
) -> None:
    tar_path = data._lfs_archive_path("dataset")
    _write_tar_gz(tar_path, "dataset", b"v1", data_layout.staging_root)

    first = data.get_data("dataset/payload.bin")
    assert first.read_bytes() == b"v1"

    call_count = _counting_decompress(monkeypatch)

    # unchanged archive: no re-extraction
    data.get_data("dataset/payload.bin")
    assert call_count() == 0

    # archive re-tarred under the same name with different content
    _write_tar_gz(
        tar_path, "dataset", b"v2, a longer payload than before", data_layout.staging_root
    )

    second = data.get_data("dataset/payload.bin")
    assert second.read_bytes() == b"v2, a longer payload than before"
    assert call_count() == 1

    # settles back to no re-extraction once the stamp matches again
    data.get_data("dataset/payload.bin")
    assert call_count() == 1


def test_get_data_reextraction_drops_members_removed_from_archive(data_layout: DataLayout) -> None:
    """Members dropped from a re-tarred archive must not survive on disk."""
    staging = data_layout.staging_root / "dataset"
    staging.mkdir(parents=True)
    tar_path = data._lfs_archive_path("dataset")

    (staging / "keep.bin").write_bytes(b"kept")
    (staging / "drop.bin").write_bytes(b"dropped")
    with tarfile.open(tar_path, "w:gz") as tar:
        tar.add(staging, arcname="dataset")

    dropped_path = data.get_data("dataset/drop.bin")
    assert dropped_path.exists()

    # re-tar without drop.bin
    (staging / "drop.bin").unlink()
    with tarfile.open(tar_path, "w:gz") as tar:
        tar.add(staging, arcname="dataset")

    kept_path = data.get_data("dataset/keep.bin")
    assert kept_path.read_bytes() == b"kept"
    assert not dropped_path.exists()


def test_get_data_mispackaged_archive_does_not_destroy_prior_extraction(
    data_layout: DataLayout,
) -> None:
    """A stale-triggering archive missing its top-level member must raise
    without deleting the still-good previous extraction."""
    tar_path = data._lfs_archive_path("dataset")
    _write_tar_gz(tar_path, "dataset", b"v1", data_layout.staging_root)

    first = data.get_data("dataset/payload.bin")
    assert first.read_bytes() == b"v1"

    # re-tar under the wrong top-level name, so extraction succeeds but the
    # expected "dataset" member never lands in the staging dir
    with tarfile.open(tar_path, "w:gz") as tar:
        info = tarfile.TarInfo("wrong_name/payload.bin")
        payload = b"v2"
        info.size = len(payload)
        tar.addfile(info, io.BytesIO(payload))

    with pytest.raises(RuntimeError, match="dataset"):
        data.get_data("dataset/payload.bin")

    assert first.exists()
    assert first.read_bytes() == b"v1"


def test_get_data_reextracts_when_stamp_is_corrupt(data_layout: DataLayout) -> None:
    tar_path = data._lfs_archive_path("dataset")
    _write_tar_gz(tar_path, "dataset", b"current", data_layout.staging_root)

    stamp_path = data._extraction_stamp_path("dataset")
    stamp_path.parent.mkdir(parents=True, exist_ok=True)
    stamp_path.write_text("not valid json")
    (data_layout.data_dir / "dataset").mkdir()
    (data_layout.data_dir / "dataset" / "payload.bin").write_bytes(b"stale")

    result = data.get_data("dataset/payload.bin")

    assert result.read_bytes() == b"current"
    assert json.loads(stamp_path.read_text())


def test_get_data_reextracts_single_file_archive(data_layout: DataLayout) -> None:
    """A prior extraction that is a bare file, not a directory, must also be replaced."""
    tar_path = data._lfs_archive_path("cafe.jpg")
    _write_single_file_tar_gz(tar_path, "cafe.jpg", b"v1")

    first = data.get_data("cafe.jpg")
    assert first.is_file()
    assert first.read_bytes() == b"v1"

    _write_single_file_tar_gz(tar_path, "cafe.jpg", b"v2, a longer payload than before")

    second = data.get_data("cafe.jpg")
    assert second.read_bytes() == b"v2, a longer payload than before"


def test_get_data_serializes_concurrent_extraction_of_same_archive(
    data_layout: DataLayout, monkeypatch: pytest.MonkeyPatch
) -> None:
    """Concurrent callers must not race: only one extraction happens."""
    tar_path = data._lfs_archive_path("dataset")
    _write_tar_gz(tar_path, "dataset", b"v1", data_layout.staging_root)

    call_count = _counting_decompress(monkeypatch)

    with ThreadPoolExecutor(max_workers=8) as executor:
        results = list(executor.map(lambda _: data.get_data("dataset/payload.bin"), range(8)))

    assert all(result.read_bytes() == b"v1" for result in results)
    assert call_count() == 1


@pytest.mark.self_hosted
def test_pull_file() -> None:
    repo_root = data.get_project_root()
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


@pytest.mark.self_hosted
def test_pull_dir() -> None:
    repo_root = data.get_project_root()
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


@pytest.mark.self_hosted
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


def test_lfs_path_division_operator() -> None:
    """Test path division operator with LfsPath."""
    # Use a directory for testing
    lfs_path = LfsPath("three_paths.png")

    # Test truediv - this should trigger download and return resolved path
    result = lfs_path / "subpath"
    assert isinstance(result, Path)

    # The result should be the resolved path with subpath appended
    assert "three_paths.png" in str(result)


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
