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

from concurrent.futures import ThreadPoolExecutor
import hashlib
from io import BytesIO
import multiprocessing
from pathlib import Path
import tarfile
import time
from typing import Any

from huggingface_hub.errors import EntryNotFoundError
import pytest

from dimos.utils import data
from dimos.utils.data import LfsPath, backup_file


def _make_backups(directory: Path, stem: str, suffix: str, timestamps: list[str]) -> None:
    for timestamp in timestamps:
        (directory / f"{stem}.{timestamp}{suffix}").write_text(timestamp)


def test_backup_file_missing_is_noop(tmp_path: Path) -> None:
    assert backup_file(tmp_path / "nope.db") is None
    assert list(tmp_path.iterdir()) == []


def test_backup_file_renames_with_timestamp(tmp_path: Path) -> None:
    path = tmp_path / "recording_go2.db"
    path.write_text("live")
    backup = backup_file(path)
    assert backup is not None and backup.read_text() == "live"
    assert not path.exists()
    timestamp = backup.name[len("recording_go2.") : -len(".db")]
    assert len(timestamp) == 14 and timestamp.isdigit()


def test_backup_file_prunes_to_keep_last(tmp_path: Path) -> None:
    path = tmp_path / "recording_go2.db"
    _make_backups(
        tmp_path,
        path.stem,
        path.suffix,
        ["20260101010101", "20260101010102", "20260101010103", "20260101010104"],
    )
    path.write_text("live")
    backup_file(path, keep_last=3)
    remaining = sorted(tmp_path.glob("recording_go2.*.db"))
    assert len(remaining) == 3
    assert not (tmp_path / "recording_go2.20260101010101.db").exists()
    assert not (tmp_path / "recording_go2.20260101010102.db").exists()


def test_backup_file_ignores_unrelated_siblings(tmp_path: Path) -> None:
    path = tmp_path / "recording_go2.db"
    (tmp_path / "recording_go2.notes.db").write_text("keep")
    (tmp_path / "other.db").write_text("keep")
    _make_backups(tmp_path, path.stem, path.suffix, ["20260101010101", "20260101010102"])
    path.write_text("live")
    backup_file(path, keep_last=1)
    assert (tmp_path / "recording_go2.notes.db").exists()
    assert (tmp_path / "other.db").exists()


def test_backup_file_keep_last_zero(tmp_path: Path) -> None:
    path = tmp_path / "recording_go2.db"
    _make_backups(tmp_path, path.stem, path.suffix, ["20260101010101"])
    path.write_text("live")
    assert backup_file(path, keep_last=0) is None
    assert not list(tmp_path.glob("recording_go2.*.db"))


def _archive(path: Path, members: list[tuple[str, bytes | None, str | None]]) -> Path:
    with tarfile.open(path, "w:gz") as tar:
        for name, content, kind in members:
            info = tarfile.TarInfo(name)
            if kind == "dir":
                info.type = tarfile.DIRTYPE
                info.mode = 0o755
                tar.addfile(info)
            elif kind == "symlink":
                info.type = tarfile.SYMTYPE
                info.linkname = "target"
                tar.addfile(info)
            elif kind == "hardlink":
                info.type = tarfile.LNKTYPE
                info.linkname = "top/file"
                tar.addfile(info)
            elif kind == "fifo":
                info.type = tarfile.FIFOTYPE
                tar.addfile(info)
            else:
                payload = content or b""
                info.size = len(payload)
                if kind == "executable":
                    info.mode = 0o751
                tar.addfile(info, BytesIO(payload))
    return path


def _hub_test_setup(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> list[dict[str, object]]:
    monkeypatch.setattr(data, "get_data_dir", lambda: tmp_path / "data")
    calls: list[dict[str, object]] = []

    def download(**kwargs: object) -> str:
        calls.append(kwargs)
        return str(tmp_path / "archive.tar.gz")

    monkeypatch.setattr(data, "hf_hub_download", download)
    return calls


def _assert_download(calls: list[dict[str, object]], filename: str) -> None:
    assert calls == [
        {
            "repo_id": "playercc7/dimensional",
            "filename": filename,
            "repo_type": "dataset",
            "revision": data.HF_DATASET_REVISION,
            "token": False,
        }
    ]


def test_get_data_file_archive_and_download_contract(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    _archive(tmp_path / "archive.tar.gz", [("cafe.jpg", b"jpeg", None)])
    calls = _hub_test_setup(tmp_path, monkeypatch)
    result = data.get_data("cafe.jpg")
    assert result.read_bytes() == b"jpeg"
    _assert_download(calls, "data/cafe.jpg.tar.gz")
    assert "local_dir" not in calls[0]


@pytest.mark.self_hosted
def test_get_data_live_occupancy_archive(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    """Exercise the public pinned dataset without using checkout data.

    ``self_hosted`` is the repository's opt-in marker for tests requiring
    network access; the default fast suite excludes it.
    """
    isolated_data_dir = tmp_path / "data"
    monkeypatch.setattr(data, "get_data_dir", lambda: isolated_data_dir)
    calls: list[dict[str, object]] = []
    download = data.hf_hub_download

    def tracked_download(**kwargs: object) -> str:
        live_kwargs = {
            **kwargs,
            "cache_dir": tmp_path / "hub-cache",
            "force_download": True,
        }
        calls.append(live_kwargs)
        return download(**live_kwargs)

    monkeypatch.setattr(data, "hf_hub_download", tracked_download)
    extracted = data.get_data("occupancy_simple.npy")

    assert hashlib.sha256(extracted.read_bytes()).hexdigest() == (
        "e8457964918d6bec7eacc253d1d3573bad878247d7debe2cb6827a61c48bcea3"
    )
    assert [call["filename"] for call in calls] == ["data/occupancy_simple.npy.tar.gz"]
    assert all(call["repo_id"] == data.HF_DATASET_REPO_ID for call in calls)
    assert all(call["revision"] == data.HF_DATASET_REVISION for call in calls)


def test_get_data_directory_archive_and_nested_lookup(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    _archive(
        tmp_path / "archive.tar.gz",
        [
            ("assets", None, "dir"),
            ("assets/frames", None, "dir"),
            ("assets/frames/a.bin", b"a", None),
            ("assets/frames/b.bin", b"b", None),
        ],
    )
    calls = _hub_test_setup(tmp_path, monkeypatch)
    assert data.get_data("assets/frames/a.bin").read_bytes() == b"a"
    _assert_download(calls, "data/assets.tar.gz")
    assert (tmp_path / "data/assets/frames/b.bin").read_bytes() == b"b"


def test_get_data_preserves_sanitized_file_mode(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    _archive(tmp_path / "archive.tar.gz", [("tool", b"#!/bin/sh", "executable")])
    _hub_test_setup(tmp_path, monkeypatch)
    result = data.get_data("tool")
    assert result.read_bytes() == b"#!/bin/sh"
    assert result.stat().st_mode & 0o777 == 0o751


@pytest.mark.parametrize(
    "name",
    [".hidden", "_leading", "-leading", "has*glob", "has?glob", "has[glob]", "é"],
)
def test_get_data_rejects_invalid_top_level_names(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch, name: str
) -> None:
    monkeypatch.setattr(data, "get_data_dir", lambda: tmp_path / "data")
    monkeypatch.setattr(data, "hf_hub_download", lambda **kwargs: pytest.fail("network used"))
    with pytest.raises(ValueError):
        data.get_data(name)


@pytest.mark.parametrize(
    "root_kind", ["file", "empty_dir", "nonempty_dir", "valid_symlink", "broken_symlink"]
)
def test_get_data_existing_root_is_authoritative(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch, root_kind: str
) -> None:
    data_dir = tmp_path / "data"
    data_dir.mkdir(parents=True)
    destination = data_dir / "top"
    if root_kind == "file":
        destination.write_bytes(b"existing")
    elif root_kind == "empty_dir":
        destination.mkdir()
    elif root_kind == "nonempty_dir":
        (destination / "existing").mkdir(parents=True)
    elif root_kind == "valid_symlink":
        (data_dir / "symlink_target").mkdir()
        destination.symlink_to(data_dir / "symlink_target", target_is_directory=True)
    else:
        destination.symlink_to(data_dir / "missing_target", target_is_directory=True)
    monkeypatch.setattr(data, "get_data_dir", lambda: data_dir)
    monkeypatch.setattr(data, "hf_hub_download", lambda **kwargs: pytest.fail("network used"))
    with pytest.raises(FileNotFoundError):
        data.get_data("top/missing")


def test_get_data_cleans_abandoned_staging(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    data_dir = tmp_path / "data"
    abandoned = data_dir / ".top.tmp-abandoned"
    abandoned.mkdir(parents=True)
    (abandoned / "partial").write_bytes(b"partial")
    archive = _archive(tmp_path / "archive.tar.gz", [("top", b"new", None)])
    monkeypatch.setattr(data, "get_data_dir", lambda: data_dir)
    monkeypatch.setattr(data, "hf_hub_download", lambda **kwargs: str(archive))
    assert data.get_data("top").read_bytes() == b"new"
    assert not abandoned.exists()


def test_asset_lock_timeout_maps_to_runtime_error(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    class TimeoutLock:
        def __init__(self, path: str) -> None:
            pass

        def acquire(self, timeout: float) -> Any:
            raise data.FileLockTimeout("busy")

    monkeypatch.setattr(data, "FileLock", TimeoutLock)
    with pytest.raises(RuntimeError, match="Timed out acquiring data asset lock"):
        with data._asset_lock(tmp_path / "data", "top"):
            pass


def test_asset_lock_filesystem_error_maps_to_runtime_error(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    class BrokenLock:
        def __init__(self, path: str) -> None:
            raise OSError("cannot create lock")

    monkeypatch.setattr(data, "FileLock", BrokenLock)
    with pytest.raises(RuntimeError, match="Could not prepare data asset lock"):
        with data._asset_lock(tmp_path / "data", "top"):
            pass


def test_abandoned_staging_enumeration_error_maps_to_runtime_error(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    data_dir = tmp_path / "data"
    data_dir.mkdir()
    iterdir = Path.iterdir

    def fail_iterdir(path: Path) -> Any:
        if path == data_dir:
            raise OSError("cannot enumerate")
        return iterdir(path)

    monkeypatch.setattr(Path, "iterdir", fail_iterdir)
    with pytest.raises(RuntimeError, match="Could not enumerate staging"):
        data._cleanup_abandoned_staging(data_dir, "top")


def test_abandoned_staging_removal_error_maps_to_runtime_error(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    data_dir = tmp_path / "data"
    staging = data_dir / ".top.tmp-abandoned"
    staging.mkdir(parents=True)
    rmtree = data.shutil.rmtree

    def fail_rmtree(path: Path) -> None:
        if path == staging:
            raise OSError("cannot remove")
        rmtree(path)

    monkeypatch.setattr(data.shutil, "rmtree", fail_rmtree)
    with pytest.raises(RuntimeError, match="Could not remove abandoned staging"):
        data._cleanup_abandoned_staging(data_dir, "top")


def test_materialization_data_dir_setup_error_maps_to_runtime_error(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    data_dir = tmp_path / "data"
    archive = _archive(tmp_path / "archive.tar.gz", [("top", b"value", None)])
    monkeypatch.setattr(data, "get_data_dir", lambda: data_dir)
    mkdir = Path.mkdir

    def fail_mkdir(path: Path, *args: Any, **kwargs: Any) -> None:
        if path == data_dir:
            raise OSError("cannot prepare materialization directory")
        mkdir(path, *args, **kwargs)

    monkeypatch.setattr(Path, "mkdir", fail_mkdir)
    with pytest.raises(RuntimeError, match="materialization failed"):
        data._materialize_archive(archive, "top", (), data_dir, "top")


def test_materialization_staging_setup_error_maps_to_runtime_error(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    archive = _archive(tmp_path / "archive.tar.gz", [("top", b"value", None)])
    data_dir = tmp_path / "data"
    monkeypatch.setattr(data, "get_data_dir", lambda: data_dir)
    monkeypatch.setattr(data, "hf_hub_download", lambda **kwargs: str(archive))
    monkeypatch.setattr(
        data.tempfile,
        "mkdtemp",
        lambda **kwargs: (_ for _ in ()).throw(OSError("cannot create staging")),
    )
    with pytest.raises(RuntimeError, match="materialization failed"):
        data.get_data("top")


def _hold_asset_lock(data_dir: str, ready: Any) -> None:
    with data._asset_lock(Path(data_dir), "process"):
        ready.set()
        time.sleep(30)


def test_asset_lock_survives_process_death(tmp_path: Path) -> None:
    data_dir = tmp_path / "data"
    context = multiprocessing.get_context("spawn")
    ready = context.Event()
    process = context.Process(target=_hold_asset_lock, args=(str(data_dir), ready))
    process.start()
    try:
        assert ready.wait(10)
        process.terminate()
        process.join(10)
        assert not process.is_alive()
        with data._asset_lock(data_dir, "process"):
            pass
    finally:
        if process.is_alive():
            process.terminate()
            process.join()


def test_get_data_serializes_repeated_materialization(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    archive = _archive(tmp_path / "archive.tar.gz", [("shared", b"value", None)])
    monkeypatch.setattr(data, "get_data_dir", lambda: tmp_path / "data")
    calls: list[dict[str, object]] = []

    def download(**kwargs: object) -> str:
        calls.append(kwargs)
        return str(archive)

    monkeypatch.setattr(data, "hf_hub_download", download)
    locations: list[Path] = []
    extract = data._extract_archive

    def record_staging(
        archive_path: Path,
        staging: Path,
        top_level: str,
        requested: tuple[str, ...],
        name: str | Path,
    ) -> Path:
        locations.append(staging)
        return extract(archive_path, staging, top_level, requested, name)

    monkeypatch.setattr(data, "_extract_archive", record_staging)
    with ThreadPoolExecutor(max_workers=2) as executor:
        results = list(executor.map(lambda _: data.get_data("shared"), range(2)))
    assert results[0].read_bytes() == b"value"
    assert results[1] == results[0]
    assert len(calls) == 1
    assert locations and all(location.parent == tmp_path / "data" for location in locations)
    assert not list((tmp_path / "data").glob(".shared.tmp-*"))


def test_get_data_local_hit_does_not_use_hub(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    local = tmp_path / "data/cafe.jpg"
    local.parent.mkdir()
    local.write_bytes(b"local")
    monkeypatch.setattr(data, "get_data_dir", lambda: tmp_path / "data")
    monkeypatch.setattr(data, "hf_hub_download", lambda **kwargs: pytest.fail("network used"))
    assert data.get_data("cafe.jpg") == local


def test_get_data_missing_archive_vs_operational_failure(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    monkeypatch.setattr(data, "get_data_dir", lambda: tmp_path / "data")
    monkeypatch.setattr(
        data,
        "hf_hub_download",
        lambda **kwargs: (_ for _ in ()).throw(EntryNotFoundError("missing")),
    )
    with pytest.raises(FileNotFoundError):
        data.get_data("missing.bin")
    monkeypatch.setattr(
        data, "hf_hub_download", lambda **kwargs: (_ for _ in ()).throw(OSError("offline"))
    )
    with pytest.raises(
        RuntimeError, match="playercc7/dimensional.*f262d7f8775c2d507b1bfde62a5aa21cffabb3a1"
    ):
        data.get_data("offline.bin")


@pytest.mark.parametrize(
    "members",
    [
        [("other/file", b"x", None)],
        [("top", b"x", None), ("top/child", b"x", None)],
        [("top", b"x", None), ("other", b"x", None)],
        [("/top/file", b"x", None)],
        [("top/../escape", b"x", None)],
        [("top/link", None, "symlink")],
        [("top/link", None, "hardlink")],
        [("top/pipe", None, "fifo")],
    ],
)
def test_get_data_rejects_unsafe_or_wrong_archives(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
    members: list[tuple[str, bytes | None, str | None]],
) -> None:
    _archive(tmp_path / "archive.tar.gz", members)
    _hub_test_setup(tmp_path, monkeypatch)
    with pytest.raises(RuntimeError):
        data.get_data("top")
    assert not list((tmp_path / "data").glob(".top.tmp-*"))


def test_get_data_existing_root_skips_malformed_remote_archive(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    destination = tmp_path / "data/top/file"
    destination.parent.mkdir(parents=True)
    destination.write_bytes(b"old")
    (tmp_path / "archive.tar.gz").write_bytes(b"not gzip")
    calls = _hub_test_setup(tmp_path, monkeypatch)
    with pytest.raises(FileNotFoundError):
        data.get_data("top/new")
    assert destination.read_bytes() == b"old"
    assert calls == []


def test_get_data_missing_nested_member_preserves_old_destination(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    destination = tmp_path / "data/top/old"
    destination.parent.mkdir(parents=True)
    destination.write_bytes(b"old")
    _archive(tmp_path / "archive.tar.gz", [("top", None, "dir"), ("top/old", b"new", None)])
    _hub_test_setup(tmp_path, monkeypatch)
    with pytest.raises(FileNotFoundError):
        data.get_data("top/missing")
    assert destination.read_bytes() == b"old"


def test_get_data_absolute_paths_and_traversal(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    existing = tmp_path / "recording.pcap"
    existing.write_bytes(b"pcap")
    monkeypatch.setattr(data, "get_data_dir", lambda: tmp_path / "data")
    assert data.get_data(existing) == existing
    with pytest.raises(FileNotFoundError):
        data.get_data(tmp_path / "missing.pcap")
    with pytest.raises(ValueError, match="Unsafe"):
        data.get_data("../secret")


def test_lfs_path_lazy_and_filesystem_operations(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    path = tmp_path / "three_paths.png"
    path.write_bytes(b"\x89PNG data")
    monkeypatch.setattr(data, "get_data", lambda name: path)
    lazy = LfsPath("three_paths.png")
    assert object.__getattribute__(lazy, "_lfs_resolved_cache") is None
    assert lazy.is_file() and not lazy.is_dir()
    assert lazy.read_bytes().startswith(b"\x89PNG")
    assert lazy.absolute().is_absolute() and lazy.resolve().is_absolute()
    assert isinstance(lazy / "subpath", Path)
    other = LfsPath("three_paths.png")
    assert other.read_bytes() == lazy.read_bytes()
    assert object.__getattribute__(other, "_lfs_resolved_cache") == path


def test_lfs_path_unload_and_reload(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    path = tmp_path / "three_paths.png"
    path.write_bytes(b"first")
    monkeypatch.setattr(data, "get_data", lambda name: path)
    assert LfsPath("three_paths.png").read_bytes() == b"first"
    path.unlink()
    path.write_bytes(b"second")
    assert LfsPath("three_paths.png").read_bytes() == b"second"
