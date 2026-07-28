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

from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
import threading

import pytest
from pytest_mock import MockerFixture

import dimos.utils.cache as cache_utils


@pytest.fixture
def cache_root(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> Path:
    cache_dir = tmp_path / "cache"
    monkeypatch.setattr(cache_utils, "CACHE_DIR", cache_dir)
    monkeypatch.setattr(cache_utils, "_CACHE_LOCK_DIR", tmp_path / "state" / "cache-users")
    monkeypatch.setattr(cache_utils, "_CACHE_GATE_PATH", tmp_path / "state" / "cache-clean.lock")
    return cache_dir


def test_clean_caches_removes_only_cache_root(
    cache_root: Path,
    tmp_path: Path,
) -> None:
    cached_file = cache_root / "entry" / "cached.bin"
    cached_file.parent.mkdir(parents=True)
    cached_file.write_bytes(b"cache")
    outside_file = tmp_path / "state" / "retained.bin"
    outside_file.parent.mkdir()
    outside_file.write_bytes(b"state")

    result = cache_utils.clean_caches()

    assert result == cache_utils.CacheCleanResult(cleaned=[cache_root.absolute()])
    assert not cache_root.exists()
    assert outside_file.read_bytes() == b"state"


def test_clean_caches_missing_root_is_noop(cache_root: Path) -> None:
    result = cache_utils.clean_caches()

    assert result == cache_utils.CacheCleanResult()


def test_clean_caches_unlinks_root_symlink_without_traversing_destination(
    cache_root: Path,
    tmp_path: Path,
) -> None:
    destination = tmp_path / "outside"
    destination.mkdir()
    retained = destination / "retained.txt"
    retained.write_text("keep")
    cache_root.symlink_to(destination, target_is_directory=True)

    result = cache_utils.clean_caches()

    assert result == cache_utils.CacheCleanResult(cleaned=[cache_root.absolute()])
    assert not cache_root.exists()
    assert retained.read_text() == "keep"


def test_clean_caches_reports_deletion_failure(
    cache_root: Path,
    mocker: MockerFixture,
) -> None:
    cache_root.mkdir()
    remove_path = mocker.patch.object(
        cache_utils,
        "_remove_path",
        side_effect=PermissionError("denied"),
    )

    result = cache_utils.clean_caches()

    assert result == cache_utils.CacheCleanResult(
        failed=[cache_utils.CacheIssue(cache_root.absolute(), "denied")]
    )
    assert cache_root.exists()
    remove_path.assert_called_once_with(cache_root.absolute())


def test_cleanup_guard_rejects_active_cache_user(cache_root: Path) -> None:
    with cache_utils.cache_usage_guard():
        with pytest.raises(cache_utils.CacheInUseError, match="DimOS caches are in use"):
            with cache_utils.cache_cleanup_guard():
                pass


def test_cleanup_guard_gates_new_cache_user(cache_root: Path) -> None:
    usage_started = threading.Event()

    def use_cache() -> None:
        with cache_utils.cache_usage_guard():
            usage_started.set()

    with ThreadPoolExecutor(max_workers=1) as executor:
        with cache_utils.cache_cleanup_guard():
            usage = executor.submit(use_cache)
            assert not usage_started.wait(timeout=0.05)

        assert usage_started.wait(timeout=1)
        usage.result(timeout=1)
