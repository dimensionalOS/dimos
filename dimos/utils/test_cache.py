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

from pathlib import Path

import pytest
from pytest_mock import MockerFixture

import dimos.utils.cache as cache_utils


@pytest.fixture
def cache_root(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> Path:
    cache_dir = tmp_path / "cache"
    monkeypatch.setattr(cache_utils, "CACHE_DIR", cache_dir)
    return cache_dir


def test_clean_caches_removes_cache_root_and_preserves_other_state(
    cache_root: Path,
    tmp_path: Path,
) -> None:
    cache_paths = (
        cache_root / "urdf",
        cache_root / "viser_urdf",
        cache_root / "ament_prefix",
        cache_root / "deno",
    )
    for path in cache_paths:
        path.mkdir(parents=True, exist_ok=True)
        (path / "cached.bin").write_bytes(b"cache")
    recording = tmp_path / "state" / "recordings" / "session.db"
    recording.parent.mkdir(parents=True)
    recording.write_bytes(b"user data")

    result = cache_utils.clean_caches()
    repeated = cache_utils.clean_caches()

    assert result.complete
    assert result.cleaned == [cache_root.absolute()]
    assert all(not path.exists() for path in cache_paths)
    assert recording.read_bytes() == b"user data"
    assert repeated == cache_utils.CacheCleanResult()


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

    assert result.complete
    assert result.cleaned == [cache_root.absolute()]
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

    assert not result.complete
    assert result.failed == [cache_utils.CacheIssue(cache_root.absolute(), "denied")]
    assert cache_root.exists()
    remove_path.assert_called_once_with(cache_root.absolute())
