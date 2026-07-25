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
import subprocess

import pytest
from pytest_mock import MockerFixture

import dimos.utils.cache as cache_utils


@pytest.fixture
def cache_root(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> Path:
    cache_dir = tmp_path / "cache"
    monkeypatch.setattr(cache_utils, "CACHE_DIR", cache_dir)
    monkeypatch.setattr(cache_utils, "ROBOT_ASSET_CACHE_DIR", cache_dir / "robot_assets")
    return cache_dir


@pytest.fixture
def robot_checkout(cache_root: Path, tmp_path: Path) -> Path:
    return _clone_test_repository(
        tmp_path,
        cache_root / "robot_assets" / "sources" / "source-key" / "robot",
    )


def test_clean_caches_removes_all_known_targets_and_preserves_other_state(
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


def test_clean_caches_preserves_dirty_robot_checkout_but_removes_derived_assets(
    cache_root: Path,
    robot_checkout: Path,
) -> None:
    (robot_checkout / "local.txt").write_text("untracked")
    derived = cache_root / "robot_assets" / "derived" / "robot.urdf"
    derived.parent.mkdir(parents=True)
    derived.write_text("<robot/>")

    result = cache_utils.clean_caches()

    assert not result.complete
    assert result.skipped == [
        cache_utils.CacheIssue(robot_checkout.absolute(), "Git checkout has local changes")
    ]
    assert robot_checkout.exists()
    assert not derived.exists()


def test_clean_caches_preserves_robot_checkout_with_local_only_commit(
    robot_checkout: Path,
) -> None:
    (robot_checkout / "local.txt").write_text("committed locally")
    _git(robot_checkout, "add", "local.txt")
    _git(robot_checkout, "commit", "-m", "local cache work")

    result = cache_utils.clean_caches()

    assert result.skipped == [
        cache_utils.CacheIssue(
            robot_checkout.absolute(),
            "Git checkout has local-only commits",
        )
    ]
    assert robot_checkout.exists()


def test_clean_caches_removes_clean_robot_checkout(
    cache_root: Path,
    robot_checkout: Path,
) -> None:
    result = cache_utils.clean_caches()

    assert result.complete
    assert not robot_checkout.exists()
    assert not cache_root.exists()


def test_clean_caches_force_removes_dirty_robot_checkout(
    cache_root: Path,
    robot_checkout: Path,
) -> None:
    (robot_checkout / "local.txt").write_text("discard me")

    result = cache_utils.clean_caches(force=True)

    assert result.complete
    assert not robot_checkout.exists()
    assert not cache_root.exists()


def test_clean_caches_unlinks_cache_symlink_without_traversing_destination(
    cache_root: Path,
    tmp_path: Path,
) -> None:
    destination = tmp_path / "outside"
    destination.mkdir()
    retained = destination / "retained.txt"
    retained.write_text("keep")
    cache_root.mkdir()
    cached_link = cache_root / "urdf"
    cached_link.symlink_to(destination, target_is_directory=True)

    result = cache_utils.clean_caches()

    assert result.complete
    assert not cached_link.exists()
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


def _clone_test_repository(tmp_path: Path, checkout: Path) -> Path:
    origin = tmp_path / "origin.git"
    seed = tmp_path / "seed"
    _git(tmp_path, "init", "--bare", "--initial-branch=main", str(origin))
    _git(tmp_path, "init", "--initial-branch=main", str(seed))
    _git(seed, "config", "user.email", "cache-test@example.invalid")
    _git(seed, "config", "user.name", "Cache Test")
    (seed / "README.md").write_text("robot assets")
    _git(seed, "add", "README.md")
    _git(seed, "commit", "-m", "initial")
    _git(seed, "remote", "add", "origin", str(origin))
    _git(seed, "push", "-u", "origin", "HEAD")
    checkout.parent.mkdir(parents=True)
    _git(tmp_path, "clone", str(origin), str(checkout))
    _git(checkout, "config", "user.email", "cache-test@example.invalid")
    _git(checkout, "config", "user.name", "Cache Test")
    return checkout


def _git(cwd: Path, *args: str) -> None:
    subprocess.run(
        ["git", "-C", str(cwd), *args],
        check=True,
        capture_output=True,
        text=True,
    )
