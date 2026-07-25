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

import dimos.utils.cache as cache_utils


@pytest.fixture
def cache_paths(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> tuple[Path, ...]:
    cache_dir = tmp_path / "cache"
    paths = (
        cache_dir,
        tmp_path / "drake",
        tmp_path / "viser",
        tmp_path / "ament",
        tmp_path / "state" / "deno",
    )
    monkeypatch.setattr(cache_utils, "CACHE_DIR", paths[0])
    monkeypatch.setattr(cache_utils, "ROBOT_ASSET_CACHE_DIR", paths[0] / "robot_assets")
    monkeypatch.setattr(cache_utils, "DRAKE_URDF_CACHE_DIR", paths[1])
    monkeypatch.setattr(cache_utils, "VISER_URDF_CACHE_DIR", paths[2])
    monkeypatch.setattr(cache_utils, "AMENT_PREFIX_CACHE_DIR", paths[3])
    monkeypatch.setattr(cache_utils, "DENO_CACHE_DIR", paths[4])
    return paths


def test_clean_caches_removes_all_known_targets_and_preserves_other_state(
    cache_paths: tuple[Path, ...],
    tmp_path: Path,
) -> None:
    for path in cache_paths:
        path.mkdir(parents=True)
        (path / "cached.bin").write_bytes(b"cache")
    recording = tmp_path / "state" / "recordings" / "session.db"
    recording.parent.mkdir(parents=True)
    recording.write_bytes(b"user data")

    result = cache_utils.clean_caches()

    assert result.complete
    assert set(result.cleaned) == {path.absolute() for path in cache_paths}
    assert all(not path.exists() for path in cache_paths)
    assert recording.read_bytes() == b"user data"


def test_clean_caches_is_an_idempotent_noop(cache_paths: tuple[Path, ...]) -> None:
    first = cache_utils.clean_caches()
    second = cache_utils.clean_caches()

    assert first == cache_utils.CacheCleanResult()
    assert second == cache_utils.CacheCleanResult()


def test_clean_caches_preserves_dirty_robot_checkout_but_removes_derived_assets(
    cache_paths: tuple[Path, ...],
    tmp_path: Path,
) -> None:
    checkout = _clone_test_repository(
        tmp_path,
        cache_paths[0] / "robot_assets" / "sources" / "source-key" / "robot",
    )
    (checkout / "local.txt").write_text("untracked")
    derived = cache_paths[0] / "robot_assets" / "derived" / "robot.urdf"
    derived.parent.mkdir(parents=True)
    derived.write_text("<robot/>")

    result = cache_utils.clean_caches()

    assert not result.complete
    assert result.skipped == [
        cache_utils.CacheIssue(checkout.absolute(), "Git checkout has local changes")
    ]
    assert checkout.exists()
    assert not derived.exists()


def test_clean_caches_preserves_robot_checkout_with_local_only_commit(
    cache_paths: tuple[Path, ...],
    tmp_path: Path,
) -> None:
    checkout = _clone_test_repository(
        tmp_path,
        cache_paths[0] / "robot_assets" / "sources" / "source-key" / "robot",
    )
    (checkout / "local.txt").write_text("committed locally")
    _git(checkout, "add", "local.txt")
    _git(checkout, "commit", "-m", "local cache work")

    result = cache_utils.clean_caches()

    assert result.skipped == [
        cache_utils.CacheIssue(checkout.absolute(), "Git checkout has local-only commits")
    ]
    assert checkout.exists()


def test_clean_caches_removes_clean_robot_checkout(
    cache_paths: tuple[Path, ...],
    tmp_path: Path,
) -> None:
    checkout = _clone_test_repository(
        tmp_path,
        cache_paths[0] / "robot_assets" / "sources" / "source-key" / "robot",
    )

    result = cache_utils.clean_caches()

    assert result.complete
    assert not checkout.exists()
    assert not cache_paths[0].exists()


def test_clean_caches_force_removes_dirty_robot_checkout(
    cache_paths: tuple[Path, ...],
    tmp_path: Path,
) -> None:
    checkout = _clone_test_repository(
        tmp_path,
        cache_paths[0] / "robot_assets" / "sources" / "source-key" / "robot",
    )
    (checkout / "local.txt").write_text("discard me")

    result = cache_utils.clean_caches(force=True)

    assert result.complete
    assert not checkout.exists()
    assert not cache_paths[0].exists()


def test_clean_caches_unlinks_cache_symlink_without_traversing_destination(
    cache_paths: tuple[Path, ...],
    tmp_path: Path,
) -> None:
    destination = tmp_path / "outside"
    destination.mkdir()
    retained = destination / "retained.txt"
    retained.write_text("keep")
    cache_paths[1].symlink_to(destination, target_is_directory=True)

    result = cache_utils.clean_caches()

    assert result.complete
    assert not cache_paths[1].exists()
    assert retained.read_text() == "keep"


def test_clean_caches_continues_after_one_target_fails(
    cache_paths: tuple[Path, ...],
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    for path in (cache_paths[0], cache_paths[4]):
        path.mkdir(parents=True)
    original_remove_path = cache_utils._remove_path

    def fail_for_deno(path: Path) -> None:
        if path == cache_paths[4].absolute():
            raise PermissionError("denied")
        original_remove_path(path)

    monkeypatch.setattr(cache_utils, "_remove_path", fail_for_deno)

    result = cache_utils.clean_caches()

    assert not result.complete
    assert result.failed == [cache_utils.CacheIssue(cache_paths[4].absolute(), "denied")]
    assert not cache_paths[0].exists()
    assert cache_paths[4].exists()


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
