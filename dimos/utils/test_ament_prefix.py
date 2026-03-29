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

"""Tests for the fake ament prefix utility."""

from __future__ import annotations

import os
from pathlib import Path

import pytest

from dimos.utils import ament_prefix
from dimos.utils.ament_prefix import ensure_ament_packages


@pytest.fixture(autouse=True)
def _isolate_ament_state(monkeypatch: pytest.MonkeyPatch, tmp_path: Path) -> None:
    """Reset module state and use a temp prefix dir for each test."""
    monkeypatch.setattr(ament_prefix, "_prefix_dir", None)
    monkeypatch.setattr(ament_prefix, "_registered", {})
    # Use a unique temp dir per test so tests don't interfere
    monkeypatch.setattr(
        ament_prefix,
        "_get_prefix_dir",
        lambda: tmp_path / "dimos_ament_prefix",
    )
    # Clear AMENT_PREFIX_PATH so tests start clean
    monkeypatch.delenv("AMENT_PREFIX_PATH", raising=False)


def test_creates_directory_structure(tmp_path: Path) -> None:
    pkg_dir = tmp_path / "my_robot_description"
    pkg_dir.mkdir()

    ensure_ament_packages({"my_robot": pkg_dir})

    prefix = tmp_path / "dimos_ament_prefix"
    marker = prefix / "share" / "ament_index" / "resource_index" / "packages" / "my_robot"
    assert marker.exists()

    share_link = prefix / "share" / "my_robot"
    assert share_link.is_symlink()
    assert share_link.resolve() == pkg_dir.resolve()


def test_sets_ament_prefix_path(tmp_path: Path) -> None:
    pkg_dir = tmp_path / "pkg"
    pkg_dir.mkdir()

    ensure_ament_packages({"pkg": pkg_dir})

    prefix = tmp_path / "dimos_ament_prefix"
    assert str(prefix) in os.environ["AMENT_PREFIX_PATH"]


def test_idempotent(tmp_path: Path) -> None:
    pkg_dir = tmp_path / "pkg"
    pkg_dir.mkdir()

    ensure_ament_packages({"pkg": pkg_dir})
    ensure_ament_packages({"pkg": pkg_dir})

    # AMENT_PREFIX_PATH should not have duplicates
    paths = os.environ["AMENT_PREFIX_PATH"].split(os.pathsep)
    prefix_str = str(tmp_path / "dimos_ament_prefix")
    assert paths.count(prefix_str) == 1


def test_updates_symlink_on_target_change(tmp_path: Path) -> None:
    old_dir = tmp_path / "old"
    old_dir.mkdir()
    new_dir = tmp_path / "new"
    new_dir.mkdir()

    ensure_ament_packages({"pkg": old_dir})
    # Force re-registration by clearing internal cache
    ament_prefix._registered.clear()
    ensure_ament_packages({"pkg": new_dir})

    prefix = tmp_path / "dimos_ament_prefix"
    share_link = prefix / "share" / "pkg"
    assert share_link.resolve() == new_dir.resolve()


def test_empty_dict_is_noop() -> None:
    ensure_ament_packages({})
    assert "AMENT_PREFIX_PATH" not in os.environ or os.environ.get("AMENT_PREFIX_PATH") == ""


def test_preserves_existing_ament_prefix_path(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    monkeypatch.setenv("AMENT_PREFIX_PATH", "/opt/ros/humble")

    pkg_dir = tmp_path / "pkg"
    pkg_dir.mkdir()
    ensure_ament_packages({"pkg": pkg_dir})

    paths = os.environ["AMENT_PREFIX_PATH"].split(os.pathsep)
    assert "/opt/ros/humble" in paths
    prefix_str = str(tmp_path / "dimos_ament_prefix")
    assert prefix_str in paths
    # Our prefix should come first
    assert paths.index(prefix_str) < paths.index("/opt/ros/humble")


def test_ament_index_resolves(tmp_path: Path) -> None:
    """Verify ament_index_python can find the package after setup."""
    pkg_dir = tmp_path / "test_pkg_data"
    pkg_dir.mkdir()

    ensure_ament_packages({"test_pkg_data": pkg_dir})

    from ament_index_python.packages import get_package_share_directory

    resolved = get_package_share_directory("test_pkg_data")
    assert Path(resolved).resolve() == pkg_dir.resolve()
