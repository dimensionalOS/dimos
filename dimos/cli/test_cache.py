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

import contextlib
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import MagicMock

import pytest
from pytest_mock import MockerFixture
from typer.testing import CliRunner

import dimos.cli.cache as cache_cli
from dimos.cli.dimos import main
from dimos.utils.cache import CacheCleanResult, CacheInUseError, CacheIssue


@pytest.fixture
def cache_dir(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> Path:
    path = tmp_path / "cache"
    monkeypatch.setattr(cache_cli, "CACHE_DIR", path)
    return path


@pytest.fixture
def cleanup_guard(mocker: MockerFixture) -> MagicMock:
    return mocker.patch.object(
        cache_cli,
        "cache_cleanup_guard",
        side_effect=contextlib.nullcontext,
    )


def test_clean_refuses_active_run(
    cache_dir: Path,
    mocker: MockerFixture,
) -> None:
    cache_dir.mkdir()
    mocker.patch.object(
        cache_cli,
        "get_most_recent",
        return_value=SimpleNamespace(run_id="active-run"),
    )
    clean_caches = mocker.patch.object(cache_cli, "clean_caches")

    result = CliRunner().invoke(main, ["cache", "clean", "--yes"])

    assert result.exit_code == 1
    assert result.output == (
        "Error: DimOS run active-run is active. Stop it before cleaning caches.\n"
    )
    clean_caches.assert_not_called()


def test_clean_previews_current_entries_and_defaults_confirmation_to_no(
    cache_dir: Path,
    mocker: MockerFixture,
) -> None:
    deno_dir = cache_dir / "deno"
    deno_dir.mkdir(parents=True)
    unknown_entry = cache_dir / "custom.bin"
    unknown_entry.write_bytes(b"cache")
    mocker.patch.object(cache_cli, "get_most_recent", return_value=None)
    clean_caches = mocker.patch.object(cache_cli, "clean_caches")

    result = CliRunner().invoke(main, ["cache", "clean"], input="\n")

    assert result.exit_code == 0
    assert result.output == (
        "DimOS cache cleanup\n"
        "\n"
        "Cache root:\n"
        f"  {cache_dir}\n"
        "\n"
        "Current entries:\n"
        f"  - {unknown_entry}\n"
        f"  - {deno_dir}\n"
        "\n"
        "Continue? [y/N]: \n"
        "Cache cleanup cancelled.\n"
    )
    clean_caches.assert_not_called()


def test_clean_refuses_run_started_during_confirmation(
    cache_dir: Path,
    cleanup_guard: MagicMock,
    mocker: MockerFixture,
) -> None:
    cache_dir.mkdir()
    mocker.patch.object(
        cache_cli,
        "get_most_recent",
        side_effect=[None, SimpleNamespace(run_id="new-run")],
    )
    clean_caches = mocker.patch.object(cache_cli, "clean_caches")

    result = CliRunner().invoke(main, ["cache", "clean"], input="y\n")

    assert result.exit_code == 1
    assert result.output.endswith(
        "Error: DimOS run new-run is active. Stop it before cleaning caches.\n"
    )
    cleanup_guard.assert_called_once_with()
    clean_caches.assert_not_called()


def test_clean_refuses_cache_in_use_during_final_guard(
    cache_dir: Path,
    mocker: MockerFixture,
) -> None:
    cache_dir.mkdir()
    mocker.patch.object(cache_cli, "get_most_recent", return_value=None)
    mocker.patch.object(
        cache_cli,
        "cache_cleanup_guard",
        side_effect=CacheInUseError,
    )
    clean_caches = mocker.patch.object(cache_cli, "clean_caches")

    result = CliRunner().invoke(main, ["cache", "clean", "--yes"])

    assert result.exit_code == 1
    assert result.output.endswith(
        "Error: DimOS is starting or running. Stop it before cleaning caches.\n"
    )
    clean_caches.assert_not_called()


def test_clean_yes_removes_cache_without_prompting(
    cache_dir: Path,
    cleanup_guard: MagicMock,
    mocker: MockerFixture,
) -> None:
    cache_dir.mkdir()
    mocker.patch.object(cache_cli, "get_most_recent", return_value=None)
    clean_caches = mocker.patch.object(
        cache_cli,
        "clean_caches",
        return_value=CacheCleanResult(cleaned=[cache_dir]),
    )

    result = CliRunner().invoke(main, ["cache", "clean", "--yes"])

    assert result.exit_code == 0
    assert f"Cleaned cache: {cache_dir}" in result.output
    assert "Continue?" not in result.output
    cleanup_guard.assert_called_once_with()
    clean_caches.assert_called_once_with()


def test_clean_reports_deletion_failure(
    cache_dir: Path,
    cleanup_guard: MagicMock,
    mocker: MockerFixture,
) -> None:
    cache_dir.mkdir()
    mocker.patch.object(cache_cli, "get_most_recent", return_value=None)
    clean_caches = mocker.patch.object(
        cache_cli,
        "clean_caches",
        return_value=CacheCleanResult(failed=[CacheIssue(cache_dir, "denied")]),
    )

    result = CliRunner().invoke(main, ["cache", "clean", "--yes"])

    assert result.exit_code == 1
    assert f"Failed to remove cache: {cache_dir} (denied)" in result.output
    cleanup_guard.assert_called_once_with()
    clean_caches.assert_called_once_with()


def test_clean_without_cache_is_a_noop(
    cache_dir: Path,
    mocker: MockerFixture,
) -> None:
    mocker.patch.object(cache_cli, "get_most_recent", return_value=None)
    clean_caches = mocker.patch.object(cache_cli, "clean_caches")

    result = CliRunner().invoke(main, ["cache", "clean"])

    assert result.exit_code == 0
    assert result.output == f"No DimOS cache found at {cache_dir}.\n"
    clean_caches.assert_not_called()
