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
from types import SimpleNamespace

import pytest
from pytest_mock import MockerFixture
from typer.testing import CliRunner

import dimos.robot.cli.cache as cache_cli
from dimos.robot.cli.dimos import main
from dimos.utils.cache import CacheCleanResult, CacheIssue


@pytest.fixture
def cache_dir(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> Path:
    path = tmp_path / "cache"
    monkeypatch.setattr(cache_cli, "CACHE_DIR", path)
    return path


def test_clean_refuses_active_run_even_when_forced(
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

    result = CliRunner().invoke(main, ["cache", "clean", "--force", "--yes"])

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
    assert f"Cache root:\n  {cache_dir}" in result.output
    assert f"  - {unknown_entry} (DimOS cache entry)" in result.output
    assert f"  - {deno_dir} (downloaded Deno runtime)" in result.output
    assert "Continue? [y/N]:" in result.output
    assert result.output.endswith("Cache cleanup cancelled.\n")
    clean_caches.assert_not_called()


def test_clean_force_forwards_option(
    cache_dir: Path,
    mocker: MockerFixture,
) -> None:
    cache_dir.mkdir()
    mocker.patch.object(cache_cli, "get_most_recent", return_value=None)
    clean_caches = mocker.patch.object(
        cache_cli,
        "clean_caches",
        return_value=CacheCleanResult(cleaned=[cache_dir]),
    )

    result = CliRunner().invoke(main, ["cache", "clean", "--force", "--yes"])

    assert result.exit_code == 0
    assert "Removes robot Git checkouts with local changes or local-only commits" in result.output
    assert f"Cleaned cache: {cache_dir}" in result.output
    clean_caches.assert_called_once_with(force=True)


def test_clean_reports_skipped_checkout(
    cache_dir: Path,
    mocker: MockerFixture,
) -> None:
    cache_dir.mkdir()
    checkout = cache_dir / "robot_assets" / "sources" / "key" / "robot"
    mocker.patch.object(cache_cli, "get_most_recent", return_value=None)
    mocker.patch.object(
        cache_cli,
        "clean_caches",
        return_value=CacheCleanResult(
            skipped=[CacheIssue(checkout, "Git checkout has local changes")]
        ),
    )

    result = CliRunner().invoke(main, ["cache", "clean", "--yes"])

    assert result.exit_code == 1
    assert f"Skipped cache: {checkout} (Git checkout has local changes)" in result.output


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
