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

from dimos.cli import dimos as cli


@pytest.fixture
def config_home(monkeypatch: pytest.MonkeyPatch, tmp_path: Path) -> Path:
    monkeypatch.setattr(cli, "CONFIG_DIR", tmp_path)
    return tmp_path


def test_legacy_flat_file_migrates_into_directory(config_home: Path) -> None:
    legacy = config_home / "dimos"
    legacy.write_text('{"viewer": "rerun"}')

    cli._migrate_legacy_config()

    assert legacy.is_dir()
    assert (legacy / "config").read_text() == '{"viewer": "rerun"}'


def test_migration_noop_without_legacy_file(config_home: Path) -> None:
    cli._migrate_legacy_config()  # nothing exists
    assert not (config_home / "dimos").exists()

    (config_home / "dimos").mkdir()  # already migrated
    cli._migrate_legacy_config()
    assert (config_home / "dimos").is_dir()
