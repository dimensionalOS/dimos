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
import typer

from dimos.cli.commands import lifecycle as cli


@pytest.fixture
def config_dir(monkeypatch: pytest.MonkeyPatch, tmp_path: Path) -> Path:
    config_dir = tmp_path / "dimos"
    monkeypatch.setattr(cli, "CONFIG_DIR", config_dir)
    return config_dir


def test_legacy_flat_file_is_refused_with_instructions(
    config_dir: Path, capsys: pytest.CaptureFixture[str]
) -> None:
    config_dir.write_text('{"viewer": "rerun"}')
    with pytest.raises(typer.Exit):
        cli._reject_legacy_config()
    assert "mv " in capsys.readouterr().err
    assert config_dir.read_text() == '{"viewer": "rerun"}'


def test_no_legacy_file_passes(config_dir: Path) -> None:
    cli._reject_legacy_config()  # nothing exists

    config_dir.mkdir()  # already a directory
    cli._reject_legacy_config()
