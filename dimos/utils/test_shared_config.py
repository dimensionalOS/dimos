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

from dimos.constants import RECORDINGS_DIR
from dimos.core.global_config import global_config
from dimos.memory.module import default_recording_dir
from dimos.memory.tap import recording_dir
from dimos.utils.shared_config import dimos_home, recordings_dir, setting


@pytest.fixture(autouse=True)
def home(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> Path:
    # never the config.yaml of the machine running the tests, nor its --record-dir
    monkeypatch.setenv("DIMOS_HOME", str(tmp_path / "dimos_home"))
    monkeypatch.setattr(global_config, "record_dir", "")
    return tmp_path / "dimos_home"


def write_config(home: Path, text: str) -> None:
    home.mkdir(parents=True, exist_ok=True)
    (home / "config.yaml").write_text(text)


def test_a_missing_or_empty_setting_is_unset(home: Path) -> None:
    assert (dimos_home(), setting("recordings", "dir")) == (home, None)
    write_config(home, "zenoh: {namespace: desk1}\n")
    assert (setting("recordings", "dir"), setting("zenoh", "namespace")) == (None, "desk1")
    write_config(home, "recordings: {dir: ''}\n")
    assert setting("recordings", "dir") is None


def test_without_a_config_file_recordings_stay_where_they_are_today() -> None:
    """A developer in a checkout who has configured nothing keeps the checkout's `recordings/`."""
    assert recordings_dir() == RECORDINGS_DIR
    assert recording_dir().parent == RECORDINGS_DIR
    assert default_recording_dir().parent == RECORDINGS_DIR


def test_the_configured_directory_is_where_every_recorder_writes(home: Path) -> None:
    write_config(home, f"recordings:\n  dir: {home.parent / 'recs'}\n")
    assert recordings_dir() == home.parent / "recs"
    # the `--record` recorder, and a recorder module that is not part of a `dimos run`
    assert recording_dir().parent == home.parent / "recs"
    assert default_recording_dir().parent == home.parent / "recs"


def test_a_tilde_in_the_configured_directory_is_the_users_home(home: Path) -> None:
    write_config(home, "recordings:\n  dir: ~/elsewhere\n")
    assert recordings_dir() == Path.home() / "elsewhere"


def test_record_dir_wins_because_that_is_how_the_gateway_hands_it_over(
    home: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    """`--record-dir` is dimos-gateway passing on what `GET /history/recordings` answers with."""
    write_config(home, f"recordings:\n  dir: {home.parent / 'recs'}\n")
    monkeypatch.setattr(global_config, "record_dir", str(home.parent / "told"))
    assert recordings_dir() == home.parent / "told"
    assert recording_dir().parent == home.parent / "told"
